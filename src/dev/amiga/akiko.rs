//! Akiko: the CD32's own gate array — its chunky-to-planar converter, the
//! CD-ROM controller's register file, and the two wires of the machine's
//! serial EEPROM.
//!
//! One class, `amiga.akiko`.
//!
//! # What Commodore published, which is one paragraph
//!
//! The *Amiga CD32 Developer Notes* (Revision 3, Commodore-Amiga Inc.) is the
//! only Commodore document about this machine, and it says:
//!
//! > Akiko is a 160-pin PQFP surface-mounted device containing most of the
//! > remainder of the logic necessary in the Amiga CD 32. It includes the
//! > CD-ROM control logic and the system timers.
//!
//! and, among the machine's features, "Amiga CD 32 Includes very fast
//! chunky-to-planar conversion hardware" and "The EEPROM is 8K bits, and
//! designed for storing settings, high scores, bookmarks, etc., on the Amiga
//! CD 32."
//!
//! **That is all of it.** The Developer Notes are a guide for a game author:
//! they say to call `WriteChunkyPixels()` in `graphics.library`,
//! `ReadJoyPort()` in `lowlevel.library`, `CD_READ` in `cd.device` and
//! `nonvolatile.library` for the EEPROM, and they print no register address,
//! no bit and no command opcode anywhere. Commodore published no hardware
//! reference for Akiko.
//!
//! So this file is written the way [`super::gayle`]'s identification register
//! was: from **what the user's own ROM does, watched from the bus**, plus the
//! facts that belong to the parts around Akiko rather than to anybody's
//! emulation of it. Each section below says which of the two it is, and an
//! inference is called an inference. `ROADMAP.md` §1: no emulator source of
//! any licence, no FPGA reimplementation, no AROS source and no Kickstart
//! disassembly was consulted.
//!
//! # How the register file is reached
//!
//! The board's select is `$B8_0000`–`$B8_7FFF` and the file is sixty-four
//! bytes of longword registers, big-endian. Everything above `$3F` in the
//! window **repeats** those sixty-four bytes; that is an inference, from the
//! shape of every other Amiga gate array whose select is wider than its
//! address pins (Gayle's four registers fill 4 KiB pages for exactly that
//! reason), and nothing observed reads above `$3F`.
//!
//! An address in the file that this model does not claim reads **zero**
//! rather than floating. That is a property of the chip and not of the board:
//! the board's policy is open bus, and the CD32's ROM reads `$19`, `$1A` and
//! `$24` before it has written them and goes on, so something is driving the
//! bus there.
//!
//! **One register is two bus cycles here.** This tree's 68000 core issues a
//! long access as two word cycles, high word first — or low word first for a
//! read-modify-write (`src/cpu/m68k/exec.rs`, `write_long`) — because a
//! 68000's data bus is sixteen bits wide. A CD32's Akiko is on a 32-bit local
//! bus and sees one cycle. So every side effect below is defined per *byte*
//! rather than per longword, which gives the same answer either way and needs
//! no rule about which half completes a register.
//!
//! # `$00`: the identification longword
//!
//! `$C0CACAFE`. **Black-box**: the first thing the CD32's extended ROM does
//! here is read the word at `$B8_0002`, and that is the only read of the
//! register in a whole boot; `$CAFE` is what lets it go on. The top half is
//! never read by the ROM and is the well-known rest of the joke.
//!
//! # `$04`–`$2F`: the CD-ROM controller
//!
//! **This is the part that is not settled, and it is worth being exact about
//! what is known.** With an empty tray — which is what a CD32 does when it
//! runs its boot screen — the ROM's `cd.device` does this, once, and then
//! polls `$04` and `$08` for the rest of the run:
//!
//! ```text
//!   read $25.b (= 0)  write $25.b = $80  read $25.b (= $80)  write $25.b = 0
//!   write $1F.b = $00            write $1D.b = $00
//!   write $10   = $0001_0000     write $14 = $001F_E400
//!   read  $19.b                  read  $24 (= 0)   write $24 = $7900_0000
//!   read  $1A.b                  write $1F.b = $01  write $1D.b = $00
//!   write $08 = $1800_0000       write $1D.b = $03
//!   write $08 = $1800_0000       write $1D.b = $05
//!   write $08 = $1800_0000
//!   then: read $04, read $08, read $04, read $08, …
//! ```
//!
//! What can be said from that, and no more:
//!
//! * `$25` is written and read back before anything else, which is a **write
//!   test**: something has to answer there with what was put in it, or the ROM
//!   stops. This model's registers do, because they are storage.
//! * `$04` and `$08` are an **interrupt request and its enable**, read as a
//!   pair in the poll and written only in bits 31–24. Nothing here ever sets
//!   a request bit, because nothing here knows which bit means what.
//! * `$10` and `$14` are two **pointers into chip RAM** — `$1F_E400` is
//!   inside this machine's 2 MiB and `$1_0000` is an allocation — almost
//!   certainly the command and status rings `cd.device` and the controller
//!   pass messages through. Their layout is not known and this file does not
//!   pretend to one: the longwords are stored and read back, and nothing
//!   walks them.
//! * `$18` and `$1C` hold **byte indices** into those rings, the chip's at
//!   `$19`/`$1A` and software's at `$1D`/`$1F`. `$18` is read-only here and
//!   stays zero, which is a controller that has produced nothing.
//! * `$24` takes `$7900_0000` once and is read back; what it configures is
//!   not known.
//!
//! **So no disc reaches the guest through this class.** [`super::cdrom`] is a
//! complete drive — it finds its sectors, lifts Mode 1 user data out of raw
//! frames and answers a table of contents — and Akiko holds it and can say
//! whether a disc is in the tray, but the message format that would carry a
//! `CD_READ` from `cd.device` to the mechanism is undocumented and could not
//! be recovered from a boot with an empty tray, because with an empty tray the
//! ROM never sends one. Inventing a plausible one would be a fiction that no
//! real disc and no real game would agree with, so this file does not. That is
//! the one substantial hole in the CD32 here, and it is written down rather
//! than papered over.
//!
//! For the same reason there is **no interrupt pin** on this class. Which of
//! the processor's levels a CD interrupt reaches, and through what, was not
//! determined; the ROM polls `$04` regardless, so the boot does not depend on
//! it, and a pin that is never asserted would only look like knowledge.
//!
//! # `$30`: the serial EEPROM's two wires
//!
//! The Developer Notes' "8K bits" is 1 KiB, and a 1 KiB serial EEPROM on two
//! wires is a 24C08. Akiko does not run the bus for software; it gives it the
//! two pins, and the ROM bit-bangs them. The **bit assignment is black-box**
//! and the trace settles it completely:
//!
//! ```text
//!   bit 31  SCL level      bit 15  drive SCL
//!   bit 30  SDA level      bit 14  drive SDA
//! ```
//!
//! The derivation, because it is the good kind of evidence. The ROM writes,
//! in order, `$C000_C000`, `$8000_C000`, `$0000_C000`, `$4000_C000`,
//! `$C000_C000`, … With bit 31 read as SCL and bit 30 as SDA, that opening is
//! a textbook I²C **start condition** — SDA falls while SCL is high — and the
//! bits clocked out after it are `1 0 1 0 0 0 0 0`, which is `$A0`: the write
//! address of a 24Cxx and of nothing else. Read the two bits the other way
//! round and the same sequence is a start condition that never happens and an
//! address of `$C0`. Then, for the ninth clock, the low half changes to
//! `$8000` — one of the two drivers lets go — and the ROM *reads* the
//! register. A master releases SDA for the acknowledge and holds SCL, so the
//! driver that stayed is SCL's: bit 15 drives SCL and bit 14 drives SDA.
//!
//! What the part on the other end does with those wires is not an inference at
//! all. It is the I²C specification (NXP UM10204: a start, a stop, a bit per
//! rising edge of SCL, the slave's acknowledge on the ninth) and the 24C08's
//! data sheet (device address `1010`, two page-select bits in the address
//! byte, a word address, sequential read, a sixteen-byte page write).
//! [`Eeprom`] is that part, and it is a slave that answers rather than a
//! recording of what the ROM asked for.
//!
//! A read of `$30` returns the **wire** levels in bits 31 and 30 — a pin
//! nobody drives is pulled up, and a pin either end pulls down is low — and
//! the direction bits as they were written. That is the only reading under
//! which the acknowledge above can be seen at all.
//!
//! The confirmation that all of this is right is a count. With nothing
//! answering at the other end, the ROM's five virtual seconds of boot contain
//! **207 005** accesses to this register file, almost all of them `$30`: the
//! transfer fails and is retried without end. With [`Eeprom`] on the wires it
//! is **311**, the transfer happens once, and the boot goes on. A model that
//! had the two bits the wrong way round could not produce that.
//!
//! The EEPROM's 1 KiB lives in this device and in its snapshot. It comes up
//! erased — `$FF` everywhere, which is what an erased floating-gate part
//! reads — and there is deliberately no media slot for it: `--media` copies
//! bytes *in* and never writes them back, so a slot would look like
//! persistence without being any.
//!
//! # `$38`: chunky to planar
//!
//! The Developer Notes call it "very fast chunky-to-planar conversion
//! hardware", and the machine's shape fixes the arithmetic: thirty-two 8-bit
//! chunky pixels in, eight 32-bit planar longwords out, a corner-turn memory.
//! Thirty-two pixels is 256 bits on either side; the converter is a square bit
//! matrix written by rows and read by columns, and there is only one such
//! matrix:
//!
//! ```text
//!   plane[p] bit (31 - i)  =  (chunky[i] >> p) & 1      i = 0..31, p = 0..7
//! ```
//!
//! [`corner_turn`] is that, and the tests compute it a second way rather than
//! calling it.
//!
//! **The port's address, its pixel order and its plane order were measured,
//! not chosen**, and the measurement is the nicest thing in this file. The
//! CD32's own ROM proves the converter on its way up: it writes eight
//! longwords of `$5555_0000` — thirty-two chunky pixels `55 55 00 00` over and
//! over — and reads the result back. Under the transform above plane 0 of that
//! is `$CCCC_CCCC`, plane 1 is zero, and so on alternating.
//!
//! * With the port at `$38`, the first pixel in the most significant byte and
//!   plane 0 first, the ROM reads **four** longwords — `$CCCC_CCCC`, `0`,
//!   `$CCCC_CCCC`, `0` — and goes on to the next thing it does, which is to
//!   write `$80` into `$25` and read it back.
//! * Turn the plane order round, so plane 7 comes out first, and the same ROM
//!   reads **one** longword, gets the `0` that is plane 7, and stops: no
//!   second read, and `$25` is never touched.
//! * Turn the pixel order round instead, so pixel 0 is the least significant
//!   bit, and it reads **one** longword of `$3333_3333` and stops in exactly
//!   the same place.
//!
//! So the ROM checks what it gets, and only this arrangement satisfies it.
//! `$38` is likewise not a guess: the writes go there.
//!
//! The one thing still inferred is that **one byte pointer serves both sides**
//! and wraps at thirty-two, which is why eight longwords written leave it back
//! at the start and the first read is plane 0. A corner-turn memory is one
//! memory; two pointers would be two. The trace cannot tell the two apart —
//! separate pointers both start at zero — so this is written down as the
//! inference it is.
//!
//! # Not modelled
//!
//! * **The system timers.** Named in one sentence of the Developer Notes and
//!   touched by nothing observed; those registers read zero.
//! * **Akiko as "system address decoder".** The CD32's decode is in
//!   `machines/amiga-cd32.machine`, where every other Amiga's is. Which chip
//!   performs it is a fact about the board, not about what a guest can see.
//! * **CD audio, subcode and CD+G.** [`super::cdrom`] says what the drive does
//!   and does not do.
//!
//! # `MemAttrs::debug`
//!
//! A debug read answers what the processor would see and moves nothing: the
//! corner turn's pointer does not advance and the EEPROM's bus is not stepped.
//! A debug write is refused, because every write here either clocks that bus
//! or fills the converter.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.akiko";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// How much of the map the chip's select covers: `$B8_0000`–`$B8_7FFF`.
pub const WINDOW: u64 = 0x8000;

/// How much of it is register file; the rest of the window repeats it.
pub const REGISTERS: u64 = 64;

/// What `$00` reads.
pub const ID: u32 = 0xC0CA_CAFE;

/// How many chunky pixels one corner-turn matrix holds.
pub const C2P_PIXELS: usize = 32;

/// How many longwords that is on either side of the turn.
pub const C2P_WORDS: usize = 8;

/// How much the EEPROM holds: the Developer Notes' "8K bits".
pub const NVRAM_BYTES: usize = 1024;

/// Register offsets within the sixty-four byte file.
mod off {
    /// The identification longword.
    pub(super) const ID: u64 = 0x00;
    /// The controller's pending events.
    pub(super) const INTREQ: u64 = 0x04;
    /// Which of them would reach the processor.
    pub(super) const INTENA: u64 = 0x08;
    /// The controller's own ring indices; read-only.
    pub(super) const CHIP_INDICES: u64 = 0x18;
    /// The EEPROM's two wires and their drivers.
    pub(super) const NVRAM: u64 = 0x30;
    /// The corner-turn memory's one port.
    pub(super) const C2P: u64 = 0x38;
}

/// `$30` bit 31: the level this end puts on SCL.
const SCL_LEVEL: u32 = 1 << 31;
/// `$30` bit 30: the level this end puts on SDA.
const SDA_LEVEL: u32 = 1 << 30;
/// `$30` bit 15: whether this end drives SCL at all.
const SCL_DRIVE: u32 = 1 << 15;
/// `$30` bit 14: whether this end drives SDA at all.
const SDA_DRIVE: u32 = 1 << 14;

/// Only bits 31–24 of the interrupt pair are writable: the ROM writes nothing
/// else, and a register that answered in the rest would be a claim.
const INT_BITS: u32 = 0xFF00_0000;

// ---------------------------------------------------------------------------
// the corner turn
// ---------------------------------------------------------------------------

/// Thirty-two chunky pixels as eight planar longwords.
///
/// `chunky[i]` is pixel *i*, leftmost first; the result's `plane[p]` carries
/// pixel *i* in bit `31 - i`. This is the whole of the converter's arithmetic
/// and it is its own specification: a 32 × 8 bit matrix read out by columns.
#[must_use]
pub fn corner_turn(chunky: &[u8; C2P_PIXELS]) -> [u32; C2P_WORDS] {
    let mut planes = [0u32; C2P_WORDS];
    for (i, &pixel) in chunky.iter().enumerate() {
        for (p, plane) in planes.iter_mut().enumerate() {
            if pixel >> p & 1 != 0 {
                *plane |= 1 << (31 - i);
            }
        }
    }
    planes
}

// ---------------------------------------------------------------------------
// the EEPROM
// ---------------------------------------------------------------------------

/// Where a 24C08 is in a transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Between a stop and the next start: not addressed.
    Idle,
    /// Started; the byte coming in is the device address.
    Device,
    /// Addressed for writing; the byte coming in is the word address.
    WordAddress,
    /// Addressed for writing; the bytes coming in are data.
    Writing,
    /// Addressed for reading; a byte is going out.
    Reading,
}

/// A 24C08: 1 KiB on a two-wire bus at device address `1010`, with the two
/// page-select bits in the address byte.
///
/// The bus behaviour is the I²C specification (NXP UM10204) and the part's
/// data sheet, not an inference: a start is SDA falling while SCL is high, a
/// stop is SDA rising while SCL is high, a bit is sampled on SCL's rising
/// edge, and the slave answers the ninth clock of every byte it accepts by
/// pulling SDA low.
#[derive(Debug, Clone)]
pub struct Eeprom {
    /// The cells. An erased part reads `$FF`.
    cells: [u8; NVRAM_BYTES],
    phase: Phase,
    /// The wire levels last seen, for edge detection.
    scl: bool,
    sda: bool,
    /// The byte being shifted in or out.
    shift: u8,
    /// How many clocks of the byte have finished, 0–8. Eight is the ninth
    /// clock: the acknowledge, whichever end drives it.
    clocks: u8,
    /// True while *this* part is holding SDA low for an acknowledge.
    acking: bool,
    /// The master's acknowledge, sampled on the ninth clock of a byte this
    /// part read out.
    master_ack: bool,
    /// Whether the ninth clock of a byte is high now.
    ninth: bool,
    /// Whether this part is presenting a data bit on SDA — true from the
    /// moment a byte is loaded to read out until the eighth clock has fallen,
    /// which is when the master takes the line back for its acknowledge.
    presenting: bool,
    /// The cell the next data byte lands on or comes from.
    addr: u16,
}

impl Default for Eeprom {
    fn default() -> Eeprom {
        Eeprom::erased()
    }
}

impl Eeprom {
    /// A part straight out of the factory: every cell `$FF`.
    #[must_use]
    pub fn erased() -> Eeprom {
        Eeprom {
            cells: [0xFF; NVRAM_BYTES],
            phase: Phase::Idle,
            scl: true,
            sda: true,
            shift: 0,
            clocks: 0,
            acking: false,
            master_ack: false,
            ninth: false,
            presenting: false,
            addr: 0,
        }
    }

    /// The same part, holding `cells` and with an idle bus.
    #[must_use]
    pub fn holding(cells: [u8; NVRAM_BYTES]) -> Eeprom {
        Eeprom {
            cells,
            ..Eeprom::erased()
        }
    }

    /// Every cell, for a snapshot or a test.
    #[must_use]
    pub fn cells(&self) -> &[u8; NVRAM_BYTES] {
        &self.cells
    }

    /// Whether the part is pulling SDA low this instant.
    fn pulls_sda_low(&self) -> bool {
        if self.acking {
            return true;
        }
        // While reading out, the part drives the bit it is presenting — and
        // lets go for the ninth clock, which is the master's to drive.
        self.presenting && self.shift >> 7 == 0
    }

    /// Step the part with the levels the **master** is putting on the two
    /// wires.
    ///
    /// Not the wire, which is the master's level and this part's pull-down
    /// together. A slave has to know its own acknowledge from a start
    /// condition, and on the wire the two look identical: both are SDA going
    /// low while SCL is high. It knows because it is the one driving.
    fn step(&mut self, scl: bool, sda: bool) {
        // Start and stop are SDA moving while SCL is high.
        if self.scl && scl && sda != self.sda {
            if sda {
                self.phase = Phase::Idle;
            } else {
                self.phase = Phase::Device;
                self.shift = 0;
            }
            self.clocks = 0;
            self.acking = false;
            self.ninth = false;
            self.presenting = false;
            self.scl = scl;
            self.sda = sda;
            return;
        }
        if scl && !self.scl {
            self.rising(sda);
        } else if !scl && self.scl {
            self.falling();
        }
        self.scl = scl;
        self.sda = sda;
    }

    /// SCL's rising edge: whatever is on SDA is the valid bit.
    ///
    /// The clock is counted here rather than on the falling edge, because a
    /// start condition leaves SCL to fall with no bit between — and a falling
    /// edge that counted would swallow the first bit of every transfer.
    fn rising(&mut self, sda: bool) {
        if self.phase == Phase::Idle {
            return;
        }
        if self.clocks == 8 {
            // The ninth clock. If this part is not driving it, the master is,
            // and what it puts there says whether it wants another byte.
            self.ninth = true;
            if self.phase == Phase::Reading && !self.acking {
                self.master_ack = !sda;
            }
            return;
        }
        if self.phase != Phase::Reading {
            self.shift = self.shift << 1 | u8::from(sda);
        }
        self.clocks += 1;
        if self.clocks == 8 && self.phase != Phase::Reading {
            self.byte_in();
        }
    }

    /// SCL's falling edge: the clock is finished and SDA may move again.
    fn falling(&mut self) {
        if self.phase == Phase::Idle {
            return;
        }
        if self.ninth {
            self.ninth = false;
            self.clocks = 0;
            if self.acking {
                self.acking = false;
                if self.phase == Phase::Reading {
                    self.shift = self.cells[self.addr as usize];
                    self.presenting = true;
                }
            } else if self.phase == Phase::Reading {
                if self.master_ack {
                    self.addr = (self.addr + 1) % NVRAM_BYTES as u16;
                    self.shift = self.cells[self.addr as usize];
                    self.presenting = true;
                } else {
                    // A master that did not acknowledge is finished; its stop
                    // condition follows.
                    self.phase = Phase::Idle;
                }
            }
            return;
        }
        // Reading: present the next bit for the master to take, or let go of
        // the line once all eight have been taken.
        if self.phase == Phase::Reading {
            if self.clocks < 8 {
                self.shift <<= 1;
            } else {
                self.presenting = false;
            }
        }
    }

    /// A whole byte arrived.
    fn byte_in(&mut self) {
        let byte = self.shift;
        match self.phase {
            Phase::Device => {
                // 1 0 1 0 P1 P0 R/W: on a 24C08 the two page bits are the top
                // of the word address rather than a chip select.
                if byte >> 4 != 0b1010 {
                    self.phase = Phase::Idle;
                    return;
                }
                let page = u16::from(byte >> 1 & 3) << 8;
                self.acking = true;
                if byte & 1 != 0 {
                    self.addr = (self.addr & 0xFF) | page;
                    self.phase = Phase::Reading;
                } else {
                    self.addr = page;
                    self.phase = Phase::WordAddress;
                }
            }
            Phase::WordAddress => {
                self.addr = (self.addr & 0x300) | u16::from(byte);
                self.acking = true;
                self.phase = Phase::Writing;
            }
            Phase::Writing => {
                self.cells[self.addr as usize] = byte;
                // A 24C08 writes within a sixteen-byte page and wraps inside
                // it: only the low four bits of the address move.
                self.addr = (self.addr & !0xF) | (self.addr.wrapping_add(1) & 0xF);
                self.acking = true;
            }
            Phase::Idle | Phase::Reading => {}
        }
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Everything the chip holds.
#[derive(Debug, Clone)]
struct State {
    /// The sixteen longwords, as software last left them.
    regs: [u32; 16],
    /// The corner-turn memory, as the thirty-two chunky bytes written into it.
    chunky: [u8; C2P_PIXELS],
    /// Which byte of it the one pointer is on.
    c2p_at: u8,
    /// The part on the other end of `$30`'s two wires.
    nvram: Eeprom,
}

impl Default for State {
    fn default() -> State {
        State {
            regs: [0; 16],
            chunky: [0; C2P_PIXELS],
            c2p_at: 0,
            nvram: Eeprom::erased(),
        }
    }
}

impl State {
    /// What this end is putting on the two wires: a pin it does not drive is
    /// released, which a pull-up takes high.
    fn driven(&self) -> (bool, bool) {
        let r = self.regs[(off::NVRAM / 4) as usize];
        (
            r & SCL_DRIVE == 0 || r & SCL_LEVEL != 0,
            r & SDA_DRIVE == 0 || r & SDA_LEVEL != 0,
        )
    }

    /// The wire levels `$30` reads back: what either end pulls low is low.
    /// The EEPROM never stretches SCL, so only SDA has two drivers.
    fn wires(&self) -> (bool, bool) {
        let (scl, sda) = self.driven();
        (scl, sda && !self.nvram.pulls_sda_low())
    }

    /// The longword at `reg`, as the processor would see it, moving nothing.
    fn read(&self, reg: u64) -> u32 {
        match reg {
            off::ID => ID,
            off::C2P => {
                let planes = corner_turn(&self.chunky);
                let at = (self.c2p_at as usize) & !3;
                let mut out = [0u8; 4];
                for (i, b) in out.iter_mut().enumerate() {
                    let byte = (at + i) % C2P_PIXELS;
                    *b = planes[byte / 4].to_be_bytes()[byte % 4];
                }
                u32::from_be_bytes(out)
            }
            off::NVRAM => {
                let (scl, sda) = self.wires();
                let mut v = self.regs[(off::NVRAM / 4) as usize] & !(SCL_LEVEL | SDA_LEVEL);
                if scl {
                    v |= SCL_LEVEL;
                }
                if sda {
                    v |= SDA_LEVEL;
                }
                v
            }
            _ => self.regs[(reg / 4) as usize],
        }
    }

    /// One byte written at `offset` in the file.
    fn write_byte(&mut self, offset: u64, byte: u8) {
        let reg = offset & !3;
        let lane = (offset & 3) as usize;
        match reg {
            // Read-only: the identification, the controller's request word and
            // the indices the controller keeps for itself.
            off::ID | off::INTREQ | off::CHIP_INDICES => {}
            off::C2P => {
                let at = self.c2p_at as usize;
                self.chunky[at] = byte;
                self.c2p_at = ((at + 1) % C2P_PIXELS) as u8;
            }
            off::NVRAM => {
                let i = (off::NVRAM / 4) as usize;
                let mut v = self.regs[i].to_be_bytes();
                v[lane] = byte;
                self.regs[i] = u32::from_be_bytes(v);
                let (scl, sda) = self.driven();
                self.nvram.step(scl, sda);
            }
            off::INTENA => {
                let i = (off::INTENA / 4) as usize;
                let mut v = self.regs[i].to_be_bytes();
                v[lane] = byte;
                self.regs[i] = u32::from_be_bytes(v) & INT_BITS;
            }
            _ => {
                let i = (reg / 4) as usize;
                let mut v = self.regs[i].to_be_bytes();
                v[lane] = byte;
                self.regs[i] = u32::from_be_bytes(v);
            }
        }
    }

    /// One byte read at `offset`, with whatever reading it moves.
    fn read_byte(&mut self, offset: u64, debug: bool) -> u8 {
        if offset & !3 == off::C2P {
            let at = self.c2p_at as usize;
            let planes = corner_turn(&self.chunky);
            let byte = planes[at / 4].to_be_bytes()[at % 4];
            if !debug {
                self.c2p_at = ((at + 1) % C2P_PIXELS) as u8;
            }
            return byte;
        }
        self.read(offset & !3).to_be_bytes()[(offset & 3) as usize]
    }
}

// ---------------------------------------------------------------------------
// the register window
// ---------------------------------------------------------------------------

struct Registers {
    state: Mutex<State>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers")
            .field("c2p_at", &self.state.lock().c2p_at)
            .finish_non_exhaustive()
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let base = offset % REGISTERS;
        if dst.is_empty() || dst.len() > 4 || (base & 3) + dst.len() as u64 > 4 {
            return Err(BusError::BadAccess);
        }
        let mut st = self.state.lock();
        for (i, b) in dst.iter_mut().enumerate() {
            *b = st.read_byte(base + i as u64, attrs.debug);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Every write here either clocks the EEPROM's bus or fills the
            // corner-turn memory; a debugger may do neither.
            return Err(BusError::BadAccess);
        }
        let base = offset % REGISTERS;
        if src.is_empty() || src.len() > 4 || (base & 3) + src.len() as u64 > 4 {
            return Err(BusError::BadAccess);
        }
        let mut st = self.state.lock();
        for (i, &b) in src.iter().enumerate() {
            st.write_byte(base + i as u64, b);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints {
            min: Width::U8,
            natural_alignment: false,
            ..AccessConstraints::word(Width::U32, Endian::Big)
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// Akiko.
#[derive(Debug)]
pub struct Akiko {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The object the board named as the mechanism, if it named one.
    drive_path: Option<String>,
    /// What that object said when bind asked it whether a disc was in it.
    disc: Mutex<Option<bool>>,
}

impl Akiko {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `drive` is not a link, or a property nothing
    /// here accepts was given.
    pub fn new(props: &Props) -> Result<Akiko> {
        let mut r = props.reader();
        let drive_path = r.optional_link("drive")?.map(|l| l.as_str().to_string());
        r.finish()?;
        Ok(Akiko::with_drive(drive_path))
    }

    fn with_drive(drive_path: Option<String>) -> Akiko {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            WINDOW,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Akiko {
            regs,
            region,
            drive_path,
            disc: Mutex::with_rank(LockRank::LEAF, None),
        }
    }

    /// The register file with no drive named.
    #[must_use]
    pub fn bare() -> Akiko {
        Akiko::with_drive(None)
    }

    /// Whether the drive the board named had a disc in it when bind asked.
    ///
    /// `None` before bind, and on a board that named no drive at all. Nothing
    /// in the register file answers from it yet, for the reason the module
    /// documentation gives: the message format `cd.device` would ask through
    /// is undocumented.
    #[must_use]
    pub fn disc_present(&self) -> Option<bool> {
        *self.disc.lock()
    }

    /// Read a register the way the processor would, without moving anything.
    #[must_use]
    pub fn peek(&self, reg: u64) -> u32 {
        self.regs.state.lock().read((reg % REGISTERS) & !3)
    }

    /// The EEPROM's cells, for a test that wants to see what was stored.
    #[must_use]
    pub fn nvram(&self) -> [u8; NVRAM_BYTES] {
        *self.regs.state.lock().nvram.cells()
    }

    /// Put `cells` in the EEPROM, as a part that has been used before.
    pub fn set_nvram(&self, cells: [u8; NVRAM_BYTES]) {
        self.regs.state.lock().nvram = Eeprom::holding(cells);
    }
}

impl Device for Akiko {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let mut st = self.regs.state.lock();
        let cells = *st.nvram.cells();
        // Everything but the cells: a reset line is not an erase, which is the
        // whole reason the part is on the board.
        *st = State {
            nvram: Eeprom::holding(cells),
            ..State::default()
        };
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.regs.state.lock();
        for r in st.regs {
            w.write_u32(r)?;
        }
        for b in st.chunky {
            w.write_u8(b)?;
        }
        w.write_u8(st.c2p_at)?;
        for b in st.nvram.cells() {
            w.write_u8(*b)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = [0u32; 16];
        for v in &mut regs {
            *v = r.read_u32()?;
        }
        let mut chunky = [0u8; C2P_PIXELS];
        for b in &mut chunky {
            *b = r.read_u8()?;
        }
        let c2p_at = r.read_u8()?;
        if c2p_at as usize >= C2P_PIXELS {
            return Err(Error::State(String::from(
                "amiga.akiko: a corner-turn pointer past its matrix",
            )));
        }
        let mut cells = [0u8; NVRAM_BYTES];
        for b in &mut cells {
            *b = r.read_u8()?;
        }
        let mut st = self.regs.state.lock();
        // Where the EEPROM's bus stood is *derived*: it is a position within a
        // transfer nobody can resume, and a part that lost its clock comes
        // back idle. The cells are the state; the shift register is not.
        *st = State {
            regs,
            chunky,
            c2p_at,
            nvram: Eeprom::holding(cells),
        };
        Ok(())
    }
}

impl Instance for Akiko {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let Some(path) = self.drive_path.as_deref() else {
            return Ok(());
        };
        #[cfg(feature = "dev-amiga-cdrom")]
        {
            use crate::core::device::ExportId;
            let drive = ctx
                .export_as::<super::cdrom::DrivePort>(path, ExportId::CD_DRIVE)
                .map_err(|e| Error::Config {
                    at: ctx.path().to_string(),
                    message: alloc::format!("`drive` has to name an `amiga.cd`: {e}"),
                })?;
            *self.disc.lock() = Some(drive.has_disc());
            Ok(())
        }
        #[cfg(not(feature = "dev-amiga-cdrom"))]
        Err(Error::Config {
            at: ctx.path().to_string(),
            message: String::from("`drive` needs the `dev-amiga-cdrom` feature"),
        })
    }
}

/// The `amiga.akiko` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Akiko, the CD32's gate array at $B80000: the chunky-to-planar corner turn, the \
              CD-ROM controller's registers and the serial EEPROM's two wires",
    properties: &[PropertySpec {
        name: "drive",
        kind: ValueKind::Link,
        required: false,
        summary: "the `amiga.cd` mechanism this controller is in front of",
    }],
    construct: |props| Ok(Box::new(Akiko::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Akiko::new(props)?)))
}

/// What the validator should know about `amiga.akiko`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("drive", ValueKind::Link))
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::vec::Vec;

    fn akiko() -> Akiko {
        Akiko::bare()
    }

    /// Read `len` bytes at `at` the way the processor would.
    fn read(a: &Akiko, at: u64, len: usize) -> u32 {
        let mut buf = [0u8; 4];
        MemOps::read(&*a.regs, at, &mut buf[..len], MemAttrs::DEFAULT).unwrap();
        let mut v = 0u32;
        for b in &buf[..len] {
            v = v << 8 | u32::from(*b);
        }
        v
    }

    fn write(a: &Akiko, at: u64, bytes: &[u8]) {
        MemOps::write(&*a.regs, at, bytes, MemAttrs::DEFAULT).unwrap();
    }

    /// A longword written the way this tree's 68000 core writes one: two word
    /// cycles, high half first.
    fn write_long(a: &Akiko, at: u64, value: u32) {
        let b = value.to_be_bytes();
        write(a, at, &b[0..2]);
        write(a, at + 2, &b[2..4]);
    }

    /// The same longword with the *low* half first, which is what a
    /// read-modify-write puts on the bus.
    fn write_long_low_first(a: &Akiko, at: u64, value: u32) {
        let b = value.to_be_bytes();
        write(a, at + 2, &b[2..4]);
        write(a, at, &b[0..2]);
    }

    fn read_long(a: &Akiko, at: u64) -> u32 {
        read(a, at, 2) << 16 | read(a, at + 2, 2)
    }

    // -- the identification --------------------------------------------------

    #[test]
    fn the_identification_reads_the_same_at_every_width_and_cannot_be_written() {
        let a = akiko();
        assert_eq!(read(&a, 0, 4), ID);
        assert_eq!(read(&a, 2, 2), ID & 0xFFFF);
        assert_eq!(read(&a, 0, 1), ID >> 24);
        write_long(&a, 0, 0);
        assert_eq!(read(&a, 0, 4), ID);
    }

    /// The register file repeats through the window; nothing observed reads
    /// above `$3F`, but the select is 32 KiB wide.
    #[test]
    fn the_file_repeats_every_sixty_four_bytes() {
        let a = akiko();
        assert_eq!(read(&a, REGISTERS, 4), ID);
        assert_eq!(read(&a, WINDOW - REGISTERS, 4), ID);
    }

    // -- the corner turn -----------------------------------------------------

    /// The transform, computed a second way: thirty-two columns of a bit
    /// matrix, assembled here rather than by calling `corner_turn`.
    fn planes_by_hand(chunky: &[u8; C2P_PIXELS]) -> [u32; C2P_WORDS] {
        let mut out = [0u32; C2P_WORDS];
        for (p, plane) in out.iter_mut().enumerate() {
            let mut bits = 0u32;
            for pixel in chunky {
                bits = bits << 1 | u32::from(pixel >> p & 1);
            }
            *plane = bits;
        }
        out
    }

    #[test]
    fn the_corner_turn_is_a_transposed_bit_matrix() {
        for seed in [0u8, 1, 0x55, 0x99, 0xFF] {
            let mut chunky = [0u8; C2P_PIXELS];
            for (i, c) in chunky.iter_mut().enumerate() {
                *c = seed.wrapping_mul(i as u8).wrapping_add(i as u8);
            }
            assert_eq!(corner_turn(&chunky), planes_by_hand(&chunky));
        }
    }

    /// The one the CD32's own ROM performs, and the numbers it insists on:
    /// thirty-two pixels of `55 55 00 00` give alternating `$CCCCCCCC` and
    /// zero, plane 0 first.
    #[test]
    fn the_converter_answers_what_the_rom_checks_for() {
        let a = akiko();
        for _ in 0..C2P_WORDS {
            write_long(&a, off::C2P, 0x5555_0000);
        }
        for plane in 0..C2P_WORDS {
            let want = if plane % 2 == 0 { 0xCCCC_CCCC } else { 0 };
            assert_eq!(read_long(&a, off::C2P), want, "plane {plane}");
        }
    }

    /// The port fills a byte at a time in **bus order**, so the two word
    /// cycles a `MOVE.L` puts out land exactly as four byte writes would.
    ///
    /// The other order is a read-modify-write, which nothing does to a
    /// write-only converter port; it is asserted here to be *different*, so
    /// that the model's rule is written down rather than assumed harmless.
    #[test]
    fn the_port_fills_in_bus_order() {
        let by_word = akiko();
        let by_byte = akiko();
        let reversed = akiko();
        for _ in 0..C2P_WORDS {
            write_long(&by_word, off::C2P, 0x0123_4567);
            for b in [0x01u8, 0x23, 0x45, 0x67] {
                write(&by_byte, off::C2P, &[b]);
            }
            write_long_low_first(&reversed, off::C2P, 0x0123_4567);
        }
        let mut differs = false;
        for _ in 0..C2P_WORDS {
            let word = read_long(&by_word, off::C2P);
            assert_eq!(word, read_long(&by_byte, off::C2P));
            differs |= word != read_long(&reversed, off::C2P);
        }
        assert!(differs, "a low-half-first long is not the same matrix");
    }

    /// One pointer serves both sides: thirty-two bytes in brings it back to
    /// the start, which is why the first read is plane 0.
    #[test]
    fn the_pointer_wraps_at_thirty_two_bytes() {
        let a = akiko();
        for i in 0..C2P_WORDS as u32 {
            write_long(&a, off::C2P, i * 0x0101_0101);
        }
        let first = read_long(&a, off::C2P);
        for _ in 1..C2P_WORDS {
            let _ = read_long(&a, off::C2P);
        }
        assert_eq!(read_long(&a, off::C2P), first);
    }

    // -- the EEPROM ----------------------------------------------------------

    /// A bit-banging master, driving the same two bits the CD32's ROM does.
    struct Master<'a> {
        akiko: &'a Akiko,
        /// Whether this end is driving SDA at all.
        sda_out: bool,
    }

    impl Master<'_> {
        fn new(akiko: &Akiko) -> Master<'_> {
            Master {
                akiko,
                sda_out: true,
            }
        }

        fn put(&self, scl: bool, sda: bool) {
            let mut v = SCL_DRIVE;
            if self.sda_out {
                v |= SDA_DRIVE;
            }
            if scl {
                v |= SCL_LEVEL;
            }
            if sda {
                v |= SDA_LEVEL;
            }
            write_long(self.akiko, off::NVRAM, v);
        }

        fn start(&mut self) {
            self.sda_out = true;
            self.put(true, true);
            self.put(true, false);
            self.put(false, false);
        }

        fn stop(&mut self) {
            self.sda_out = true;
            self.put(false, false);
            self.put(true, false);
            self.put(true, true);
        }

        /// Clock out eight bits and return whether the slave acknowledged.
        fn send(&mut self, byte: u8) -> bool {
            self.sda_out = true;
            for i in (0..8).rev() {
                let bit = byte >> i & 1 != 0;
                self.put(false, bit);
                self.put(true, bit);
                self.put(false, bit);
            }
            self.sda_out = false;
            self.put(false, true);
            self.put(true, true);
            let ack = read_long(self.akiko, off::NVRAM) & SDA_LEVEL == 0;
            self.put(false, true);
            self.sda_out = true;
            ack
        }

        /// Clock in eight bits, then acknowledge or not.
        fn receive(&mut self, more: bool) -> u8 {
            self.sda_out = false;
            let mut byte = 0u8;
            for _ in 0..8 {
                self.put(false, true);
                self.put(true, true);
                let bit = read_long(self.akiko, off::NVRAM) & SDA_LEVEL != 0;
                byte = byte << 1 | u8::from(bit);
                self.put(false, true);
            }
            self.sda_out = true;
            let ack = !more;
            self.put(false, ack);
            self.put(true, ack);
            self.put(false, ack);
            byte
        }
    }

    #[test]
    fn the_eeprom_acknowledges_its_own_address_and_nothing_else() {
        let a = akiko();
        let mut m = Master::new(&a);
        m.start();
        assert!(m.send(0xA0), "$A0 is a 24C08 being addressed for writing");
        m.stop();
        m.start();
        assert!(!m.send(0xC0), "$C0 is nobody");
        m.stop();
    }

    #[test]
    fn a_byte_written_comes_back_out() {
        let a = akiko();
        let mut m = Master::new(&a);
        m.start();
        assert!(m.send(0xA0));
        assert!(m.send(0x40), "the word address");
        assert!(m.send(0x5A), "the byte");
        m.stop();
        assert_eq!(a.nvram()[0x40], 0x5A);

        m.start();
        assert!(m.send(0xA0));
        assert!(m.send(0x40));
        m.start();
        assert!(m.send(0xA1), "addressed for reading");
        assert_eq!(m.receive(false), 0x5A);
        m.stop();
    }

    /// The two page bits in the device address are the top of the word
    /// address on a 24C08, not a chip select.
    #[test]
    fn the_page_bits_reach_the_upper_three_quarters() {
        let a = akiko();
        let mut m = Master::new(&a);
        m.start();
        assert!(m.send(0xA6), "page 3");
        assert!(m.send(0x10));
        assert!(m.send(0x99));
        m.stop();
        assert_eq!(a.nvram()[0x310], 0x99);
    }

    #[test]
    fn a_sequential_read_walks_on() {
        let a = akiko();
        let mut cells = [0xFFu8; NVRAM_BYTES];
        cells[0..4].copy_from_slice(&[1, 2, 3, 4]);
        a.set_nvram(cells);

        let mut m = Master::new(&a);
        m.start();
        assert!(m.send(0xA0));
        assert!(m.send(0x00));
        m.start();
        assert!(m.send(0xA1));
        assert_eq!(m.receive(true), 1);
        assert_eq!(m.receive(true), 2);
        assert_eq!(m.receive(true), 3);
        assert_eq!(m.receive(false), 4);
        m.stop();
    }

    /// An erased part reads `$FF`, which is what a floating-gate cell that has
    /// never been written holds.
    #[test]
    fn a_fresh_part_is_erased() {
        let a = akiko();
        assert!(a.nvram().iter().all(|b| *b == 0xFF));
    }

    /// A pin nobody drives is pulled up; either end may pull it down.
    #[test]
    fn a_released_pin_reads_high() {
        let a = akiko();
        write_long(&a, off::NVRAM, 0);
        assert_eq!(
            read_long(&a, off::NVRAM) & (SCL_LEVEL | SDA_LEVEL),
            SCL_LEVEL | SDA_LEVEL
        );
        write_long(&a, off::NVRAM, SCL_DRIVE | SDA_DRIVE);
        assert_eq!(read_long(&a, off::NVRAM) & (SCL_LEVEL | SDA_LEVEL), 0);
    }

    // -- the controller's registers -----------------------------------------

    /// The write test the ROM does at `$25` before it will go on.
    #[test]
    fn the_configuration_register_holds_what_is_put_in_it() {
        let a = akiko();
        assert_eq!(read(&a, 0x25, 1), 0);
        write(&a, 0x25, &[0x80]);
        assert_eq!(read(&a, 0x25, 1), 0x80);
        write(&a, 0x25, &[0x00]);
        assert_eq!(read(&a, 0x25, 1), 0);
    }

    /// The interrupt pair: only bits 31–24 of the enable answer, and the
    /// request is the controller's to set, which nothing here does.
    #[test]
    fn the_enable_takes_the_top_byte_and_the_request_does_not_move() {
        let a = akiko();
        write_long(&a, off::INTENA, 0x1800_1234);
        assert_eq!(read_long(&a, off::INTENA), 0x1800_0000);
        write_long(&a, off::INTREQ, 0xFFFF_FFFF);
        assert_eq!(read_long(&a, off::INTREQ), 0);
    }

    /// The pointers the ROM programs are stored and read back, and nothing
    /// walks them.
    #[test]
    fn the_ring_pointers_are_storage() {
        let a = akiko();
        write_long(&a, 0x10, 0x0001_0000);
        write_long(&a, 0x14, 0x001F_E400);
        assert_eq!(read_long(&a, 0x10), 0x0001_0000);
        assert_eq!(read_long(&a, 0x14), 0x001F_E400);
        write_long(&a, off::CHIP_INDICES, 0xFFFF_FFFF);
        assert_eq!(read_long(&a, off::CHIP_INDICES), 0);
    }

    // -- the debugger --------------------------------------------------------

    #[test]
    fn a_debugger_reads_without_moving_and_may_not_write() {
        let a = akiko();
        for _ in 0..C2P_WORDS {
            write_long(&a, off::C2P, 0x5555_0000);
        }
        let mut buf = [0u8; 4];
        MemOps::read(&*a.regs, off::C2P, &mut buf, MemAttrs::DEBUG).unwrap();
        assert_eq!(u32::from_be_bytes(buf), 0xCCCC_CCCC);
        MemOps::read(&*a.regs, off::C2P, &mut buf, MemAttrs::DEBUG).unwrap();
        assert_eq!(u32::from_be_bytes(buf), 0xCCCC_CCCC);
        assert_eq!(read_long(&a, off::C2P), 0xCCCC_CCCC);

        assert!(MemOps::write(&*a.regs, off::C2P, &[0, 0], MemAttrs::DEBUG).is_err());
        assert!(MemOps::write(&*a.regs, off::NVRAM, &[0, 0], MemAttrs::DEBUG).is_err());
    }

    /// A debug read of `$30` does not step the EEPROM's bus either: the wires
    /// are where the register left them.
    #[test]
    fn a_debug_read_does_not_clock_the_eeprom() {
        let a = akiko();
        let mut m = Master::new(&a);
        m.start();
        assert!(m.send(0xA0));
        let mut buf = [0u8; 4];
        for _ in 0..8 {
            MemOps::read(&*a.regs, off::NVRAM, &mut buf, MemAttrs::DEBUG).unwrap();
        }
        assert!(m.send(0x20));
        assert!(m.send(0x77));
        m.stop();
        assert_eq!(a.nvram()[0x20], 0x77);
    }

    // -- snapshots -----------------------------------------------------------

    fn snapshot(a: &Akiko) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("akiko", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("akiko", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(a, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_and_resumes_identically() {
        let saved = akiko();
        let mut m = Master::new(&saved);
        m.start();
        assert!(m.send(0xA0));
        assert!(m.send(0x11));
        assert!(m.send(0x22));
        m.stop();
        write_long(&saved, off::INTENA, 0x1800_0000);
        write_long(&saved, 0x10, 0x0001_0000);
        for _ in 0..C2P_WORDS {
            write_long(&saved, off::C2P, 0x5555_0000);
        }
        let _ = read_long(&saved, off::C2P);

        let bytes = snapshot(&saved);
        let restored = akiko();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("akiko", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        assert_eq!(snapshot(&restored), bytes);

        assert_eq!(read_long(&restored, off::C2P), read_long(&saved, off::C2P));
        assert_eq!(restored.nvram()[0x11], 0x22);
        assert_eq!(snapshot(&restored), snapshot(&saved));
    }

    /// A reset clears the registers and the converter and leaves the cells
    /// alone, which is the whole point of the part.
    #[test]
    fn a_reset_does_not_erase_the_eeprom() {
        let a = akiko();
        let mut cells = [0xFFu8; NVRAM_BYTES];
        cells[7] = 0x42;
        a.set_nvram(cells);
        write_long(&a, off::INTENA, 0x1800_0000);
        a.reset(ResetKind::Cold);
        assert_eq!(read_long(&a, off::INTENA), 0);
        assert_eq!(a.nvram()[7], 0x42);
    }
}
