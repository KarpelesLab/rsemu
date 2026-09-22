//! The Integrated Woz Machine and the 400K/800K drive on the end of its
//! cable.
//!
//! # Sources
//!
//! * Apple's *IWM Specification* (Apple Computer, 1982), for the sixteen soft
//!   switches, the four register pairs `Q6`/`Q7` select between, the mode
//!   register's bits and the write handshake.
//! * *Guide to the Macintosh Family Hardware*, 2nd edition, chapter 9 — where
//!   the chip is decoded and what is on the twenty-pin cable. It does **not**
//!   carry the drive's register file; its chapter 9 tables are connector
//!   signal assignments and nothing more.
//! * Neil Parker, *Controlling the 3.5 Drive Hardware on the Apple IIGS*
//!   (version 1.00, February 1994) — the published table of the Sony
//!   mechanism's sixteen one-bit status registers and its control registers,
//!   addressed by `CA2`, `CA1`, `CA0` and `SEL`. The same mechanism hangs off
//!   a Macintosh Plus, and this is the document that settles the polarities.
//!
//! No emulator source was consulted and no ROM was disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`); where the documents were ambiguous the
//! answer came from watching which addresses a real ROM touches and what it
//! waits for.
//!
//! # The decode, and why every access is a switch
//!
//! The IWM has no address pins in the ordinary sense. Its sixteen addresses
//! are **soft switches**: the low bit of the address number sets or clears one
//! internal line, and it does so whether the access was a read or a write.
//! Reading `$DFE1FF + 2 * 512` clears `CA1`; reading `$DFE1FF + 3 * 512` sets
//! it. A ROM therefore drives this chip almost entirely with *reads*, which is
//! exactly what a trace of one shows.
//!
//! ```text
//!   0/1   CA0     off / on
//!   2/3   CA1     off / on
//!   4/5   CA2     off / on
//!   6/7   LSTRB   off / on     the strobe that writes a drive register
//!   8/9   ENABLE  off / on     the drive motor
//!   10/11 SELECT  drive 1 / 2
//!   12/13 Q6      off / on
//!   14/15 Q7      off / on
//! ```
//!
//! and `Q7:Q6` then choose what a read of *any* of those sixteen addresses
//! returns:
//!
//! ```text
//!   0 0   the data register — what the head has shifted in
//!   0 1   the status register
//!   1 0   the write handshake
//!   1 1   nothing (a write here loads the mode register)
//! ```
//!
//! A Macintosh puts the chip's register selects on **A9-A12** like the VIA's,
//! so its sixteen addresses are 512 bytes apart, the block repeats every
//! 8 KiB, and the published base is `$DFE1FF` — an *odd* address, because the
//! IWM sits on the low byte lane where the VIA sits on the high one.
//!
//! # The drive's own registers
//!
//! The drive is addressed by `CA2:CA1:CA0` **and the `SEL` line**, which comes
//! from the VIA's `PA5` rather than from the IWM: four bits, sixteen readable
//! status lines, whose selected value appears as bit 7 of the IWM's status
//! register (`SENSE`).
//!
//! ```text
//!   CA2 CA1 CA0 SEL                          asserted
//!    0   0   0   0   step direction          1 = outward, toward track 0
//!    0   0   0   1   disk in place           0 = a disk is in the drive
//!    0   0   1   0   disk is stepping        0 = the head is moving
//!    0   0   1   1   disk locked             0 = write protected
//!    0   1   0   0   motor on                0 = the spindle is turning
//!    0   1   0   1   track 0                 0 = the head is over track 0
//!    0   1   1   0   disk switched           0 = the user ejected a disk
//!    0   1   1   1   tachometer              60 pulses a revolution
//!    1   0   0   0   lower head's read line  and selects that head
//!    1   0   0   1   upper head's read line  and selects that head
//!    1   0   1   0   (unassigned)
//!    1   0   1   1   is a SuperDrive         0 = an FDHD mechanism
//!    1   1   0   0   number of sides         1 = double sided
//!    1   1   0   1   disk ready for reading  0 = ready
//!    1   1   1   x   drive installed         0 = a drive is connected
//! ```
//!
//! Writing is addressed by `CA1:CA0:SEL` — `SEL` is part of the address here
//! too — with the data on `CA2`, latched when `LSTRB` goes high:
//!
//! ```text
//!   CA1 CA0 SEL  CA2=0            CA2=1
//!    0   0   0   step toward 79   step toward 0
//!    0   0   1   —                reset the disk-switched flag
//!    0   1   0   issue one step   —
//!    1   0   0   motor on         motor off
//!    1   1   0   —                eject
//! ```
//!
//! Most of those lines are asserted **low**, and the table above is the whole
//! of what this model knows about the mechanism. Getting it wrong is not a
//! detail: a first version of it invented the four high addresses, answered
//! the pull-up where the ROM looks for "drive installed", and the machine sat
//! on the insert-disk screen for ever without once turning the motor.
//!
//! # The read data path
//!
//! A disk goes in as an image ([`super::disk`]), becomes a cylinder of bit
//! cells ([`super::gcr`]), and is shifted past the head one cell per tick of
//! this device's clock — so **one tick is one bit cell**, and a board gives it
//! 500 kHz, the rate the IWM's own cell time works out at. A different rate is
//! not refused; a device cannot see its domain's frequency.
//!
//! The shifter is the IWM's own and there is only one rule to it: the register
//! shifts left, and a byte is complete when a one reaches bit 7. Leading zeros
//! are skipped rather than counted, which is what makes a self-sync run —
//! `$FF` in ten cells instead of eight — resynchronise a shifter that came into
//! it on the wrong boundary. Reading the data register takes the byte and
//! leaves zero behind, so a guest polls until bit 7 is set, which is what a
//! ROM does.
//!
//! **The chip names each latch as an event** ([`Device::next_event_tick`]),
//! and that is what makes the polling work. The scheduler bounds a round by
//! the earliest event any lazily-advanced device names, and an access is
//! answered at the position the round has reached — a 68000 publishes no live
//! cursor of its own. With no event named, a round ran on past two byte times
//! at a stretch and the guest was handed the *last* byte of it; a real
//! Macintosh Plus ROM lost one byte in three that way, which is every sector
//! it tried.
//!
//! **Which head** is decided by the two read lines above: they differ only in
//! `SEL`, and the note says reading one of them *configures the drive* to do
//! its I/O with that head. So addressing one is how the computer says which
//! head it wants, and this model latches the side there.
//!
//! # What is modelled, and what is not
//!
//! The switches, the mode and status registers, the write handshake, the
//! drive's status lines, stepping, the motor, a disk that can be present,
//! absent or write protected, and the **read** data path above. **Writing is
//! not here**: a byte written to the data register is kept and goes nowhere, so
//! a disk is read-only however its tab is set.

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use super::disk::Disk;
use super::gcr::Track;
use super::mfm;
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "mac.iwm";

/// The snapshot chunk version. Bump with the encoding, never on its own.
pub const STATE_VERSION: u32 = 5;

/// How many bytes of address space the sixteen switches occupy: the board puts
/// the register selects on A9-A12, so `16 * 512`.
pub const REGISTER_SPAN: u64 = 0x2000;

/// How far apart two switches are, in bytes.
const REGISTER_STRIDE: u64 = 0x200;

/// The input pin the VIA's `PA5` drives: the drive register file's fourth
/// address bit.
const SEL_PIN: &str = "sel";

/// The highest track a 400K/800K mechanism can step to.
pub const MAX_TRACK: u8 = 79;

// -- the soft switches -------------------------------------------------------

const SW_CA0: u8 = 1 << 0;
const SW_CA1: u8 = 1 << 1;
const SW_CA2: u8 = 1 << 2;
const SW_LSTRB: u8 = 1 << 3;
const SW_ENABLE: u8 = 1 << 4;
const SW_SELECT: u8 = 1 << 5;
const SW_Q6: u8 = 1 << 6;
const SW_Q7: u8 = 1 << 7;

/// The status register's bit 5: the drive enable line, as software reads it.
const STATUS_ENABLE: u8 = 1 << 5;
/// Its bit 7: the selected drive status line.
const STATUS_SENSE: u8 = 1 << 7;
/// The write handshake's bit 6: no underrun has happened.
const HANDSHAKE_NO_UNDERRUN: u8 = 1 << 6;
/// Its bit 7: the write buffer will take another byte.
const HANDSHAKE_READY: u8 = 1 << 7;

// -- the MFM byte framer -----------------------------------------------------
//
// This is the one thing in this file that an IWM does not have, and it is here
// rather than in `mac.swim` because *the medium is here*: the head position,
// the cylinder under it and the motor all live in `Mechanism`, and a second
// copy of them so that a SWIM could frame its own bytes is exactly the
// duplication `mac.swim` exists to avoid. It is inert — `Framer::on` is false
// and nothing below it runs — until a SWIM in ISM mode switches it on, so a
// Macintosh Plus is byte-identical.

/// How many cells one MFM byte spends: two per data bit (`super::mfm`).
const MFM_CELLS_PER_BYTE: u8 = 16;

/// How many framed bytes the FIFO holds.
///
/// *SWIM Chip User's Reference*, rev. 1.5, page 13: "The ISM uses a 2-byte
/// read/write FIFO, so the software can 'slip' out a byte from time to time
/// without causing an overrun (reading too quickly) or underrun (writing too
/// slowly)."
pub const MFM_FIFO: usize = 2;

/// The state of the MFM separator: where the byte boundary is and what it has
/// framed that the guest has not taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Framer {
    /// Whether a SWIM has asked for MFM framing at all. An IWM leaves this
    /// clear for ever.
    on: bool,
    /// The last **thirty-two** cells past the head, oldest in bit 31 — which
    /// is the order `mfm::cells` packs a byte and `mfm::Writer::push` lays it
    /// down. The low sixteen are the byte being assembled; the high sixteen
    /// are the one before it, which is what anchors the mark search to a sync
    /// field (see [`Framer::cell`]).
    cells: u32,
    /// Cells since the last byte boundary, `0..16`. Meaningless until
    /// `synced`.
    phase: u8,
    /// Whether a mark byte has been found, so byte boundaries are known.
    ///
    /// Page 23: "after setting ACTION on a read operation, the first byte that
    /// will be returned will be a mark byte... The search for the mark byte is
    /// invisible to the software since it is handled entirely by the SWIM
    /// chip."
    synced: bool,
    /// The framed bytes the guest has not taken, oldest first.
    fifo: [u8; MFM_FIFO],
    /// One bit per slot of `fifo`: whether that byte was a mark byte.
    marks: u8,
    /// How many of `fifo` are live.
    count: u8,
    /// A byte was framed while the FIFO was already full, so it was lost.
    overrun: bool,
    /// The CRC generator, run over every byte framed off the medium.
    ///
    /// It lives here rather than in the ISM's register file because it is fed
    /// **from the medium**, not from the processor's reads: the ISM's handshake
    /// register reports the CRC over "the bytes up to and including the byte
    /// about to be read", which only a generator tapped where the bytes are
    /// framed can answer. A byte lost to an overrun still goes through it, for
    /// the same reason — the hardware's generator never saw the FIFO.
    /// `swim::ism` owns the seed and the meaning.
    crc: u16,
}

impl Framer {
    /// Forget the byte boundary and go back to looking for a mark, emptying
    /// the FIFO and reseeding the CRC. What the ISM's "clear FIFO" bit and a
    /// fresh `ACTION` do.
    fn restart(&mut self, seed: u16) {
        let on = self.on;
        *self = Framer::default();
        self.on = on;
        self.crc = seed;
    }

    /// The sixteen cells of the byte the head has just finished.
    fn byte_cells(&self) -> u16 {
        self.cells as u16
    }

    /// And of the one before it.
    fn previous_cells(&self) -> u16 {
        (self.cells >> 16) as u16
    }

    /// Take one cell off the medium and frame a byte if this was the
    /// sixteenth.
    fn cell(&mut self, cell: bool) {
        self.cells = (self.cells << 1) | u32::from(cell);
        if self.synced {
            self.phase += 1;
            if self.phase < MFM_CELLS_PER_BYTE {
                return;
            }
            self.phase = 0;
            let (byte, mark) = decode_cells(self.byte_cells());
            self.push(byte, mark);
            return;
        }
        // Searching. A mark byte is one whose clock pulse is deliberately
        // missing, and `mfm::sync_cells` derives the pattern from the encoding
        // rule rather than quoting a magic number.
        //
        // **Only `$A1`'s.** The format has two marks, but a search that has no
        // byte boundary to align to can only look for a *bit pattern*, and
        // `$C2`'s `$5224` is not unique at an arbitrary cell offset:
        // `swim::tests::only_the_a1_sync_is_unique_at_every_cell_alignment`
        // walks a formatted track and finds `$4489` **108** times — three per
        // ID field and three per data field, exactly where the format puts
        // them and nowhere else — against **192** hits of `$5224` on a track
        // that carries three. Syncing on the second pattern made this chip
        // report an index mark eighteen times a revolution, and Apple's own
        // ROM, which reads a mark and then the address mark behind it, got
        // `$C2` where a sector's `$A1` should have been and started over. An
        // ID or data field is prefixed by `$A1` and only by `$A1`, so nothing
        // is lost: `super::mfm` says the same thing from the other side —
        // "nothing in the read path here looks for it; a controller finds
        // sectors by their own marks".
        //
        // **And only the first `$A1` of the three**, which is what the
        // preceding-sync-byte test is for. The format writes three of them in
        // a row and the CRC covers all three (`super::mfm`), so a separator
        // that locked onto the second or the third would seed its generator a
        // byte or two into the field and every CRC on the disk would read as
        // bad — which is precisely what Apple's ROM was told, over and over,
        // before this test was here: the handshake register's bit 1 came back
        // set on every field it read and it put the disk straight back out.
        //
        // The chip locks onto the **sync field** rather than onto a bare
        // pattern, and page 19 says so while describing the correction
        // machine: "The CSM looks for 32 pairs of minimum cells which
        // coincidently show up in a run of zero bytes, such as a sync field.
        // After that it looks to see if the first non-minimum cell belongs to
        // a mark byte. If not, it starts looking for minimum cells again."
        // Sixteen cells of `$00` in front of the mark is that rule at the
        // resolution this model works at: a run of minimum cells, then the
        // *first* mark after it. The second and third `$A1` have `$4489`
        // behind them rather than `$AAAA`, so they are framed as the ordinary
        // marks they are and the generator has already seen the first.
        let sync_byte = mfm::cells(0x00, false, None);
        if self.byte_cells() == mfm::sync_cells(mfm::SYNC_A1) && self.previous_cells() == sync_byte
        {
            self.synced = true;
            self.phase = 0;
            let (byte, _) = decode_cells(self.byte_cells());
            self.push(byte, true);
        }
    }

    /// Put a framed byte in the FIFO, or record that it was lost.
    fn push(&mut self, byte: u8, mark: bool) {
        self.crc = mfm::crc16(self.crc, &[byte]);
        let slot = usize::from(self.count);
        if slot >= MFM_FIFO {
            // Page 24, Error register bit 0: "The processor is not
            // reading/writing fast enough to keep up with the chip."
            self.overrun = true;
            return;
        }
        self.fifo[slot] = byte;
        if mark {
            self.marks |= 1 << slot;
        } else {
            self.marks &= !(1 << slot);
        }
        self.count += 1;
    }

    /// The oldest framed byte and whether it is a mark, without taking it.
    fn peek(&self) -> Option<(u8, bool)> {
        (self.count > 0).then(|| (self.fifo[0], self.marks & 1 != 0))
    }

    /// Take it.
    fn pop(&mut self) -> Option<(u8, bool)> {
        let head = self.peek()?;
        self.count -= 1;
        self.fifo.rotate_left(1);
        self.marks >>= 1;
        Some(head)
    }

    /// How many cells until the next byte could be framed.
    ///
    /// Exact while synced: a byte is sixteen cells and `phase` says how far
    /// into one the head is. While *searching* a mark can complete on any
    /// cell, so naming the truth would be an event every cell — a million a
    /// virtual second. Sixteen is named instead, which is the same rate the
    /// synced path costs and is never *late*: at most one byte can be framed
    /// in sixteen cells either way, so the FIFO cannot be overrun by the
    /// scheduler running a round longer than this.
    fn cells_ahead(&self) -> u64 {
        u64::from(MFM_CELLS_PER_BYTE - if self.synced { self.phase } else { 0 })
    }
}

// -- the write side ----------------------------------------------------------

/// How many bytes the write buffer holds.
///
/// Two, which covers both parts. The IWM has one byte of buffer behind its
/// shift register and its handshake reports it in one bit — *SWIM Chip User's
/// Reference*, page 11: "The write buffer empty bit will be set to '1'
/// whenever the chip is ready to accept another byte from the processor" — and
/// the ISM has the two-byte FIFO its own handshake register counts in two bits
/// (page 13: "The ISM uses a 2-byte read/write FIFO"). A buffer of two answers
/// both: the IWM's bit is "at least one slot free", which a buffer of two
/// answers as well as a buffer of one does.
pub const WRITE_FIFO: usize = 2;

/// The write side of the head: the bytes the processor has handed over and the
/// cells they become on the way to the medium.
///
/// # The rule, and where it comes from
///
/// Apple, *Software Control of the Disk II or IWM Controller* (26 April 1984,
/// revision 1 of 10 May 1984), page 3, gives the whole of the register decode
/// the write path hangs off:
///
/// ```text
///   Q6   Q7   FUNCTION
///   L    L    READ
///   H    L    SENSE WRITE PROTECT OR PREWRITE STATE
///   L    H    WRITE
///   H    H    WRITE LOAD
/// ```
///
/// and page 4 the sequence, which is what says that a *store* is the load and
/// that the shifting is the chip's own: "The STA instruction loads the contents
/// of the accumulator into the controller's data shift register. The next
/// instruction [LDA Q6L, X] causes the data in the register to shift out
/// serially. Q6H and Q7H are the conditions required for parallel loading the
/// data into the controller. Shifting out the data serially to the disk drive
/// requires Q6L and Q7H."
///
/// The *SWIM Chip User's Reference*, page 10, gives the same decode for the
/// part a Macintosh has, with the `MotorOn` latch as its third address bit —
/// `[110]` is `Set Mode` and `[111]` is `Write Data` — and page 12 says what a
/// load does: "Writing to either L7=, L6=1 or MotorOn=1 while in this state
/// will write a byte of data to the write buffer. This byte will in turn be
/// loaded into the shifting hardware and sent out serially to the disk."
///
/// # What happens when the processor is late is an **inference**
///
/// Neither document says in so many words what the head lays down at a byte
/// boundary with nothing loaded. Two things are quotable and they settle it
/// between them.
///
/// * Apple's own self-sync example, page 4 of *Software Control*, spends
///   **forty** clock cycles on an `$FF` where a data byte spends **thirty-two**
///   — "SELF SYNC BYTE / 40 CLOCK CYCLES" against "DATA BYTE / 32 CLOCK
///   CYCLES". Eight 6502 cycles are two bit cells, and what comes off a disk
///   there is a self-sync byte: eight ones and two zeros. So the two extra
///   cells carry **no transition**, and there is no other mechanism in the part
///   for writing one — the sync byte a formatter lays down is made by being
///   late on purpose.
/// * Neil Parker, *Controlling the 3.5 Drive Hardware on the Apple IIGS*
///   (1994), ends its write routine on `BIT Q6 / BVS WLAST  ;wait until last
///   data underruns`, so an underrun is the *ordinary* end of a write rather
///   than a fault.
///
/// So: a boundary with an empty buffer lays down a zero cell and sets the
/// underrun flag, and the head keeps writing. That is what
/// [`Writer::cell`] does and it is commented here rather than presented as a
/// reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Writer {
    /// Whether the head is laying cells down at all.
    on: bool,
    /// Whether a byte becomes **sixteen MFM cells** rather than its own eight.
    /// An IWM writes Apple GCR and leaves this clear; an ISM out of GCR mode
    /// sets it.
    mfm: bool,
    /// The bytes the processor has handed over and the head has not reached,
    /// oldest first.
    fifo: [u8; WRITE_FIFO],
    /// What each of them is, which is not always the same as its value: a CRC
    /// byte is whatever the generator holds when the head **reaches** it, not
    /// when the processor asked for it.
    kinds: [WriteKind; WRITE_FIFO],
    /// How many of `fifo` are live.
    count: u8,
    /// The cells of the byte being laid down, the next one in bit `bits - 1`.
    cells: u16,
    /// How many of them are left.
    bits: u8,
    /// The last data bit laid down, which is what MFM's clock rule needs:
    /// `super::mfm::cells` states it as `clock = !prev && !bit`.
    prev: bool,
    /// The processor was late: the head reached a byte boundary with an empty
    /// buffer. Sticky, page 11: "this bit will be reset to '0' until either the
    /// chip is reset or taken out of write mode".
    underrun: bool,
    /// The CRC generator, the mirror of [`Framer::crc`].
    ///
    /// **It is preset at the first mark byte of a field**, which is the one
    /// thing about the write side that neither document states and is an
    /// *inference* — from the read side's own rule, which is the same rule
    /// seen from the other end. `super::mfm` puts it as: "A field's CRC is
    /// seeded with `$FFFF` and covers the three sync bytes as data — `$A1 $A1
    /// $A1` — then the address mark, then the field." On the read side that
    /// falls out of where framing begins, because the separator locks onto the
    /// first `$A1` and nothing in front of it is ever framed. On the write
    /// side the chip is *told* which bytes are marks — that is the whole point
    /// of the ISM's Write Mark register — so the first of a run of them is the
    /// same instant. A generator running from the Clear FIFO toggle instead
    /// would have absorbed the sync field of `$00`s the processor writes in
    /// front of the marks, and every field on the disk would read as bad.
    ///
    /// It is **checkable** rather than assumed, which is the reason to accept
    /// it: a field written this way and read straight back leaves the read
    /// side's generator at zero, which is exactly what the ISM's handshake
    /// register reports in bit 1.
    crc: u16,
    /// What `crc` goes back to at the first mark of a field.
    seed: u16,
    /// Whether the byte last taken into the shifter was a mark, so that the
    /// *first* of a field's three is the one that presets the generator.
    after_mark: bool,
    /// The generator as it stood when the head reached the first of the two
    /// CRC bytes, so that the second is the low half of the same value.
    pending_crc: u16,
}

/// What a byte in the write buffer is.
///
/// A real enum rather than the `#[repr(transparent)]` newtype `CLAUDE.md`
/// prefers, because these four are the whole of it: the ISM has one register
/// per kind — Write Data, Write Mark and Write CRC — and an IWM, which has
/// only the first, never produces the other three.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WriteKind {
    /// An ordinary byte, from the Data register.
    #[default]
    Data,
    /// A byte "that has a transition missing between two adjacent zero-bits"
    /// (page 20) — a field's `$A1` sync. The ISM's Write Mark register.
    Mark,
    /// The high half of the generator, taken when the head reaches it.
    CrcHigh,
    /// And the low half of the same value.
    CrcLow,
}

impl WriteKind {
    /// The one a snapshot byte names; anything else is an ordinary byte,
    /// because a restore must not be able to invent a fifth.
    fn from_byte(byte: u8) -> WriteKind {
        match byte {
            1 => WriteKind::Mark,
            2 => WriteKind::CrcHigh,
            3 => WriteKind::CrcLow,
            _ => WriteKind::Data,
        }
    }
}

impl Writer {
    /// Empty the buffer and reseed the generator, leaving the mode alone. What
    /// the ISM's "clear FIFO" bit and a fresh `ACTION` do.
    fn restart(&mut self, seed: u16) {
        let (on, mfm) = (self.on, self.mfm);
        *self = Writer::default();
        self.on = on;
        self.mfm = mfm;
        self.crc = seed;
        self.seed = seed;
    }

    /// How many slots are free.
    fn space(&self) -> u8 {
        WRITE_FIFO as u8 - self.count
    }

    /// Hand a byte over. `false` if there was no room, which page 12 says is
    /// harmless: "Writing to the chip faster than this won't hurt anything".
    fn push(&mut self, byte: u8, kind: WriteKind) -> bool {
        let slot = usize::from(self.count);
        if slot >= WRITE_FIFO {
            return false;
        }
        self.fifo[slot] = byte;
        self.kinds[slot] = kind;
        self.count += 1;
        true
    }

    /// Take the next byte into the shifter. `false` when there is none.
    fn load(&mut self) -> bool {
        if self.count == 0 {
            return false;
        }
        let (mut byte, kind) = (self.fifo[0], self.kinds[0]);
        self.count -= 1;
        self.fifo.rotate_left(1);
        self.kinds.rotate_left(1);
        match kind {
            WriteKind::Data => {}
            // The first mark of a field presets the generator; the second and
            // third `$A1` are covered by it. See `Writer::crc`.
            WriteKind::Mark => {
                if !self.after_mark {
                    self.crc = self.seed;
                }
            }
            // The generator's own value, taken **here** rather than where the
            // processor asked for it: bytes handed over earlier may still have
            // been in the buffer then, and the CRC has to cover them.
            WriteKind::CrcHigh => {
                self.pending_crc = self.crc;
                byte = (self.pending_crc >> 8) as u8;
            }
            WriteKind::CrcLow => byte = self.pending_crc as u8,
        }
        self.after_mark = kind == WriteKind::Mark;
        self.crc = mfm::crc16(self.crc, &[byte]);
        if self.mfm {
            self.cells = if kind == WriteKind::Mark {
                mfm::sync_cells(byte)
            } else {
                mfm::cells(byte, self.prev, None)
            };
            self.bits = MFM_CELLS_PER_BYTE;
        } else {
            // Apple GCR: the byte is the cells, most significant first.
            self.cells = u16::from(byte);
            self.bits = 8;
        }
        self.prev = byte & 1 != 0;
        true
    }

    /// The cell the head lays down now.
    fn cell(&mut self) -> bool {
        if self.bits == 0 && !self.load() {
            // Late. See the module-level inference above: a zero cell, and the
            // flag stays set until the chip leaves write mode.
            self.underrun = true;
            self.prev = false;
            return false;
        }
        self.bits -= 1;
        self.cells & (1 << self.bits) != 0
    }

    /// How many cells until the head wants the next byte.
    ///
    /// Exact: the shifter owes `bits` more cells of the byte it is on, and a
    /// shifter with nothing in it wants one now. Naming it is what keeps a
    /// scheduler round from running past several byte times at once and
    /// underrunning a guest that was keeping up — the defect
    /// [`Shared::publish_latch`] records on the read side, in the other
    /// direction.
    fn cells_ahead(&self) -> u64 {
        u64::from(self.bits.max(1))
    }
}

/// The byte in sixteen MFM cells, and whether a clock pulse is missing from
/// them — which is what makes a byte a mark byte.
///
/// The data bits sit in the odd-numbered cells counting from the bottom: cell
/// 14 is the most significant data bit, cell 0 the least. The clock cells
/// between them follow one rule and the whole of it — `mfm::cells` states it
/// as `clock = !prev && !bit` — so re-encoding the decoded byte and comparing
/// is what says whether a pulse was left out.
fn decode_cells(cells: u16) -> (u8, bool) {
    let mut byte = 0u8;
    for i in 0..8u32 {
        let bit = cells >> (14 - 2 * i) & 1;
        byte = (byte << 1) | bit as u8;
    }
    // The data bit before this byte is the previous byte's last, which is the
    // cell two below the window — not available here. It only decides the one
    // clock cell at the top, so a byte whose only difference is that cell is
    // reported as ordinary rather than as a mark: `mfm::cells` is asked for
    // both possibilities and either match is an ordinary byte.
    let mark = cells != mfm::cells(byte, false, None) && cells != mfm::cells(byte, true, None);
    (byte, mark)
}

/// One 400K/800K mechanism.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Mechanism {
    /// Whether a drive is plugged in at all. An external port with nothing on
    /// it has this clear, and the drive-installed line then reads high.
    installed: bool,
    /// Whether a disk is in it.
    disk: bool,
    /// Whether that disk is write protected.
    write_protect: bool,
    /// Whether it is a double-sided (800K) mechanism.
    double_sided: bool,
    /// Whether it is a **SuperDrive** — an FDHD mechanism, which reads
    /// high-density MFM as well as Apple's GCR.
    ///
    /// Apple's note leaves `CA2:CA1:CA0 = 101` unassigned on the 800K
    /// mechanism and says nothing about later ones; a Macintosh Classic ROM
    /// reads that address while working out what is on the cable, so this is
    /// where a SuperDrive answers. A Plus's 800K drive has it clear and the
    /// address reads as the cable's pull-up, which is exactly what it did
    /// before this field existed.
    superdrive: bool,
    /// Which cylinder the head is over, 0 to [`MAX_TRACK`].
    track: u8,
    /// The step direction the last write to register 0 set: `false` steps
    /// toward track 79, `true` toward track 0.
    outward: bool,
    /// Whether the motor is running.
    motor: bool,
    /// The "the user ejected a disk" line, set when a disk leaves the drive
    /// and cleared by the drive's own reset-disk-switched register.
    ///
    /// Not set when a disk goes *in*: the documented meaning of the line is
    /// "0 = user ejected disk by pressing the eject button", so a machine that
    /// came up with a disk already in the drive has not switched anything.
    switched: bool,
    /// Which head the computer last asked for, by addressing `RDDATA0` or
    /// `RDDATA1`.
    side: bool,
    /// The tachometer's output now.
    tach: bool,
    /// The bit under the head now.
    read_line: bool,
    /// How far round the cylinder the head is, in bit cells.
    bit: u64,
    /// The read shift register: shifts left, and latches when a one reaches
    /// bit 7.
    rsr: u8,
}

impl Mechanism {
    fn fresh(installed: bool) -> Mechanism {
        Mechanism {
            installed,
            disk: false,
            write_protect: false,
            double_sided: true,
            superdrive: false,
            track: 0,
            outward: false,
            motor: false,
            switched: false,
            side: false,
            tach: false,
            read_line: false,
            bit: 0,
            rsr: 0,
        }
    }

    /// The status line `addr` selects, where `addr` is `CA2:CA1:CA0:SEL`.
    ///
    /// The table is Apple's, from Neil Parker's *Controlling the 3.5 Drive
    /// Hardware on the Apple IIGS* (1994), "Accessing Disk Drive Status and
    /// Control Bits" — the only published listing of the Sony mechanism's
    /// sixteen one-bit registers. Chapter 9 of the *Guide to the Macintosh
    /// Family Hardware* names the cable's signals but does **not** carry this
    /// table, which is how the first version of this function came to invent
    /// one; see `docs/platforms/mac-plus.md`.
    ///
    /// Its own summary of the polarities is the thing to keep in mind: "the
    /// settings of most of these bits are *backwards*: 0 means yes and 1 means
    /// no". A drive that is not installed lets go of the cable and every line
    /// reads as the pull-up, which is `true` — which is the same thing as
    /// answering "no" to all sixteen.
    fn sense(&self, addr: u8) -> bool {
        if !self.installed {
            return true;
        }
        match addr {
            // Step direction: 0 steps inward, toward higher-numbered tracks.
            0 => self.outward,
            // Disk in place: 0 while a disk is in the drive.
            1 => !self.disk,
            // Disk is stepping: 0 while the head is moving. This model steps
            // instantly, so it is never caught in between.
            2 => true,
            // Disk locked: 0 while the disk is write protected.
            3 => !(self.disk && self.write_protect),
            // Motor on: 0 while the spindle is turning.
            4 => !self.motor,
            // Track 0: 0 while the head is over track 0.
            5 => self.track != 0,
            // Disk switched: 0 once the user has ejected a disk.
            6 => !self.switched,
            // Tachometer: sixty pulses a revolution. A line that never moves
            // is a drive that reports no rotation, which is what a stopped
            // motor looks like.
            7 => self.tach,
            // The two heads' instantaneous read lines. These two addresses
            // differ only in `SEL`, and the note says reading one of them
            // *configures the drive* to do its I/O with that head — so
            // addressing one selects the side as well as reading it.
            8 | 9 => self.read_line,
            // `CA2:CA1:CA0 = 101`, which Apple's note leaves unassigned on the
            // 800K mechanism. A **SuperDrive** answers at `SEL` *high* —
            // address 11 — asserted low like every other line on this cable,
            // and leaves address 10 to the pull-up.
            //
            // **Which of the two is measured, not read**, and the measurement
            // is the sharpest differential this board has produced. A real
            // Macintosh Classic ROM, on the same board with the same disk:
            //
            // ```text
            //   neither asserted   the ROM drives the mechanism as an IWM,
            //                      spins it up, steps to track 79 and reads
            //                      GCR — a plain 800K drive, and the path a
            //                      Plus uses
            //   11 asserted        the ROM switches the controller into ISM
            //                      mode, loads the parameter RAM with Apple's
            //                      own published MFM table, and reads the
            //                      1.44 MB disk
            //   both asserted      the ROM never touches the mechanism at all:
            //                      279 accesses in twenty virtual seconds, no
            //                      motor, no step, and the insert-disk icon
            //                      for ever
            // ```
            //
            // So address 11 is the line that says "this is a SuperDrive", and
            // address 10 is a *different* line that a SuperDrive does not
            // assert. **What address 10 is for was not established here** and
            // nothing in this file guesses: it reads as the cable's pull-up,
            // which is what the ROM requires and what an unassigned line does.
            // The previous model answered both halves from one flag, on the
            // reasoning that "the mechanism has one such line and no way to
            // make it depend on `SEL`" — true of *drive installed*, and the
            // thing that kept this board off the Finder.
            10 => true,
            11 => !self.superdrive,
            // Number of sides: 1 on a double-sided mechanism. One of the two
            // lines in this table that is *not* inverted.
            12 => self.double_sided,
            // Disk ready for reading: 0 once the drive will hand over data.
            // The note is unsure of this one and says only that the firmware
            // waits for it to go low before looking for a sector's address
            // field; a Plus ROM does exactly that. Nothing here models
            // spin-up, so a turning spindle with a disk on it is ready.
            13 => !(self.motor && self.disk),
            // Drive installed: 0 while a drive is connected. The note lists
            // this at `SEL` on (address 15) and a Macintosh Plus ROM reads it
            // at `SEL` off (address 14) — it is the line the ROM tests before
            // it will touch the drive at all, and answering `true` at the
            // address it uses is what kept this board on the insert-disk
            // screen. Both halves answer, because the mechanism has one such
            // line and no way to make it depend on `SEL`.
            _ => !self.installed,
        }
    }

    /// Apply the write register `CA1:CA0:SEL` addresses, with `CA2` as its
    /// data.
    ///
    /// Same source as [`Mechanism::sense`], "The control functions are as
    /// follows". Note that `SEL` is part of the *address* here as well, which
    /// is what separates "set the step direction" from "reset the
    /// disk-switched flag".
    fn control(&mut self, addr: u8, data: bool) {
        if !self.installed {
            return;
        }
        match addr {
            // Step direction: a one sets it outward, toward track 0.
            0b000 => self.outward = data,
            // Reset the disk-switched flag, on the one.
            0b001 => {
                if data {
                    self.switched = false;
                }
            }
            // One step in the current direction, on the zero.
            0b010 => {
                if !data {
                    if self.outward {
                        self.track = self.track.saturating_sub(1);
                    } else if self.track < MAX_TRACK {
                        self.track += 1;
                    }
                }
            }
            // The spindle motor: on for a zero, off for a one.
            0b100 => self.motor = !data,
            // Eject, on the one — **but not while the spindle is turning**.
            //
            // That interlock is a measurement of Apple's own code rather than
            // a reading, and it is what `docs/platforms/mac-classic.md`'s
            // ledger item 1 was about. A Macintosh Classic running Mac OS
            // 6.0.8 drives the phase lines to `CA2:CA1:CA0 = 111` and strobes
            // `LSTRB` at the instant the Finder finishes with the startup
            // volume, which by Apple's own table (`Mechanism::sense`'s source)
            // is this register with `CA2` as a one. The two readings the
            // ledger offered were that `SEL` is not the VIA's `PA5` on a
            // Classic, or that the mechanism refuses. The first is **refuted**:
            // the trace prints `SEL` beside every phase write and it is low
            // here, and it has to be `PA5` anyway or the head could not be
            // chosen, since `RDDATA0` and `RDDATA1` differ only in that line
            // and this machine reads both sides of its disk.
            //
            // What is left is the mechanism, and the register write
            // immediately before the strobe is what points at it: the ROM sets
            // the ISM's mode register to `$82` — `MotorOn` and `ENBL1` — and
            // *then* asks to eject. Nothing stops the spindle first. A Sony
            // mechanism will not throw a disk out from under a turning
            // spindle, so the command is refused and the guest agrees: the
            // startup volume's icon stays on the desktop, undimmed, and the
            // machine goes on reading and writing the disk for another virtual
            // minute. Modelling the command literally took the disk away while
            // the Finder still had it mounted.
            //
            // **Inferred**, in this sense: no document here states the
            // interlock. What is measured is that the guest does not accept
            // the eject, and this is the mechanism that would explain it.
            0b110 if data && self.disk && !self.motor => {
                self.disk = false;
                self.switched = true;
            }
            // The note lists no function for the other three addresses.
            _ => {}
        }
    }
}

/// Everything the chip owns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The eight soft switches, one per bit — see the `SW_*` constants.
    switches: u8,
    /// The mode register, loaded by a write while `Q7:Q6` is `11`.
    mode: u8,
    /// The data register: the byte the shifter last latched, or zero once the
    /// guest has taken it. Bit 7 is set in every byte a disk can carry, so a
    /// zero here is "nothing yet" and that is what a guest polls on.
    data: u8,
    /// Bit cells simulated. One tick of this device's clock is one cell.
    ticks: u64,
    /// The two mechanisms the chip's `SELECT` line picks between.
    drives: [Mechanism; 2],
    /// `SEL`, which the VIA drives and the drive register file uses as its
    /// fourth address bit.
    ///
    /// Snapshotted, for the reason `mac.via` gives about its own pins: a level
    /// that came back wrong turns the realize sweep at the end of a restore
    /// into a change.
    sel: bool,
    /// The MFM separator. Off, and therefore invisible, unless a SWIM in ISM
    /// mode turned it on. See [`Framer`].
    mfm: Framer,
    /// The write side of the head. See [`Writer`].
    writer: Writer,
    /// How many times the guest has loaded the mode register.
    ///
    /// The count alone, with no opinion about what was written: it is what
    /// lets `mac.swim` recognise the ISM mode switch — four consecutive mode
    /// writes with bit 6 going `1, 0, 1, 1` — without this file knowing what
    /// an ISM is, and without a second copy of the `Q7:Q6` switch state over
    /// in the SWIM to work out which write was a mode write.
    mode_writes: u32,
}

impl State {
    fn fresh(installed: [bool; 2]) -> State {
        State {
            switches: 0,
            mode: 0,
            data: 0,
            ticks: 0,
            drives: [
                Mechanism::fresh(installed[0]),
                Mechanism::fresh(installed[1]),
            ],
            sel: true,
            mfm: Framer::default(),
            writer: Writer::default(),
            mode_writes: 0,
        }
    }

    /// Which mechanism `SELECT` is pointing at.
    fn selected(&self) -> usize {
        usize::from(self.switches & SW_SELECT != 0)
    }

    /// The drive register address: `CA2:CA1:CA0:SEL`.
    fn drive_address(&self) -> u8 {
        let s = self.switches;
        (u8::from(s & SW_CA2 != 0) << 3)
            | (u8::from(s & SW_CA1 != 0) << 2)
            | (u8::from(s & SW_CA0 != 0) << 1)
            | u8::from(self.sel)
    }

    /// Pick the head, if the status register was just read at one of the two
    /// addresses that does so.
    ///
    /// Apple's note is explicit that it is the *read* that configures the
    /// drive — "Instantaneous data from lower head. Reading this bit
    /// configures the drive to do I/O with the lower head" — not merely having
    /// the `CA` lines sitting there. That distinction is load-bearing: a ROM
    /// walking the sixteen switches passes through `CA2:CA1:CA0 = 100` on its
    /// way to somewhere else, and a model that latched on the switch alone
    /// came out of the ROM's startup sweep reading the upper head.
    fn pick_head(&mut self) {
        let addr = self.drive_address();
        if addr & 0xe == 0x8 {
            let which = self.selected();
            self.drives[which].side = addr & 1 != 0;
        }
    }

    /// The status register as software reads it.
    ///
    /// The IWM specification: bits 0-4 mirror the mode register, bit 5 is the
    /// drive enable, bit 6 reads zero, and bit 7 is the selected drive's
    /// status line.
    fn status(&self) -> u8 {
        let mut value = self.mode & 0x1f;
        if self.switches & SW_ENABLE != 0 {
            value |= STATUS_ENABLE;
        }
        // `SENSE` is the drive's line, and the drive's lines are asserted low;
        // the bit reads the *pin*, so a line at rest reads one.
        if self.drives[self.selected()].sense(self.drive_address()) {
            value |= STATUS_SENSE;
        }
        value
    }

    /// The write handshake register, `[10X]`.
    ///
    /// *SWIM Chip User's Reference*, page 11, bit by bit:
    ///
    /// > 5 - 0  These bits are reserved for future expansion and will always
    /// > read as "1"s.
    /// > 6  This bit will be a "1" while writing data. However if a byte hasn't
    /// > been loaded into the Write Data register before the IWM hardware goes
    /// > looking for it (called an underrun), then this bit will be reset to
    /// > "0" until either the chip is reset or taken out of write mode.
    /// > 7  The write buffer empty bit will be set to "1" whenever the chip is
    /// > ready to accept another byte from the processor.
    fn handshake(&self) -> u8 {
        let mut value = 0x3f;
        if !self.writer.underrun {
            value |= HANDSHAKE_NO_UNDERRUN;
        }
        if self.writer.space() > 0 {
            value |= HANDSHAKE_READY;
        }
        value
    }

    /// What a read of any of the sixteen addresses returns now.
    fn read_value(&self) -> u8 {
        match (self.switches & SW_Q7 != 0, self.switches & SW_Q6 != 0) {
            (false, false) => self.data,
            (false, true) => self.status(),
            (true, false) => self.handshake(),
            // Reading the mode register is not a thing the chip does.
            (true, true) => 0xff,
        }
    }

    /// Set the soft switch `index` selects, and apply whatever that changed.
    fn switch(&mut self, index: u8) {
        let bit = 1u8 << (index >> 1);
        let on = index & 1 != 0;
        let before = self.switches;
        if on {
            self.switches |= bit;
        } else {
            self.switches &= !bit;
        }
        // `LSTRB` going high is what latches a drive control register; the
        // address and the data are the `CA` lines as they stand at that moment.
        // `[111]` is Write Data and `[110]` is Set Mode (*SWIM Chip User's
        // Reference*, page 10), so **the chip is writing exactly while `L7`
        // and the `MotorOn` latch are both set** — `Q6` moves between Write
        // Load and Write, and both of those are write states. Recomputed only
        // when one of those two latches actually moved, because a SWIM in ISM
        // mode drives the phase lines through `State::switch` as well and owns
        // the write mode itself ([`Iwm::set_writing`]).
        if bit == SW_Q7 || bit == SW_ENABLE {
            let writing = self.switches & (SW_Q7 | SW_ENABLE) == (SW_Q7 | SW_ENABLE);
            if writing != self.writer.on {
                // Leaving write mode is what clears the underrun flag, page 11.
                self.writer.restart(0);
                self.writer.on = writing;
                // An IWM writes Apple GCR and nothing else.
                self.writer.mfm = false;
            }
        }
        if bit == SW_LSTRB && on && before & SW_LSTRB == 0 {
            let s = self.switches;
            let addr = (u8::from(s & SW_CA1 != 0) << 2)
                | (u8::from(s & SW_CA0 != 0) << 1)
                | u8::from(self.sel);
            let data = s & SW_CA2 != 0;
            let which = self.selected();
            self.drives[which].control(addr, data);
        }
    }
}

/// How many cells go by before the shifter latches its next byte, given the
/// cylinder, where the head is on it and what is in the register.
///
/// The shifter's one rule is that a byte is complete when a one reaches bit 7
/// (see the module docs), so the answer is arithmetic rather than a
/// simulation: a register already holding a one needs only the shifts that
/// carry its highest one up to bit 7, and an empty one waits for the next one
/// on the medium and then eight more cells. `None` is a cylinder with no one
/// on it at all — an erased track, which never latches anything.
fn cells_to_latch(bits: &Track, len: u64, bit: u64, rsr: u8) -> Option<u64> {
    if len == 0 {
        // An unformatted cylinder shifts nothing past the head, so whatever is
        // in the register stays there. Answering from `rsr` here would name an
        // event a cell or two out that then never happens, over and over.
        return None;
    }
    if rsr != 0 {
        // `leading_zeros` counts from bit 7, so it *is* the number of shifts
        // the highest one still owes. A latched byte clears the register, so
        // bit 7 is never already set here.
        return Some(u64::from(rsr.leading_zeros()));
    }
    (0..len)
        .find(|d| bits.bit((bit + d) as usize))
        .map(|d| d + 8)
}

/// The disks in the two mechanisms, and the cylinder under the head.
///
/// One lock for both because the second is built out of the first, and because
/// it sits **below** the chip's own state lock: `advance_to` holds `State` and
/// reaches in here, which is `DEVICE` then `LEAF` and is the ranked order.
///
/// `bits` is **derived state**: never serialized, and thrown away whenever the
/// head moves, the side changes or a disk comes or goes (`CLAUDE.md`,
/// *Devices*).
#[derive(Debug, Default)]
struct Media {
    disks: [Option<Disk>; 2],
    /// Which mechanism, cylinder and side `bits` holds, or `None` when nothing
    /// has been built.
    under: Option<(usize, u8, bool)>,
    bits: Track,
    /// Whether the head has written to `bits` since it was built.
    ///
    /// The cells are the medium and the image is the record of it, so the two
    /// have to be put back together before the cache is thrown away — which is
    /// the one place a track's cells can be lost. Doing it *here* rather than
    /// per byte is what makes a write cost what a read costs: decoding a whole
    /// cylinder is the expensive half and it happens once per head movement,
    /// not once per sector.
    dirty: bool,
}

impl Media {
    /// Put whatever the head has written back into the image.
    ///
    /// The sectors come off the cells through the *same* decoder the read path
    /// is tested against ([`Disk::absorb`]), so a write that did not come out
    /// as a readable field is not absorbed and the image keeps what it had.
    fn flush(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let Some((which, track, side)) = self.under else {
            return;
        };
        if let Some(disk) = self.disks[which].as_mut() {
            disk.absorb(track, side, &self.bits);
        }
    }
}

/// The chip, as something an address space can dispatch to.
struct Shared {
    state: Mutex<State>,
    /// The disks and the cylinder under the head. See [`Media`].
    media: Mutex<Media>,
    /// Published without a lock for the scheduler.
    ticks: AtomicU64,
    /// The cell at which the shifter will next latch a byte, or [`u64::MAX`]
    /// when nothing is turning. Published for [`LazyDevice::next_event_tick`],
    /// which is asked under the scheduler's own leaf lock and so may not take
    /// one of ours.
    next_latch: AtomicU64,
    /// The catch-up handle a register access syncs through (§4.2).
    lazy: Mutex<Option<LazyHandle>>,
}

impl Shared {
    /// Bring the chip up to date before a register access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, debug: bool) {
        if debug {
            return;
        }
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            // A refusal means catch-up for this chip is already running further
            // up the stack; the access still has to be answered from where the
            // head stands.
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    /// Shift the disk past the head until `target` bit cells have gone by.
    fn advance_to(&self, target: u64) {
        let mut state = self.state.lock();
        if target > state.ticks {
            self.shift(&mut state, target);
        }
        self.publish_latch(&state);
    }

    /// Shift `target - state.ticks` cells past the head.
    fn shift(&self, state: &mut State, target: u64) {
        let cells = target - state.ticks;
        state.ticks = target;
        self.ticks.store(target, Ordering::Relaxed);
        let which = state.selected();
        let drive = state.drives[which];
        // A motor that is not turning moves no medium past the head, and a
        // drive with nothing in it has none to move.
        if !drive.motor || !drive.disk {
            return;
        }
        let mut media = self.media.lock();
        let len = Shared::cylinder(&mut media, which, drive.track, drive.side);
        if len == 0 {
            // An unformatted cylinder: the head sees nothing and the shifter
            // stays where it is.
            return;
        }
        // A whole revolution is the same bits again, so anything beyond one
        // lands in the same place; only the remainder has to be walked.
        let steps = if cells >= len {
            len + cells % len
        } else {
            cells
        };
        let (mut bit, mut rsr, mut data) = (drive.bit, drive.rsr, state.data);
        let (mut line, mut tach) = (drive.read_line, drive.tach);
        let mut framer = state.mfm;
        let mut writer = state.writer;
        // A head cannot read and write at once. A tab over the hole stops the
        // *medium* changing and nothing else: the chip has no idea, goes on
        // shifting, and its handshake reads exactly as it would on a disk it
        // was allowed to write — which is what the drive's `disk locked` line
        // is for and why software is expected to check it first.
        let writing = writer.on;
        let protect = drive.write_protect;
        for _ in 0..steps {
            if writing {
                line = writer.cell();
                if !protect {
                    media.bits.set_bit(bit as usize, line);
                    media.dirty = true;
                }
            } else {
                line = media.bits.bit(bit as usize);
                if framer.on {
                    // ISM mode: the byte boundary is the format's, not the
                    // shifter's, so the GCR rule below is not run at all. The
                    // two are exclusive — a chip cannot be framing both ways at
                    // once.
                    framer.cell(line);
                } else {
                    rsr = (rsr << 1) | u8::from(line);
                    if rsr & 0x80 != 0 {
                        data = rsr;
                        rsr = 0;
                    }
                }
            }
            bit = (bit + 1) % len;
            // Sixty tachometer pulses a revolution: a hundred and twenty half
            // cycles, so the line is which of them the head is in.
            tach = (bit * 120 / len) % 2 == 1;
        }
        state.data = data;
        state.mfm = framer;
        state.writer = writer;
        let drive = &mut state.drives[which];
        drive.bit = bit;
        drive.rsr = rsr;
        drive.read_line = line;
        drive.tach = tach;
    }

    /// Put the cylinder under the head into `media.bits` and say how long it
    /// is. Derived state: rebuilt whenever the head, the side or the disk
    /// moves, never serialized.
    fn cylinder(media: &mut Media, which: usize, track: u8, side: bool) -> u64 {
        let want = (which, track, side);
        if media.under != Some(want) {
            // The cells about to be thrown away may be the only copy of what
            // the head wrote, so they go back into the image first.
            media.flush();
            media.bits = match &media.disks[which] {
                Some(disk) => disk.track(track, side),
                None => Track::new(),
            };
            media.under = Some(want);
        }
        media.bits.len() as u64
    }

    /// Publish the cell at which the shifter will next latch a byte.
    ///
    /// **This is what makes a read land on the right byte.** The scheduler
    /// bounds a round by the earliest event any lazily-advanced device names
    /// (`Scheduler::lazy_deadline`), and an access is answered at the position
    /// the round has reached — a 68000 publishes no live cursor of its own, so
    /// without an event here the chip stands still for a whole round and a
    /// round longer than a byte hands the guest the *last* byte of it and
    /// loses the rest. A real Macintosh ROM reading a track drops one byte in
    /// three that way, which is every sector it tries.
    ///
    /// So the latch is named as what it is: an internal event, past which a
    /// read of the data register answers differently. It costs a round per
    /// byte — about fifty thousand a second — and only while a disk is
    /// actually turning under the head.
    fn publish_latch(&self, state: &State) {
        let which = state.selected();
        let drive = state.drives[which];
        if !drive.motor || !drive.disk {
            self.next_latch.store(u64::MAX, Ordering::Relaxed);
            return;
        }
        let mut media = self.media.lock();
        let len = Shared::cylinder(&mut media, which, drive.track, drive.side);
        let ahead = if state.writer.on {
            // Writing: the event is the cell the shifter wants the next byte
            // on, for exactly the reason the read side names its latch — a
            // round that ran past several byte times at once would underrun a
            // guest that was keeping up perfectly well.
            (len != 0).then(|| state.writer.cells_ahead())
        } else if state.mfm.on {
            // ISM mode frames on the format's boundaries; see
            // `Framer::cells_ahead`.
            (len != 0).then(|| state.mfm.cells_ahead())
        } else {
            cells_to_latch(&media.bits, len, drive.bit, drive.rsr)
        };
        drop(media);
        self.next_latch.store(
            ahead.map_or(u64::MAX, |n| state.ticks.saturating_add(n)),
            Ordering::Relaxed,
        );
    }

    /// Throw the cached cylinder away and say when the next byte lands from
    /// wherever the head now stands.
    fn invalidate(&self) {
        {
            let mut media = self.media.lock();
            // Throwing the cylinder away is the one place written cells can be
            // lost, and `Media::flush` needs `under` to know where they came
            // from — so it has to happen *before* the forgetting, not inside
            // the rebuild.
            media.flush();
            media.under = None;
        }
        let state = self.state.lock();
        self.publish_latch(&state);
    }

    /// Re-announce the next byte's cell **without** throwing the cylinder
    /// away.
    ///
    /// The distinction is not a micro-optimisation. Rebuilding the cache means
    /// re-encoding a whole track — two hundred thousand cells for a 1.44 MB
    /// cylinder — and an ISM pushes its configuration at the mechanism after
    /// *every* register write, of which Apple's ROM makes thousands a second
    /// while it reads a disk. Invalidating on each of those re-encoded the
    /// same cylinder over and over and made a virtual second cost a wall
    /// minute. Only the three things that change which cells are under the
    /// head — the cylinder, the side, and the disk itself — may invalidate.
    fn republish(&self) {
        let state = self.state.lock();
        self.publish_latch(&state);
    }
}

impl core::fmt::Debug for Shared {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut s = f.debug_struct("Iwm");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl MemOps for Shared {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        let index = ((offset / REGISTER_STRIDE) & 0xf) as u8;
        self.sync(attrs.debug);
        let mut state = self.state.lock();
        if attrs.debug {
            // A debugger must be able to look without moving a switch, and
            // every address here moves one. So a debug read answers from the
            // state as it stands, changes nothing, and does not take the byte
            // out of the data register (invariant 5).
            *byte = state.read_value();
            return Ok(());
        }
        state.switch(index);
        *byte = state.read_value();
        if state.switches & (SW_Q7 | SW_Q6) == SW_Q6 {
            // A read of the status register at one of the two read-data
            // addresses is what tells the drive which head to use.
            state.pick_head();
        }
        if state.switches & (SW_Q7 | SW_Q6) == 0 {
            // Reading the data register takes the byte: a guest polls it until
            // bit 7 is set, and every byte a disk can carry has bit 7 set, so
            // zero is the "nothing yet" this leaves behind.
            state.data = 0;
        }
        // A read is how the motor gets turned on and how a head is picked, so
        // the next latch is re-announced from where this access left things.
        self.publish_latch(&state);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Every address is a switch, so there is no harmless debug write.
            return Err(BusError::BadAccess);
        }
        let index = ((offset / REGISTER_STRIDE) & 0xf) as u8;
        self.sync(false);
        let mut state = self.state.lock();
        state.switch(index);
        // *SWIM Chip User's Reference*, page 10: the three latches `L7`, `L6`
        // and `MotorOn` name the register, and "Writing to a register must be
        // done from a '1' state". `[110]` is `Set Mode` and `[111]` is `Write
        // Data`, so **which of the two a store lands in is the `MotorOn`
        // latch** — the same bit the sixteen soft switches call `ENABLE`.
        //
        // That distinction is why a Macintosh can load its mode register at all
        // and still write a disk through the same address: Apple's own note
        // (Neil Parker, 1994) is explicit that "the write to the mode register
        // will fail unless the drive is fully deactivated".
        let enabled = state.switches & SW_ENABLE != 0;
        match (state.switches & SW_Q7 != 0, state.switches & SW_Q6 != 0) {
            (true, true) if enabled => {
                // Page 12: "Writing to either L7=, L6=1 or MotorOn=1 while in
                // this state will write a byte of data to the write buffer."
                // A buffer that is already full drops it, which page 12 says is
                // harmless: "Writing to the chip faster than this won't hurt
                // anything, but the last byte written to the chip when the
                // buffer is emptied is the one that will be used."
                state.writer.push(*value, WriteKind::Data);
            }
            (true, true) => {
                state.mode = *value & 0x7f;
                state.mode_writes = state.mode_writes.wrapping_add(1);
            }
            _ => {}
        }
        // A write moves `LSTRB`, which is how the motor and the stepper are
        // driven; both change when the next byte arrives.
        self.publish_latch(&state);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An eight-bit part on the low lane of the 68000's word bus.
        AccessConstraints::word(Width::U8, Endian::Big)
    }
}

/// The `SEL` input.
#[derive(Debug)]
struct SelPin {
    shared: Arc<Shared>,
    inputs: FanIn,
}

impl WireSink for SelPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        self.shared.state.lock().sel = high;
    }
}

/// An Integrated Woz Machine and the mechanisms on its cable.
#[derive(Debug)]
pub struct Iwm {
    shared: Arc<Shared>,
    region: RegionRef,
    /// Which of the two cable positions has a drive on it, for reset.
    installed: [bool; 2],
    /// Whether those mechanisms are SuperDrives, for the same.
    superdrive: bool,
    /// The pin, kept alive here: a net holds only a `Weak` to its sinks.
    pins: Mutex<Vec<Arc<SelPin>>>,
}

impl Iwm {
    /// Build the chip.
    ///
    /// `drives` says how many mechanisms are on the cable: 1 for a Macintosh
    /// Plus with only its internal drive, 2 with an external one plugged in.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `drives` is not 1 or 2, or if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Iwm> {
        let mut r = props.reader();
        let drives = r.or("drives", 1u64)?;
        let image = r.optional_media("image")?.map(|m| m.bytes().to_vec());
        let image2 = r.optional_media("image2")?.map(|m| m.bytes().to_vec());
        r.finish()?;
        if drives == 0 || drives > 2 {
            return Err(Error::Property(alloc::format!(
                "property `drives`: an IWM's cable takes one or two mechanisms, not {drives}"
            )));
        }
        let iwm = Iwm::with_drives([true, drives == 2]);
        // An empty slot is an empty drive rather than a bad image: a Macintosh
        // with no disk in it is the ordinary case and the one the ROM draws a
        // picture for.
        if let Some(bytes) = image.filter(|b| !b.is_empty()) {
            iwm.insert(0, Disk::from_image(&bytes)?);
        }
        if let Some(bytes) = image2.filter(|b| !b.is_empty()) {
            if drives < 2 {
                return Err(Error::Property(alloc::format!(
                    "property `image2`: there is a disk for the second drive and `drives` is                      {drives}; a cable with one mechanism on it has nowhere to put it"
                )));
            }
            iwm.insert(1, Disk::from_image(&bytes)?);
        }
        Ok(iwm)
    }

    /// The same, saying exactly which cable positions are occupied.
    #[must_use]
    pub fn with_drives(installed: [bool; 2]) -> Iwm {
        Iwm::with_mechanisms(installed, false)
    }

    /// The same, with **SuperDrive** mechanisms on the cable: they answer the
    /// drive register at `CA2:CA1:CA0 = 101`, which is how a computer finds out
    /// it can ask for high-density media.
    ///
    /// A `mac.iwm` never builds one — a Plus has 800K drives — but `mac.swim`
    /// does.
    #[must_use]
    pub fn with_superdrives(installed: [bool; 2]) -> Iwm {
        Iwm::with_mechanisms(installed, true)
    }

    fn with_mechanisms(installed: [bool; 2], superdrive: bool) -> Iwm {
        let mut fresh = State::fresh(installed);
        for drive in &mut fresh.drives {
            drive.superdrive = superdrive;
        }
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, fresh),
            media: Mutex::with_rank(LockRank::LEAF, Media::default()),
            ticks: AtomicU64::new(0),
            next_latch: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_SPAN,
            Arc::clone(&shared) as Arc<dyn MemOps>,
        ));
        Iwm {
            shared,
            region,
            installed,
            superdrive,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Read a switch the way the address space would, for a test.
    #[must_use]
    pub fn peek(&self, index: u8) -> u8 {
        let mut byte = [0u8; 1];
        let _ = self.shared.read(
            u64::from(index & 0xf) * REGISTER_STRIDE,
            &mut byte,
            MemAttrs::DEFAULT,
        );
        byte[0]
    }

    /// Write one the same way.
    pub fn poke(&self, index: u8, value: u8) {
        let _ = self.shared.write(
            u64::from(index & 0xf) * REGISTER_STRIDE,
            &[value],
            MemAttrs::DEFAULT,
        );
    }

    /// Set the level the VIA is driving onto `SEL`, for a test with no wire
    /// graph.
    ///
    /// It is an address bit into the drive's register file and nothing else:
    /// it does not move the head, the medium or the shifter, so the cell the
    /// next byte lands on is none of its business.
    pub fn set_sel(&self, high: bool) {
        self.shared.state.lock().sel = high;
    }

    /// Whether the motor of drive `which` is running.
    #[must_use]
    pub fn motor(&self, which: usize) -> bool {
        self.shared.state.lock().drives[which & 1].motor
    }

    /// Which cylinder drive `which`'s head is over.
    #[must_use]
    pub fn track(&self, which: usize) -> u8 {
        self.shared.state.lock().drives[which & 1].track
    }

    /// Whether drive `which` has a disk in it.
    #[must_use]
    pub fn has_disk(&self, which: usize) -> bool {
        self.shared.state.lock().drives[which & 1].disk
    }

    /// Put `disk` in drive `which`, taking out whatever was there.
    pub fn insert(&self, which: usize, disk: Disk) {
        let which = which & 1;
        {
            let mut state = self.shared.state.lock();
            let protect = disk.write_protected();
            let drive = &mut state.drives[which];
            drive.disk = true;
            drive.write_protect = protect;
            drive.bit = 0;
            drive.rsr = 0;
        }
        {
            let mut media = self.shared.media.lock();
            // Whatever the head wrote belongs to the disk coming *out*.
            media.flush();
            media.disks[which] = Some(disk);
            media.under = None;
        }
        self.shared.invalidate();
    }

    /// Take the disk out of drive `which`.
    pub fn eject(&self, which: usize) {
        let which = which & 1;
        {
            let mut state = self.shared.state.lock();
            let drive = &mut state.drives[which];
            if drive.disk {
                drive.switched = true;
            }
            drive.disk = false;
            drive.write_protect = false;
        }
        {
            let mut media = self.shared.media.lock();
            // The same: a disk leaving the drive takes what was written to it.
            media.flush();
            media.disks[which] = None;
            media.under = None;
        }
        self.shared.invalidate();
    }

    /// Put a blank disk in drive `which`, or take one out — the short form a
    /// test uses when what is on the disk does not matter.
    pub fn set_disk(&self, which: usize, present: bool, write_protect: bool) {
        if !present {
            self.eject(which);
            return;
        }
        let mut disk = Disk::blank(2);
        disk.set_write_protected(write_protect);
        self.insert(which, disk);
    }

    /// The disk in drive `which`, if there is one.
    ///
    /// Whatever the head has written and not yet put back goes into the image
    /// first, so what comes out is the medium as it stands rather than as it
    /// was when the cylinder was last built.
    #[must_use]
    pub fn disk(&self, which: usize) -> Option<Disk> {
        let mut media = self.shared.media.lock();
        media.flush();
        media.disks[which & 1].clone()
    }

    /// Put whatever the head has written back into the image, without
    /// disturbing anything else.
    pub fn flush_writes(&self) {
        self.shared.media.lock().flush();
    }

    // -- the write side ------------------------------------------------------

    /// Turn the write head on or off, restarting the buffer either way with
    /// `seed` in its CRC generator.
    ///
    /// `mfm` says whether a byte becomes sixteen MFM cells or its own eight:
    /// an ISM out of GCR mode writes the first, an IWM and an ISM in GCR mode
    /// the second. Idempotent, for the reason [`Iwm::set_mfm_framing`] gives —
    /// an ISM pushes its whole configuration at the mechanism after every
    /// register write, and a restart on each of those would throw away the byte
    /// the head is in the middle of.
    pub fn set_writing(&self, on: bool, mfm: bool, seed: u16) {
        {
            let mut state = self.shared.state.lock();
            if state.writer.on == on && state.writer.mfm == mfm {
                return;
            }
            state.writer.restart(seed);
            state.writer.on = on;
            state.writer.mfm = mfm;
        }
        self.shared.republish();
    }

    /// Whether the head is in write mode.
    #[must_use]
    pub fn writing(&self) -> bool {
        self.shared.state.lock().writer.on
    }

    /// Hand the head a byte. `false` if the buffer was full and it was
    /// dropped, which page 12 says is harmless.
    pub fn push_write(&self, byte: u8, kind: WriteKind) -> bool {
        let took = self.shared.state.lock().writer.push(byte, kind);
        if took {
            // A slot filled changes when the shifter next wants one.
            self.shared.republish();
        }
        took
    }

    /// How many bytes the buffer will take, 0 to [`WRITE_FIFO`].
    #[must_use]
    pub fn write_space(&self) -> u8 {
        self.shared.state.lock().writer.space()
    }

    /// Whether the head has reached a byte boundary with an empty buffer since
    /// the last restart.
    #[must_use]
    pub fn write_underrun(&self) -> bool {
        self.shared.state.lock().writer.underrun
    }

    /// The generator over every byte the head has laid down since the last
    /// restart.
    #[must_use]
    pub fn write_crc(&self) -> u16 {
        self.shared.state.lock().writer.crc
    }

    /// The byte the shifter last latched and the guest has not taken.
    #[must_use]
    pub fn latched(&self) -> u8 {
        self.shared.state.lock().data
    }

    /// Bit cells shifted past the head.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Shift the disk past the head until `target` bit cells have gone by.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// The soft switches, for a test that wants to see what a ROM set.
    #[must_use]
    pub fn switches(&self) -> u8 {
        self.shared.state.lock().switches
    }

    /// The mode register the guest last wrote.
    #[must_use]
    pub fn mode(&self) -> u8 {
        self.shared.state.lock().mode
    }

    /// The level on `SEL`, the drive register file's fourth address bit.
    #[must_use]
    pub fn sel(&self) -> bool {
        self.shared.state.lock().sel
    }

    /// The drive register address the switches and `SEL` name now:
    /// `CA2:CA1:CA0:SEL`.
    ///
    /// For a trace that wants to know *which* of the mechanism's sixteen
    /// status lines an access read, which a log of switch numbers alone cannot
    /// say.
    #[must_use]
    pub fn drive_address(&self) -> u8 {
        self.shared.state.lock().drive_address()
    }

    /// Which mechanism `SELECT` is pointing at.
    #[must_use]
    pub fn selected_drive(&self) -> usize {
        self.shared.state.lock().selected()
    }

    /// How many times the guest has loaded the mode register.
    ///
    /// For `mac.swim`, which counts the four in a row that ask for ISM mode.
    #[must_use]
    pub fn mode_writes(&self) -> u32 {
        self.shared.state.lock().mode_writes
    }

    // -- what a SWIM in ISM mode needs of the mechanism ----------------------
    //
    // The ISM's register file is a different sixteen registers from the IWM's
    // sixteen soft switches, but *the drive on the end of the cable is the
    // same drive*: the same four lines address its register file, the same
    // enables run its spindle, the same head reads it. So `mac.swim` drives it
    // through these rather than through a second copy of `Mechanism`.

    /// Bring the head up to the cycle the guest is looking at.
    ///
    /// Every access through this chip's own aperture does it already; a SWIM
    /// answering out of the ISM register set does not go through that aperture
    /// and has to ask. A debug access advances nothing (`ROADMAP.md` §15,
    /// invariant 5).
    pub fn sync(&self, debug: bool) {
        self.shared.sync(debug);
    }

    /// Drive the four phase lines and latch a drive control register if
    /// `PHASE3` rose.
    ///
    /// `phases` is `PHASE3:PHASE2:PHASE1:PHASE0` in bits 3-0, which are the
    /// mechanism's `LSTRB`, `CA2`, `CA1` and `CA0` — the same four lines the
    /// IWM's soft switches 0-7 drive, so this walks them through
    /// `State::switch` and the control register latches exactly as it does
    /// for a Plus.
    pub fn set_phases(&self, phases: u8) {
        let moved = {
            let mut state = self.shared.state.lock();
            let which = state.selected();
            let before = state.drives[which].track;
            for line in 0..4u8 {
                let want = phases & (1 << line) != 0;
                let index = (line << 1) | u8::from(want);
                state.switch(index);
            }
            // `LSTRB` rising may have stepped the head, which is one of the
            // three things that change the cells under it.
            state.drives[which].track != before
        };
        if moved {
            self.shared.invalidate();
        } else {
            self.shared.republish();
        }
    }

    /// The four phase lines as they stand, in the same bit order.
    #[must_use]
    pub fn phases(&self) -> u8 {
        self.shared.state.lock().switches & 0x0f
    }

    /// The selected drive's status line at the address the phase lines and
    /// `SEL` name, as a *pin level*: the cable's lines are asserted low, so
    /// `true` is "no".
    ///
    /// Side-effect free, which is what a debug read and a test want.
    #[must_use]
    pub fn sense(&self) -> bool {
        let state = self.shared.state.lock();
        state.drives[state.selected()].sense(state.drive_address())
    }

    /// The same, but **the read picks the head** when the address is one of
    /// the two instantaneous-read-line registers.
    ///
    /// That is the mechanism's rule and not the controller's, so it holds
    /// however the line is read — through an IWM's status register or through
    /// an ISM's handshake register. Apple's note is explicit that it is the
    /// *read* that does it: "Instantaneous data from lower head. Reading this
    /// bit configures the drive to do I/O with the lower head."
    ///
    /// This is what a SWIM in ISM mode needs, because that is the only way the
    /// head gets chosen there: page 22 makes the chip's own `HDSEL` pin an
    /// output only when the Setup register's bit 0 says so, and a Macintosh
    /// Classic ROM never sets it — it writes the phase lines to
    /// `CA2:CA1:CA0 = 100` and reads the handshake, exactly as it would drive
    /// an IWM.
    pub fn sense_and_pick_head(&self) -> bool {
        let (level, moved) = {
            let mut state = self.shared.state.lock();
            let which = state.selected();
            let addr = state.drive_address();
            let level = state.drives[which].sense(addr);
            let before = state.drives[which].side;
            state.pick_head();
            (level, state.drives[which].side != before)
        };
        if moved {
            // A different side is a different cylinder of cells under the
            // head: derived state, thrown away rather than adjusted.
            self.shared.invalidate();
        }
        level
    }

    /// The cell under the head now — the `RDDATA` line.
    #[must_use]
    pub fn read_line(&self) -> bool {
        let state = self.shared.state.lock();
        state.drives[state.selected()].read_line
    }

    /// Pick the drive, run or stop its spindle, and — only if the controller
    /// is driving the head-select pin at all — choose the head.
    ///
    /// `drive` is which cable position the ISM's two enable bits name, or
    /// `None` when neither does. This is the ISM's own path to the mechanism
    /// and it does **not** go through the drive's `LSTRB` register file the
    /// way an IWM's motor does: page 23 makes the enables and `MotorOn` bits
    /// of the mode register.
    ///
    /// `head` is `None` when the chip's `HDSEL` pin is *not* an output, which
    /// page 23 makes the ordinary case — "Sets the state of the HDSEL pin if
    /// the Q3*/HDSEL bit in the Setup register is set to '1'" — and a
    /// Macintosh Classic ROM never sets that bit. Passing `Some(false)` there
    /// instead of `None` put the head back on side 0 after every single ISM
    /// register write, so the side the drive had just been told to use through
    /// its own register file ([`Iwm::sense_and_pick_head`]) never survived to
    /// the read.
    pub fn set_enables(&self, drive: Option<usize>, motor: bool, head: Option<bool>) {
        let moved = {
            let mut state = self.shared.state.lock();
            let was = (state.selected(), state.drives[state.selected()].side);
            // `SELECT` picks which mechanism every other line addresses, so
            // the ISM's enable bits set it as well.
            if let Some(which) = drive {
                if which & 1 == 0 {
                    state.switches &= !SW_SELECT;
                } else {
                    state.switches |= SW_SELECT;
                }
            }
            for (i, mech) in state.drives.iter_mut().enumerate() {
                let picked = drive == Some(i);
                mech.motor = picked && motor;
                if let (true, Some(side)) = (picked, head) {
                    mech.side = side;
                }
            }
            (state.selected(), state.drives[state.selected()].side) != was
        };
        if moved {
            self.shared.invalidate();
        } else {
            // The spindle starting or stopping changes when the next byte
            // lands but not which cells are under the head.
            self.shared.republish();
        }
    }

    /// Turn the MFM separator on or off, restarting it either way with `seed`
    /// in its CRC generator.
    pub fn set_mfm_framing(&self, on: bool, seed: u16) {
        {
            let mut state = self.shared.state.lock();
            if state.mfm.on == on {
                // Idempotent: the ISM pushes its whole configuration onto the
                // mechanism after every register write, and a restart on each
                // of those would throw away the byte boundary the chip is in
                // the middle of.
                return;
            }
            state.mfm.restart(seed);
            state.mfm.on = on;
        }
        self.shared.republish();
    }

    /// Forget the byte boundary and empty the FIFO, reseeding the CRC and
    /// leaving the separator as it is.
    pub fn restart_mfm(&self, seed: u16) {
        {
            let mut state = self.shared.state.lock();
            state.mfm.restart(seed);
        }
        self.shared.republish();
    }

    /// The separator's CRC over every byte it has framed since the last
    /// restart.
    #[must_use]
    pub fn mfm_crc(&self) -> u16 {
        self.shared.state.lock().mfm.crc
    }

    /// The oldest framed byte and whether it is a mark, without taking it —
    /// which is what a debug read and the handshake register both want.
    #[must_use]
    pub fn peek_mfm(&self) -> Option<(u8, bool)> {
        self.shared.state.lock().mfm.peek()
    }

    /// Take it.
    pub fn take_mfm(&self) -> Option<(u8, bool)> {
        let popped = self.shared.state.lock().mfm.pop();
        if popped.is_some() {
            // A slot came free, so the next boundary matters again — but the
            // cells under the head have not moved.
            self.shared.republish();
        }
        popped
    }

    /// How many framed bytes are waiting, 0 to [`MFM_FIFO`].
    #[must_use]
    pub fn mfm_queued(&self) -> u8 {
        self.shared.state.lock().mfm.count
    }

    /// Whether a framed byte has been lost because the FIFO was full, clearing
    /// the flag.
    pub fn take_mfm_overrun(&self) -> bool {
        let mut state = self.shared.state.lock();
        core::mem::take(&mut state.mfm.overrun)
    }
}

impl Device for Iwm {
    fn class(&self) -> &'static DeviceClass {
        &IWM_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the wire
        // graph brings `SEL`.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            let ticks = state.ticks;
            let sel = state.sel;
            // A disk stays in the drive across a reset: it is a thing in a
            // slot, not a register. The chip's switches and the mechanism's
            // motor and head position do come back.
            let disks = [
                (state.drives[0].disk, state.drives[0].write_protect),
                (state.drives[1].disk, state.drives[1].write_protect),
            ];
            *state = State::fresh(self.installed);
            state.sel = sel;
            state.ticks = ticks;
            for (drive, (disk, wp)) in state.drives.iter_mut().zip(disks) {
                drive.disk = disk;
                drive.write_protect = wp;
                drive.superdrive = self.superdrive;
            }
        }
        self.shared.invalidate();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u8(state.switches)?;
        w.write_u8(state.mode)?;
        w.write_u8(state.data)?;
        w.write_u64(state.ticks)?;
        for drive in &state.drives {
            w.write_bool(drive.installed)?;
            w.write_bool(drive.disk)?;
            w.write_bool(drive.write_protect)?;
            w.write_bool(drive.double_sided)?;
            w.write_bool(drive.superdrive)?;
            w.write_u8(drive.track)?;
            w.write_bool(drive.outward)?;
            w.write_bool(drive.motor)?;
            w.write_bool(drive.switched)?;
            w.write_bool(drive.side)?;
            w.write_bool(drive.tach)?;
            w.write_bool(drive.read_line)?;
            w.write_u64(drive.bit)?;
            w.write_u8(drive.rsr)?;
        }
        w.write_bool(state.sel)?;
        // The separator is chip state, not a cache: the guest can read the
        // FIFO and the byte boundary is where the last mark left it.
        w.write_bool(state.mfm.on)?;
        w.write_u32(state.mfm.cells)?;
        w.write_u8(state.mfm.phase)?;
        w.write_bool(state.mfm.synced)?;
        for byte in state.mfm.fifo {
            w.write_u8(byte)?;
        }
        w.write_u8(state.mfm.marks)?;
        w.write_u8(state.mfm.count)?;
        w.write_bool(state.mfm.overrun)?;
        w.write_u16(state.mfm.crc)?;
        w.write_u32(state.mode_writes)?;
        // The write side is chip state for the same reason the separator is:
        // a machine snapshotted in the middle of laying a sector down comes
        // back in the middle of it, with the same bytes owed and the same
        // generator behind them.
        w.write_bool(state.writer.on)?;
        w.write_bool(state.writer.mfm)?;
        for byte in state.writer.fifo {
            w.write_u8(byte)?;
        }
        for kind in state.writer.kinds {
            w.write_u8(kind as u8)?;
        }
        w.write_u8(state.writer.count)?;
        w.write_u16(state.writer.cells)?;
        w.write_u8(state.writer.bits)?;
        w.write_bool(state.writer.prev)?;
        w.write_bool(state.writer.underrun)?;
        w.write_u16(state.writer.crc)?;
        w.write_u16(state.writer.seed)?;
        w.write_bool(state.writer.after_mark)?;
        w.write_u16(state.writer.pending_crc)?;
        // And the **medium**, when it has been written to. An image that has
        // never been written is rebuilt from the machine's own media slot on a
        // restore, so putting it in every chunk would be a megabyte and a half
        // of bytes the machine file already carries; one that *has* been
        // written cannot be rebuilt from anywhere, and dropping it would make
        // a snapshot silently undo the guest's work.
        let mut media = self.shared.media.lock();
        media.flush();
        for slot in 0..2 {
            match media.disks[slot].as_ref().filter(|d| d.written()) {
                Some(disk) => {
                    w.write_bool(true)?;
                    w.write_bytes(disk.data())?;
                    w.write_bytes(disk.tags())?;
                }
                None => w.write_bool(false)?,
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = self.shared.state.lock();
        let mut next = State::fresh(self.installed);
        next.switches = r.read_u8()?;
        next.mode = r.read_u8()? & 0x7f;
        next.data = r.read_u8()?;
        next.ticks = r.read_u64()?;
        for drive in &mut next.drives {
            drive.installed = r.read_bool()?;
            drive.disk = r.read_bool()?;
            drive.write_protect = r.read_bool()?;
            drive.double_sided = r.read_bool()?;
            drive.superdrive = r.read_bool()?;
            drive.track = r.read_u8()?.min(MAX_TRACK);
            drive.outward = r.read_bool()?;
            drive.motor = r.read_bool()?;
            drive.switched = r.read_bool()?;
            drive.side = r.read_bool()?;
            drive.tach = r.read_bool()?;
            drive.read_line = r.read_bool()?;
            drive.bit = r.read_u64()?;
            drive.rsr = r.read_u8()?;
        }
        next.sel = r.read_bool()?;
        next.mfm.on = r.read_bool()?;
        next.mfm.cells = r.read_u32()?;
        next.mfm.phase = r.read_u8()? % MFM_CELLS_PER_BYTE;
        next.mfm.synced = r.read_bool()?;
        for slot in 0..MFM_FIFO {
            next.mfm.fifo[slot] = r.read_u8()?;
        }
        next.mfm.marks = r.read_u8()?;
        next.mfm.count = r.read_u8()?.min(MFM_FIFO as u8);
        next.mfm.overrun = r.read_bool()?;
        next.mfm.crc = r.read_u16()?;
        next.mode_writes = r.read_u32()?;
        next.writer.on = r.read_bool()?;
        next.writer.mfm = r.read_bool()?;
        for slot in 0..WRITE_FIFO {
            next.writer.fifo[slot] = r.read_u8()?;
        }
        for slot in 0..WRITE_FIFO {
            next.writer.kinds[slot] = WriteKind::from_byte(r.read_u8()?);
        }
        next.writer.count = r.read_u8()?.min(WRITE_FIFO as u8);
        next.writer.cells = r.read_u16()?;
        next.writer.bits = r.read_u8()?.min(MFM_CELLS_PER_BYTE);
        next.writer.prev = r.read_bool()?;
        next.writer.underrun = r.read_bool()?;
        next.writer.crc = r.read_u16()?;
        next.writer.seed = r.read_u16()?;
        next.writer.after_mark = r.read_bool()?;
        next.writer.pending_crc = r.read_u16()?;
        *state = next;
        // The published tick is not in the chunk and it is what the scheduler
        // reads: a restore that left it at zero would have the head advanced
        // from the wrong cell, or not at all.
        self.shared.ticks.store(next.ticks, Ordering::Relaxed);
        drop(state);
        {
            // A written medium comes back out of the chunk; an unwritten one
            // is whatever the media slot put in the drive, which the restore
            // has not touched.
            let mut media = self.shared.media.lock();
            media.dirty = false;
            for slot in 0..2 {
                if !r.read_bool()? {
                    continue;
                }
                let data = r.read_bytes()?.to_vec();
                let tags = r.read_bytes()?.to_vec();
                if let Some(disk) = media.disks[slot].as_mut() {
                    disk.set_contents(&data, &tags)?;
                }
            }
        }
        // The cylinder under the head is derived and is rebuilt from wherever
        // the restored head turns out to be, and the next byte's cell with it.
        self.shared.invalidate();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. The head is where it is at the cycle the guest looks, and a guest
    /// polling the data register is asking exactly that.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Iwm::advance_to(self, tick);
    }

    /// The cell the shifter will next latch a byte on, while a disk is
    /// turning under the head; `None` when none is.
    ///
    /// This used to be `None`, on the argument that a read syncs the chip
    /// anyway. It does — but only to where the *round* has reached, because a
    /// 68000 publishes no live cursor, so a round longer than a byte time
    /// delivered the last byte of the round and lost the rest.
    /// `Shared::publish_latch` has the measurement.
    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_latch.load(Ordering::Relaxed) {
            u64::MAX => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != SEL_PIN {
            return None;
        }
        let pin = Arc::new(SelPin {
            shared: Arc::clone(&self.shared),
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }
}

impl Instance for Iwm {}

/// The `mac.iwm` device class.
pub static IWM_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Integrated Woz Machine: sixteen soft switches, and the 400K/800K drive on its \
              cable",
    properties: &[
        PropertySpec {
            name: "drives",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many mechanisms are on the cable, 1 or 2 (default 1)",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the disk in the internal drive; empty is no disk",
        },
        PropertySpec {
            name: "image2",
            kind: ValueKind::Media,
            required: false,
            summary: "the same for the second mechanism on the cable, which needs `drives = 2`",
        },
    ],
    construct: |props| Ok(Box::new(Iwm::new(props)?)),
};

/// Add [`IWM_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&IWM_CLASS)
}

/// Bind [`IWM_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Iwm::new(props)?)))
}

/// What the validator should know about `mac.iwm`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("drives", ValueKind::Uint).range(1, 2))
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("image2", ValueKind::Media))
        .region("")
        .region("regs")
        .port(SEL_PIN, PortDir::In)
}

#[cfg(test)]
mod tests;
