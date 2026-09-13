//! Paula: the Amiga's interrupts, disk controller, UART and four audio
//! channels.
//!
//! One class, `amiga.paula`. It attaches to the custom-chip register space
//! ([`custom`](super::custom)) for the registers Appendix B assigns to `P`, and
//! it is the chip every interrupt in the machine passes through: the manual's
//! *System Control Hardware* chapter says the hardware interrupts "are brought
//! into the peripherals chip and are translated into six of the seven
//! available interrupts of the 680x0".
//!
//! ```text
//!   object paula "amiga.paula" { clock = clk / 8, custom = custom, port = "serial" }
//!   wire paula.ipl0 -> cpu.ipl0
//!   wire paula.ipl1 -> cpu.ipl1
//!   wire paula.ipl2 -> cpu.ipl2
//!   wire cia_a.irq  -> paula.int2
//!   wire cia_b.irq  -> paula.int6
//! ```
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition, and
//! nothing else:
//!
//! * Chapter 7, *System Control Hardware*, "Interrupts": the two registers, the
//!   `SET/CLR` bit, the master enable, what each bit is and which level it
//!   raises, and Figure 7-4's priorities ("external INT2 & CIAA", "external
//!   INT6 & CIAB").
//! * Chapter 8, *Interface Hardware*: "Floppy Disk Controller" (Tables 8-5 to
//!   8-8 — the CIA lines, `DSKLEN` and its write-twice rule, `DSKBYTR`,
//!   `ADKCON`, `DSKSYNC`, the disk interrupts) and "Serial Interface"
//!   (`SERPER`, `SERDAT`, `SERDATR`, Table 8-9).
//! * Chapter 5, *Audio Hardware*: the location, length, period and volume
//!   registers, the back-up registers and when the interrupt comes, attach
//!   modes (Tables 5-4 and 5-5), and the named signals of Figure 5-8's state
//!   diagram (`AUDxDR`, `AUDxDSR`, `lencount`, `pbufld1`).
//! * Appendix A for every register's bits, Appendix B for the table.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1); every
//! Amiga emulator the author knows of is GPL.
//!
//! # Interrupts
//!
//! `INTENA` and `INTREQ` share one bit layout (Appendix A, `INTENA`):
//!
//! | bit | name | level | | bit | name | level |
//! | --- | --- | --- | --- | --- | --- | --- |
//! | 15 | `SET/CLR` | | | 6 | `BLIT` | 3 |
//! | 14 | `INTEN` | master | | 5 | `VERTB` | 3 |
//! | 13 | `EXTER` | 6 | | 4 | `COPER` | 3 |
//! | 12 | `DSKSYN` | 5 | | 3 | `PORTS` | 2 |
//! | 11 | `RBF` | 5 | | 2 | `SOFT` | 1 |
//! | 10–7 | `AUD3`–`AUD0` | 4 | | 1 | `DSKBLK` | 1 |
//! | | | | | 0 | `TBE` | 1 |
//!
//! The processor is shown the highest level among the bits set in both
//! registers, provided `INTEN` is set in `INTENA`, as an **encoded level** on
//! `ipl0`–`ipl2` — which is what the 68000's three pins are, and what
//! `cpu.m68k` reads. Level 7 is never generated. `INTEN` "creates no interrupt
//! request", so `INTREQ` does not store bit 14.
//!
//! `int2` and `int6` are the manual's `INT2*` and `INT6*`: "Bit 3, PORTS,
//! becomes a 1 when the system line called INT2* becomes a logic 0." They are
//! modelled **level-sensitive** — while a line is asserted the bit is held set
//! and a clear does not stick — because both are wired-OR lines shared by a
//! CIA and the expansion bus, and software clears the source (the CIA's `ICR`)
//! before it clears `INTREQ`. The pins take this tree's polarity, high is
//! requesting, which is what `mos.8520`'s `irq` drives.
//!
//! The chip's other interrupt sources arrive on their own: `COPER` because the
//! copper writes `INTREQ` through the register bus, and `VERTB` and `BLIT`
//! through [`PaulaPort::request`], which Agnus calls.
//!
//! # Time
//!
//! One clock: the **colour clock**, `clk / 8` on an A500 — 3.546895 MHz PAL,
//! 3.579545 MHz NTSC. Every period in the chapters is written in it: `SERPER`
//! "N+1 color clocks", the audio period "124 color clocks" at minimum, and
//! Agnus's memory cycles "280 ns". The device is lazily advanced (§4.2) and
//! counts colour clocks.
//!
//! The disk's bit cell is the one number that is not a whole count of it. The
//! manual gives "two microseconds per bit cell" with `FAST` set and four
//! without; that is 7.094 colour clocks, and a cell here is **7** (14 slow) —
//! 1.974 µs PAL. A drive model spins at the same rate, so what is written reads
//! back cell for cell.
//!
//! # Disk
//!
//! Paula's half of the floppy controller, as Chapter 8 describes it:
//!
//! * the raw MFM cell stream from whichever drives are presenting data is
//!   shifted into a 16-bit register; every eighth cell becomes a `DSKBYTR` byte
//!   with `DSKBYT` set, and the read clears it;
//! * the register is compared with `DSKSYNC` on every cell: a match sets
//!   `DSKSYN` in `INTREQ` "independent of the WORDSYNC enable", holds
//!   `WORDEQUAL` for one cell, and restarts byte alignment;
//! * `DSKLEN` starts DMA only when `DMAEN` is written **twice in a row** — the
//!   manual's "must be turned on twice" — and any write without it stops DMA;
//! * a read DMA assembles words, waiting for the first sync match when
//!   `WORDSYNC` is set and realigning on every match after that, and a write
//!   DMA shifts words out most significant cell first; the length counts down
//!   per word transferred and `DSKBLK` is set when it reaches zero.
//!
//! The drive mechanism — motor, select, step, direction, side, `/RDY`,
//! `/TRK0`, `/WPRO`, `/CHNG` and the index pulse — is on the two CIAs' ports
//! (Table 8-5) and belongs to `amiga.floppy`, which also supplies the cells
//! through [`DiskDrive`].
//!
//! Not modelled: the two documented hardware bugs ("the last three bits of data
//! sent to the disk" are lost, and "one less word may be read than you asked
//! for"), write precompensation (`PRECOMP`, `MFMPREC` are stored), and
//! `MSBSYNC`, whose GCR byte alignment the manual describes in one sentence.
//!
//! # Serial
//!
//! A UART timed by `SERPER` and joined to a host character port
//! ([`host::chardev`](crate::host::chardev), named by `port`). A word written
//! to `SERDAT` is sent after one start bit, least significant bit first, until
//! the highest one bit — which is how the manual's stop bits work — and the
//! low eight bits reach the host when the frame ends. `TBE` is set in `INTREQ`
//! as the word moves into the shift register; `SERDATR`'s `TBE` and `TSRE` are
//! the buffer and the shifter. Chapter 8 calls `SERDATR`'s `TBE` "not a mirror"
//! and Appendix A calls it one; this model follows the chapter.
//!
//! A byte from the host is received as a start bit, eight data bits and a stop
//! bit at the `SERPER` rate, and sets `RBF`; a byte arriving while `RBF` is
//! still set sets `OVRUN`, which clearing `RBF` resets. The host is **not a
//! wire**, and waits: the next host byte is not taken while `RBF` is set, so a
//! paste into a slow guest is not an overrun. `UARTBRK` is stored.
//!
//! # Audio
//!
//! The register face and the interrupts, and the channel state machine as far
//! as the prose describes it — the state diagram itself (Figure 5-8) is a
//! drawing whose transitions the available copy does not render legibly, so
//! what is here is the chapter's words rather than the diagram's arrows:
//!
//! * a channel starts when `DMAEN` and its `AUDxEN` are both set in `DMACON`:
//!   the length is copied to a back-up counter, Agnus is asked to reset its
//!   pointer and fetch a word, and the channel's interrupt is set — "an
//!   interrupt for the 680x0 saying that it has completed retrieving working
//!   copies of length and location";
//! * each word plays as two samples, high byte then low, each lasting the
//!   period; each word boundary loads the output buffer from the data latch
//!   and asks for the next word; when the length runs out the back-up is
//!   reloaded, the pointer reset, and the interrupt set again;
//! * with the channel off, a processor write to `AUDxDAT` plays that word and
//!   sets the interrupt when the latch is ready for another — "manual mode";
//! * attach modes send a modulator channel's words, one per period, to the
//!   next channel's volume, period, or alternately both.
//!
//! [`Paula::audio_output`] reports each channel's current sample and volume.
//! No host audio stream is produced yet.
//!
//! # What Agnus must provide
//!
//! The DMA fetch is Agnus's: the pointers (`DSKPT`, `AUDxLC`) are Agnus
//! registers, and only Agnus reaches chip RAM. Agnus names Paula with a link
//! property, takes [`PaulaPort`] from [`ExportId::PAULA`], and in its DMA
//! slots, with `at` a tick of the same colour-clock domain:
//!
//! * **disk read** — while [`PaulaPort::disk_read_word`] returns a word, store
//!   it at `DSKPT` and add two;
//! * **disk write** — while [`PaulaPort::disk_write_wanted`] is true, fetch the
//!   word at `DSKPT`, add two, and hand it to [`PaulaPort::disk_write_word`];
//! * **audio** — for each channel whose `AUDxEN` it sees, ask
//!   [`PaulaPort::audio_request`]; on `restart` reload that channel's pointer
//!   from `AUDxLC`, on `fetch` read the word at the pointer, add two, and hand
//!   it to [`PaulaPort::audio_word`] (or write `AUDxDAT` through the bus with
//!   [`Origin::dma`], which lands in the same latch);
//! * **interrupts** — [`PaulaPort::request`] with `VERTB` at the start of
//!   vertical blank and `BLIT` when the blitter finishes;
//! * **`DMACON`** — nothing: the bus already delivers it to both chips, and
//!   Paula keeps its own copy of the enable bits. Paula drives no bits of
//!   `DMACONR`, which is Agnus's to answer.
//!
//! Each call advances Paula to `at` first, so Agnus need not care which of the
//! two was caught up last; a word Paula assembled before Agnus came for it
//! waits in a two-word queue.
//!
//! # Pots
//!
//! `POTGO` is stored and `POTGOR` answers from it: a pin set as an output
//! reads its data bit, and an input reads one, which is an unpressed button
//! with the pull-up Chapter 8 describes. The proportional counters (`POT0DAT`,
//! `POT1DAT`) read zero and `START` does nothing.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::any::Any;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, Budget, Consumed, LazyHandle};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{FanIn, Level, WireId, WireSink, WireSource};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

use super::custom::{CustomBus, CustomChip, Driver, Origin};
use super::regs::{ChipId, Reg};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.paula";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// The character port a board gets when it does not name one.
const DEFAULT_PORT: &str = "serial";

/// The three encoded interrupt-level outputs, least significant first.
pub const IPL_PINS: [&str; 3] = ["ipl0", "ipl1", "ipl2"];

/// The `INT2*` input: `PORTS`, level 2.
pub const INT2_PIN: &str = "int2";

/// The `INT6*` input: `EXTER`, level 6.
pub const INT6_PIN: &str = "int6";

/// A tick no event is scheduled for.
const NO_EVENT: u64 = u64::MAX;

// ---------------------------------------------------------------------------
// register bits
// ---------------------------------------------------------------------------

/// The bits of `INTENA` and `INTREQ` (Appendix A, `INTENA`).
pub mod int {
    /// `SET/CLR`: a write sets the selected bits when this is one and clears
    /// them when it is zero.
    pub const SETCLR: u16 = 1 << 15;
    /// `INTEN`, the master enable. `INTENA` only.
    pub const INTEN: u16 = 1 << 14;
    /// `EXTER`: `INT6*`, level 6.
    pub const EXTER: u16 = 1 << 13;
    /// `DSKSYN`: the input stream matched `DSKSYNC`, level 5.
    pub const DSKSYN: u16 = 1 << 12;
    /// `RBF`: serial receive buffer full, level 5.
    pub const RBF: u16 = 1 << 11;
    /// `AUD3`, level 4.
    pub const AUD3: u16 = 1 << 10;
    /// `AUD2`, level 4.
    pub const AUD2: u16 = 1 << 9;
    /// `AUD1`, level 4.
    pub const AUD1: u16 = 1 << 8;
    /// `AUD0`, level 4.
    pub const AUD0: u16 = 1 << 7;
    /// `BLIT`: the blitter finished, level 3.
    pub const BLIT: u16 = 1 << 6;
    /// `VERTB`: start of vertical blank, level 3.
    pub const VERTB: u16 = 1 << 5;
    /// `COPER`: the copper, level 3.
    pub const COPER: u16 = 1 << 4;
    /// `PORTS`: `INT2*`, level 2.
    pub const PORTS: u16 = 1 << 3;
    /// `SOFT`: software, level 1.
    pub const SOFT: u16 = 1 << 2;
    /// `DSKBLK`: disk DMA finished, level 1.
    pub const DSKBLK: u16 = 1 << 1;
    /// `TBE`: serial transmit buffer empty, level 1.
    pub const TBE: u16 = 1 << 0;
    /// Every request bit — the fourteen that `INTREQ` stores.
    pub const REQUESTS: u16 = 0x3fff;
}

/// The level each request bit raises, bit 0 first (Appendix A, `INTENA`'s
/// "LEVEL" column). Monotonic, which is what lets the highest set bit decide.
const LEVELS: [u8; 14] = [1, 1, 1, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 6];

/// `DMACON` bit 9, `DMAEN`.
const DMA_DMAEN: u16 = 1 << 9;
/// `DMACON` bit 4, `DSKEN`.
const DMA_DSKEN: u16 = 1 << 4;
/// The `DMACON` bits Paula keeps: `DMAEN` and everything below it.
const DMA_BITS: u16 = 0x03ff;

/// `ADKCON` bit 10, `WORDSYNC`.
const ADK_WORDSYNC: u16 = 1 << 10;
/// `ADKCON` bit 8, `FAST`: two microseconds per cell rather than four.
const ADK_FAST: u16 = 1 << 8;

/// `DSKLEN` bit 15, `DMAEN`.
const LEN_DMAEN: u16 = 1 << 15;
/// `DSKLEN` bit 14, `WRITE`.
const LEN_WRITE: u16 = 1 << 14;
/// `DSKLEN` bits 13–0, the word count.
const LEN_COUNT: u16 = 0x3fff;

/// `SERPER` bit 15, `LONG`: nine data bits on receive.
const SER_LONG: u16 = 1 << 15;

/// A `FAST` disk cell in colour clocks. See the module docs for why 7.
pub const FAST_CELL_TICKS: u64 = 7;

/// A slow disk cell.
pub const SLOW_CELL_TICKS: u64 = 14;

/// How many assembled read words wait for Agnus before the oldest is lost.
const READY_DEPTH: usize = 2;

/// How many cells the disk path fetches from a drive at a time.
const CHUNK: usize = 512;

/// The offsets of the registers Paula answers, from Appendix B. Checked against
/// [`regs`](super::regs) by a test, so the two cannot drift.
mod off {
    pub(super) const ADKCONR: u16 = 0x010;
    pub(super) const POTGOR: u16 = 0x016;
    pub(super) const SERDATR: u16 = 0x018;
    pub(super) const DSKBYTR: u16 = 0x01a;
    pub(super) const INTENAR: u16 = 0x01c;
    pub(super) const INTREQR: u16 = 0x01e;
    pub(super) const DSKLEN: u16 = 0x024;
    pub(super) const DSKDAT: u16 = 0x026;
    pub(super) const SERDAT: u16 = 0x030;
    pub(super) const SERPER: u16 = 0x032;
    pub(super) const POTGO: u16 = 0x034;
    pub(super) const DSKSYNC: u16 = 0x07e;
    pub(super) const DMACON: u16 = 0x096;
    pub(super) const INTENA: u16 = 0x09a;
    pub(super) const INTREQ: u16 = 0x09c;
    pub(super) const ADKCON: u16 = 0x09e;
    /// Channel 0's length register, the first audio register Paula owns.
    pub(super) const AUD0LEN: u16 = 0x0a4;
    /// Channel 3's data register, the last.
    pub(super) const AUD3DAT: u16 = 0x0da;
}

/// The audio interrupt bit of channel `ch`.
const fn aud_bit(ch: usize) -> u16 {
    int::AUD0 << ch
}

// ---------------------------------------------------------------------------
// the drive seam
// ---------------------------------------------------------------------------

/// A floppy drive, as the disk controller sees its data lines.
///
/// Implemented by `amiga.floppy` and attached through
/// [`PaulaPort::attach_drive`]. Every tick is in the colour-clock domain Paula
/// counts in. Several drives share the read line: the cells of every drive
/// that is presenting data are **or**ed, and a drive that is not presenting any
/// contributes nothing.
///
/// Called with Paula's own state lock held, so an implementation's lock must
/// rank above [`LockRank::DEVICE`] — and it must not drive a wire or call back
/// into Paula from inside any of these.
pub trait DiskDrive: Send + Sync + fmt::Debug {
    /// Whether the drive is putting data on the read line: selected, spinning,
    /// with a disk in it.
    fn reading(&self) -> bool;

    /// Or the cells under the head into `out`, one entry per cell, the first
    /// starting at tick `start` and each `cell` ticks after the last. An entry
    /// is `0` or `1`.
    fn read_cells(&self, start: u64, cell: u64, out: &mut [u8]);

    /// Record `cells` under the head from tick `start`, if the drive is
    /// selected, spinning, holding a disk and not write-protected.
    fn write_cells(&self, start: u64, cell: u64, cells: &[u8]);
}

/// What an audio channel is asking Agnus's DMA for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AudioRequest {
    /// `AUDxDSR`: reset the pointer to `AUDxLC` before fetching.
    pub restart: bool,
    /// `AUDxDR`: fetch one word.
    pub fetch: bool,
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// The UART.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Serial {
    /// `SERPER` as last written.
    serper: u16,
    /// `SERDAT` holding a word the shifter has not taken.
    buffer: u16,
    /// Whether it does.
    buffered: bool,
    /// Whether a word is being shifted out.
    shifting: bool,
    /// The word being shifted out.
    shift: u16,
    /// The tick its last bit ends.
    tx_end: u64,
    /// Whether a frame is arriving.
    receiving: bool,
    /// The byte arriving.
    rx_byte: u8,
    /// The tick its start bit began.
    rx_start: u64,
    /// The bit period it arrives at.
    rx_period: u64,
    /// The tick its stop bit ends.
    rx_end: u64,
    /// `SERDATR` bits 9–0 as last received.
    data: u16,
    /// `OVRUN`.
    overrun: bool,
}

/// The disk controller.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Disk {
    /// `DSKLEN` as last written.
    dsklen: u16,
    /// Whether the last `DSKLEN` write had `DMAEN` set.
    armed: bool,
    /// Whether a DMA transfer has been started and not finished.
    active: bool,
    /// Its direction: RAM to disk.
    write: bool,
    /// Words still to transfer.
    remaining: u16,
    /// `DSKSYNC`.
    sync: u16,
    /// The input shift register.
    shift: u16,
    /// Cells since the last byte boundary.
    bits: u8,
    /// Cells since the last word boundary, for a read DMA.
    word_bits: u8,
    /// `DSKBYTR`'s data byte.
    byte: u8,
    /// `DSKBYT`.
    byte_ready: bool,
    /// `WORDEQUAL` holds while the tick is below this.
    equal_until: u64,
    /// A `WORDSYNC` read waiting for its first match.
    waiting: bool,
    /// Read words waiting for Agnus.
    ready: VecDeque<u16>,
    /// The tick the next cell starts on.
    next_cell: u64,
    /// The word being written out.
    out: u16,
    /// Cells of it left.
    out_bits: u8,
    /// The next word Agnus has handed over for writing.
    pending: Option<u16>,
}

/// One audio channel.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Channel {
    /// `AUDxLEN`.
    len: u16,
    /// `AUDxPER`.
    per: u16,
    /// `AUDxVOL`.
    vol: u16,
    /// The `AUDxDAT` holding latch.
    latch: u16,
    /// Whether the latch holds a word the buffer has not taken.
    fresh: bool,
    /// `AUDxON` as last seen: `DMAEN` and `AUDxEN` both set.
    on: bool,
    /// Whether the channel is producing samples.
    playing: bool,
    /// The output buffer.
    word: u16,
    /// Whether the low byte is playing (the next boundary loads a word).
    low_half: bool,
    /// The tick of the next sample boundary.
    next_at: u64,
    /// Words left in the block before the back-up length is reloaded.
    left: u32,
    /// `AUDxDR`.
    dr: bool,
    /// `AUDxDSR`.
    dsr: bool,
    /// For a channel attached as both kinds of modulator: whether the next
    /// word is a period rather than a volume.
    alt_period: bool,
}

/// Everything guest-visible, behind one lock.
#[derive(Debug, Clone, Default)]
struct State {
    /// Colour clocks simulated.
    ticks: u64,
    intena: u16,
    intreq: u16,
    /// `INT2*` asserted.
    int2: bool,
    /// `INT6*` asserted.
    int6: bool,
    /// Paula's copy of `DMACON`'s enables.
    dmacon: u16,
    adkcon: u16,
    potgo: u16,
    serial: Serial,
    disk: Disk,
    aud: [Channel; 4],
    /// Whether any attached drive was presenting data when last asked.
    /// Derived, never saved: it only decides whether an event is scheduled.
    drive_reading: bool,
    /// Bytes the UART finished sending, for the host once the lock is gone.
    /// Host state, never saved.
    host_out: Vec<u8>,
}

/// A period register as a tick count: zero is a full turn of the counter.
fn period_ticks(per: u16) -> u64 {
    if per == 0 { 0x1_0000 } else { u64::from(per) }
}

/// A length register as a word count: zero is a full turn of the counter.
fn length_words(len: u16) -> u32 {
    if len == 0 { 0x1_0000 } else { u32::from(len) }
}

/// Apply a `SET/CLR` write to `reg`, touching only `mask`.
fn set_clr(reg: u16, value: u16, mask: u16) -> u16 {
    let bits = value & mask;
    if value & int::SETCLR != 0 {
        reg | bits
    } else {
        reg & !bits
    }
}

impl State {
    fn fresh(ticks: u64) -> State {
        State {
            ticks,
            ..State::default()
        }
    }

    // -- interrupts ---------------------------------------------------------

    /// The encoded level the processor sees.
    fn ipl(&self) -> u8 {
        if self.intena & int::INTEN == 0 {
            return 0;
        }
        let pending = self.intreq & self.intena & int::REQUESTS;
        if pending == 0 {
            0
        } else {
            LEVELS[15 - pending.leading_zeros() as usize]
        }
    }

    /// Hold the external bits up while their lines are asserted.
    fn apply_lines(&mut self) {
        if self.int2 {
            self.intreq |= int::PORTS;
        }
        if self.int6 {
            self.intreq |= int::EXTER;
        }
    }

    fn write_intreq(&mut self, value: u16) {
        let before = self.intreq;
        self.intreq = set_clr(self.intreq, value, int::REQUESTS);
        if before & int::RBF != 0 && self.intreq & int::RBF == 0 {
            // "Reset by resetting bit 11 of INTREQ" (Appendix A, SERDATR).
            self.serial.overrun = false;
        }
        self.apply_lines();
    }

    fn write_dmacon(&mut self, value: u16) {
        self.dmacon = set_clr(self.dmacon, value, DMA_BITS);
        let now = self.ticks;
        for ch in 0..4 {
            let on = self.dmacon & DMA_DMAEN != 0 && self.dmacon & (1 << ch) != 0;
            if on == self.aud[ch].on {
                continue;
            }
            self.aud[ch].on = on;
            if on {
                self.audio_start(ch, now);
            } else {
                let c = &mut self.aud[ch];
                c.playing = false;
                c.dr = false;
                c.dsr = false;
            }
        }
    }

    // -- serial -------------------------------------------------------------

    fn bit_ticks(&self) -> u64 {
        u64::from(self.serial.serper & !SER_LONG) + 1
    }

    /// Move `word` into the shifter at `now`.
    fn serial_load(&mut self, word: u16, now: u64) {
        // One start bit, then everything up to and including the highest one
        // bit, which is the last stop bit (Chapter 8, "Specifying the Register
        // Contents").
        let bits = 1 + u64::from(16 - word.leading_zeros());
        let s = &mut self.serial;
        s.shifting = true;
        s.shift = word;
        s.tx_end = now + bits * (u64::from(s.serper & !SER_LONG) + 1);
        self.intreq |= int::TBE;
    }

    fn write_serdat(&mut self, word: u16) {
        // "If this register is written with all zeros, no data transmission
        // is initiated."
        if word == 0 {
            return;
        }
        if self.serial.shifting {
            self.serial.buffer = word;
            self.serial.buffered = true;
        } else {
            self.serial.buffered = false;
            self.serial_load(word, self.ticks);
        }
    }

    /// Whether a host byte may start arriving now.
    fn can_receive(&self) -> bool {
        !self.serial.receiving && self.intreq & int::RBF == 0
    }

    fn serial_receive(&mut self, byte: u8) {
        let period = self.bit_ticks();
        let bits = if self.serial.serper & SER_LONG != 0 {
            11
        } else {
            10
        };
        let s = &mut self.serial;
        s.receiving = true;
        s.rx_byte = byte;
        s.rx_start = self.ticks;
        s.rx_period = period;
        s.rx_end = self.ticks + bits * period;
    }

    fn serdatr(&self) -> u16 {
        let s = &self.serial;
        let rxd = if s.receiving {
            match self.ticks.saturating_sub(s.rx_start) / s.rx_period.max(1) {
                0 => false,
                n @ 1..=8 => (s.rx_byte >> (n - 1)) & 1 != 0,
                _ => true,
            }
        } else {
            true
        };
        (u16::from(s.overrun) << 15)
            | (u16::from(self.intreq & int::RBF != 0) << 14)
            | (u16::from(!s.buffered) << 13)
            | (u16::from(!s.shifting) << 12)
            | (u16::from(rxd) << 11)
            | (s.data & 0x03ff)
    }

    // -- disk ---------------------------------------------------------------

    fn cell_ticks(&self) -> u64 {
        if self.adkcon & ADK_FAST != 0 {
            FAST_CELL_TICKS
        } else {
            SLOW_CELL_TICKS
        }
    }

    /// `DMAON`: the transfer is started and `DMACON` lets it run.
    fn disk_dma_on(&self) -> bool {
        self.disk.active && self.dmacon & DMA_DMAEN != 0 && self.dmacon & DMA_DSKEN != 0
    }

    fn disk_reading_dma(&self) -> bool {
        self.disk_dma_on() && !self.disk.write
    }

    fn disk_writing_dma(&self) -> bool {
        self.disk_dma_on() && self.disk.write
    }

    fn disk_finish(&mut self) {
        let d = &mut self.disk;
        d.active = false;
        d.ready.clear();
        d.pending = None;
        d.out_bits = 0;
        self.intreq |= int::DSKBLK;
    }

    fn write_dsklen(&mut self, value: u16) {
        let adkcon = self.adkcon;
        let d = &mut self.disk;
        d.dsklen = value;
        if value & LEN_DMAEN == 0 {
            // Any write without DMAEN turns the DMA off and disarms it; step
            // 2 of the manual's sequence is exactly this.
            d.armed = false;
            d.active = false;
            d.ready.clear();
            d.pending = None;
            d.out_bits = 0;
            return;
        }
        if !d.armed {
            // The first of the two: "the DMAEN bit in the DSKLEN register must
            // be turned on twice in order to actually enable the disk DMA".
            d.armed = true;
            return;
        }
        d.active = true;
        d.write = value & LEN_WRITE != 0;
        d.remaining = value & LEN_COUNT;
        d.waiting = !d.write && adkcon & ADK_WORDSYNC != 0;
        d.word_bits = 0;
        d.ready.clear();
        d.pending = None;
        d.out_bits = 0;
        if d.remaining == 0 {
            self.disk_finish();
        }
    }

    fn dskbytr(&self) -> u16 {
        let d = &self.disk;
        (u16::from(d.byte_ready) << 15)
            | (u16::from(self.disk_dma_on()) << 14)
            | (u16::from(d.dsklen & LEN_WRITE != 0) << 13)
            | (u16::from(self.ticks < d.equal_until) << 12)
            | u16::from(d.byte)
    }

    /// One cell off the read line, starting at `at`.
    fn disk_read_cell(&mut self, cell: u8, at: u64, cell_ticks: u64) {
        let collecting = self.disk_reading_dma();
        let wordsync = self.adkcon & ADK_WORDSYNC != 0;
        let d = &mut self.disk;
        d.shift = (d.shift << 1) | u16::from(cell & 1);
        d.bits += 1;
        if d.bits == 8 {
            d.byte = d.shift as u8;
            d.byte_ready = true;
            d.bits = 0;
        }
        let matched = d.shift == d.sync;
        let mut word = None;
        if collecting && !(d.waiting && !matched) {
            if matched && wordsync {
                // Realign on the match. A sync word that was already on a word
                // boundary is itself a word of the stream; one that was not
                // restarts the count, and the next word starts after it.
                if !d.waiting && d.word_bits == 15 {
                    word = Some(d.shift);
                }
                d.waiting = false;
                d.word_bits = 0;
            } else if !d.waiting {
                d.word_bits += 1;
                if d.word_bits == 16 {
                    d.word_bits = 0;
                    word = Some(d.shift);
                }
            }
        }
        if let Some(w) = word
            && d.ready.len() < usize::from(d.remaining)
        {
            if d.ready.len() == READY_DEPTH {
                // Agnus did not come for the oldest in time.
                d.ready.pop_front();
            }
            d.ready.push_back(w);
        }
        if matched {
            d.bits = 0;
            d.equal_until = at + cell_ticks;
            self.intreq |= int::DSKSYN;
        }
    }

    /// The next cell of a write DMA, if there is one to write.
    fn disk_write_cell(&mut self) -> Option<u8> {
        let d = &mut self.disk;
        if d.out_bits == 0 {
            match d.pending.take() {
                Some(w) => {
                    d.out = w;
                    d.out_bits = 16;
                }
                None => {
                    if d.remaining == 0 {
                        self.disk_finish();
                    }
                    return None;
                }
            }
        }
        let cell = (d.out >> 15) as u8;
        d.out <<= 1;
        d.out_bits -= 1;
        if d.out_bits == 0 && d.pending.is_none() && d.remaining == 0 {
            self.disk_finish();
        }
        Some(cell)
    }

    /// Run the disk path over every cell that starts before `to`.
    fn disk_run(&mut self, to: u64, drives: &[Arc<dyn DiskDrive>]) {
        let cell = self.cell_ticks();
        if self.disk.next_cell < self.ticks {
            self.disk.next_cell = self.ticks.div_ceil(cell) * cell;
        }
        while self.disk.next_cell < to {
            let at = self.disk.next_cell;
            if self.disk_writing_dma() {
                if let Some(bit) = self.disk_write_cell() {
                    for d in drives {
                        d.write_cells(at, cell, &[bit]);
                    }
                }
                self.disk.next_cell += cell;
                continue;
            }
            let reading = drives.iter().any(|d| d.reading());
            self.drive_reading = reading;
            if !reading && self.disk.shift == 0 && !self.disk_reading_dma() {
                // Nothing on the line and no transfer waiting on it: the
                // register stays zero, so every cell does the same thing and
                // the stretch can be counted rather than stepped.
                let n = (to - at).div_ceil(cell);
                self.disk.next_cell += n * cell;
                if self.disk.sync == 0 {
                    // A zero register equals a zero DSKSYNC on every cell,
                    // and each match restarts the byte clock before it can
                    // reach eight.
                    self.disk.bits = 0;
                    self.disk.equal_until = self.disk.next_cell;
                    self.intreq |= int::DSKSYN;
                } else {
                    let total = u64::from(self.disk.bits) + n;
                    if total >= 8 {
                        self.disk.byte = 0;
                        self.disk.byte_ready = true;
                    }
                    self.disk.bits = (total % 8) as u8;
                }
                continue;
            }
            let n = (to - at).div_ceil(cell).min(CHUNK as u64) as usize;
            let mut buf = [0u8; CHUNK];
            if reading {
                for d in drives {
                    if d.reading() {
                        d.read_cells(at, cell, &mut buf[..n]);
                    }
                }
            }
            for (i, bit) in buf[..n].iter().enumerate() {
                let start = at + i as u64 * cell;
                self.disk_read_cell(*bit, start, cell);
            }
            self.disk.next_cell = at + n as u64 * cell;
        }
    }

    // -- audio --------------------------------------------------------------

    fn attached(&self, ch: usize) -> (bool, bool) {
        (
            self.adkcon & (1 << ch) != 0,
            self.adkcon & (1 << (4 + ch)) != 0,
        )
    }

    fn audio_start(&mut self, ch: usize, now: u64) {
        let c = &mut self.aud[ch];
        c.playing = true;
        c.left = length_words(c.len);
        c.dsr = true;
        c.dr = true;
        c.low_half = true;
        c.next_at = now + period_ticks(c.per);
        self.intreq |= aud_bit(ch);
    }

    /// A processor write to `AUDxDAT`.
    fn audio_manual(&mut self, ch: usize, word: u16, now: u64) {
        let c = &mut self.aud[ch];
        c.latch = word;
        c.fresh = true;
        if !c.on && !c.playing {
            c.playing = true;
            c.low_half = true;
            c.next_at = now;
            self.audio_boundary(ch);
        }
    }

    /// The sample boundary channel `ch` has reached at its `next_at`.
    fn audio_boundary(&mut self, ch: usize) {
        let (av, ap) = self.attached(ch);
        let modulator = av || ap;
        let per = period_ticks(self.aud[ch].per);
        {
            let c = &mut self.aud[ch];
            if !c.playing {
                return;
            }
            if !modulator && !c.low_half {
                c.low_half = true;
                c.next_at += per;
                return;
            }
        }
        // A word boundary.
        let mut raise = false;
        let word;
        {
            let c = &mut self.aud[ch];
            if c.on {
                if c.fresh {
                    c.word = c.latch;
                    c.fresh = false;
                }
                c.left -= 1;
                if c.left == 0 {
                    c.left = length_words(c.len);
                    c.dsr = true;
                    raise = true;
                }
                c.dr = true;
            } else if c.fresh {
                c.word = c.latch;
                c.fresh = false;
                raise = true;
            } else {
                c.playing = false;
                return;
            }
            c.low_half = false;
            c.next_at += per;
            word = c.word;
        }
        if raise {
            self.intreq |= aud_bit(ch);
        }
        if modulator && ch < 3 {
            // Table 5-4: volume and period alternate, volume first, when a
            // channel modulates both.
            let to_period = if av && ap {
                let p = self.aud[ch].alt_period;
                self.aud[ch].alt_period = !p;
                p
            } else {
                ap
            };
            let target = &mut self.aud[ch + 1];
            if to_period {
                target.per = word;
            } else {
                target.vol = word;
            }
        }
    }

    /// The tick the channel next sets its interrupt, if it will.
    fn audio_event(&self, ch: usize) -> Option<u64> {
        let c = &self.aud[ch];
        if !c.playing {
            return None;
        }
        let per = period_ticks(c.per);
        let (av, ap) = self.attached(ch);
        let modulator = av || ap;
        let next_word = if modulator || c.low_half {
            c.next_at
        } else {
            c.next_at + per
        };
        if !c.on {
            return Some(next_word);
        }
        let word_ticks = if modulator { per } else { 2 * per };
        Some(next_word + u64::from(c.left.saturating_sub(1)) * word_ticks)
    }

    // -- time ---------------------------------------------------------------

    /// The earliest boundary after `ticks` the run loop must stop at.
    fn next_boundary(&self, target: u64) -> u64 {
        let mut next = target;
        if self.serial.shifting {
            next = next.min(self.serial.tx_end);
        }
        if self.serial.receiving {
            next = next.min(self.serial.rx_end);
        }
        for c in &self.aud {
            if c.playing {
                next = next.min(c.next_at);
            }
        }
        next.max(self.ticks + 1).min(target)
    }

    /// Everything that happens exactly at `now`.
    fn fire(&mut self, now: u64) {
        if self.serial.shifting && self.serial.tx_end <= now {
            self.serial.shifting = false;
            self.host_out.push(self.serial.shift as u8);
            if self.serial.buffered {
                self.serial.buffered = false;
                let word = self.serial.buffer;
                self.serial_load(word, now);
            }
        }
        if self.serial.receiving && self.serial.rx_end <= now {
            let s = &mut self.serial;
            s.receiving = false;
            // Bit 8 is the stop bit, or on a LONG receive the ninth data bit,
            // which from an eight-bit host is the idle line; bit 9 the stop.
            s.data = u16::from(s.rx_byte) | 0x0300;
            if self.intreq & int::RBF != 0 {
                s.overrun = true;
            }
            self.intreq |= int::RBF;
        }
        for ch in 0..4 {
            while self.aud[ch].playing && self.aud[ch].next_at <= now {
                self.audio_boundary(ch);
            }
        }
    }

    fn run_to(&mut self, target: u64, drives: &[Arc<dyn DiskDrive>]) {
        while self.ticks < target {
            let step = self.next_boundary(target);
            self.disk_run(step, drives);
            self.ticks = step;
            self.fire(step);
        }
    }

    /// The next tick the scheduler must bring this chip to.
    fn next_event(&self) -> u64 {
        let mut next = NO_EVENT;
        if self.serial.shifting {
            next = next.min(self.serial.tx_end);
        }
        if self.serial.receiving {
            next = next.min(self.serial.rx_end);
        }
        for ch in 0..4 {
            if let Some(at) = self.audio_event(ch) {
                next = next.min(at);
            }
        }
        let word = 16 * self.cell_ticks();
        let dsksyn_armed = self.intena & int::INTEN != 0
            && self.intena & int::DSKSYN != 0
            && self.intreq & int::DSKSYN == 0
            && self.drive_reading;
        if self.disk_dma_on() || dsksyn_armed {
            // A word boundary: the finest grain a DMA word or a sync match is
            // promised at. A match inside the word raises its interrupt at
            // the boundary, at most 32 µs late.
            next = next.min(self.disk.next_cell.max(self.ticks) + word);
        }
        if next == NO_EVENT {
            NO_EVENT
        } else {
            next.max(self.ticks + 1)
        }
    }

    // -- registers ----------------------------------------------------------

    fn potgor(&self) -> u16 {
        let mut value = 0;
        for pin in 0..4 {
            let out = 1 << (9 + 2 * pin);
            let dat = 1 << (8 + 2 * pin);
            let level = if self.potgo & out != 0 {
                self.potgo & dat != 0
            } else {
                true
            };
            if level {
                value |= dat;
            }
        }
        value
    }
}

// ---------------------------------------------------------------------------
// shared
// ---------------------------------------------------------------------------

/// The interrupt outputs.
#[derive(Debug, Clone, Default)]
struct Outputs {
    ipl: [Option<WireSource>; 3],
}

/// What the register block, the pins, the export and the device all hold.
struct Shared {
    state: Mutex<State>,
    /// [`State::ticks`], for the scheduler to read without a lock.
    ticks: AtomicU64,
    /// [`State::next_event`], likewise.
    next_event: AtomicU64,
    out: Mutex<Outputs>,
    lazy: Mutex<Option<LazyHandle>>,
    drives: Mutex<Vec<Arc<dyn DiskDrive>>>,
    port: Arc<dyn CharDevice>,
    port_name: String,
    /// Host bytes the port refused, oldest first.
    backlog: Mutex<VecDeque<u8>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("port", &self.port_name);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event.store(state.next_event(), Ordering::Relaxed);
    }

    fn drives(&self) -> Vec<Arc<dyn DiskDrive>> {
        self.drives.lock().clone()
    }

    /// Drive the interrupt level, holding no lock while the wires move.
    fn refresh(&self) {
        let level = self.state.lock().ipl();
        let out = self.out.lock().clone();
        for (bit, src) in out.ipl.iter().enumerate() {
            if let Some(src) = src {
                src.set(Level::from_bool(level >> bit & 1 != 0));
            }
        }
    }

    /// Hand finished bytes to the host, keeping what it refuses.
    fn flush_host(&self, bytes: Vec<u8>) {
        let mut backlog = self.backlog.lock();
        backlog.extend(bytes);
        while let Some(&byte) = backlog.front() {
            if !self.port.write_byte(byte) {
                break;
            }
            backlog.pop_front();
        }
    }

    /// Bring the chip up to date before an access.
    fn sync(&self, debug: bool) {
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            let kind = if debug {
                AccessKind::Debug
            } else {
                AccessKind::Guest
            };
            // A refusal means catch-up is already running further up the
            // stack; answering from where the chip stands is all there is.
            let _ = handle.sync(kind);
        }
    }

    fn advance_to(&self, target: u64) {
        let drives = self.drives();
        let (moved, bytes) = {
            let mut state = self.state.lock();
            if target <= state.ticks {
                return;
            }
            let before = state.ipl();
            state.run_to(target, &drives);
            self.publish(&state);
            (before != state.ipl(), core::mem::take(&mut state.host_out))
        };
        if !bytes.is_empty() {
            self.flush_host(bytes);
        }
        if moved {
            self.refresh();
        }
    }

    /// Mutate the state at the current tick, then republish and re-drive.
    fn with_state<R>(&self, f: impl FnOnce(&mut State) -> R) -> R {
        let (r, moved) = {
            let mut state = self.state.lock();
            let before = state.ipl();
            let r = f(&mut state);
            self.publish(&state);
            (r, before != state.ipl())
        };
        if moved {
            self.refresh();
        }
        r
    }

    /// Retry what the host refused, and take a host byte into the receiver if
    /// it can start one.
    fn pump(&self) {
        self.flush_host(Vec::new());
        let mut state = self.state.lock();
        if state.can_receive()
            && let Some(byte) = self.port.read_byte()
        {
            state.serial_receive(byte);
            self.publish(&state);
        }
    }
}

// ---------------------------------------------------------------------------
// the register block
// ---------------------------------------------------------------------------

/// Paula's registers, as the custom-chip bus reaches them.
struct PaulaRegs {
    shared: Arc<Shared>,
}

impl fmt::Debug for PaulaRegs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PaulaRegs").finish_non_exhaustive()
    }
}

/// The channel an audio register at `offset` belongs to, `$0A0` being
/// channel 0's first.
fn audio_channel(offset: u16) -> usize {
    usize::from((offset - 0x0a0) / 0x10)
}

impl CustomChip for PaulaRegs {
    fn which(&self) -> ChipId {
        ChipId::PAULA
    }

    fn read(&self, reg: &Reg, from: Origin) -> u16 {
        self.shared.sync(from.debug);
        match reg.offset {
            off::INTENAR => self.shared.state.lock().intena & !int::SETCLR,
            off::INTREQR => self.shared.state.lock().intreq & int::REQUESTS,
            off::ADKCONR => self.shared.state.lock().adkcon & !int::SETCLR,
            off::SERDATR => self.shared.state.lock().serdatr(),
            off::DSKBYTR => {
                let mut state = self.shared.state.lock();
                let value = state.dskbytr();
                if !from.debug {
                    // "DSKBYT is cleared when the DSKBYTR register is read."
                    state.disk.byte_ready = false;
                }
                value
            }
            off::POTGOR => self.shared.state.lock().potgor(),
            // The proportional counters (`POT0DAT`, `POT1DAT`) are not
            // modelled, and `DMACONR` is Agnus's to answer: Paula drives none
            // of its bits.
            _ => 0,
        }
    }

    fn write(&self, reg: &Reg, value: u16, from: Origin) {
        self.shared.sync(from.debug);
        let offset = reg.offset;
        self.shared.with_state(|st| match offset {
            off::INTENA => st.intena = set_clr(st.intena, value, !int::SETCLR),
            off::INTREQ => st.write_intreq(value),
            off::ADKCON => st.adkcon = set_clr(st.adkcon, value, !int::SETCLR),
            off::DMACON => st.write_dmacon(value),
            off::DSKLEN => st.write_dsklen(value),
            off::DSKSYNC => st.disk.sync = value,
            off::DSKDAT => {
                if from.driver == Driver::Dma && st.disk_writing_dma() {
                    give_disk_word(st, value);
                }
            }
            off::SERDAT => st.write_serdat(value),
            off::SERPER => st.serial.serper = value,
            off::POTGO => st.potgo = value,
            off::AUD0LEN..=off::AUD3DAT => {
                let ch = audio_channel(offset);
                match (offset - 0x0a0) % 0x10 {
                    0x4 => st.aud[ch].len = value,
                    0x6 => st.aud[ch].per = value,
                    0x8 => st.aud[ch].vol = value,
                    0xa => {
                        if from.driver == Driver::Dma {
                            st.aud[ch].latch = value;
                            st.aud[ch].fresh = true;
                        } else {
                            let now = st.ticks;
                            st.audio_manual(ch, value, now);
                        }
                    }
                    _ => {}
                }
            }
            // `STRHOR`, the horizontal strobe, which the pot counters would
            // count; nothing here counts.
            _ => {}
        });
    }
}

/// Hand a write DMA its next word, counting it as transferred.
fn give_disk_word(st: &mut State, word: u16) {
    let d = &mut st.disk;
    if d.pending.is_none() && d.remaining > 0 {
        d.pending = Some(word);
        d.remaining -= 1;
    }
}

// ---------------------------------------------------------------------------
// the export
// ---------------------------------------------------------------------------

/// Paula's private seams: Agnus's DMA, a drive's data lines, and the interrupt
/// sources that live on other chips.
///
/// Published as [`ExportId::PAULA`]. See the module documentation for what
/// Agnus does with it. Every `at` is a tick of the colour-clock domain Paula
/// counts in; each call advances Paula to it first, and a tick Paula has
/// already passed is answered from where it stands.
#[derive(Debug, Clone)]
pub struct PaulaPort {
    shared: Arc<Shared>,
}

impl PaulaPort {
    /// The tick Paula has simulated up to.
    #[must_use]
    pub fn tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Advance Paula to `at`.
    pub fn advance_to(&self, at: u64) {
        self.shared.advance_to(at);
    }

    /// Set `bits` in `INTREQ`, as `VERTB` and `BLIT` arrive from Agnus.
    pub fn request(&self, at: u64, bits: u16) {
        self.advance_to(at);
        self.shared
            .with_state(|st| st.write_intreq(int::SETCLR | (bits & int::REQUESTS)));
    }

    /// Disk read DMA: the next word Paula has assembled, which Agnus stores at
    /// `DSKPT`. Counts it as transferred, and sets `DSKBLK` on the last.
    pub fn disk_read_word(&self, at: u64) -> Option<u16> {
        self.advance_to(at);
        self.shared.with_state(|st| {
            if !st.disk_reading_dma() {
                return None;
            }
            let word = st.disk.ready.pop_front()?;
            st.disk.remaining -= 1;
            if st.disk.remaining == 0 {
                st.disk_finish();
            }
            Some(word)
        })
    }

    /// Disk write DMA: whether Paula wants the next word from `DSKPT`.
    #[must_use]
    pub fn disk_write_wanted(&self, at: u64) -> bool {
        self.advance_to(at);
        let st = self.shared.state.lock();
        st.disk_writing_dma() && st.disk.pending.is_none() && st.disk.remaining > 0
    }

    /// Disk write DMA: the word Agnus fetched from `DSKPT`.
    pub fn disk_write_word(&self, at: u64, word: u16) {
        self.advance_to(at);
        self.shared.with_state(|st| {
            if st.disk_writing_dma() {
                give_disk_word(st, word);
            }
        });
    }

    /// Audio DMA: what channel `ch` wants, clearing the request.
    pub fn audio_request(&self, ch: usize, at: u64) -> AudioRequest {
        self.advance_to(at);
        self.shared.with_state(|st| {
            let Some(c) = st.aud.get_mut(ch) else {
                return AudioRequest::default();
            };
            let req = AudioRequest {
                restart: c.dsr,
                fetch: c.dr,
            };
            c.dsr = false;
            c.dr = false;
            req
        })
    }

    /// Audio DMA: the word Agnus fetched for channel `ch`, into its latch.
    pub fn audio_word(&self, ch: usize, at: u64, word: u16) {
        self.advance_to(at);
        self.shared.with_state(|st| {
            if let Some(c) = st.aud.get_mut(ch) {
                c.latch = word;
                c.fresh = true;
            }
        });
    }

    /// Add a drive to the read line.
    pub fn attach_drive(&self, drive: Arc<dyn DiskDrive>) {
        self.shared.drives.lock().push(drive);
        self.drive_changed();
    }

    /// Tell Paula a drive started or stopped presenting data, so the
    /// interrupt it may now owe is on the calendar.
    pub fn drive_changed(&self) {
        let drives = self.shared.drives();
        let mut state = self.shared.state.lock();
        state.drive_reading = drives.iter().any(|d| d.reading());
        self.shared.publish(&state);
    }
}

// ---------------------------------------------------------------------------
// the pins
// ---------------------------------------------------------------------------

/// `int2` or `int6`.
#[derive(Debug)]
struct IntPin {
    shared: Arc<Shared>,
    six: bool,
    inputs: FanIn,
}

impl WireSink for IntPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let asserted = self.inputs.any_high();
        let six = self.six;
        self.shared.with_state(|st| {
            if six {
                st.int6 = asserted;
            } else {
                st.int2 = asserted;
            }
            st.apply_lines();
        });
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// Paula.
#[derive(Debug)]
pub struct Paula {
    shared: Arc<Shared>,
    regs: Arc<PaulaRegs>,
    custom_path: String,
    pins: Mutex<Vec<Arc<IntPin>>>,
}

impl Paula {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `custom` is missing, a property is of the wrong
    /// kind, or one this class does not know was given.
    pub fn new(props: &Props) -> Result<Paula> {
        let mut r = props.reader();
        let custom_path = r.require_link("custom")?.as_str().to_string();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        r.finish()?;
        let port = ports::attach(props, &port_name)?;
        Ok(Paula::with_port(
            custom_path,
            port as Arc<dyn CharDevice>,
            port_name,
        ))
    }

    /// Build one against a character device the caller already has.
    #[must_use]
    pub fn with_port(custom_path: String, port: Arc<dyn CharDevice>, port_name: String) -> Paula {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::fresh(0)),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            // WIRE: cloned out after the state lock is released, and held while
            // nothing else is.
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            // LEAF: cloned out before the state lock is taken.
            drives: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            port,
            port_name,
            backlog: Mutex::with_rank(LockRank::DEVICE, VecDeque::new()),
        });
        let regs = Arc::new(PaulaRegs {
            shared: Arc::clone(&shared),
        });
        Paula {
            shared,
            regs,
            custom_path,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// The seam Agnus and the drives use.
    #[must_use]
    pub fn port(&self) -> PaulaPort {
        PaulaPort {
            shared: Arc::clone(&self.shared),
        }
    }

    /// The register block, for a test or an embedder attaching it by hand.
    #[must_use]
    pub fn chip(&self) -> Arc<dyn CustomChip> {
        Arc::clone(&self.regs) as Arc<dyn CustomChip>
    }

    /// The interrupt level the processor is being shown.
    #[must_use]
    pub fn ipl(&self) -> u8 {
        self.shared.state.lock().ipl()
    }

    /// `INTENA`, without disturbing anything.
    #[must_use]
    pub fn intena(&self) -> u16 {
        self.shared.state.lock().intena
    }

    /// `INTREQ`, without disturbing anything.
    #[must_use]
    pub fn intreq(&self) -> u16 {
        self.shared.state.lock().intreq
    }

    /// Channel `ch`'s current sample and effective volume, 0–64.
    ///
    /// Silent while the channel is stopped or is a modulator.
    #[must_use]
    pub fn audio_output(&self, ch: usize) -> (i8, u8) {
        let st = self.shared.state.lock();
        let Some(c) = st.aud.get(ch) else {
            return (0, 0);
        };
        let (av, ap) = st.attached(ch);
        if !c.playing || av || ap {
            return (0, 0);
        }
        let byte = if c.low_half {
            c.word as u8
        } else {
            (c.word >> 8) as u8
        };
        let vol = if c.vol & 0x40 != 0 {
            64
        } else {
            (c.vol & 0x3f) as u8
        };
        (byte as i8, vol)
    }

    /// The name of the character port the UART is joined to.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.shared.port_name
    }

    /// Move bytes between the UART and the host.
    ///
    /// What [`Device::run`] does; a test with no scheduler calls it directly.
    pub fn pump(&self) {
        self.shared.pump();
    }

    /// Run the chip until `target` colour clocks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Colour clocks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }
}

impl Device for Paula {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `bind` attaches to the register bus, and the wire
        // graph brings the pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            // The input lines are what other devices drive, and survive.
            let (int2, int6, reading) = (state.int2, state.int6, state.drive_reading);
            *state = State::fresh(state.ticks);
            state.int2 = int2;
            state.int6 = int6;
            state.drive_reading = reading;
            state.apply_lines();
            self.shared.publish(&state);
        }
        self.shared.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.shared.state.lock().clone();
        w.write_u64(st.ticks)?;
        for v in [st.intena, st.intreq, st.dmacon, st.adkcon, st.potgo] {
            w.write_u16(v)?;
        }
        w.write_bool(st.int2)?;
        w.write_bool(st.int6)?;
        let s = &st.serial;
        for v in [s.serper, s.buffer, s.shift, s.data] {
            w.write_u16(v)?;
        }
        for v in [s.buffered, s.shifting, s.receiving, s.overrun] {
            w.write_bool(v)?;
        }
        w.write_u8(s.rx_byte)?;
        for v in [s.tx_end, s.rx_start, s.rx_period, s.rx_end] {
            w.write_u64(v)?;
        }
        let d = &st.disk;
        for v in [d.dsklen, d.remaining, d.sync, d.shift, d.out] {
            w.write_u16(v)?;
        }
        for v in [d.armed, d.active, d.write, d.byte_ready, d.waiting] {
            w.write_bool(v)?;
        }
        for v in [d.bits, d.word_bits, d.byte, d.out_bits] {
            w.write_u8(v)?;
        }
        w.write_u64(d.equal_until)?;
        w.write_u64(d.next_cell)?;
        w.write_seq_len(d.ready.len() as u64)?;
        for v in &d.ready {
            w.write_u16(*v)?;
        }
        w.write_bool(d.pending.is_some())?;
        w.write_u16(d.pending.unwrap_or(0))?;
        for c in &st.aud {
            for v in [c.len, c.per, c.vol, c.latch, c.word] {
                w.write_u16(v)?;
            }
            for v in [
                c.fresh,
                c.on,
                c.playing,
                c.low_half,
                c.dr,
                c.dsr,
                c.alt_period,
            ] {
                w.write_bool(v)?;
            }
            w.write_u64(c.next_at)?;
            w.write_u32(c.left)?;
        }
        Ok(())
        // `host_out` and the backlog are the host's, and `drive_reading` is
        // derived; none is machine state (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut st = State::fresh(r.read_u64()?);
        st.intena = r.read_u16()? & !int::SETCLR;
        st.intreq = r.read_u16()? & int::REQUESTS;
        st.dmacon = r.read_u16()? & DMA_BITS;
        st.adkcon = r.read_u16()? & !int::SETCLR;
        st.potgo = r.read_u16()?;
        st.int2 = r.read_bool()?;
        st.int6 = r.read_bool()?;
        {
            let s = &mut st.serial;
            s.serper = r.read_u16()?;
            s.buffer = r.read_u16()?;
            s.shift = r.read_u16()?;
            s.data = r.read_u16()?;
            s.buffered = r.read_bool()?;
            s.shifting = r.read_bool()?;
            s.receiving = r.read_bool()?;
            s.overrun = r.read_bool()?;
            s.rx_byte = r.read_u8()?;
            s.tx_end = r.read_u64()?;
            s.rx_start = r.read_u64()?;
            s.rx_period = r.read_u64()?;
            s.rx_end = r.read_u64()?;
        }
        {
            let d = &mut st.disk;
            d.dsklen = r.read_u16()?;
            d.remaining = r.read_u16()? & LEN_COUNT;
            d.sync = r.read_u16()?;
            d.shift = r.read_u16()?;
            d.out = r.read_u16()?;
            d.armed = r.read_bool()?;
            d.active = r.read_bool()?;
            d.write = r.read_bool()?;
            d.byte_ready = r.read_bool()?;
            d.waiting = r.read_bool()?;
            d.bits = r.read_u8()? % 8;
            d.word_bits = r.read_u8()? % 16;
            d.byte = r.read_u8()?;
            d.out_bits = r.read_u8()?.min(16);
            d.equal_until = r.read_u64()?;
            d.next_cell = r.read_u64()?;
            let count = r.read_seq_len(2)?;
            if count > READY_DEPTH as u64 {
                return Err(Error::State(format!(
                    "snapshot has {count} disk word(s) waiting in a {READY_DEPTH}-word queue"
                )));
            }
            for _ in 0..count {
                d.ready.push_back(r.read_u16()?);
            }
            let has = r.read_bool()?;
            let word = r.read_u16()?;
            d.pending = has.then_some(word);
        }
        for c in &mut st.aud {
            c.len = r.read_u16()?;
            c.per = r.read_u16()?;
            c.vol = r.read_u16()?;
            c.latch = r.read_u16()?;
            c.word = r.read_u16()?;
            c.fresh = r.read_bool()?;
            c.on = r.read_bool()?;
            c.playing = r.read_bool()?;
            c.low_half = r.read_bool()?;
            c.dr = r.read_bool()?;
            c.dsr = r.read_bool()?;
            c.alt_period = r.read_bool()?;
            c.next_at = r.read_u64()?;
            c.left = r.read_u32()?.min(0x1_0000);
        }
        {
            let mut state = self.shared.state.lock();
            st.drive_reading = state.drive_reading;
            *state = st;
            self.shared.publish(&state);
        }
        self.shared.refresh();
        Ok(())
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::PAULA)
            .then(|| Export::Opaque(Arc::new(self.port()) as Arc<dyn Any + Send + Sync>))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(bit) = IPL_PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from(
                    "Paula drives three pins, `ipl0`, `ipl1` and `ipl2`: the encoded \
                     interrupt level",
                ),
            });
        };
        self.shared.out.lock().ipl[bit] = Some(source);
        self.shared.refresh();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let six = match port {
            INT2_PIN => false,
            INT6_PIN => true,
            _ => return None,
        };
        let pin = Arc::new(IntPin {
            shared: Arc::clone(&self.shared),
            six,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: u32::from(six),
        })
    }

    fn is_runnable(&self) -> bool {
        // Not because it executes anything: a byte from the host has to be
        // picked up, and a refused one retried, and the scheduler is the only
        // thing allowed to decide when.
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.shared.pump();
        Consumed::new(budget.ticks)
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Paula {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bus = ctx
            .export_as::<CustomBus>(&self.custom_path, ExportId::CUSTOM_BUS)
            .map_err(|e| Error::Config {
                at: ctx.path().to_string(),
                message: format!("`custom` has to name the `amiga.custom` register space: {e}"),
            })?;
        bus.attach(self.chip())
    }
}

/// The `amiga.paula` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Amiga Paula: interrupts onto the 68000's levels, the disk controller, the UART \
              and the four audio channels",
    properties: &[
        PropertySpec {
            name: "custom",
            kind: ValueKind::Link,
            required: true,
            summary: "the `amiga.custom` register space the chip answers in",
        },
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the host character port the UART is joined to (default `serial`)",
        },
    ],
    construct: |props| Ok(Box::new(Paula::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Paula::new(props)?)))
}

/// What the validator should know about `amiga.paula`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("custom", ValueKind::Link).required())
        .prop(PropSchema::new("port", ValueKind::Str))
        .port(INT2_PIN, PortDir::In)
        .port(INT6_PIN, PortDir::In);
    for pin in IPL_PINS {
        schema = schema.port(pin, PortDir::Out);
    }
    schema
}

#[cfg(test)]
mod tests;
