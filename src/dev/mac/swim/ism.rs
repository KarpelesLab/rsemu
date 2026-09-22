//! The **ISM register set**: the sixteen registers a SWIM answers with once
//! software has asked it to stop pretending to be an IWM.
//!
//! # The source, quoted rather than cited
//!
//! Apple's *SWIM Chip User's Reference*, **revision 1.5, 11 January 1988**
//! (Apple Computer, Inc., marked CONFIDENTIAL at the time and published since),
//! together with the *SWIM Chip Specification* of 29 September 1987. Every
//! register, every bit and every rule below carries the sentence or the table
//! row it came from, with its page — because the last register file written for
//! this cable was written from a chapter reference to a chapter that had no
//! such table, eight of its sixteen addresses were invented, five were wrong,
//! and the tests passed because they encoded the same invention
//! (`docs/platforms/mac-plus.md`, "The drive's register file was invented").
//! A number here that is not quoted is marked **inferred** and says what it was
//! inferred from.
//!
//! No Macintosh emulator source was consulted and no ROM was disassembled
//! (`ROADMAP.md` §1, `CLAUDE.md`).
//!
//! # Getting in
//!
//! Page 12, the IWM mode register's bit 6:
//!
//! > The *ISM/IWM* bit selects which register set will be used. To select the
//! > ISM set, you must write to the GCR mode register **four times in a row**
//! > with this bit set to "1", "0", "1","1", respectively. This somewhat
//! > torturous route is set up to prevent unintentional intrusions into the ISM
//! > world by existing software. After the switch, all further accesses to the
//! > SWIM will then be routed to the ISM register set until you clear bit 6 in
//! > the ISM mode register.
//!
//! `1, 0, 1, 1` — **not** `1, 0, 1, 0` — and that is what Apple's own Macintosh
//! Classic ROM writes: `$57`, `$17`, `$57`, `$57`, four consecutive loads of
//! the IWM mode register whose bit 6 goes exactly that way.
//! [`Switch`] is the four-entry shift register that watches for it.
//!
//! # The addresses
//!
//! Page 13:
//!
//! > Unlike the IWM, the four address lines directly select one of 16
//! > registers. The address bit A3 acts as the read/write line for the
//! > registers; registers are read when A3=1 and written when A3=0. Since
//! > there are no "forbidden" paths for moving from state to state, registers
//! > may be accessed in any order.
//!
//! and page 26's table, "IWM State/ISM Register Mapping", in full:
//!
//! ```text
//!   Address   IWM State      ISM Register
//!      0      PHASE0 = 0     Write Data
//!      1      PHASE0 = 1     Write Mark
//!      2      PHASE1 = 0     Write CRC/IWM Config
//!      3      PHASE1 = 1     Write Parameter RAM
//!      4      PHASE2 = 0     Write Phases
//!      5      PHASE2 = 1     Write Setup
//!      6      PHASE3 = 0     Write Mode (0's)
//!      7      PHASE3 = 1     Write Mode (1's)
//!      8      MOTORON = 0    Read Data
//!      9      MOTORON = 1    Read Mark
//!     10      DRIVESEL = 0   Read CRC            <- struck out by hand
//!     11      DRIVESEL = 1   Read Parameter RAM
//!     12      L6 = 0         Read Phases
//!     13      L6 = 1         Read Setup
//!     14      L7 = 0         Read Status
//!     15      L7 = 1         Read Handshake
//! ```
//!
//! **Address 10 is the ERROR register, not a CRC read.** The printed table says
//! "Read CRC" and the word `CRC` is *struck through by hand on the scan*, and
//! the register's own section on page 24 heads itself `ERROR Register  R
//! [1010]`. The per-register headings are the authority and they are what this
//! file implements; the table's row is a typo its own author corrected.
//!
//! The headings also settle which registers answer at both halves of the
//! address space, because they write `x` for an address bit they do not care
//! about:
//!
//! ```text
//!   DATA Register           R/W [x000]   (ACTION=1)
//!   CORRECTION Register     R   [1000]   (ACTION=0)
//!   MARK Register           R/W [x001]
//!   CRC Register            W   [0010]   (ACTION=1)
//!   IWM Configuration       W   [0010]   (ACTION=0)
//!   PARAMETER RAM           R/W [x011]
//!   PHASE Register          R/W [x100]                 Reset to 11110000
//!   SETUP Register          R/W [x101]                 Reset to 00000000
//!   MODE Register           W   [011x]                 Reset to 00000000
//!   STATUS Register         R   [1110]
//!   ERROR Register          R   [1010]                 Reset to 00000000
//!   HANDSHAKE Register      R   [1111]
//! ```
//!
//! # The drive is still the drive
//!
//! None of these sixteen registers is a second mechanism. The phase lines are
//! the same `CA0`, `CA1`, `CA2` and `LSTRB` that address the Sony drive's own
//! register file, the enables run the same spindle, and `SENSE` is the same
//! status line — so this module reaches all of them through
//! [`super::super::iwm::Iwm`], which owns the cable, and keeps no copy.

use super::super::iwm::{self, Iwm};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::error::Result;

// -- the sixteen addresses ---------------------------------------------------

/// `Write Data` / `Read Data`, and `Read Correction` when `ACTION` is clear.
pub const REG_DATA: u8 = 0;
/// `Write Mark` / `Read Mark`.
pub const REG_MARK: u8 = 1;
/// `Write CRC` when `ACTION` is set, `Write IWM Config` when it is clear.
pub const REG_CRC: u8 = 2;
/// `Write`/`Read Parameter RAM`.
pub const REG_PARAM: u8 = 3;
/// `Write`/`Read Phases`.
pub const REG_PHASE: u8 = 4;
/// `Write`/`Read Setup`.
pub const REG_SETUP: u8 = 5;
/// `Write Mode (0's)` — every bit set in the value written is *cleared*.
pub const REG_MODE_ZEROS: u8 = 6;
/// `Write Mode (1's)` — every bit set in the value written is *set*.
pub const REG_MODE_ONES: u8 = 7;
/// `Read Status`: the mode register read back.
pub const REG_STATUS: u8 = 14;
/// `Read Error`.
pub const REG_ERROR: u8 = 10;
/// `Read Handshake`.
pub const REG_HANDSHAKE: u8 = 15;

// -- the mode register, page 23 ----------------------------------------------

/// Bit 0: "Toggling the clear FIFO bit high then low clears the FIFO to begin
/// a read or write operation, and initializes the CRC generator with its
/// starting value."
pub const MODE_CLEAR_FIFO: u8 = 1 << 0;
/// Bit 1: "Setting this bit along with bit 7 (MotorOn) will enable drive 1."
pub const MODE_ENABLE1: u8 = 1 << 1;
/// Bit 2: "Setting this bit along with bit 7 (MotorOn) will enable drive 2."
pub const MODE_ENABLE2: u8 = 1 << 2;
/// Bit 3: "Setting the ACTION bit to '1' starts a read or write operation."
pub const MODE_ACTION: u8 = 1 << 3;
/// Bit 4: "This bit determines whether an operation will be a read (0) or
/// write (1) operation."
pub const MODE_WRITE: u8 = 1 << 4;
/// Bit 5: "Sets the state of the HDSEL pin if the Q3*/HDSEL bit in the Setup
/// register is set to '1'."
pub const MODE_HDSEL: u8 = 1 << 5;
/// Bit 6: "Clearing this bit switches to the IWM register set[.] As long as
/// this bit remains a '1' the ISM register set will stay selected."
pub const MODE_ISM: u8 = 1 << 6;
/// Bit 7: "Enables/disables the /ENBL1 and /ENBL2 drive enables (assuming bit
/// 1 or 2 is set)."
pub const MODE_MOTOR_ON: u8 = 1 << 7;

// -- the setup register, page 22 ---------------------------------------------

/// Bit 0: "'0' makes the Q3*/HDSEL pin an input to support the Q3 clock; '1'
/// makes the pin an output to use as a drive head select line."
pub const SETUP_HDSEL_PIN: u8 = 1 << 0;
/// Bit 1: "Sets the state of the 3.5SEL* pin (note the output state is the
/// inverse of the bit value)."
pub const SETUP_35SEL: u8 = 1 << 1;
/// Bit 2: "Setting the bit selects GCR mode; clearing it selects the normal
/// operating mode."
pub const SETUP_GCR: u8 = 1 << 2;
/// Bit 3: "Setting the bit causes the FCLK clock frequency to be divided by 2".
pub const SETUP_FCLK_DIV2: u8 = 1 << 3;
/// Bit 4: "Enables the Error Correction Machine".
pub const SETUP_ECM: u8 = 1 << 4;
/// Bit 5: "Sets up the RDDATA and WRDATA signals to be either pulses (1) or
/// transitions (0)".
pub const SETUP_PULSES: u8 = 1 << 5;
/// Bit 6: "Causes the Trans-Space logic to be bypassed. This bit must be set
/// for GCR operation."
pub const SETUP_BYPASS_TSM: u8 = 1 << 6;
/// Bit 7: "This bit is used to enable/disable the MotorOn timer".
pub const SETUP_MOTOR_TIMER: u8 = 1 << 7;

// -- the error register, page 24 ---------------------------------------------

/// Bit 0: "The processor is not reading/writing fast enough to keep up with
/// the chip."
pub const ERROR_UNDERRUN: u8 = 1 << 0;
/// Bit 1: "A mark byte (missing transition) was read from the Data register."
pub const ERROR_MARK_IN_DATA: u8 = 1 << 1;
/// Bit 2: "The processor is reading faster than bytes are available or writing
/// faster than the FIFO is requesting bytes."
pub const ERROR_OVERRUN: u8 = 1 << 2;
/// Bit 3: "The correction number obtained in the Error Correction Machine is
/// so large that the error cannot be corrected."
pub const ERROR_CORRECTION: u8 = 1 << 3;
/// Bit 4: "A transition occurred before the MIN cell time, making the cell too
/// narrow to be legal."
pub const ERROR_TOO_NARROW: u8 = 1 << 4;
/// Bit 5: "A transition didn't occur before MIN+xSx+xLx+RPT clocks, making the
/// cell too wide to be legal."
pub const ERROR_TOO_WIDE: u8 = 1 << 5;
/// Bit 6: "There were three marginal transitions in a row which implies that
/// the transitions cannot be resolved."
pub const ERROR_UNRESOLVED: u8 = 1 << 6;

// -- the handshake register, page 25 -----------------------------------------

/// Bit 0: "If set to '1' it indicates that the next byte to be read is a mark
/// byte (i.e., has a dropped clock pulse)."
pub const HS_MARK: u8 = 1 << 0;
/// Bit 1: "The CRC error bit is cleared to zero if the CRC generated on the
/// bytes up to and including the byte about to be read is zero (meaning all
/// the bytes are correct). It's set to '1' if the internal CRC is currently
/// non-zero."
pub const HS_CRC_ERROR: u8 = 1 << 1;
/// Bit 2: "This bit returns the current state of the RDDATA input from the
/// drive."
pub const HS_RDDATA: u8 = 1 << 2;
/// Bit 3: "This bit returns the current state of the SENSE input."
pub const HS_SENSE: u8 = 1 << 3;
/// Bit 4: "This bit is set to '1' if either the MotorOn bit in the mode
/// register is a '1' or the timer is timing out."
pub const HS_MOTOR_ON: u8 = 1 << 4;
/// Bit 5: "If this bit is set, it indicates that one of the bits in the Error
/// register is set."
pub const HS_ERROR: u8 = 1 << 5;
/// Bit 6: "In read mode, this bit indicates that the FIFO contains 2 bytes to
/// be read. In write mode, it indicates that 2 bytes can be written to the
/// FIFO."
pub const HS_TWO_BYTES: u8 = 1 << 6;
/// Bit 7: "In read mode, this bit indicates that the FIFO contains at least 1
/// byte to be read. In write mode, it indicates that at least 1 byte can be
/// written to the FIFO."
pub const HS_ONE_BYTE: u8 = 1 << 7;

// -- the phase register, page 21 ---------------------------------------------

/// The phase register's reset value: "Reset to 11110000" — all four lines
/// outputs, all four low.
pub const PHASE_RESET: u8 = 0b1111_0000;

/// How many bytes of parameter RAM. Page 21: "This location consists of 16
/// bytes of parameter data used to control the read/write timing."
pub const PARAM_BYTES: usize = 16;

/// The CRC generator's starting value.
///
/// **Inferred**, and this is what from: page 23 says only that the Clear FIFO
/// bit "initializes the CRC generator with its starting value" and that "this
/// value is different for reading or writing", without giving either. The
/// format does give it — the preset for the CRC-16/CCITT that an IBM System 34
/// double-density field carries is `$FFFF` (`super::super::mfm::crc16`) — and
/// it is *checkable* rather than assumed: a field followed by its own CRC
/// leaves this generator holding zero only for the right preset, which is
/// exactly what the handshake register's bit 1 is for, and
/// `tests.rs::the_crc_comes_out_zero_over_a_field_and_its_own_crc` asserts it
/// over a track this project's own encoder laid down.
pub const CRC_SEED: u16 = 0xffff;

/// Watches the IWM mode register for the four writes that ask for ISM mode.
///
/// "Four times in a row" is read here as four *consecutive loads of the mode
/// register* — reads of other addresses in between do not break the run,
/// because the sentence is about writes to one register and the run Apple's own
/// ROM performs has nothing at all between its four. That reading is the one
/// thing here that the document does not state outright; it is marked as an
/// **inference** and it is the weakest possible one, since the stricter reading
/// (nothing whatever between) also accepts the ROM's sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Switch {
    /// Bit 6 of the last four mode-register writes, most recent in bit 0.
    history: u8,
    /// The mode-write count this last looked at, so one write is counted once.
    seen: u32,
}

impl Switch {
    /// The pattern the document asks for.
    ///
    /// [`Switch::observe`] shifts each new write in at the bottom, so the
    /// oldest of the four ends up in bit 3 and reading the nibble from bit 3
    /// down to bit 0 gives the writes in the order they happened: page 12's
    /// `"1", "0", "1","1"` is the literal `0b1011`.
    const WANTED: u8 = 0b1011;

    /// Look at the chip after a forwarded access and say whether that access
    /// completed the sequence.
    pub fn observe(&mut self, iwm: &Iwm) -> bool {
        let writes = iwm.mode_writes();
        if writes == self.seen {
            return false;
        }
        self.seen = writes;
        let bit = u8::from(iwm.mode() & MODE_ISM != 0);
        self.history = ((self.history << 1) | bit) & 0xf;
        self.history == Switch::WANTED
    }

    /// Forget the run — after a reset, or after the chip has switched.
    pub fn forget(&mut self, iwm: &Iwm) {
        self.history = 0;
        self.seen = iwm.mode_writes();
    }

    /// Write the run into a snapshot.
    ///
    /// # Errors
    ///
    /// Whatever the writer returns.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u8(self.history)?;
        w.write_u32(self.seen)
    }

    /// And read it back.
    ///
    /// # Errors
    ///
    /// Whatever the reader returns.
    pub fn load(r: &mut ChunkReader<'_>) -> Result<Switch> {
        Ok(Switch {
            history: r.read_u8()? & 0xf,
            seen: r.read_u32()?,
        })
    }
}

/// The ISM's own sixteen registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ism {
    /// The mode register (page 23). `Read Status` is this read back.
    mode: u8,
    /// The setup register (page 22).
    setup: u8,
    /// The phase register (page 21): directions in bits 4-7, states in 0-3.
    phase: u8,
    /// The error register (page 24). Cleared by reading it.
    error: u8,
    /// The parameter RAM (page 21) and its auto-increment counter.
    param: [u8; PARAM_BYTES],
    /// "An auto-increment counter accesses consecutive RAM addresses every
    /// time that a read or write is made to this register."
    param_at: u8,
    /// Which of the two correction bytes the next read of the correction
    /// register hands over: "two consecutive reads from this location will
    /// provide error correction information".
    correction_at: bool,
    /// The last byte written to the data register. Writing to a disk is not
    /// modelled, so it is kept and goes nowhere, exactly as the IWM's is.
    written: u8,
}

impl Default for Ism {
    fn default() -> Ism {
        Ism {
            // "Reset to 00000000" for mode, setup and error; "Reset to
            // 11110000" for the phase register.
            mode: 0,
            setup: 0,
            phase: PHASE_RESET,
            error: 0,
            param: [0; PARAM_BYTES],
            param_at: 0,
            correction_at: false,
            written: 0,
        }
    }
}

impl Ism {
    /// The register file as it comes up out of a chip reset.
    #[must_use]
    pub fn fresh() -> Ism {
        Ism::default()
    }

    /// The register file as it is the instant the mode switch happens.
    ///
    /// Bit 6 is set because that is what the switch means — page 12: "all
    /// further accesses to the SWIM will then be routed to the ISM register set
    /// until you clear bit 6 in the ISM mode register" — and the rest of the
    /// mode register takes its documented reset value. The **phase lines are
    /// carried over** from where the IWM's `CA` lines stand, which page 21
    /// requires in as many words:
    ///
    /// > NOTE: when the SWIM switches between the IWM and ISM register sets,
    /// > the current levels of the phase lines are carried over so that no
    /// > glitches occur.
    ///
    /// The four *directions* are not carried over, because the IWM has none to
    /// carry: page 4 says the phase lines "are forced to be outputs" while the
    /// IWM register set is selected, so all four arrive as outputs — which is
    /// also the phase register's own reset value.
    #[must_use]
    pub fn entered(iwm: &Iwm) -> Ism {
        Ism {
            mode: MODE_ISM,
            phase: PHASE_RESET | (iwm.phases() & 0x0f),
            ..Ism::default()
        }
    }

    /// The mode register, which `Read Status` returns.
    #[must_use]
    pub fn mode(&self) -> u8 {
        self.mode
    }

    /// Whether the chip is still answering as an ISM.
    #[must_use]
    pub fn selected(&self) -> bool {
        self.mode & MODE_ISM != 0
    }

    /// The setup register.
    #[must_use]
    pub fn setup(&self) -> u8 {
        self.setup
    }

    /// The phase register.
    #[must_use]
    pub fn phase(&self) -> u8 {
        self.phase
    }

    /// The error register, without clearing it — for a debug read and a test.
    #[must_use]
    pub fn error(&self) -> u8 {
        self.error
    }

    /// The parameter RAM.
    #[must_use]
    pub fn param(&self) -> &[u8; PARAM_BYTES] {
        &self.param
    }

    /// Which drive the two enable bits name, if either does.
    fn drive(&self) -> Option<usize> {
        if self.mode & MODE_ENABLE1 != 0 {
            Some(0)
        } else if self.mode & MODE_ENABLE2 != 0 {
            Some(1)
        } else {
            None
        }
    }

    /// Whether a read operation is running: `ACTION` set and the read/write
    /// bit saying read.
    fn reading(&self) -> bool {
        self.mode & MODE_ACTION != 0 && self.mode & MODE_WRITE == 0
    }

    /// Push everything the mode and phase registers say onto the mechanism.
    ///
    /// Called after the lock on this state is **released**, never with it held:
    /// it calls into the drive, and `CLAUDE.md`'s re-entrancy contract wants
    /// the short critical section finished first.
    pub fn apply(&self, iwm: &Iwm) {
        // The head comes from the mode register's `HDSEL` bit **only** when
        // the Setup register has made that pin an output; otherwise the chip
        // is not driving it and the drive keeps whichever head its own
        // register file last selected.
        let head = (self.setup & SETUP_HDSEL_PIN != 0).then_some(self.mode & MODE_HDSEL != 0);
        iwm.set_enables(self.drive(), self.mode & MODE_MOTOR_ON != 0, head);
        // Only a line configured as an output drives the cable; page 21: "Bits
        // 4-7 control the direction of each phase line. Clearing a bit causes
        // the line to be an input, while setting a bit makes the line an
        // output." An input line drives nothing, and this cable's lines are
        // pulled up, so it stands high.
        let driven = (self.phase & 0x0f) | (!(self.phase >> 4) & 0x0f);
        iwm.set_phases(driven);
        iwm.set_mfm_framing(self.reading(), CRC_SEED);
    }

    /// Read register `reg`, which is the four address lines as they stand.
    ///
    /// `debug` is a debugger looking rather than the guest reading, and it must
    /// not pop the FIFO, clear the error register or advance the parameter
    /// RAM's counter (`CLAUDE.md`, *Devices*). An ISM register file is mostly
    /// side effects, so that is real work here and not a checkbox.
    pub fn read(&mut self, reg: u8, iwm: &Iwm, debug: bool) -> u8 {
        // Page 13: "registers are read when A3=1". The five registers whose
        // heading writes `[x...]` answer at both halves, so the low three bits
        // name the register and A3 only distinguishes the three that do not.
        match reg & 7 {
            REG_DATA if reg == 8 && self.mode & MODE_ACTION == 0 => self.correction(debug),
            REG_DATA => self.data(iwm, debug),
            REG_MARK => self.mark(iwm, debug),
            // Address 2 is `Write CRC/IWM Config`, address 10 is the error
            // register. The low half has no read function of its own.
            REG_CRC if reg == REG_ERROR => self.take_error(debug),
            REG_CRC => 0,
            REG_PARAM => self.read_param(debug),
            REG_PHASE => self.read_phase(iwm),
            REG_SETUP => self.setup,
            // Addresses 6 and 7 are the two write halves of the mode register;
            // 14 is `Read Status` and 15 is `Read Handshake`.
            REG_MODE_ZEROS if reg == REG_STATUS => self.mode,
            REG_MODE_ONES if reg == REG_HANDSHAKE => self.handshake(iwm, debug),
            // **Inferred**: the document gives addresses 6 and 7 no read
            // function at all, and a chip that decodes A3 as its read/write
            // line has nothing to put on the bus for them. Zero is what is
            // answered, and the access still *completes* — a compact Macintosh
            // has no bus-error timeout, so a chip that is fitted may not fault
            // (`docs/platforms/mac-classic.md`, "A word access to the VIA").
            _ => 0,
        }
    }

    /// The data register (page 20).
    ///
    /// > When ACTION is set, this register reads data from and writes data to
    /// > the FIFO. If a mark byte is read from this location, an error will
    /// > occur (see Error register, bit 1).
    fn data(&mut self, iwm: &Iwm, debug: bool) -> u8 {
        if debug {
            return iwm.peek_mfm().map_or(0, |(byte, _)| byte);
        }
        let Some((byte, mark)) = iwm.take_mfm() else {
            // Nothing framed. Page 24, bit 2: "The processor is reading faster
            // than bytes are available".
            self.raise(ERROR_OVERRUN);
            return 0;
        };
        if mark {
            self.raise(ERROR_MARK_IN_DATA);
        }
        byte
    }

    /// The mark register (page 20).
    ///
    /// > Reading from this register will allow a mark byte to be read without
    /// > causing an error.
    fn mark(&mut self, iwm: &Iwm, debug: bool) -> u8 {
        if debug {
            return iwm.peek_mfm().map_or(0, |(byte, _)| byte);
        }
        match iwm.take_mfm() {
            Some((byte, _)) => byte,
            None => {
                self.raise(ERROR_OVERRUN);
                0
            }
        }
    }

    /// The correction register (page 20), read when `ACTION` is clear.
    ///
    /// > When ACTION is not set, two consecutive reads from this location will
    /// > provide error correction information (see the section on error
    /// > correction).
    ///
    /// Page 19 says what the two bytes mean: "The first byte is the cumulative
    /// error for 'even' transitions and the second byte is for 'odd'
    /// transitions... If the value is in the range 0 to 192, then cell times
    /// were too long and this value is the amount of error." A disk this
    /// project laid down has cells of exactly the length they should be, so
    /// the error is zero both ways and there is nothing for the ROM to correct.
    fn correction(&mut self, debug: bool) -> u8 {
        if !debug {
            self.correction_at = !self.correction_at;
        }
        0
    }

    /// The error register, which reading clears (page 24): "The register is
    /// cleared by either reading it or resetting the chip."
    fn take_error(&mut self, debug: bool) -> u8 {
        let value = self.error;
        if !debug {
            self.error = 0;
        }
        value
    }

    /// The parameter RAM, advancing the auto-increment counter (page 21).
    fn read_param(&mut self, debug: bool) -> u8 {
        let value = self.param[usize::from(self.param_at)];
        if !debug {
            self.param_at = (self.param_at + 1) % PARAM_BYTES as u8;
        }
        value
    }

    /// The phase register (page 21): the four directions, and each line's
    /// state — the bit for an output, the pin for an input.
    fn read_phase(&self, iwm: &Iwm) -> u8 {
        let dirs = self.phase & 0xf0;
        let live = iwm.phases() & 0x0f;
        let outputs = (self.phase >> 4) & 0x0f;
        // An output reads back what was written to it; an input reads the pin,
        // and nothing on this board drives these four, so an input stands at
        // the cable's pull-up.
        dirs | (self.phase & outputs & 0x0f) | (live & !outputs & 0x0f) | (!outputs & !live & 0x0f)
    }

    /// The handshake register (page 25).
    ///
    /// `debug` matters here beyond the FIFO: bit 3 is the drive's `SENSE`
    /// line, and *reading* that line at one of the two instantaneous-read
    /// addresses is how the mechanism is told which head to use, so a debugger
    /// looking at this register must not move the head
    /// ([`Iwm::sense_and_pick_head`]).
    fn handshake(&self, iwm: &Iwm, debug: bool) -> u8 {
        let mut value = 0u8;
        let queued = iwm.mfm_queued();
        if self.mode & MODE_WRITE == 0 {
            // Read mode: the bits count bytes *available*.
            if queued >= 2 {
                value |= HS_TWO_BYTES;
            }
            if queued >= 1 {
                value |= HS_ONE_BYTE;
            }
        } else {
            // Write mode: they count *empty* slots. Writing to a disk is not
            // modelled, so the buffer is always empty and always will take a
            // byte — the same answer the IWM's own write handshake gives.
            value |= HS_TWO_BYTES | HS_ONE_BYTE;
        }
        if let Some((_, true)) = iwm.peek_mfm() {
            value |= HS_MARK;
        }
        if iwm.mfm_crc() != 0 {
            value |= HS_CRC_ERROR;
        }
        if iwm.read_line() {
            value |= HS_RDDATA;
        }
        let sense = if debug {
            iwm.sense()
        } else {
            iwm.sense_and_pick_head()
        };
        if sense {
            value |= HS_SENSE;
        }
        if self.mode & MODE_MOTOR_ON != 0 {
            value |= HS_MOTOR_ON;
        }
        if self.error != 0 {
            value |= HS_ERROR;
        }
        value
    }

    /// Write `value` to register `reg`.
    ///
    /// Page 13 makes A3 "the read/write line for the registers", so a bus
    /// *write* names its register with the low three address bits and A3 says
    /// nothing further — which is why this decodes `reg & 7` and the read path
    /// above does not. **Inferred** from that sentence rather than stated.
    ///
    /// Returns whether the write left ISM mode, which the caller has to act on
    /// because it is the caller that decides where the next access goes.
    pub fn write(&mut self, reg: u8, value: u8, iwm: &Iwm, debug: bool) -> bool {
        if debug {
            // There is no harmless debug write here: every address either
            // loads a register or moves a counter.
            return false;
        }
        let before = self.mode;
        match reg & 7 {
            REG_DATA => self.written = value,
            // Page 20: "Writing to this register will cause a byte to be
            // written that has a transition missing between two adjacent
            // zero-bits." Writing to a disk is not modelled here.
            REG_MARK => self.written = value,
            REG_CRC => {
                if self.mode & MODE_ACTION == 0 {
                    // Page 20, the IWM Configuration register: "the uppermost
                    // three bits modify some of the IWM-mode operations. This
                    // feature is not supported in the standard ISM." So it is
                    // not supported here either, and the write is kept only so
                    // that a read of it can be told from a fault.
                }
            }
            REG_PARAM => {
                self.param[usize::from(self.param_at)] = value;
                self.param_at = (self.param_at + 1) % PARAM_BYTES as u8;
            }
            REG_PHASE => self.phase = value,
            REG_SETUP => self.setup = value,
            REG_MODE_ZEROS => {
                // Page 23: "One or more bits can be set to '0' by writing a
                // byte with those bit(s) set to the 'zeroes' location (0110)".
                self.mode &= !value;
                // Page 21: "The counter is set to zero after any access is
                // made to the Mode 0 register (register 6) or the chip is
                // reset."
                self.param_at = 0;
            }
            // "the bit(s) can be set to '1' by writing to the 'ones' location
            // (0111)".
            REG_MODE_ONES => self.mode |= value,
            _ => {}
        }
        self.after_mode(before, iwm);
        !self.selected()
    }

    /// Act on whatever the mode register's bits just became.
    fn after_mode(&mut self, before: u8, iwm: &Iwm) {
        let now = self.mode;
        // The Clear FIFO bit is a *toggle*, page 23: "Toggling the clear FIFO
        // bit high then low clears the FIFO to begin a read or write
        // operation, and initializes the CRC generator with its starting
        // value. Since this value is different for reading or writing, the
        // read*/write mode bit must be set to the appropriate state before
        // toggling the Clear FIFO bit."
        let cleared = before & MODE_CLEAR_FIFO != 0 && now & MODE_CLEAR_FIFO == 0;
        // `ACTION` rising is what starts an operation, page 23: "Setting the
        // ACTION bit to '1' starts a read or write operation. It should be set
        // only after everything else has been set up."
        let started = before & MODE_ACTION == 0 && now & MODE_ACTION != 0;
        if cleared || started {
            iwm.restart_mfm(CRC_SEED);
        }
        if started || (before & MODE_ACTION != 0 && now & MODE_ACTION == 0) {
            self.error = 0;
        }
    }

    /// Pick up an overrun the separator recorded, before answering anything
    /// that reports it.
    pub fn note_overrun(&mut self, iwm: &Iwm) {
        if iwm.take_mfm_overrun() {
            self.raise(ERROR_UNDERRUN);
        }
    }

    /// Set an error bit, if none is set yet.
    ///
    /// Page 24: "Once one error bit is set, no other bits can be set until the
    /// register is cleared."
    fn raise(&mut self, bit: u8) {
        if self.error == 0 {
            self.error = bit;
        }
    }

    /// Write the register file into a snapshot.
    ///
    /// # Errors
    ///
    /// Whatever the writer returns.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u8(self.mode)?;
        w.write_u8(self.setup)?;
        w.write_u8(self.phase)?;
        w.write_u8(self.error)?;
        for byte in self.param {
            w.write_u8(byte)?;
        }
        w.write_u8(self.param_at)?;
        w.write_bool(self.correction_at)?;
        w.write_u8(self.written)
    }

    /// And read it back.
    ///
    /// # Errors
    ///
    /// Whatever the reader returns.
    pub fn load(r: &mut ChunkReader<'_>) -> Result<Ism> {
        let mode = r.read_u8()?;
        let setup = r.read_u8()?;
        let phase = r.read_u8()?;
        let error = r.read_u8()?;
        let mut param = [0u8; PARAM_BYTES];
        for slot in &mut param {
            *slot = r.read_u8()?;
        }
        Ok(Ism {
            mode,
            setup,
            phase,
            error,
            param,
            param_at: r.read_u8()? % PARAM_BYTES as u8,
            correction_at: r.read_bool()?,
            written: r.read_u8()?,
        })
    }
}

/// How many bytes of address space one ISM register occupies, which is the
/// IWM's stride because it is the same decode: the board puts the register
/// selects on A9-A12.
pub const REGISTER_STRIDE: u64 = iwm::REGISTER_SPAN / 16;
