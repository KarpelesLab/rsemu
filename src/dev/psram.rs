//! QSPI pseudo-static RAM: an AP Memory **APS6404L**-class part, on the SPI bus.
//!
//! # What pseudo-static RAM is, and why it is not a `ram` object
//!
//! It is DRAM with a refresh controller welded on, in a package with six pins.
//! The array needs refreshing and the part does it itself, which is what makes
//! it *pseudo*-static; everything a board has to care about follows from the
//! one thing the part cannot do on its own:
//!
//! > **It can only refresh while the chip select is high.**
//!
//! So the datasheet specifies **tCEM**, a *maximum* time the chip select may
//! stay low — 8 µs — and a master that clocks a burst longer than that starves
//! the array and loses data. That constraint is the whole reason a QSPI PSRAM
//! is a device model rather than a `ram` object on a chip select, and it is why
//! `stm32.octospi` grew `DCR3`'s `CSBOUND` and `MAXTRAN`: those fields exist to
//! chop a burst up so this part can breathe.
//!
//! The other two things that make it a device rather than memory:
//!
//! * **It is a quad part.** Its fast read is `EBh` with the address, dummy and
//!   data phases on four wires, and after `35h` *every* phase is four wires —
//!   at which point a one-line command is not a command the part can parse at
//!   all. [`Lines`] on [`SpiSlave::transfer_wide`] is what carries that, and a
//!   model that ignored the width would answer commands the silicon never
//!   understood.
//! * **A burst wraps.** A linear read runs to the end of a 1 KiB page and
//!   starts again at the beginning of *that page*, not the next one. `C0h`
//!   toggles the boundary to 32 bytes. Firmware that memcpys across a page
//!   boundary in one frame gets its own data back, shuffled, and nothing
//!   reports an error.
//!
//! # Time, and how this part gets it
//!
//! [`SpiSlave::select`] carries no timestamp, and this part has no clock
//! domain for exactly the reasons [`crate::dev::flash::spinor`] sets out: `SCK`
//! is the *master's* clock, the on-die refresh oscillator is not a crystal any
//! board wires, and a slave is reached from inside its controller's own
//! catch-up, where arming a scheduler event is not available.
//!
//! So it counts the only clock on the link: **the master's**. Every word it is
//! handed carries the width the master drove, and a byte on `n` wires is
//! exactly `8 / n` clocks, so the length of a chip-select assertion is integer
//! arithmetic over the frame — no floats, no wall clock, and the same number
//! `stm32.octospi` computes on its side of the same bus. `tcem-cycles` is that
//! budget, and it is denominated in cycles rather than nanoseconds because a
//! slave has no rate to convert with: 8 µs is 672 cycles at 84 MHz, 480 at 60,
//! and the board is what knows which.
//!
//! `tcem-check` says what a violation does. **Never data corruption**: a
//! starved array loses bits in a pattern nothing can model usefully, and a
//! model that scrambled the guest's memory would be inventing a failure rather
//! than reporting one. `log` counts it, `fault` additionally discards the rest
//! of the frame — a failure a firmware author sees immediately — and `off`
//! does not look.
//!
//! # Command set
//!
//! | Opcode | Name | Address | Dummy | Notes |
//! | --- | --- | --- | --- | --- |
//! | `03h` | Read | 24-bit | none | single line only, and ≤ 33 MHz on silicon |
//! | `0Bh` | Fast Read | 24-bit | 8 clocks | single line |
//! | `EBh` | Fast Read Quad | 24-bit, quad | 6 clocks, quad | opcode single in SPI mode, quad in QPI |
//! | `02h` | Write | 24-bit | none | single line |
//! | `38h` | Quad Write | 24-bit, quad | none | opcode single in SPI mode, quad in QPI |
//! | `35h` | Enter Quad Mode | — | — | every phase is four wires afterwards |
//! | `F5h` | Exit Quad Mode | — | — | |
//! | `66h` / `99h` | Reset Enable / Reset | — | — | `99h` only does anything straight after `66h` |
//! | `C0h` | Wrap Boundary Toggle | — | — | 1 KiB page ⇄ 32 bytes |
//! | `9Fh` | Read ID | 24-bit | none | `MFID`, `KGD`, then the EID |
//!
//! `03h`, `0Bh`, `02h` and `35h` are **SPI-mode instructions** and this model
//! ignores them in QPI mode, where the part is looking for four-bit nibbles and
//! a one-line opcode is not one. `F5h` is how a driver gets back.
//!
//! # Time, again: a write lands where it is clocked
//!
//! Unlike [`crate::dev::flash::spinor`], there is no staging and no "the
//! instruction is not executed" rule. This is RAM: a byte clocked into a write
//! frame is in the array before the next one arrives, and a frame cut short by
//! the chip select has written exactly the bytes that got there. That is the
//! difference between a part that programs a page latch and a part that does
//! not have one.
//!
//! # Sources
//!
//! * **AP Memory APS6404L-3SQR** *64 Mb (8 Mx8) QSPI pseudo-SRAM* datasheet —
//!   the instruction table, the 24-bit addressing, tCEM and its 8 µs, the 1 KiB
//!   page wrap and `C0h`'s toggle, the `9Fh` identification bytes (`MFID` 0Dh,
//!   `KGD` 5Dh, then the EID), and the `66h`/`99h` reset pair. The APS1604M,
//!   ISSI IS66WVS2M8, Espressif ESP-PSRAM64H and Lyontek LY68L6400 differ in
//!   the density and the manufacturer byte, which are properties here.
//! * **ST RM0432** (STM32L4+) and **RM0456** (STM32U5), chapter *"Octo-SPI
//!   interface"*, for `DCR3`'s `CSBOUND` and `MAXTRAN` — the two fields whose
//!   documented purpose is complying with this part's tCEM.
//!
//! The datasheet does **not** guarantee the array's contents across `99h`, and
//! this model leaves them alone rather than clearing them: "not guaranteed"
//! permits retention, and a guest that reset and re-read is better served by
//! the conservative reading than by data loss it cannot reproduce on hardware
//! it happens to hold.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::spi::{
    BitOrder, ChipSelect, Format, Lines, MAX_CHIP_SELECTS, Mode, SlavePins, SpiSlave, buses,
    pin as spi_pin,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{WireId, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "psram.qspi";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How big an APS6404L is: 64 Mbit, and the default here.
pub const DEFAULT_SIZE: u64 = 8 * 1024 * 1024;

/// The largest part 24-bit addressing reaches.
pub const MAX_SIZE: u64 = 16 * 1024 * 1024;

/// The linear-burst boundary a part powers up with: one page.
pub const PAGE: u64 = 1024;

/// The boundary `C0h` toggles to.
pub const SHORT_WRAP: u64 = 32;

/// AP Memory's manufacturer byte, the first thing `9Fh` returns.
pub const AP_MEMORY: u8 = 0x0d;

/// ISSI's, on an IS66WVS2M8.
pub const ISSI: u8 = 0x9d;

/// Known-good-die: the second `9Fh` byte, and the same on every part here.
pub const KGD: u8 = 0x5d;

// -- the instruction set (APS6404L datasheet, the instruction table) ---------

/// Read, up to 33 MHz, no dummy cycles.
const CMD_READ: u8 = 0x03;
/// Fast Read: eight dummy clocks.
const CMD_FAST_READ: u8 = 0x0b;
/// Fast Read Quad: six dummy clocks, address and data on four wires.
const CMD_FAST_READ_QUAD: u8 = 0xeb;
/// Write.
const CMD_WRITE: u8 = 0x02;
/// Quad Write: address and data on four wires.
const CMD_QUAD_WRITE: u8 = 0x38;
/// Enter Quad Mode. Afterwards every phase of every frame is four wires.
const CMD_ENTER_QUAD: u8 = 0x35;
/// Exit Quad Mode.
const CMD_EXIT_QUAD: u8 = 0xf5;
/// Reset Enable. Only `99h` immediately after it resets the part.
const CMD_RESET_ENABLE: u8 = 0x66;
/// Reset.
const CMD_RESET: u8 = 0x99;
/// Wrap Boundary Toggle: 1 KiB page ⇄ 32 bytes.
const CMD_WRAP_TOGGLE: u8 = 0xc0;
/// Read ID, after a 24-bit address phase.
const CMD_READ_ID: u8 = 0x9f;

/// How many dummy *clocks* `0Bh` takes.
const DUMMY_FAST_READ: u64 = 8;
/// And `EBh`.
const DUMMY_FAST_READ_QUAD: u64 = 6;

/// How many address bytes every command that has an address phase carries.
const ADDRESS_BYTES: u8 = 3;

/// What an undriven, pulled-up MISO reads as, and what this part presents when
/// it has nothing to say.
const IDLE_BYTE: u8 = 0xff;

fn config(message: String) -> Error {
    Error::Config {
        at: CLASS_NAME.to_string(),
        message,
    }
}

// ---------------------------------------------------------------------------
// what a tCEM violation does
// ---------------------------------------------------------------------------

/// What the part does when the chip select stays low past `tcem-cycles`.
///
/// **None of these corrupts the array**, and that is the design rather than an
/// omission. A refresh-starved DRAM cell decays in a pattern that depends on
/// the die, the temperature and how long ago the row was last touched; a model
/// that scrambled guest memory would be inventing a failure rather than
/// reporting one, and a firmware author would then be debugging the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TcemCheck {
    /// Do not look. What a board with no `tcem-cycles` gets.
    Off,
    /// Count the violation and carry on unharmed. The default.
    #[default]
    Log,
    /// Count it, and discard the rest of the frame — a failure firmware sees
    /// on the very next byte rather than a counter someone has to go and read.
    Fault,
}

impl TcemCheck {
    /// Parse the property's spelling.
    fn from_name(name: &str) -> Option<TcemCheck> {
        match name {
            "off" => Some(TcemCheck::Off),
            "log" => Some(TcemCheck::Log),
            "fault" => Some(TcemCheck::Fault),
            _ => None,
        }
    }

    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            TcemCheck::Off => "off",
            TcemCheck::Log => "log",
            TcemCheck::Fault => "fault",
        }
    }

    /// Every spelling, for an error message and for the validator.
    pub const NAMES: &'static [&'static str] = &["off", "log", "fault"];
}

impl fmt::Display for TcemCheck {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// ---------------------------------------------------------------------------
// where a frame is
// ---------------------------------------------------------------------------

/// Which part of the current frame the next byte belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Waiting for the instruction byte.
    Opcode,
    /// Collecting address bytes.
    Address,
    /// Consuming dummy clocks.
    Dummy,
    /// Streaming data, in whichever direction [`Stream`] says.
    Data,
    /// A frame this part cannot parse — an unimplemented opcode, an SPI-mode
    /// instruction arriving in QPI mode, a phase clocked on the wrong number
    /// of wires, or a tCEM violation under `tcem-check = "fault"`. Everything
    /// to the next rising edge of the chip select is discarded.
    Ignored,
}

impl Phase {
    const fn tag(self) -> u8 {
        match self {
            Phase::Opcode => 0,
            Phase::Address => 1,
            Phase::Dummy => 2,
            Phase::Data => 3,
            Phase::Ignored => 4,
        }
    }

    fn from_tag(tag: u8) -> Result<Phase> {
        match tag {
            0 => Ok(Phase::Opcode),
            1 => Ok(Phase::Address),
            2 => Ok(Phase::Dummy),
            3 => Ok(Phase::Data),
            4 => Ok(Phase::Ignored),
            other => Err(Error::State(format!("{other} is not a PSRAM frame phase"))),
        }
    }
}

/// What the data phase of the current frame carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stream {
    /// Nothing; the part drives the idle level.
    None,
    /// The array, from `addr` upwards, wrapping inside the current page.
    Read,
    /// Incoming bytes, into the array at `addr`.
    Write,
    /// `MFID`, `KGD` and the EID, repeating (`9Fh`).
    Id,
}

impl Stream {
    const fn tag(self) -> u8 {
        match self {
            Stream::None => 0,
            Stream::Read => 1,
            Stream::Write => 2,
            Stream::Id => 3,
        }
    }

    fn from_tag(tag: u8) -> Result<Stream> {
        match tag {
            0 => Ok(Stream::None),
            1 => Ok(Stream::Read),
            2 => Ok(Stream::Write),
            3 => Ok(Stream::Id),
            other => Err(Error::State(format!("{other} is not a PSRAM stream"))),
        }
    }
}

/// A mode change that takes effect when the chip select rises.
///
/// The commands that carry no address and no data — `35h`, `F5h`, `66h`,
/// `99h`, `C0h` — are single bytes, and a frame is not over until the chip
/// select says so. Applying them where their byte lands would let a master
/// that clocked an extra byte see the *new* mode part way through the frame
/// that changed it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Staged {
    /// Nothing to do.
    None,
    /// `35h` or `F5h`.
    QuadMode(bool),
    /// `66h`.
    EnableReset,
    /// `99h`, after a `66h`.
    Reset,
    /// `C0h`.
    ToggleWrap,
}

impl Staged {
    const fn tag(self) -> u8 {
        match self {
            Staged::None => 0,
            Staged::QuadMode(false) => 1,
            Staged::QuadMode(true) => 2,
            Staged::EnableReset => 3,
            Staged::Reset => 4,
            Staged::ToggleWrap => 5,
        }
    }

    fn from_tag(tag: u8) -> Result<Staged> {
        match tag {
            0 => Ok(Staged::None),
            1 => Ok(Staged::QuadMode(false)),
            2 => Ok(Staged::QuadMode(true)),
            3 => Ok(Staged::EnableReset),
            4 => Ok(Staged::Reset),
            5 => Ok(Staged::ToggleWrap),
            other => Err(Error::State(format!("{other} is not a staged PSRAM mode"))),
        }
    }
}

// ---------------------------------------------------------------------------
// the part
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    phase: Phase,
    stream: Stream,
    /// The byte the part is presenting on MISO *now*. Full duplex: what
    /// [`SpiSlave::transfer`] returns is what was already here when the word
    /// began, never a reply to it.
    out: u8,
    /// The address the frame is working at.
    addr: u64,
    /// How many address bytes have arrived.
    got: u8,
    /// How many dummy *bits* are still to come. Bits rather than clocks
    /// because a byte is eight bits at any width, and it is bytes that arrive.
    dummy_bits: u64,
    /// How far into the repeating identifier response the master has clocked.
    count: u64,
    /// Whether every phase is four wires: `35h` set it, `F5h` clears it.
    quad: bool,
    /// The linear-burst boundary in force: [`PAGE`] or [`SHORT_WRAP`].
    wrap: u64,
    /// How many wires the rest of this frame runs on.
    ///
    /// Set when the opcode lands and unchanged until the chip select rises,
    /// because that is exactly what a mixed-width frame is: `EBh` in SPI mode
    /// is a one-line opcode whose address, dummy and data are four-line, and
    /// the *only* thing that says so is which opcode it was.
    lines: Lines,
    /// `66h` was the last completed frame, so `99h` will reset the part.
    reset_armed: bool,
    /// What the rising edge of the chip select will do.
    staged: Staged,
    /// How many serial clocks the chip select has been low for.
    ///
    /// The part's only clock. See the module docs: a byte on `n` wires is
    /// `8 / n` clocks, and the master announces `n` with every word.
    cs_cycles: u64,
    /// How many frames have outlasted tCEM. A diagnostic, saved so that a
    /// snapshot round trip does not silently reset the tally a test is
    /// asserting on.
    tcem_violations: u64,
}

impl State {
    const fn new() -> State {
        State {
            phase: Phase::Opcode,
            stream: Stream::None,
            out: IDLE_BYTE,
            addr: 0,
            got: 0,
            dummy_bits: 0,
            count: 0,
            // SPI, one wire, and a 1 KiB linear burst: the power-on state.
            quad: false,
            wrap: PAGE,
            lines: Lines::SINGLE,
            reset_armed: false,
            staged: Staged::None,
            cs_cycles: 0,
            tcem_violations: 0,
        }
    }

    /// Start a fresh frame, keeping everything that survives a chip select.
    fn begin_frame(&mut self) {
        self.phase = Phase::Opcode;
        self.stream = Stream::None;
        self.addr = 0;
        self.got = 0;
        self.dummy_bits = 0;
        self.count = 0;
        self.staged = Staged::None;
        self.cs_cycles = 0;
        self.out = IDLE_BYTE;
        self.lines = self.opcode_lines();
    }

    /// The width the *opcode* of the next frame arrives on.
    const fn opcode_lines(&self) -> Lines {
        if self.quad {
            Lines::QUAD
        } else {
            Lines::SINGLE
        }
    }
}

/// Everything both halves of the device reach.
struct Shared {
    /// The contents. A [`RamStore`] for the reasons every array in this tree
    /// uses one: byte addressed, `Sync` without `unsafe`, never handed out as
    /// a slice, so it can live in a `SharedArrayBuffer`.
    array: Arc<RamStore>,
    size: u64,
    /// How this part frames a word on the wire.
    format: Format,
    /// What `9Fh` answers with: `MFID`, `KGD`, then six EID bytes.
    id: [u8; 8],
    /// What the array holds at power-on.
    fill: u8,
    /// How many serial clocks the chip select may stay low, or zero for no
    /// check. 8 µs × the bus clock; see the module docs for why a slave cannot
    /// convert that itself.
    tcem_cycles: u64,
    /// What a violation does.
    tcem_check: TcemCheck,
    state: Mutex<State>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Psram");
        s.field("size", &self.size).field("id", &self.id);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    /// One byte of the array. The address is already inside the part.
    fn byte(&self, addr: u64) -> u8 {
        self.array.read_u8(addr).unwrap_or(self.fill)
    }

    /// Advance the burst address, wrapping inside the current boundary.
    ///
    /// The rule the datasheet states and firmware forgets: a linear burst runs
    /// to the end of its 1 KiB page and resumes at the *start of that page*,
    /// not at the next one. `C0h` makes the boundary 32 bytes instead.
    fn advance(&self, state: &mut State) {
        let wrap = state.wrap.max(1);
        let base = state.addr & !(wrap - 1);
        let within = (state.addr.wrapping_add(1)) & (wrap - 1);
        state.addr = (base | within) % self.size.max(1);
    }

    /// Present the byte the part will drive during the next word.
    fn present(&self, state: &mut State) {
        state.out = match state.stream {
            Stream::Read => self.byte(state.addr),
            Stream::Id => self.id[(state.count % 8) as usize],
            Stream::None | Stream::Write => IDLE_BYTE,
        };
    }

    /// One word has been exchanged: fold `mosi` into the frame.
    fn step(&self, state: &mut State, mosi: u8) {
        match state.phase {
            Phase::Opcode => self.opcode(state, mosi),
            Phase::Address => {
                state.addr = (state.addr << 8) | u64::from(mosi);
                state.got += 1;
                if state.got >= ADDRESS_BYTES {
                    // A 24-bit address on a part smaller than 16 MiB simply
                    // has fewer pins inside: the top bits are not decoded.
                    state.addr %= self.size.max(1);
                    self.addressed(state);
                }
            }
            Phase::Dummy => {
                state.dummy_bits = state.dummy_bits.saturating_sub(8);
                if state.dummy_bits == 0 {
                    state.phase = Phase::Data;
                    self.present(state);
                }
            }
            Phase::Data => match state.stream {
                Stream::Read | Stream::Id => {
                    state.count += 1;
                    if state.stream == Stream::Read {
                        self.advance(state);
                    }
                    self.present(state);
                }
                Stream::Write => {
                    // RAM, not flash: the byte is in the array before the next
                    // one arrives. There is no page latch and no rule about an
                    // instruction that the chip select cut short.
                    let _ = self.array.write_u8(state.addr, mosi);
                    self.advance(state);
                }
                Stream::None => {}
            },
            Phase::Ignored => {}
        }
    }

    /// The instruction byte landed.
    fn opcode(&self, state: &mut State, opcode: u8) {
        // The instructions that only exist in one mode. In QPI the part is
        // sampling four bits a clock and a single-line opcode is not something
        // it can parse; `F5h` is the way back out.
        let spi_only = matches!(
            opcode,
            CMD_READ | CMD_FAST_READ | CMD_WRITE | CMD_ENTER_QUAD
        );
        if state.quad && spi_only {
            state.phase = Phase::Ignored;
            return;
        }
        match opcode {
            CMD_READ | CMD_FAST_READ | CMD_FAST_READ_QUAD | CMD_WRITE | CMD_QUAD_WRITE
            | CMD_READ_ID => {
                state.phase = Phase::Address;
                state.got = 0;
                state.addr = 0;
                state.stream = match opcode {
                    CMD_WRITE | CMD_QUAD_WRITE => Stream::Write,
                    CMD_READ_ID => Stream::Id,
                    _ => Stream::Read,
                };
                // The dummy clocks each read takes, at the width the data
                // phase will run on: the conversion `Lines::bits` exists for.
                let lines = data_lines(opcode, state.quad);
                state.lines = lines;
                state.dummy_bits = match opcode {
                    CMD_FAST_READ => lines.bits(DUMMY_FAST_READ),
                    CMD_FAST_READ_QUAD => lines.bits(DUMMY_FAST_READ_QUAD),
                    _ => 0,
                };
            }
            CMD_ENTER_QUAD => {
                state.staged = Staged::QuadMode(true);
                state.phase = Phase::Ignored;
            }
            CMD_EXIT_QUAD => {
                state.staged = Staged::QuadMode(false);
                state.phase = Phase::Ignored;
            }
            CMD_RESET_ENABLE => {
                state.staged = Staged::EnableReset;
                state.phase = Phase::Ignored;
            }
            CMD_RESET => {
                // Only straight after `66h`, which is the whole point of the
                // pair: a lone `99h` on a noisy line does not wipe the mode.
                state.staged = if state.reset_armed {
                    Staged::Reset
                } else {
                    Staged::None
                };
                state.phase = Phase::Ignored;
            }
            CMD_WRAP_TOGGLE => {
                state.staged = Staged::ToggleWrap;
                state.phase = Phase::Ignored;
            }
            _ => state.phase = Phase::Ignored,
        }
    }

    /// The address phase finished: work out what comes next.
    fn addressed(&self, state: &mut State) {
        if state.dummy_bits > 0 {
            state.phase = Phase::Dummy;
        } else {
            state.phase = Phase::Data;
            self.present(state);
        }
    }

    /// Apply what the frame staged, at the rising edge of the chip select.
    fn commit(&self, state: &mut State, staged: Staged) {
        // `reset_armed` survives only to the *next* frame, so it is cleared
        // here and set again below if this frame was the `66h`.
        state.reset_armed = false;
        match staged {
            Staged::None => {}
            Staged::QuadMode(on) => state.quad = on,
            Staged::EnableReset => state.reset_armed = true,
            Staged::Reset => {
                // Back to SPI, back to a 1 KiB burst. The array is left alone;
                // the module docs say why.
                state.quad = false;
                state.wrap = PAGE;
            }
            Staged::ToggleWrap => {
                state.wrap = if state.wrap == PAGE { SHORT_WRAP } else { PAGE };
            }
        }
    }

    /// Charge `cycles` to the open chip-select assertion and see whether the
    /// part has now been held past tCEM.
    fn charge(&self, state: &mut State, cycles: u64) {
        let before = state.cs_cycles;
        state.cs_cycles = before.saturating_add(cycles);
        if self.tcem_check == TcemCheck::Off || self.tcem_cycles == 0 {
            return;
        }
        // Only on the crossing, so one over-long frame is one violation
        // however many bytes follow it.
        if before <= self.tcem_cycles && state.cs_cycles > self.tcem_cycles {
            state.tcem_violations = state.tcem_violations.saturating_add(1);
            if self.tcem_check == TcemCheck::Fault {
                state.phase = Phase::Ignored;
                state.out = IDLE_BYTE;
            }
        }
    }
}

/// How many wires the address, dummy and data phases of `opcode` run on.
///
/// In QPI mode every phase is four wires. In SPI mode the opcode is always one
/// wire and only the two quad instructions widen afterwards — which is what
/// makes `EBh` a *mixed-width frame*, and what [`Lines`] on the word exists to
/// carry.
const fn data_lines(opcode: u8, quad_mode: bool) -> Lines {
    if quad_mode {
        return Lines::QUAD;
    }
    match opcode {
        CMD_FAST_READ_QUAD | CMD_QUAD_WRITE => Lines::QUAD,
        _ => Lines::SINGLE,
    }
}

impl SpiSlave for Shared {
    fn format(&self) -> Format {
        self.format
    }

    fn select(&self, selected: bool) {
        let mut state = self.state.lock();
        if selected {
            state.begin_frame();
            self.present(&mut state);
            return;
        }
        let staged = core::mem::replace(&mut state.staged, Staged::None);
        self.commit(&mut state, staged);
        state.phase = Phase::Opcode;
        state.stream = Stream::None;
        state.out = IDLE_BYTE;
        state.cs_cycles = 0;
    }

    fn transfer(&self, mosi: u32) -> u32 {
        self.transfer_wide(mosi, Lines::SINGLE)
    }

    fn transfer_wide(&self, mosi: u32, lines: Lines) -> u32 {
        let mut state = self.state.lock();
        // Full duplex: what goes out is what was already in the shift register
        // when this word began, which is what `present` last put there.
        let presented = state.out;
        // The clock this part has: a byte on `n` wires took `8 / n` of them.
        self.charge(&mut state, lines.cycles(8));
        // Then the width itself. A phase clocked on the wrong number of wires
        // is not a phase this part can parse — in QPI mode especially, where
        // the same eight clocks would have carried four bytes.
        let want = match state.phase {
            Phase::Opcode => state.opcode_lines(),
            // An ignored frame swallows whatever arrives, at any width: the
            // part has stopped parsing until the chip select rises.
            Phase::Ignored => lines,
            _ => state.lines,
        };
        if lines != want {
            state.phase = Phase::Ignored;
            state.out = IDLE_BYTE;
            return u32::from(presented);
        }
        self.step(&mut state, mosi as u8);
        u32::from(presented)
    }

    fn peek(&self) -> u32 {
        u32::from(self.state.lock().out)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An APS6404L-class QSPI pseudo-static RAM.
#[derive(Debug)]
pub struct Psram {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
}

impl Psram {
    /// Validate `props` and allocate the array.
    ///
    /// Properties:
    ///
    /// * `size` — how many bytes the part holds, a power of two from 1 KiB to
    ///   16 MiB. Defaults to 8 MiB, an APS6404L. The ceiling is 24-bit
    ///   addressing, which is what the whole family has.
    /// * `bus`, `cs` — the named SPI bus and the chip select to answer on.
    /// * `mode` — 0 or 3. The part accepts both and the seam names one.
    /// * `id` — `"aps6404"` (default) or `"is66"`, which differ only in the
    ///   manufacturer byte `9Fh` returns.
    /// * `eid` — the 48 bits of identifier that follow `MFID` and `KGD`.
    /// * `fill` — the byte the array holds at power-on. Real pseudo-static RAM
    ///   powers up indeterminate; a model has to choose, and a board that
    ///   wants to see uninitialised reads can say `0xff`.
    /// * `tcem-cycles` — how many serial clocks the chip select may stay low.
    ///   Zero, the default, does not check. 8 µs is 672 clocks at 84 MHz.
    /// * `tcem-check` — `off`, `log` (default) or `fault`.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property, [`Error::Config`] for an
    /// impossible size, an unsupported mode, an unknown `id` or `tcem-check`,
    /// or a chip select out of range.
    pub fn new(props: &Props) -> Result<Psram> {
        let mut r = props.reader();
        let size = r.or_size("size", DEFAULT_SIZE)?;
        let bus_name = r.optional_str("bus")?.map(String::from);
        let cs = r.or_range("cs", 0u64, 0..=(MAX_CHIP_SELECTS as u64 - 1))?;
        let mode = r.or_range("mode", 0u64, 0..=3)?;
        let id = r.or_str("id", "aps6404")?.to_string();
        let eid: u64 = r.or("eid", 0)?;
        let fill = r.or_range("fill", 0u64, 0..=0xff)?;
        let tcem_cycles: u64 = r.or("tcem-cycles", 0)?;
        let tcem_check = r.or_str("tcem-check", "log")?.to_string();
        r.finish()?;

        if !(PAGE..=MAX_SIZE).contains(&size) || !size.is_power_of_two() {
            return Err(config(format!(
                "a pseudo-static RAM of {size} byte(s): the family is 24-bit addressed, so the \
                 part is a power of two from {PAGE} to {MAX_SIZE} bytes"
            )));
        }
        if usize::try_from(size).is_err() {
            return Err(config(format!(
                "a part of {size} byte(s) is larger than this host's address space"
            )));
        }
        if mode != 0 && mode != 3 {
            return Err(config(format!(
                "`mode` is {mode}; a QSPI PSRAM samples on the rising edge of a clock that idles \
                 either low or high, which is SPI mode 0 or mode 3"
            )));
        }
        let mfid = match id.as_str() {
            "aps6404" | "aps1604" => AP_MEMORY,
            "is66" => ISSI,
            other => {
                return Err(config(format!(
                    "`id` is {other:?}; this model knows \"aps6404\" and \"is66\", which differ \
                     only in the manufacturer byte `9Fh` returns"
                )));
            }
        };
        let Some(tcem_check) = TcemCheck::from_name(&tcem_check) else {
            return Err(config(format!(
                "`tcem-check` is {tcem_check:?}; it is one of {:?}",
                TcemCheck::NAMES
            )));
        };

        let array = Arc::new(RamStore::new(size));
        array
            .fill(0, size, fill as u8)
            .map_err(|_| config(String::from("the array could not be filled")))?;

        let mut id_bytes = [0u8; 8];
        id_bytes[0] = mfid;
        id_bytes[1] = KGD;
        for (i, byte) in id_bytes[2..].iter_mut().enumerate() {
            *byte = (eid >> (8 * (5 - i))) as u8;
        }

        let shared = Arc::new(Shared {
            array,
            size,
            format: Format::new(
                if mode == 3 { Mode::Mode3 } else { Mode::Mode0 },
                8,
                BitOrder::MsbFirst,
            ),
            id: id_bytes,
            fill: fill as u8,
            tcem_cycles,
            tcem_check,
            state: Mutex::with_rank(LockRank::DEVICE, State::new()),
        });
        let pins = Arc::new(SlavePins::new(Arc::clone(&shared) as Arc<dyn SpiSlave>));
        let part = Psram { shared, pins };
        if let Some(name) = bus_name {
            let bus = buses::attach(props, &name)?;
            bus.attach(
                ChipSelect(cs as u8),
                Arc::clone(&part.shared) as Arc<dyn SpiSlave>,
            )?;
        }
        Ok(part)
    }

    /// How many bytes the part holds.
    #[must_use]
    pub fn size(&self) -> u64 {
        self.shared.size
    }

    /// The eight bytes `9Fh` answers with, repeating.
    #[must_use]
    pub fn id(&self) -> [u8; 8] {
        self.shared.id
    }

    /// Whether the part is in QPI mode — `35h` put it there.
    #[must_use]
    pub fn quad_mode(&self) -> bool {
        self.shared.state.lock().quad
    }

    /// The linear-burst boundary in force, in bytes.
    #[must_use]
    pub fn wrap(&self) -> u64 {
        self.shared.state.lock().wrap
    }

    /// How many frames have held the chip select past tCEM.
    ///
    /// The diagnostic `tcem-check` produces. Zero on a board that never
    /// programmed a budget, because nothing was checked.
    #[must_use]
    pub fn tcem_violations(&self) -> u64 {
        self.shared.state.lock().tcem_violations
    }

    /// This part's wire pins, for a controller that drives them directly.
    ///
    /// **One data line.** `bus::spi`'s wired link is a bit per edge, so a part
    /// reached this way is reachable in SPI mode only: `35h` puts it into a
    /// mode whose every phase is four wires, and the pins cannot carry that.
    #[must_use]
    pub fn pins(&self) -> &Arc<SlavePins> {
        &self.pins
    }

    /// This part as a bus slave, for a test or an embedder that wires its own
    /// [`SpiBus`](crate::bus::spi::SpiBus).
    #[must_use]
    pub fn slave(&self) -> Arc<dyn SpiSlave> {
        Arc::clone(&self.shared) as Arc<dyn SpiSlave>
    }

    /// The contents, for a test, a debugger, or a host that inspects them.
    ///
    /// Never has a side effect and never touches the frame state machine.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs off the end of the part.
    pub fn read_contents(&self, offset: u64, dst: &mut [u8]) -> Result<()> {
        self.shared
            .array
            .read_at(offset, dst)
            .map_err(|_| Error::State(format!("{offset:#x} is outside this PSRAM")))
    }

    /// Put `bytes` into the array at `offset`, without a frame.
    ///
    /// The loader's door, for a test or a board that pre-seeds the array.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if it does not fit.
    pub fn load_image(&self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.shared.array.write_at(offset, bytes).map_err(|_| {
            config(format!(
                "an image of {} byte(s) at {offset:#x} does not fit in a part of {}",
                bytes.len(),
                self.shared.size
            ))
        })
    }

    /// Whether this part's SCK rests high between frames.
    ///
    /// The snapshot stores SCK relative to this so that "power-on" has one
    /// encoding whatever the mode.
    fn idle_sck(&self) -> bool {
        self.shared.format.mode.idle_level().is_high()
    }
}

impl Device for Psram {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the part is already on its bus and a `wire`
        // statement connects its pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // **The contents do not survive**, and that is the difference from
        // every flash part in this tree: this is volatile memory, and a reset
        // is a power cycle as far as a device model can tell.
        let _ = self
            .shared
            .array
            .fill(0, self.shared.size, self.shared.fill);
        {
            let mut state = self.shared.state.lock();
            *state = State::new();
        }
        self.pins.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let mut contents = alloc::vec![0u8; self.shared.size as usize];
        let _ = self.shared.array.read_at(0, &mut contents);
        w.write_bytes(&contents)?;
        // The shifter first, though it is written last: its lock ranks *above*
        // a device's own state (`bus::spi::SHIFTER_RANK` against
        // `LockRank::DEVICE`), so taking it while holding the state lock would
        // climb the ladder backwards.
        let (rx, tx, count, selected, sck, mosi, loaded) = self.pins.snapshot();
        let state = *self.shared.state.lock();
        w.write_u8(state.phase.tag())?;
        w.write_u8(state.stream.tag())?;
        w.write_u8(state.out)?;
        w.write_u64(state.addr)?;
        w.write_u8(state.got)?;
        w.write_u64(state.dummy_bits)?;
        w.write_u64(state.count)?;
        w.write_bool(state.quad)?;
        w.write_u64(state.wrap)?;
        w.write_u8(state.lines.0)?;
        w.write_bool(state.reset_armed)?;
        w.write_u8(state.staged.tag())?;
        w.write_u64(state.cs_cycles)?;
        w.write_u64(state.tcem_violations)?;
        // The bit-level shifter. Everything above is a *byte* machine, and the
        // bits that have arrived since its last whole word live in the
        // `SlavePins` this part holds; a snapshot without them restores a
        // decoder that disagrees with the wire about where the frame is.
        w.write_u32(rx)?;
        w.write_u32(tx)?;
        w.write_u8(count)?;
        w.write_bool(selected)?;
        // SCK **relative to this part's idle level**: the `mode` property
        // decides whether idle is high or low, so an absolute bit has no
        // power-on value a static function could name.
        w.write_bool(sck != self.idle_sck())?;
        w.write_bool(mosi)?;
        w.write_bool(loaded)
        // MISO is not saved: it is a level *this* part drives, and
        // `SlavePins::restore` republishes it from the shifter.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        if bytes.len() as u64 != self.shared.size {
            return Err(Error::State(format!(
                "snapshot has {} byte(s) of PSRAM, this part has {}",
                bytes.len(),
                self.shared.size
            )));
        }
        self.shared
            .array
            .write_at(0, bytes)
            .map_err(|_| Error::State(String::from("the PSRAM refused its snapshot contents")))?;
        let phase = Phase::from_tag(r.read_u8()?)?;
        let stream = Stream::from_tag(r.read_u8()?)?;
        let out = r.read_u8()?;
        let addr = r.read_u64()?;
        let got = r.read_u8()?;
        let dummy_bits = r.read_u64()?;
        let count = r.read_u64()?;
        let quad = r.read_bool()?;
        let wrap = r.read_u64()?;
        let lines = Lines(r.read_u8()?);
        let reset_armed = r.read_bool()?;
        let staged = Staged::from_tag(r.read_u8()?)?;
        let cs_cycles = r.read_u64()?;
        let tcem_violations = r.read_u64()?;
        let rx = r.read_u32()?;
        let tx = r.read_u32()?;
        let bits = r.read_u8()?;
        let selected = r.read_bool()?;
        let sck_moved = r.read_bool()?;
        let mosi = r.read_bool()?;
        let loaded = r.read_bool()?;

        {
            let mut state = self.shared.state.lock();
            *state = State {
                phase,
                stream,
                out,
                // Bounded by the part rather than trusted: a corrupt snapshot
                // must not become a read outside the array.
                addr: addr % self.shared.size.max(1),
                got: got.min(ADDRESS_BYTES),
                dummy_bits,
                count,
                quad,
                // One of the two boundaries the part has, and neither is zero
                // — `advance` divides by it.
                wrap: if wrap == SHORT_WRAP { SHORT_WRAP } else { PAGE },
                // A width the fabric does not have is read as one wire rather
                // than refused: `Lines::cycles` divides by it.
                lines: if lines.is_standard() {
                    lines
                } else {
                    Lines::SINGLE
                },
                reset_armed,
                staged,
                cs_cycles,
                tcem_violations,
            };
        }
        self.pins.restore((
            rx,
            tx,
            bits,
            selected,
            sck_moved != self.idle_sck(),
            mosi,
            loaded,
        ));
        Ok(())
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            spi_pin::SCK_NAME => spi_pin::SCK,
            spi_pin::MOSI_NAME => spi_pin::MOSI,
            spi_pin::CS_NAME => spi_pin::CS,
            _ => return None,
        };
        Some(SinkPin {
            sink: self.pins.sink(line),
            line,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != spi_pin::MISO_NAME {
            return Err(config(format!(
                "`{port}` is not an output of {CLASS_NAME}; it drives `{}`",
                spi_pin::MISO_NAME
            )));
        }
        self.pins.connect_miso(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == spi_pin::MISO_NAME {
            self.pins.publish_miso();
        }
    }
}

impl Instance for Psram {}

/// The `psram.qspi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "APS6404L-class QSPI pseudo-static RAM: single and quad frames, the 1 KiB burst \
              wrap, `9Fh` identification, and the tCEM chip-select-low check",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how many bytes the part holds, a power of two (default 8M, an APS6404L)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named SPI bus to attach to, for a transactional link",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select on that bus (default 0)",
        },
        PropertySpec {
            name: "mode",
            kind: ValueKind::Uint,
            required: false,
            summary: "SPI mode 0 or 3; the part accepts both and the fabric names one",
        },
        PropertySpec {
            name: "id",
            kind: ValueKind::Str,
            required: false,
            summary: "\"aps6404\" (default) or \"is66\": the manufacturer byte `9Fh` returns",
        },
        PropertySpec {
            name: "eid",
            kind: ValueKind::Uint,
            required: false,
            summary: "the 48 identifier bits that follow MFID and KGD in `9Fh`",
        },
        PropertySpec {
            name: "fill",
            kind: ValueKind::Uint,
            required: false,
            summary: "the byte the array holds at power-on (default 0; silicon is indeterminate)",
        },
        PropertySpec {
            name: "tcem-cycles",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many serial clocks CS may stay low; 8 us is 672 at 84 MHz (0 = no check)",
        },
        PropertySpec {
            name: "tcem-check",
            kind: ValueKind::Str,
            required: false,
            summary: "what a tCEM violation does: \"off\", \"log\" (default) or \"fault\"",
        },
    ],
    construct: |props| Ok(Box::new(Psram::new(props)?)),
};

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Psram::new(props)?)))
}

/// What the validator should know about `psram.qspi`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, MAX_CHIP_SELECTS as u64 - 1))
        .prop(PropSchema::new("mode", ValueKind::Uint).range(0, 3))
        .prop(PropSchema::new("id", ValueKind::Str))
        .prop(PropSchema::new("eid", ValueKind::Uint))
        .prop(PropSchema::new("fill", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("tcem-cycles", ValueKind::Uint))
        .prop(PropSchema::new("tcem-check", ValueKind::Str))
        .port(spi_pin::SCK_NAME, PortDir::In)
        .port(spi_pin::MOSI_NAME, PortDir::In)
        .port(spi_pin::CS_NAME, PortDir::In)
        .port(spi_pin::MISO_NAME, PortDir::Out)
}

#[cfg(test)]
mod tests;
