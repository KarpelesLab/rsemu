//! NCR 53C710: a SCSI I/O Processor.
//!
//! The second initiator in `dev/scsi`'s split, and a very different animal from
//! [`crate::dev::wd33c93`]. A WD33C93A is a peripheral: the host writes a
//! command register and the chip does one SCSI step. A 53C710 is a **processor**
//! — it fetches its own instructions out of host memory, masters the bus for
//! both the fetch and the data, and only interrupts the host when its program
//! says to. A driver for it is two programs: one in 68000 code and one in
//! SCRIPTS.
//!
//! As with the WD33C93A, this file **contains no SCSI command opcode** — no
//! `INQUIRY`, no `READ(10)`, no sense key, no mode page. What a target does with
//! a command descriptor block is the target's business; this chip moves bytes in
//! the phase its program asked for and counts them.
//!
//! # The register file, and which end of it is up
//!
//! Sixty-four bytes. The data manual prints them as sixteen rows of four, and
//! the numbering it gives is the **little-endian** one: `SCNTL0` at `00`,
//! `SCNTL1` at `01`, `SDID` at `02`, `SIEN` at `03`, and the thirty-two-bit
//! registers — `DSA`, `TEMP`, `DNAD`, `DSP`, `DSPS`, `SCRATCH`, `ADDER` — with
//! their least significant byte at the lowest number.
//!
//! The chip has a **big-endian mode**, which is what a 68000-family board wires
//! it into, and in that mode the four byte lanes of every row swap: the eight-bit
//! register the manual numbers `n` answers at address `n XOR 3`, and a
//! thirty-two-bit register reads as a natural big-endian longword. That single
//! rule is [`Order`], it is the whole of the difference, and it is why an A4000T
//! driver polls `ISTAT` — manual number `21` — at `$00DD0062`, which is
//! `$00DD0040 + (0x21 ^ 3)`.
//!
//! Every constant in this file is a **manual number**. Nothing here knows what
//! address a board put the chip at.
//!
//! # SCRIPTS
//!
//! The processor fetches two longwords at [`DSP`] (three for a Memory Move),
//! puts the first byte in [`DCMD`] and the rest in [`DBC`], the second longword
//! in [`DSPS`], and executes. Bits 31–30 of `DCMD` choose between four
//! instruction types, and all four are implemented:
//!
//! * **Block Move** (`00`) — move `DBC` bytes between memory at [`DNAD`] and the
//!   SCSI bus in the phase bits 26–24 name. *Indirect* (bit 29) makes `DSPS` a
//!   pointer to the data address; *table indirect* (bit 28) makes it a signed
//!   offset from [`DSA`] to a count/address pair. A target asking for a
//!   different phase is a **phase mismatch**, which is how a SCSI transfer ends.
//! * **I/O** (`01`) — `Select`, `Wait Disconnect`, `Wait Reselect`, `Set` and
//!   `Clear` in opcodes 0 to 4, and the **register read/write** instructions in
//!   opcodes 6 and 7: an eight-bit ALU between a register, an immediate and
//!   [`SFBR`], with a carry, which is what lets a SCRIPTS program add 4 to a
//!   thirty-two-bit `SCRATCH` a byte at a time.
//! * **Transfer Control** (`10`) — `Jump`, `Call`, `Return` and `Interrupt`,
//!   each conditional on the phase, on [`SFBR`] compared under a mask, or on the
//!   carry, and addressed absolutely or relative to the instruction after.
//! * **Memory Move** (`11`) — three longwords: a count, a source and a
//!   destination, copied through the same address space everything else uses.
//!   A driver uses it to patch its own SCRIPTS and to load the chip's own
//!   registers, so the destination is quite often this very chip.
//!
//! # What is not implemented
//!
//! * **Target role.** `SCNTL0`'s `TRG` bit is stored and ignored, and the
//!   target-mode I/O opcodes (`Disconnect`, `Set`/`Clear` of `TARGET`) do
//!   nothing to the bus. This chip is an initiator; the targets in this tree are
//!   [`crate::dev::scsi::disk`] and friends.
//! * **Reselection.** No target here disconnects (see [`crate::dev::scsi`],
//!   which says so and says why), so `Wait Reselect` only ever leaves by its
//!   alternate address when the host sets `SIGP`. That is the path Commodore's
//!   driver actually uses, and it is exercised; a reselection that arrives from
//!   the cable is not.
//! * **Synchronous transfer.** `SXFER` is stored and reported back. Nothing here
//!   has a transfer *rate*, so a synchronous transfer would be the same bytes in
//!   the same order.
//! * **Parity, the DMA FIFO and the watchdog.** `PAR` is never set, because no
//!   byte on this bus was carried by a wire; `DSTAT`'s `DFE` is always set,
//!   because a transfer that completes inside one instruction never leaves the
//!   FIFO dirty; `DWT` is stored and never expires, because the bus it watches
//!   never stalls.
//! * **Single stepping.** `DCNTL`'s `SSM` is stored; the step interrupt `SSI`
//!   is raised when it is set, which is the whole of what a single step is here.
//!
//! # What a running ROM settled, and the manual to hand did not
//!
//! Five things, each of which was wrong here first and each of which stops a
//! machine booting on its own. They are commented where they are decided; this
//! is the index.
//!
//! * **Which byte of [`DSP`] starts the processor** — the one at the highest
//!   *address*, not a particular register number. See [`DSP_START`].
//! * **A selection's destination is a bus line**, not an encoded number. See
//!   `decode_id` and `Chip::select`.
//! * **Which half of a transfer control comparison is the value** — `DBC` bits
//!   7–0, with the mask in 15–8. See `Chip::condition`.
//! * **`ISTAT`'s `SIP` and `DIP` are levels**, computed from the status
//!   registers and their masks rather than latched. See `State::sip`.
//! * **`Wait Reselect` does not consume `SIGP`**; only a read of `CTEST2`
//!   does. See `Chip::wait_reselect`.
//!
//! `docs/platforms/amiga.md`, "A4000T", has the traces each was read out of.
//!
//! # Sources
//!
//! *NCR 53C710 SCSI I/O Processor Data Manual* — the register map and every
//! register bit, the interrupt model (`ISTAT`, `DSTAT`/`DIEN`, `SSTAT0`/`SIEN`),
//! the SCRIPTS instruction formats, and the big-endian mode. X3.131-1994 for
//! what the phases mean; see [`crate::dev::scsi`].
//!
//! **No emulator source of any licence was consulted, no FPGA reimplementation
//! was opened, and no Kickstart was disassembled** (`CLAUDE.md`, provenance).
//! Where this model answers a question the manual to hand does not settle, it
//! says so in a comment and names the black-box observation that settled it.

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::scsi::{self, Bus, Phase, Target, buses, message};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "ncr.53c710";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The `IRQ/` output pin.
pub const IRQ_PIN: &str = "irq";

/// The register window a board maps.
pub const REGS_REGION: &str = "regs";

/// How much the register file decodes: sixty-four bytes, and no more. A board
/// that answers a wider window says so with `mirror()`.
pub const REGS_WINDOW_LEN: u64 = 0x40;

// ---------------------------------------------------------------------------
// the register map, in the data manual's own numbering
// ---------------------------------------------------------------------------

/// SCSI control zero: the arbitration mode, `START`, `WATN`, the parity
/// enables and `TRG`.
pub const SCNTL0: usize = 0x00;
/// SCSI control one: `ADB`, `CON`, the `RST` the host asserts by hand, and the
/// low-level send and receive bits.
pub const SCNTL1: usize = 0x01;
/// SCSI destination ID, encoded.
pub const SDID: usize = 0x02;
/// SCSI interrupt enable: the mask over [`SSTAT0`].
pub const SIEN: usize = 0x03;
/// SCSI chip ID: this initiator's own address.
pub const SCID: usize = 0x04;
/// SCSI transfer: the synchronous period and offset.
pub const SXFER: usize = 0x05;
/// SCSI output data latch.
pub const SODL: usize = 0x06;
/// SCSI output control latch.
pub const SOCL: usize = 0x07;
/// SCSI first byte received — the accumulator every conditional transfer
/// control instruction compares against.
pub const SFBR: usize = 0x08;
/// SCSI input data latch.
pub const SIDL: usize = 0x09;
/// SCSI bus data lines, as they stand.
pub const SBDL: usize = 0x0a;
/// SCSI bus control lines on a read; the synchronous clock divisor on a write.
pub const SBCL: usize = 0x0b;
/// DMA status: the six DMA interrupt conditions and `DFE`.
pub const DSTAT: usize = 0x0c;
/// SCSI status zero: the eight SCSI interrupt conditions.
pub const SSTAT0: usize = 0x0d;
/// SCSI status one.
pub const SSTAT1: usize = 0x0e;
/// SCSI status two.
pub const SSTAT2: usize = 0x0f;
/// Data structure address: what a table-indirect instruction is relative to.
pub const DSA: usize = 0x10;
/// The first of the eight chip test registers.
pub const CTEST0: usize = 0x14;
/// Chip test two: `SIGP` is readable here, and reading it clears it.
pub const CTEST2: usize = 0x16;
/// Chip test seven.
pub const CTEST7: usize = 0x1b;
/// A scratch longword the chip uses for a Memory Move.
pub const TEMP: usize = 0x1c;
/// How many bytes are in the DMA FIFO.
pub const DFIFO: usize = 0x20;
/// Interrupt status: `ABRT`, `RST`, `SIGP`, `CON`, `SIP`, `DIP`. The one
/// register a host may touch while SCRIPTS is running.
pub const ISTAT: usize = 0x21;
/// Chip test eight.
pub const CTEST8: usize = 0x22;
/// The longitudinal parity byte.
pub const LCRC: usize = 0x23;
/// DMA byte count, three bytes: the low twenty-four bits of an instruction.
pub const DBC: usize = 0x24;
/// DMA command: the high byte of an instruction.
pub const DCMD: usize = 0x27;
/// DMA next address: where a Block Move's next byte goes.
pub const DNAD: usize = 0x28;
/// DMA SCRIPTS pointer: writing it starts the processor.
pub const DSP: usize = 0x2c;
/// DMA SCRIPTS pointer save: the second longword of an instruction.
pub const DSPS: usize = 0x30;
/// A longword of scratch the host and SCRIPTS share.
pub const SCRATCH: usize = 0x34;
/// DMA mode: the burst length, the function code and `MAN`.
pub const DMODE: usize = 0x38;
/// DMA interrupt enable: the mask over [`DSTAT`].
pub const DIEN: usize = 0x39;
/// DMA watchdog timer. Stored; it never expires here.
pub const DWT: usize = 0x3a;
/// DMA control: the clock divisor, `SSM`, `STD`, `IRQD` and `COM`.
pub const DCNTL: usize = 0x3b;
/// The adder's output, which a test program reads back.
pub const ADDER: usize = 0x3c;

/// The window offset whose write starts the processor: the **highest** address
/// of [`DSP`]'s four bytes, whichever end of the longword the lane order puts
/// there — which is the register the manual numbers `2C` in [`Order::Big`] and
/// `2F` in [`Order::Little`].
pub const DSP_START: u64 = 0x2f;

/// How many registers there are.
const REGS: usize = 0x40;

// -- ISTAT (data manual, "Interrupt Status") ---------------------------------

/// `ABRT`: abort the operation in progress.
pub const ISTAT_ABRT: u8 = 0x80;
/// `RST`: hold the chip in software reset.
pub const ISTAT_RST: u8 = 0x40;
/// `SIGP`: signal process — the host poking a waiting SCRIPTS program.
pub const ISTAT_SIGP: u8 = 0x20;
/// `CON`: the chip is connected to the SCSI bus.
pub const ISTAT_CON: u8 = 0x08;
/// `SIP`: a SCSI interrupt is pending in [`SSTAT0`].
pub const ISTAT_SIP: u8 = 0x02;
/// `DIP`: a DMA interrupt is pending in [`DSTAT`].
pub const ISTAT_DIP: u8 = 0x01;

// -- DSTAT and DIEN ----------------------------------------------------------

/// `DFE`: the DMA FIFO is empty. Status only — [`DIEN`] has no bit for it.
pub const DSTAT_DFE: u8 = 0x80;
/// `MDPE`: master data parity error.
pub const DSTAT_MDPE: u8 = 0x40;
/// `BF`: bus fault — an access the address space refused.
pub const DSTAT_BF: u8 = 0x20;
/// `ABRT`: the host wrote [`ISTAT_ABRT`].
pub const DSTAT_ABRT: u8 = 0x10;
/// `SSI`: a single step completed.
pub const DSTAT_SSI: u8 = 0x08;
/// `SIR`: a SCRIPTS `Interrupt` instruction executed; [`DSPS`] carries its
/// vector.
pub const DSTAT_SIR: u8 = 0x04;
/// `WTD`: the watchdog timer expired. Never set here.
pub const DSTAT_WTD: u8 = 0x02;
/// `IID`: an illegal instruction was detected.
pub const DSTAT_IID: u8 = 0x01;

// -- SSTAT0 and SIEN ---------------------------------------------------------

/// `M/A`: a phase mismatch — the target asked for a phase the instruction did
/// not, which is how a SCSI information transfer ends.
pub const SSTAT0_MA: u8 = 0x80;
/// `FCMP`: function complete — a selection finished.
pub const SSTAT0_FCMP: u8 = 0x40;
/// `STO`: selection or reselection timed out; nobody answered.
pub const SSTAT0_STO: u8 = 0x20;
/// `SEL`: this chip was selected or reselected as a target.
pub const SSTAT0_SEL: u8 = 0x10;
/// `SGE`: SCSI gross error.
pub const SSTAT0_SGE: u8 = 0x08;
/// `UDC`: an unexpected disconnect.
pub const SSTAT0_UDC: u8 = 0x04;
/// `RST`: `RST/` was seen on the cable — including this chip's own.
pub const SSTAT0_RST: u8 = 0x02;
/// `PAR`: a parity error. Never set here.
pub const SSTAT0_PAR: u8 = 0x01;

// -- the control registers ---------------------------------------------------

/// `SCNTL1`'s `RST`: drive `RST/` on the cable for as long as it is set.
pub const SCNTL1_RST: u8 = 0x08;
/// `SCNTL1`'s `CON`: connected.
pub const SCNTL1_CON: u8 = 0x10;
/// `CTEST2`'s copy of [`ISTAT_SIGP`]. Reading `CTEST2` clears it.
pub const CTEST2_SIGP: u8 = 0x40;
/// `DMODE`'s `MAN`: do not start on a [`DSP`] write; wait for `DCNTL`'s `STD`.
pub const DMODE_MAN: u8 = 0x01;
/// `DCNTL`'s `SSM`: single step — interrupt after every instruction.
pub const DCNTL_SSM: u8 = 0x10;
/// `DCNTL`'s `STD`: start the processor now.
pub const DCNTL_STD: u8 = 0x04;
/// `DCNTL`'s `IRQD`: hold the `IRQ/` pin off whatever the status says.
pub const DCNTL_IRQD: u8 = 0x02;

// -- the SCRIPTS instruction encoding ----------------------------------------

/// `DCMD` bits 31–30: which of the four instruction types this is.
pub const DCMD_TYPE: u8 = 0xc0;
/// A Block Move.
pub const TYPE_BLOCK_MOVE: u8 = 0x00;
/// An I/O or register read/write instruction.
pub const TYPE_IO: u8 = 0x40;
/// A transfer control instruction.
pub const TYPE_TRANSFER: u8 = 0x80;
/// A Memory Move.
pub const TYPE_MEMORY_MOVE: u8 = 0xc0;

/// Bits 26–24 of `DCMD`: the `MSG`, `C/D` and `I/O` a phase is made of.
pub const DCMD_PHASE: u8 = 0x07;
/// Block Move bit 29: `DSPS` points at the data address rather than being it.
pub const BLOCK_INDIRECT: u8 = 0x20;
/// Block Move bit 28: `DSPS` is a signed offset from [`DSA`] to a count and an
/// address.
pub const BLOCK_TABLE: u8 = 0x10;

/// Bits 29–27 of `DCMD` for an I/O or transfer control instruction.
pub const OPCODE: u8 = 0x38;
/// I/O opcode 0: arbitrate for the bus and select a target.
pub const IO_SELECT: u8 = 0x00;
/// I/O opcode 1: wait for the target to let go of the bus.
pub const IO_WAIT_DISCONNECT: u8 = 0x08;
/// I/O opcode 2: wait to be reselected, or for the host to set `SIGP`.
pub const IO_WAIT_RESELECT: u8 = 0x10;
/// I/O opcode 3: assert the signals named in `DBC`.
pub const IO_SET: u8 = 0x18;
/// I/O opcode 4: negate them.
pub const IO_CLEAR: u8 = 0x20;
/// I/O opcode 6: `SFBR := op(register, data)`.
pub const IO_TO_SFBR: u8 = 0x30;
/// I/O opcode 7: `register := op(register, data)`.
pub const IO_TO_REG: u8 = 0x38;
/// I/O opcode 5: `register := op(SFBR, data)`.
pub const IO_FROM_SFBR: u8 = 0x28;

/// Transfer control opcode 0.
pub const XFER_JUMP: u8 = 0x00;
/// Transfer control opcode 1: push the return address into [`TEMP`].
pub const XFER_CALL: u8 = 0x08;
/// Transfer control opcode 2: pop it.
pub const XFER_RETURN: u8 = 0x10;
/// Transfer control opcode 3: stop and interrupt the host, with the vector in
/// [`DSPS`].
pub const XFER_INTERRUPT: u8 = 0x18;

/// `Select`'s "with `ATN`" bit, `DCMD` bit 24.
pub const SELECT_ATN: u8 = 0x01;
/// `Select`'s table-indirect bit, `DCMD` bit 25: the identify byte and the
/// transfer parameters come from [`DSA`] plus `DBC`.
pub const SELECT_TABLE: u8 = 0x02;
/// `Select`'s and a transfer control instruction's relative addressing bit.
pub const ADDR_RELATIVE: u8 = 0x04;

/// `DBC` bit 23 on a transfer control instruction: the address in [`DSPS`] is
/// relative to the instruction after this one.
pub const DBC_RELATIVE: u32 = 0x0080_0000;
/// `DBC` bit 21: test the carry instead of anything else.
pub const DBC_CARRY: u32 = 0x0020_0000;
/// `DBC` bit 19: jump when the comparison is true rather than when it is false.
pub const DBC_TRUE: u32 = 0x0008_0000;
/// `DBC` bit 18: compare [`SFBR`] with the data and mask in the low sixteen
/// bits.
pub const DBC_COMPARE_DATA: u32 = 0x0004_0000;
/// `DBC` bit 17: compare the SCSI phase with `DCMD` bits 26–24.
pub const DBC_COMPARE_PHASE: u32 = 0x0002_0000;
/// `DBC` bit 16: wait for a valid phase before comparing.
pub const DBC_WAIT_PHASE: u32 = 0x0001_0000;

/// `Set`/`Clear`'s `ATN` bit.
pub const SET_ATN: u32 = 0x0000_0008;
/// `Set`/`Clear`'s `ACK` bit.
pub const SET_ACK: u32 = 0x0000_0040;
/// `Set`/`Clear`'s target-mode bit. Stored and ignored.
pub const SET_TARGET: u32 = 0x0000_0200;
/// `Set`/`Clear`'s carry bit.
pub const SET_CARRY: u32 = 0x0000_0400;

/// How many instructions one start may execute before the model calls the
/// program a runaway.
///
/// A real processor has no such limit; it would loop until the host stopped it,
/// and a host with a stopped processor has other things wrong with it. An
/// emulator has to bound the work a single register write can do, so a program
/// that neither interrupts, waits, nor mismatches inside this many instructions
/// is stopped with `IID` — which is the manual's "the chip could not carry on
/// with this program" and the only interrupt whose meaning fits.
const MAX_STEPS: u32 = 1_000_000;

/// How many bytes a Block Move crosses at a time. A `READ(10)` of 32 MiB should
/// not become a 32 MiB allocation, and a bus tenure was never the whole
/// transfer anyway.
const BURST: usize = 4096;

// ---------------------------------------------------------------------------
// byte order
// ---------------------------------------------------------------------------

/// Which way round the four byte lanes of the register file are.
///
/// The chip has a mode pin for this and a 68000-family board wires it to
/// [`Order::Big`], which is why every constant above is a *manual* number and
/// the translation lives in one function.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Order {
    /// The data manual's own numbering: `SCNTL0` at `00`, and a thirty-two-bit
    /// register's least significant byte at its lowest address.
    Little,
    /// The lanes of every row swapped, so the register the manual numbers `n`
    /// answers at `n XOR 3` and a longword reads as a big-endian longword.
    Big,
}

impl Order {
    /// The manual's register number an access at `offset` in the window lands
    /// on.
    #[must_use]
    #[inline]
    pub const fn register(self, offset: u64) -> usize {
        let at = (offset & 0x3f) as usize;
        match self {
            Order::Little => at,
            Order::Big => at ^ 3,
        }
    }

    /// The name a machine file writes.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Order::Little => "little",
            Order::Big => "big",
        }
    }
}

/// A phase from the three `MSG`, `C/D`, `I/O` bits of a `DCMD`.
///
/// The two combinations X3.131-1994 §5.1 reserves are not phases, and an
/// instruction that names one is an illegal instruction rather than a near
/// miss.
const fn phase_of(mci: u8) -> Option<Phase> {
    match mci & 7 {
        0b000 => Some(Phase::DataOut),
        0b001 => Some(Phase::DataIn),
        0b010 => Some(Phase::Command),
        0b011 => Some(Phase::Status),
        0b110 => Some(Phase::MessageOut),
        0b111 => Some(Phase::MessageIn),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// the chip's state
// ---------------------------------------------------------------------------

/// What the SCRIPTS processor is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Run {
    /// Stopped, whether because it has never started or because it
    /// interrupted.
    Idle,
    /// Executing. Only ever observed from inside the processor itself.
    Running,
    /// Parked on a `Wait Reselect` at [`DSP`], which `SIGP` or a reselection
    /// will leave.
    WaitingReselect,
}

#[derive(Debug)]
struct State {
    /// The sixty-four registers, in the manual's numbering. A longword register
    /// is held little-endian inside its four, which is what makes
    /// [`Order::register`] the whole of the byte-order difference.
    regs: [u8; REGS],
    /// Where the processor is.
    run: Run,
    /// Connected as an initiator, and to whom.
    connected: bool,
    selected_id: u8,
    /// `ATN` asserted.
    atn: bool,
    /// The carry the register read/write instructions produce and the transfer
    /// control instructions test.
    carry: bool,
    /// The chip's own reset address, kept across a software reset.
    own_id: u8,
}

impl State {
    fn new(own_id: u8) -> State {
        let mut state = State {
            regs: [0; REGS],
            run: Run::Idle,
            connected: false,
            selected_id: 0,
            atn: false,
            carry: false,
            own_id,
        };
        state.reset_regs();
        state
    }

    /// Power-on and software-reset values. The manual's reset column is zero
    /// for everything but `DSTAT`, whose `DFE` says the FIFO is empty — which
    /// on a chip with no FIFO of its own it always is.
    fn reset_regs(&mut self) {
        self.regs = [0; REGS];
        self.regs[DSTAT] = DSTAT_DFE;
        // The chip's own address, as the bus carries one: a data line, not a
        // number. Black box, and the same reading the `Select` instruction's
        // destination field needs — an A4000T's Kickstart writes `SCID := $01`
        // and then scans addresses 1 to 7, which is an adapter at address 0
        // skipping itself.
        self.regs[SCID] = 1 << (self.own_id & 0x07);
    }

    fn long(&self, at: usize) -> u32 {
        u32::from_le_bytes([
            self.regs[at],
            self.regs[at + 1],
            self.regs[at + 2],
            self.regs[at + 3],
        ])
    }

    fn set_long(&mut self, at: usize, value: u32) {
        self.regs[at..at + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn set_dbc(&mut self, value: u32) {
        let bytes = value.to_le_bytes();
        self.regs[DBC..DBC + 3].copy_from_slice(&bytes[..3]);
    }

    /// Whether a SCSI condition that [`SIEN`] enables is standing.
    ///
    /// A **level**, recomputed from the two registers, not a bit set once when
    /// something happened. That is what the manual's summary bit is, and the
    /// difference is a boot: an A4000T's Kickstart asserts `RST/` through
    /// `SCNTL1` and only *then* writes `SIEN`, and it is waiting for the
    /// interrupt that unmasking the `RST` already in `SSTAT0` produces. Latch
    /// `SIP` at the moment of the event instead and that interrupt never comes,
    /// the driver polls `ISTAT` for ever and the machine never gets a boot
    /// device.
    fn sip(&self) -> bool {
        self.regs[SSTAT0] & self.regs[SIEN] != 0
    }

    /// The same for a DMA condition and [`DIEN`]. `DSTAT`'s `DFE` is not in it
    /// because `DIEN` has no bit opposite it — the manual's own way of saying
    /// that a level is not an interrupt.
    fn dip(&self) -> bool {
        self.regs[DSTAT] & self.regs[DIEN] & !DSTAT_DFE != 0
    }

    /// `ISTAT` as a read returns it: the bits the host owns, plus the two
    /// summary levels.
    fn istat(&self) -> u8 {
        (self.regs[ISTAT] & !(ISTAT_SIP | ISTAT_DIP))
            | if self.sip() { ISTAT_SIP } else { 0 }
            | if self.dip() { ISTAT_DIP } else { 0 }
    }

    /// Whether anything enabled is asking for the pin.
    fn interrupting(&self) -> bool {
        (self.sip() || self.dip()) && self.regs[DCNTL] & DCNTL_IRQD == 0
    }
}

// ---------------------------------------------------------------------------
// the chip
// ---------------------------------------------------------------------------

/// The shared half of the device: the registers, the cable, the memory it
/// masters and the pin.
struct Chip {
    state: Mutex<State>,
    order: Order,
    bus: Arc<Bus>,
    bus_name: String,
    /// The memory SCRIPTS fetches from and DMA moves through, weakly so the
    /// space may own this device without a cycle.
    space: Mutex<Option<Weak<AddressSpace>>>,
    requester: Mutex<RequesterId>,
    irq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Chip {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Chip")
            .field("bus", &self.bus_name)
            .field("order", &self.order)
            .field("state", &*self.state.lock())
            .finish_non_exhaustive()
    }
}

/// Why the processor stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stop {
    /// It ran out of program to run without being told to stop: an interrupt
    /// was raised and the host has it.
    Interrupted,
    /// It parked on a `Wait Reselect`.
    Parked,
}

impl Chip {
    // -- the host interface --------------------------------------------------

    /// One byte out of the register file.
    ///
    /// `debug` suppresses every read side effect — the two status registers
    /// clearing their interrupt, and `CTEST2` clearing `SIGP` — which is the
    /// whole of [`MemAttrs::debug`] for this chip: a monitor that read `DSTAT`
    /// would take the interrupt the guest's handler is about to look for.
    fn read_reg(&self, reg: usize, debug: bool) -> u8 {
        let mut state = self.state.lock();
        let value = match reg {
            // The manual: reading a status register clears the conditions it
            // reports and, with nothing left, the summary bit in `ISTAT`.
            DSTAT | SSTAT0 => {
                let value = state.regs[reg];
                if !debug {
                    // The manual: reading a status register clears the
                    // conditions it reports, and the summary level in `ISTAT`
                    // goes with them because it is computed from these. `DFE`
                    // is a level, not an event: it says the FIFO is empty and
                    // it stays said.
                    state.regs[reg] = if reg == DSTAT { DSTAT_DFE } else { 0 };
                }
                value
            }
            ISTAT => state.istat(),
            // `CTEST2` bit 6 is `SIGP`, and reading it is how a SCRIPTS program
            // consumes the host's signal.
            CTEST2 => {
                let sigp = state.regs[ISTAT] & ISTAT_SIGP != 0;
                let value =
                    (state.regs[CTEST2] & !CTEST2_SIGP) | if sigp { CTEST2_SIGP } else { 0 };
                if !debug {
                    state.regs[ISTAT] &= !ISTAT_SIGP;
                }
                value
            }
            // `SBCL` reads the cable rather than the latch a write to it loads.
            SBCL => {
                let connected = state.connected;
                drop(state);
                return self.bus_control(connected);
            }
            _ => state.regs[reg],
        };
        drop(state);
        if !debug {
            self.refresh();
        }
        value
    }

    /// The `MSG`, `C/D` and `I/O` the target is driving, in `SBCL`'s own bit
    /// order (bits 2–0), plus `BSY` (bit 5) while a target is on the bus.
    fn bus_control(&self, connected: bool) -> u8 {
        let id = self.state.lock().selected_id;
        let phase = if connected {
            self.bus.target(id).map_or(Phase::BusFree, |t| t.phase())
        } else {
            Phase::BusFree
        };
        match phase.mci() {
            Some(mci) => 0x20 | mci,
            None => 0,
        }
    }

    /// One byte into the register file.
    fn write_reg(&self, reg: usize, value: u8) {
        match reg {
            ISTAT => self.write_istat(value),
            SCNTL1 => self.write_scntl1(value),
            DCNTL => {
                let start = {
                    let mut state = self.state.lock();
                    state.regs[DCNTL] = value & !DCNTL_STD;
                    value & DCNTL_STD != 0 && state.run == Run::Idle
                };
                self.refresh();
                if start {
                    self.start();
                }
            }
            // Every other register is a latch, including the ones this model
            // stores and ignores. The manual's read-only registers are read-only
            // to *software*; nothing here is harmed by letting a driver's
            // read-modify-write of a reserved byte land.
            _ => {
                self.state.lock().regs[reg] = value;
            }
        }
    }

    /// One byte through the *board's window*, which is where the processor's
    /// start trigger lives.
    ///
    /// The manual starts the processor when the host finishes writing [`DSP`],
    /// and a 68000 finishes it with the byte at the **highest address** — which
    /// in [`Order::Big`] is the register the manual numbers `2C`, the longword's
    /// least significant byte, and not `2F`. Deciding it from the register
    /// number rather than the address is a defect that costs the whole boot:
    /// an A4000T's Kickstart writes `DSP` as two words, high half first, so a
    /// chip that started on register `2F` would start with three quarters of
    /// the address still missing and run whatever is at `$07000000`.
    fn write_window(&self, offset: u64, value: u8) {
        self.write_reg(self.order.register(offset), value);
        if offset & 0x3f != DSP_START {
            return;
        }
        let start = {
            let state = self.state.lock();
            state.run == Run::Idle && state.regs[DMODE] & DMODE_MAN == 0
        };
        if start {
            self.start();
        }
    }

    /// `ISTAT`, which is the only register with more than a latch behind it.
    fn write_istat(&self, value: u8) {
        let mut state = self.state.lock();
        // Bits the host owns.
        let keep = state.regs[ISTAT] & ISTAT_CON;
        state.regs[ISTAT] = (value & (ISTAT_ABRT | ISTAT_RST | ISTAT_SIGP)) | keep;
        if value & ISTAT_RST != 0 {
            // A software reset: everything but this bit and the chip's own
            // strapped address goes back to power-on, and the chip stays there
            // until the host writes the bit away.
            let own = state.own_id;
            let mut fresh = State::new(own);
            fresh.regs[ISTAT] = ISTAT_RST;
            *state = fresh;
            drop(state);
            self.refresh();
            return;
        }
        if value & ISTAT_ABRT != 0 {
            // "Abort the operation in progress": the processor stops where it
            // is and the host is told, if it asked to be.
            state.run = Run::Idle;
            state.regs[ISTAT] &= !ISTAT_ABRT;
            Chip::raise_dma(&mut state, DSTAT_ABRT);
            drop(state);
            self.refresh();
            return;
        }
        let resume = state.run == Run::WaitingReselect && value & ISTAT_SIGP != 0;
        drop(state);
        self.refresh();
        if resume {
            // `SIGP` is exactly how a host gets a parked program moving again.
            self.start();
        }
    }

    /// `SCNTL1`, whose `RST` bit drives `RST/` on the cable for as long as it
    /// is set.
    fn write_scntl1(&self, value: u8) {
        let pulse = {
            let mut state = self.state.lock();
            let was = state.regs[SCNTL1];
            state.regs[SCNTL1] = value;
            was & SCNTL1_RST == 0 && value & SCNTL1_RST != 0
        };
        if !pulse {
            return;
        }
        // Nothing of this chip's is held while the cable is reset: every target
        // takes its own lock.
        self.bus.reset();
        let mut state = self.state.lock();
        state.connected = false;
        state.regs[ISTAT] &= !ISTAT_CON;
        // The manual: `RST/` sets `SSTAT0`'s `RST` whoever asserted it, and this
        // chip's own assertion is no exception.
        Chip::raise_scsi(&mut state, SSTAT0_RST);
        drop(state);
        self.refresh();
    }

    /// Record a DMA condition.
    ///
    /// Whether it *interrupts* is [`State::dip`]'s to say, because the mask may
    /// change afterwards. A masked condition is recorded and does not
    /// interrupt, which is what lets a driver poll `DSTAT` for something it did
    /// not want a pin for.
    fn raise_dma(state: &mut State, bit: u8) {
        state.regs[DSTAT] |= bit;
    }

    /// The same for a SCSI condition.
    fn raise_scsi(state: &mut State, bit: u8) {
        state.regs[SSTAT0] |= bit;
    }

    /// Drive `IRQ/` from the registers, with nothing held while the net
    /// delivers.
    fn refresh(&self) {
        let high = self.state.lock().interrupting();
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(high));
        }
    }

    // -- mastering memory ----------------------------------------------------

    /// The space and the attributes a bus tenure uses, if a board gave this
    /// chip one.
    fn memory(&self) -> Option<(Arc<AddressSpace>, MemAttrs)> {
        let space = self.space.lock().clone()?.upgrade()?;
        let requester = *self.requester.lock();
        Some((space, MemAttrs::DEFAULT.with_requester(requester)))
    }

    /// Fetch a longword, big-endian, the way the processor fetches an
    /// instruction.
    ///
    /// `None` is a bus fault, which the caller turns into `DSTAT`'s `BF`.
    /// **No lock of this chip's is held**: the address may be this chip's own
    /// register file, which a driver patching its own program relies on.
    fn fetch(&self, at: u32) -> Option<u32> {
        let (space, attrs) = self.memory()?;
        let mut buf = [0u8; 4];
        space.read_bytes(u64::from(at), &mut buf, attrs).ok()?;
        Some(u32::from_be_bytes(buf))
    }

    fn read_mem(&self, at: u32, dst: &mut [u8]) -> bool {
        let Some((space, attrs)) = self.memory() else {
            return false;
        };
        space.read_bytes(u64::from(at), dst, attrs).is_ok()
    }

    fn write_mem(&self, at: u32, src: &[u8]) -> bool {
        let Some((space, attrs)) = self.memory() else {
            return false;
        };
        space.write_bytes(u64::from(at), src, attrs).is_ok()
    }

    // -- the SCRIPTS processor -----------------------------------------------

    /// Run from [`DSP`] until something stops the processor.
    fn start(&self) {
        {
            let mut state = self.state.lock();
            if state.run == Run::Running {
                // A Memory Move whose destination was this chip's own `DSP`.
                // The manual leaves that undefined; here it lands in the
                // register and the instruction after it is fetched from the new
                // address, which is what a driver that does it wants.
                return;
            }
            state.run = Run::Running;
        }
        let stop = self.execute();
        let mut state = self.state.lock();
        state.run = match stop {
            Stop::Parked => Run::WaitingReselect,
            Stop::Interrupted => Run::Idle,
        };
        drop(state);
        self.refresh();
    }

    /// The instruction loop.
    fn execute(&self) -> Stop {
        for _ in 0..MAX_STEPS {
            let at = self.state.lock().long(DSP);
            let (Some(first), Some(second)) = (self.fetch(at), self.fetch(at.wrapping_add(4)))
            else {
                self.bus_fault();
                return Stop::Interrupted;
            };
            {
                let mut state = self.state.lock();
                state.regs[DCMD] = (first >> 24) as u8;
                state.set_dbc(first & 0x00ff_ffff);
                state.set_long(DSPS, second);
                state.set_long(DSP, at.wrapping_add(8));
            }
            let dcmd = (first >> 24) as u8;
            let step = match dcmd & DCMD_TYPE {
                TYPE_BLOCK_MOVE => self.block_move(dcmd, first & 0x00ff_ffff, second),
                TYPE_IO => self.io(dcmd, first & 0x00ff_ffff, second),
                TYPE_TRANSFER => self.transfer(dcmd, first & 0x00ff_ffff, second),
                _ => self.memory_move(first & 0x00ff_ffff, second, at.wrapping_add(8)),
            };
            match step {
                Step::Next => {}
                Step::Stop(stop) => return stop,
            }
            // `SSM`: the manual stops after each instruction so a debugger can
            // look. A driver that sets it gets one instruction per start.
            let single = {
                let mut state = self.state.lock();
                if state.regs[DCNTL] & DCNTL_SSM != 0 {
                    Chip::raise_dma(&mut state, DSTAT_SSI);
                    true
                } else {
                    false
                }
            };
            if single {
                return Stop::Interrupted;
            }
        }
        // See [`MAX_STEPS`].
        let mut state = self.state.lock();
        Chip::raise_dma(&mut state, DSTAT_IID);
        Stop::Interrupted
    }

    /// An access the address space refused.
    fn bus_fault(&self) {
        let mut state = self.state.lock();
        Chip::raise_dma(&mut state, DSTAT_BF);
    }

    /// An instruction this processor cannot make sense of.
    fn illegal(&self) -> Step {
        let mut state = self.state.lock();
        Chip::raise_dma(&mut state, DSTAT_IID);
        Step::Stop(Stop::Interrupted)
    }

    /// The target this chip is connected to, if it is.
    fn connected_target(&self) -> Option<Arc<dyn Target>> {
        let (connected, id) = {
            let state = self.state.lock();
            (state.connected, state.selected_id)
        };
        connected.then(|| self.bus.target(id)).flatten()
    }

    /// An unexpected bus free: the target went away mid-operation.
    fn unexpected_disconnect(&self) -> Step {
        let mut state = self.state.lock();
        state.connected = false;
        state.regs[ISTAT] &= !ISTAT_CON;
        Chip::raise_scsi(&mut state, SSTAT0_UDC);
        Step::Stop(Stop::Interrupted)
    }

    // -- Block Move ----------------------------------------------------------

    /// Move `DBC` bytes between memory and the cable in the phase `DCMD` names.
    fn block_move(&self, dcmd: u8, dbc: u32, dsps: u32) -> Step {
        let Some(want) = phase_of(dcmd & DCMD_PHASE) else {
            return self.illegal();
        };
        // Where the bytes are and how many, which three encodings answer.
        let (count, addr) = if dcmd & BLOCK_TABLE != 0 {
            // Table indirect: `DSPS` is a signed offset from `DSA` to a
            // count/address pair. Black box: Commodore's driver puts the offset
            // in `DSPS` with `DBC` zero, and the pair it points at is a
            // twenty-four-bit count in the first longword and the address in the
            // second.
            let dsa = self.state.lock().long(DSA);
            let at = dsa.wrapping_add(dsps);
            let (Some(c), Some(a)) = (self.fetch(at), self.fetch(at.wrapping_add(4))) else {
                self.bus_fault();
                return Step::Stop(Stop::Interrupted);
            };
            (c & 0x00ff_ffff, a)
        } else if dcmd & BLOCK_INDIRECT != 0 {
            let Some(a) = self.fetch(dsps) else {
                self.bus_fault();
                return Step::Stop(Stop::Interrupted);
            };
            (dbc, a)
        } else {
            (dbc, dsps)
        };
        {
            let mut state = self.state.lock();
            state.set_dbc(count);
            state.set_long(DNAD, addr);
        }

        let Some(target) = self.connected_target() else {
            return self.unexpected_disconnect();
        };
        let phase = target.phase();
        if phase == Phase::BusFree {
            return self.unexpected_disconnect();
        }
        if phase != want {
            // The manual's phase mismatch: the chip stops with `M/A`, `DSP`
            // pointing at the instruction after this one, and `DBC`/`DNAD`
            // holding what is left. That is how *every* SCSI transfer of an
            // unknown length ends, so it is a normal event and not an error.
            let mut state = self.state.lock();
            Chip::raise_scsi(&mut state, SSTAT0_MA);
            return Step::Stop(Stop::Interrupted);
        }
        let moved = self.pump(&target, want.is_input(), count);
        {
            let mut state = self.state.lock();
            state.set_dbc(count - moved);
            state.set_long(DNAD, addr.wrapping_add(moved));
        }
        if moved < count {
            // The phase ended before the count did, which is the same mismatch
            // by another route: the target had less than the program asked for.
            let now = target.phase();
            if now == Phase::BusFree {
                return self.unexpected_disconnect();
            }
            let mut state = self.state.lock();
            Chip::raise_scsi(&mut state, SSTAT0_MA);
            return Step::Stop(Stop::Interrupted);
        }
        Step::Next
    }

    /// Move `count` bytes, and say how many went.
    ///
    /// **No lock of this chip's is held**: the space may be this chip's own
    /// register file and the target takes its own lock.
    fn pump(&self, target: &Arc<dyn Target>, input: bool, count: u32) -> u32 {
        let was = target.phase();
        let mut at = self.state.lock().long(DNAD);
        let mut moved = 0u32;
        let mut first = true;
        while moved < count {
            if target.phase() != was {
                break;
            }
            let want = (count - moved).min(BURST as u32) as usize;
            let mut buf = alloc::vec![0u8; want];
            let n = if input {
                let got = target.read(&mut buf);
                if got == 0 {
                    break;
                }
                if first {
                    // The manual: the first byte of every inbound Block Move is
                    // latched in `SFBR`, which is what makes a one-byte
                    // `MESSAGE IN` followed by a compare the idiom it is.
                    self.state.lock().regs[SFBR] = buf[0];
                }
                if !self.write_mem(at, &buf[..got]) {
                    self.bus_fault();
                    break;
                }
                got
            } else {
                if !self.read_mem(at, &mut buf) {
                    self.bus_fault();
                    break;
                }
                let took = target.write(&buf);
                if took == 0 {
                    break;
                }
                took
            };
            first = false;
            at = at.wrapping_add(n as u32);
            moved += n as u32;
        }
        moved
    }

    // -- the I/O and register instructions -----------------------------------

    fn io(&self, dcmd: u8, dbc: u32, dsps: u32) -> Step {
        match dcmd & OPCODE {
            IO_SELECT => self.select(dcmd, dbc, dsps),
            IO_WAIT_DISCONNECT => self.wait_disconnect(),
            IO_WAIT_RESELECT => self.wait_reselect(dcmd, dsps),
            IO_SET => self.set_clear(dbc, true),
            IO_CLEAR => self.set_clear(dbc, false),
            IO_FROM_SFBR | IO_TO_SFBR | IO_TO_REG => self.register_op(dcmd, dbc),
            _ => self.illegal(),
        }
    }

    /// Arbitrate and select. On nobody answering, the manual jumps to `DSPS`
    /// *and* raises a timeout, which is what lets a driver both retry and know.
    fn select(&self, dcmd: u8, dbc: u32, dsps: u32) -> Step {
        let atn = dcmd & SELECT_ATN != 0;
        // Bits 23–16 are the destination, in both forms: in the instruction
        // itself, or in the longword at `DSA + DBC` when bit 25 asks for it.
        //
        // **Black box, and it decides which drive answers.** Commodore's
        // `scsi.device` scans an A4000T's cable with the seven entries
        // `$0002_0000`, `$0004_0000`, `$0008_0000`, `$0010_0000`,
        // `$0020_0000`, `$0040_0000` and `$0080_0000` — one bit walking up
        // bits 23–16, and seven of them rather than eight because a host
        // adapter does not select itself. That is the *bus line*, which is how
        // a selection carries an address (X3.131-1994 §5.1.3.2): the initiator
        // asserts its own data line and the target's. Read as an encoded
        // number instead and every one of those seven selects address 0, which
        // is what this model did — the machine then found the same drive seven
        // times over, mounted its partitions seven times, and AmigaDOS asked
        // for the volume back because there were seven of it.
        //
        // The entry's other three bytes are zero in everything that ROM
        // writes, so what they carry is not established here and nothing is
        // taken from them — in particular `SXFER` is left as the host set it.
        let field = if dcmd & SELECT_TABLE != 0 {
            let dsa = self.state.lock().long(DSA);
            let Some(entry) = self.fetch(dsa.wrapping_add(dbc)) else {
                self.bus_fault();
                return Step::Stop(Stop::Interrupted);
            };
            ((entry >> 16) & 0xff) as u8
        } else {
            ((dbc >> 16) & 0xff) as u8
        };
        let id = decode_id(field);
        self.state.lock().regs[SDID] = id;

        let answered = self.bus.target(id).is_some_and(|target| target.select(atn));
        if !answered {
            let mut state = self.state.lock();
            state.connected = false;
            state.regs[ISTAT] &= !ISTAT_CON;
            state.atn = false;
            Chip::raise_scsi(&mut state, SSTAT0_STO);
            // The manual's "jump on failure": the program gets to decide what a
            // missing target means, and does so *after* the interrupt is
            // recorded — a masked `STO` leaves the program running down its own
            // alternate path, which is how a driver scans a bus.
            let jump = self.address(dcmd & ADDR_RELATIVE != 0, dsps, state.long(DSP));
            state.set_long(DSP, jump);
            let stop = state.sip();
            drop(state);
            return if stop {
                Step::Stop(Stop::Interrupted)
            } else {
                Step::Next
            };
        }
        let mut state = self.state.lock();
        state.connected = true;
        state.selected_id = id;
        state.atn = atn;
        state.regs[ISTAT] |= ISTAT_CON;
        state.regs[SCNTL1] |= SCNTL1_CON;
        Chip::raise_scsi(&mut state, SSTAT0_FCMP);
        let stop = state.sip();
        drop(state);
        if stop {
            Step::Stop(Stop::Interrupted)
        } else {
            Step::Next
        }
    }

    /// Wait for the bus to go free.
    ///
    /// No target in this tree disconnects in the middle of a command, so by the
    /// time a program reaches this instruction the target has already let go —
    /// and if it has not, letting go is what the instruction asks for.
    fn wait_disconnect(&self) -> Step {
        if let Some(target) = self.connected_target()
            && target.phase() != Phase::BusFree
        {
            target.release();
        }
        let mut state = self.state.lock();
        state.connected = false;
        state.atn = false;
        state.regs[ISTAT] &= !ISTAT_CON;
        state.regs[SCNTL1] &= !SCNTL1_CON;
        Step::Next
    }

    /// Wait to be reselected, or for `SIGP`.
    ///
    /// Taking the alternate address **does not clear `SIGP`**. The only thing
    /// that clears it is a read of `CTEST2`, and the reason matters: Commodore's
    /// driver leaves this instruction and immediately does
    /// `SFBR := CTEST2 AND $40`, which is the program asking *why* it woke up.
    /// A model that consumed the signal here hands it `$00`, the program
    /// concludes nothing happened and interrupts with its "I was woken for
    /// nothing" vector, and the driver resets the chip and starts again — which
    /// is exactly what this one did until the trace said so.
    fn wait_reselect(&self, dcmd: u8, dsps: u32) -> Step {
        let mut state = self.state.lock();
        if state.regs[ISTAT] & ISTAT_SIGP != 0 {
            let here = state.long(DSP);
            let jump = self.address(dcmd & ADDR_RELATIVE != 0, dsps, here);
            state.set_long(DSP, jump);
            return Step::Next;
        }
        // Park *on this instruction*, so that the `SIGP` the host is about to
        // write finds the program where it left it.
        let here = state.long(DSP).wrapping_sub(8);
        state.set_long(DSP, here);
        Step::Stop(Stop::Parked)
    }

    /// `Set` and `Clear`.
    fn set_clear(&self, dbc: u32, set: bool) -> Step {
        if dbc & SET_ATN != 0 {
            self.state.lock().atn = set;
            // `ATN` is an initiator's "I have a message"; the targets here take
            // their message-out phase from the selection's `ATN`, so a later
            // change has nothing to change. Recorded so a snapshot round-trips.
        }
        if dbc & SET_ACK != 0 {
            // `ACK` is a handshake wire. Every transfer in this model completes
            // its handshakes inside the instruction that started it, so there
            // is never an `ACK` left asserted for this to let go of.
        }
        if dbc & SET_CARRY != 0 {
            self.state.lock().carry = set;
        }
        // `SET_TARGET` puts the chip in target mode, which this model does not
        // have. Stored in `SCNTL0` by whoever wrote it there; ignored here.
        Step::Next
    }

    /// The eight-bit ALU between a register, an immediate and `SFBR`.
    ///
    /// `DCMD` bits 26–24 are the manual's eight operations. The two that add
    /// are what makes four of these instructions a thirty-two-bit add: the
    /// A4000T's Kickstart adds 4 to the low byte of `SCRATCH` and then adds
    /// zero *with carry* to each of the three above it.
    fn register_op(&self, dcmd: u8, dbc: u32) -> Step {
        let reg = ((dbc >> 16) & 0x3f) as usize;
        let data = ((dbc >> 8) & 0xff) as u8;
        let op = dcmd & 7;

        // The source register is read through `read_reg`, so `CTEST2`'s `SIGP`
        // and the two status registers behave for SCRIPTS exactly as they do
        // for the host — which is the point of `CTEST2`.
        let source = match dcmd & OPCODE {
            IO_FROM_SFBR => self.state.lock().regs[SFBR],
            _ => self.read_reg(reg, false),
        };
        let carry_in = self.carry();
        let (value, carry) = match op {
            // Move: the immediate wins outright, which is how a program loads a
            // register with a constant.
            0 => (data, carry_in),
            1 => (source << 1, source & 0x80 != 0),
            2 => (source | data, carry_in),
            3 => (source ^ data, carry_in),
            4 => (source & data, carry_in),
            5 => (source >> 1, source & 1 != 0),
            _ => {
                let sum = u16::from(source) + u16::from(data) + u16::from(op == 7 && carry_in);
                ((sum & 0xff) as u8, sum > 0xff)
            }
        };
        self.state.lock().carry = carry;
        match dcmd & OPCODE {
            IO_TO_SFBR => {
                self.state.lock().regs[SFBR] = value;
            }
            _ => self.write_reg(reg, value),
        }
        Step::Next
    }

    fn carry(&self) -> bool {
        self.state.lock().carry
    }

    // -- transfer control ----------------------------------------------------

    /// Where a jump goes: `DSPS`, or `DSPS` added to the address after this
    /// instruction.
    fn address(&self, relative: bool, dsps: u32, after: u32) -> u32 {
        if relative {
            // `DSPS` is a signed displacement; two's complement makes a
            // wrapping add the whole of the sign extension.
            after.wrapping_add(dsps)
        } else {
            dsps
        }
    }

    fn transfer(&self, dcmd: u8, dbc: u32, dsps: u32) -> Step {
        let taken = self.condition(dcmd, dbc);
        if !taken {
            return Step::Next;
        }
        let after = self.state.lock().long(DSP);
        let relative = dbc & DBC_RELATIVE != 0;
        match dcmd & OPCODE {
            XFER_JUMP => {
                let to = self.address(relative, dsps, after);
                self.state.lock().set_long(DSP, to);
                Step::Next
            }
            XFER_CALL => {
                let to = self.address(relative, dsps, after);
                let mut state = self.state.lock();
                // The manual keeps one return address, in `TEMP`. One deep is
                // the whole of the call stack a 53C710 has.
                state.set_long(TEMP, after);
                state.set_long(DSP, to);
                Step::Next
            }
            XFER_RETURN => {
                let mut state = self.state.lock();
                let to = state.long(TEMP);
                state.set_long(DSP, to);
                Step::Next
            }
            _ => {
                // `Interrupt`: stop, and hand the host the vector in `DSPS`.
                let mut state = self.state.lock();
                Chip::raise_dma(&mut state, DSTAT_SIR);
                Step::Stop(Stop::Interrupted)
            }
        }
    }

    /// Whether a transfer control instruction's condition holds.
    ///
    /// With no comparison asked for the condition is *true*, so `DBC` bit 19
    /// alone is an unconditional jump and a bare instruction with neither is a
    /// no-operation — which is exactly how a driver patches a jump on and off
    /// by rewriting one byte of it.
    fn condition(&self, dcmd: u8, dbc: u32) -> bool {
        if dbc & DBC_CARRY != 0 {
            return self.carry() == (dbc & DBC_TRUE != 0);
        }
        let mut met = true;
        if dbc & DBC_COMPARE_PHASE != 0 {
            let Some(want) = phase_of(dcmd & DCMD_PHASE) else {
                return false;
            };
            let phase = self
                .connected_target()
                .map_or(Phase::BusFree, |t| t.phase());
            met &= phase == want;
        }
        if dbc & DBC_COMPARE_DATA != 0 {
            let sfbr = self.state.lock().regs[SFBR];
            // **Black box.** The A4000T's Kickstart fixes which half of the low
            // sixteen bits is the value: its idle loop is
            //
            // ```text
            //   74 16 40 00   SFBR := CTEST2 AND $40      ; SIGP?
            //   80 0c 00 40   Jump $0700B3D8 if <compare>  ; yes: take the work
            // ```
            //
            // and the pair is only coherent if `$40` — bits 7–0 — is the value
            // `SFBR` is compared against. Read the other way round every
            // comparison in that driver becomes vacuous, and the program parks
            // for ever, which is what this model did until the trace said so.
            // Bits 15–8 are then the mask, and this driver leaves them zero, so
            // the mask's own meaning is taken from the manual — the bits *not*
            // compared — and is not established here.
            let data = (dbc & 0xff) as u8;
            let mask = ((dbc >> 8) & 0xff) as u8;
            met &= (sfbr & !mask) == (data & !mask);
        }
        met == (dbc & DBC_TRUE != 0)
    }

    // -- Memory Move ---------------------------------------------------------

    /// Three longwords: a count, a source and a destination.
    fn memory_move(&self, dbc: u32, source: u32, after: u32) -> Step {
        let Some(dest) = self.fetch(after) else {
            self.bus_fault();
            return Step::Stop(Stop::Interrupted);
        };
        self.state.lock().set_long(DSP, after.wrapping_add(4));
        let count = dbc as usize;
        if count == 0 {
            return Step::Next;
        }
        let mut at = 0usize;
        while at < count {
            let want = (count - at).min(BURST);
            let mut buf = alloc::vec![0u8; want];
            if !self.read_mem(source.wrapping_add(at as u32), &mut buf)
                || !self.write_mem(dest.wrapping_add(at as u32), &buf)
            {
                self.bus_fault();
                return Step::Stop(Stop::Interrupted);
            }
            at += want;
        }
        Step::Next
    }

    // -- reset ---------------------------------------------------------------

    /// The `RST/` pin a board pulls, which is the hardware reset of the manual:
    /// every register to its power-on value and the cable reset with it.
    fn hard_reset(&self) {
        let own = self.state.lock().own_id;
        {
            let mut state = self.state.lock();
            *state = State::new(own);
        }
        self.bus.reset();
        self.refresh();
    }
}

/// One step's outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Carry on with whatever `DSP` now says.
    Next,
    /// Stop, for the reason given.
    Stop(Stop),
}

/// The address a `Select`'s destination field names.
///
/// The field is the **data bus line** the selection asserts (X3.131-1994
/// §5.1.3.2), so one bit set is address *n*. A field with no bits set is
/// address 0 rather than nothing: that is the same answer a bare zero gives,
/// it is what a program that never selects address 0 cannot tell apart, and it
/// keeps a hand-assembled `Select 0` working either way round.
const fn decode_id(field: u8) -> u8 {
    if field == 0 {
        0
    } else {
        field.trailing_zeros() as u8
    }
}

// ---------------------------------------------------------------------------
// the register window
// ---------------------------------------------------------------------------

/// The chip's sixty-four registers, for a board that maps them directly.
#[derive(Debug)]
struct RegsWindow(Arc<Chip>);

impl MemOps for RegsWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        for (i, byte) in dst.iter_mut().enumerate() {
            let reg = self.0.order.register(offset + i as u64);
            *byte = self.0.read_reg(reg, attrs.debug);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // A write to `DSP` starts a processor that masters the bus and
            // moves a disk block; one to `SCNTL1` resets the cable. Neither can
            // be made harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        for (i, byte) in src.iter().enumerate() {
            self.0.write_window(offset + i as u64, *byte);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Big)
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(false)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The chip, as whatever holds it sees it.
///
/// [`ExportId::SCSI_CONTROLLER`] carries this. Cloning it is cloning the
/// handle, not the chip.
#[derive(Debug, Clone)]
pub struct ControllerPort {
    chip: Arc<Chip>,
}

impl ControllerPort {
    /// Whether `IRQ/` is asserted, without disturbing anything.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.chip.state.lock().interrupting()
    }

    /// One register, by the data manual's number, with no read side effect.
    #[must_use]
    pub fn peek(&self, reg: usize) -> u8 {
        if reg >= REGS {
            return 0xff;
        }
        self.chip.read_reg(reg, true)
    }
}

/// An NCR 53C710 on a named SCSI bus.
#[derive(Debug)]
pub struct Ncr53c710 {
    chip: Arc<Chip>,
    regs: RegionRef,
}

/// Where the controller's lock sits in the ranked order.
///
/// Below [`scsi::BUS_RANK`], so an instruction may hold it while it looks a
/// target up. A distinct number from [`crate::dev::wd33c93::CHIP_RANK`] because
/// a board that someday held both would otherwise have no deterministic order.
///
/// Nothing here holds it across an outward call, and it is not a style point:
/// a Memory Move's destination is quite often **this chip's own register
/// file**, so a lock held across the space access would deadlock against
/// itself.
pub const CHIP_RANK: LockRank = LockRank::new(0x4c10);

impl Ncr53c710 {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, out of range or
    /// unknown.
    pub fn new(props: &Props) -> Result<Ncr53c710> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", scsi::DEFAULT_BUS)?.to_string();
        let own = r.or_range("id", 7u64, 0..=7)? as u8;
        let order = match r.or_str("byte-order", Order::Big.as_str())? {
            "big" => Order::Big,
            "little" => Order::Little,
            other => {
                return Err(Error::Config {
                    at: String::from("byte-order"),
                    message: format!("`{other}` is not `big` or `little`"),
                });
            }
        };
        r.finish()?;
        let bus = buses::attach(props, &bus_name)?;
        Ok(Ncr53c710::with_bus(bus, bus_name, own, order))
    }

    /// Build one around a bus the caller already has.
    #[must_use]
    pub fn with_bus(bus: Arc<Bus>, bus_name: String, own: u8, order: Order) -> Ncr53c710 {
        let chip = Arc::new(Chip {
            state: Mutex::with_rank(CHIP_RANK, State::new(own & 7)),
            order,
            bus,
            bus_name,
            space: Mutex::with_rank(LockRank::LEAF, None),
            requester: Mutex::with_rank(LockRank::LEAF, RequesterId(0)),
            irq: Mutex::with_rank(LockRank::LEAF, None),
        });
        Ncr53c710 {
            regs: Arc::new(Region::io(
                format!("{CLASS_NAME}.{REGS_REGION}"),
                REGS_WINDOW_LEN,
                Arc::new(RegsWindow(Arc::clone(&chip))),
            )),
            chip,
        }
    }

    /// The handle a host adapter holds.
    #[must_use]
    pub fn port(&self) -> ControllerPort {
        ControllerPort {
            chip: Arc::clone(&self.chip),
        }
    }

    /// The bus it is on.
    #[must_use]
    pub fn bus(&self) -> Arc<Bus> {
        Arc::clone(&self.chip.bus)
    }

    /// Give the chip the memory it masters.
    pub fn attach_space(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        *self.chip.space.lock() = Some(Arc::downgrade(space));
        *self.chip.requester.lock() = requester;
    }

    /// One register, by the data manual's number, with no read side effect.
    #[must_use]
    pub fn peek(&self, reg: usize) -> u8 {
        self.port().peek(reg)
    }

    /// Whether `IRQ/` is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.chip.state.lock().interrupting()
    }

    /// Which way round this chip's byte lanes are.
    #[must_use]
    pub fn order(&self) -> Order {
        self.chip.order
    }
}

/// The `ncr.53c710` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an NCR 53C710 SCSI I/O Processor: the register file, the interrupt model and a \
              SCRIPTS processor that masters the bus for its own instructions and its data",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the SCSI bus this controller arbitrates for (default `scsi0`)",
        },
        PropertySpec {
            name: "id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the initiator's own SCSI address out of reset, 0 to 7 (default 7)",
        },
        PropertySpec {
            name: "byte-order",
            kind: ValueKind::Str,
            required: false,
            summary: "which way round the register file's byte lanes are: `big` for a \
                      68000-family board (the default), `little` for the data manual's own \
                      numbering",
        },
    ],
    construct: |props| Ok(Box::new(Ncr53c710::new(props)?)),
};

impl Device for Ncr53c710 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` places the window, `bind` brings the space and
        // the wire graph brings the pin.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: `RST/` is a pin, and a board reset pulls it.
        self.chip.hard_reset();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | REGS_REGION => Some(Arc::clone(&self.regs)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a 53C710 drives one pin: `irq`"),
            });
        }
        *self.chip.irq.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            self.chip.refresh();
        }
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::SCSI_CONTROLLER)
            .then(|| Export::Opaque(Arc::new(self.port()) as Arc<dyn core::any::Any + Send + Sync>))
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.chip.state.lock();
        w.write_all(&state.regs)?;
        w.write_u8(match state.run {
            Run::Idle | Run::Running => 0,
            Run::WaitingReselect => 1,
        })?;
        w.write_bool(state.connected)?;
        w.write_u8(state.selected_id)?;
        w.write_bool(state.atn)?;
        w.write_bool(state.carry)?;
        // The phase the cable is in is the target's to save; what is kept here
        // is only what this chip knows that the cable does not.
        w.write_u8(state.own_id)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = [0u8; REGS];
        regs.copy_from_slice(r.take(REGS)?);
        let run = match r.read_u8()? {
            0 => Run::Idle,
            1 => Run::WaitingReselect,
            other => {
                return Err(Error::State(format!(
                    "{other} is not a state a 53C710's processor can be in"
                )));
            }
        };
        let connected = r.read_bool()?;
        let selected_id = r.read_u8()? & 7;
        let atn = r.read_bool()?;
        let carry = r.read_bool()?;
        let own_id = r.read_u8()? & 7;
        let mut state = self.chip.state.lock();
        *state = State {
            regs,
            run,
            connected,
            selected_id,
            atn,
            carry,
            own_id,
        };
        drop(state);
        // A restore does not re-run the wire graph; put the pin where the
        // registers say.
        self.chip.refresh();
        Ok(())
    }
}

/// The machine layer's half: the chip has to be told which memory it masters,
/// because a SCRIPTS processor fetches its own instructions out of it.
impl Instance for Ncr53c710 {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        if let Some(space) = ctx.space() {
            self.attach_space(space, ctx.requester());
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ncr53c710::new(props)?)))
}

/// What the validator should know about `ncr.53c710`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("id", ValueKind::Uint))
        .prop(PropSchema::new("byte-order", ValueKind::Str))
        .region("")
        .region(REGS_REGION)
        .port(IRQ_PIN, PortDir::Out)
}

/// The `IDENTIFY` message a table-indirect selection would carry, for a caller
/// assembling one.
///
/// Exposed because a test that hand-assembles a SCRIPTS program needs the same
/// byte Commodore's driver puts in its message-out buffer, and
/// [`crate::dev::scsi::message`] is where its parts are defined.
#[must_use]
pub const fn identify(lun: u8, disconnect: bool) -> u8 {
    message::IDENTIFY | if disconnect { message::DISC_PRIV } else { 0 } | (lun & message::LUN_MASK)
}
