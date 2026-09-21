//! Western Digital WD33C93A: a SCSI Bus Interface Controller.
//!
//! The initiator half of `dev/scsi`'s split. This file drives the phase
//! sequence [`crate::dev::scsi`] defines and **contains no SCSI command
//! opcode** — no `INQUIRY`, no `READ(10)`, no sense key, no mode page. What a
//! target does with a command descriptor block is the target's business; this
//! chip counts its bytes and puts them on the wire.
//!
//! The falsifiable form is in [`crate::dev::scsi`]'s documentation: a grep of
//! this file's *code* for a command name finds nothing, and if it ever starts
//! to, the split has rotted.
//!
//! # Two register addresses, twenty-six registers
//!
//! The chip has **two** addresses (datasheet §6.2.2). With `A0` low a write
//! loads the Address register and a read returns Auxiliary Status; with `A0`
//! high the access goes to whichever of the twenty-six registers the Address
//! register names, and the Address register then **auto-increments** — except
//! after Auxiliary Status, Data and Command, which are the three a driver
//! hammers and which therefore stay put.
//!
//! That is the whole of the host interface, and it is why a board needs almost
//! nothing to attach one: two addresses, a chip select, an interrupt line and,
//! if it wants speed, a `DRQ`/`DACK` pair into a DMA controller.
//!
//! # Levels, and why the combination command matters
//!
//! The datasheet sorts the command set into levels (§7.2):
//!
//! * **Level I** — `Reset`, `Abort`, `Assert ATN`, `Negate ACK`, `Disconnect`:
//!   they act on a signal and return.
//! * **Simple Level II** — `Select`, `Transfer Info`: one bus phase each, with
//!   the host deciding what comes next from the `MCI` field of every interrupt.
//! * **Combination Level II** — `Select-And-Transfer` (§7.6.1): the chip's own
//!   microprocessor walks selection, message out, command, data, status and
//!   message in, and raises **one** interrupt at the end. The Command Phase
//!   register records how far it got, which is what lets the host recover.
//!
//! All three are implemented, because a driver may use any of them and because
//! the simple ones are how the combination one is tested: the same transfer,
//! driven two ways, must reach the same target state.
//!
//! # What is not implemented
//!
//! * **Target-role commands** — `Reselect`, `Reselect-And-Transfer`,
//!   `Wait-For-Select-And-Receive`, `Send-Status-And-Command-Complete`,
//!   `Send-Disconnect-Message`, and the four `Receive`/four `Send` commands
//!   (§7.5.4, §7.5.5, §7.6.2). This chip is an initiator; a machine wanting a
//!   WD33C93A pretending to be a disk is a different job. They answer with the
//!   *invalid command* interrupt (`40` Hex), which is what the chip does with a
//!   command that is not valid in its current state.
//! * **`Translate Address` (§7.5.7)** — a division the host can do itself, for
//!   a geometry no SCSI-2 target reports.
//! * **Synchronous transfer.** The Synchronous Transfer register is readable
//!   and writable and its offset field is reported back unchanged, but every
//!   transfer is asynchronous: nothing here has a transfer *rate*, so a
//!   synchronous one would be the same bytes in the same order.
//! * **Host and SCSI parity.** `PE` is never set, because no byte on this bus
//!   was ever carried by a wire.
//! * **Reselection.** The targets in this tree never disconnect, so nothing
//!   reselects; see [`crate::dev::scsi`], which says so and says why.
//!
//! # Sources
//!
//! *WD33C93A SCSI Bus Interface Controller*, Western Digital, data sheet and
//! application notes, November 1990 — §6.1 the register map, §6.2 every
//! register bit by bit, §6.2.19 the interrupt status codes, §6.3 the two
//! resets, §7 the commands. X3.131 for what the phases mean; see
//! [`crate::dev::scsi`].
//!
//! **No emulator source of any licence was consulted** (`CLAUDE.md`,
//! provenance).

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::scsi::{self, Bus, Phase, Target, buses, cdb_len, message};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "wd.33c93";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The `INTRQ` output pin.
pub const INTRQ_PIN: &str = "intrq";

/// The two-address register window a board maps: `A0` low, `A0` high.
pub const REGS_REGION: &str = "regs";

/// How much the register window decodes: the chip has one address line.
pub const REGS_WINDOW_LEN: u64 = 2;

// -- the register map (datasheet §6.1) ---------------------------------------

/// Own ID / CDB Size.
pub const OWN_ID: u8 = 0x00;
/// Control.
pub const CONTROL: u8 = 0x01;
/// Timeout Period.
pub const TIMEOUT: u8 = 0x02;
/// The first of the twelve Command Descriptor Block registers.
pub const CDB1: u8 = 0x03;
/// Target LUN — and, after a `Select-And-Transfer`, the status byte.
pub const TARGET_LUN: u8 = 0x0F;
/// Command Phase: how far a combination command got.
pub const COMMAND_PHASE: u8 = 0x10;
/// Synchronous Transfer.
pub const SYNC_TRANSFER: u8 = 0x11;
/// Transfer Count, most significant of three.
pub const COUNT_MSB: u8 = 0x12;
/// Destination ID: who to select.
pub const DEST_ID: u8 = 0x15;
/// Source ID: who selected us, and the two response enables.
pub const SOURCE_ID: u8 = 0x16;
/// SCSI Status: why the interrupt. Read-only, and reading it clears `INTRQ`.
pub const SCSI_STATUS: u8 = 0x17;
/// Command.
pub const COMMAND: u8 = 0x18;
/// Data: the port into the twelve-byte FIFO.
pub const DATA: u8 = 0x19;
/// Auxiliary Status, in direct-addressing mode.
pub const AUX_STATUS: u8 = 0x1F;

/// How many host-addressable registers there are, `00` through `19`.
const REGS: usize = 0x1a;

// -- Auxiliary Status bits (§6.2.1) ------------------------------------------

/// `INT`: `INTRQ` is asserted.
pub const AUX_INT: u8 = 0x80;
/// `LCI`: the last command was ignored.
pub const AUX_LCI: u8 = 0x40;
/// `BSY`: a Level II command is executing.
pub const AUX_BSY: u8 = 0x20;
/// `CIP`: a command is being interpreted.
pub const AUX_CIP: u8 = 0x10;
/// `PE`: a parity error was seen. Never set here — see the module docs.
pub const AUX_PE: u8 = 0x02;
/// `DBR`: the Data register is ready.
pub const AUX_DBR: u8 = 0x01;

// -- Control register bits (§6.2.4) ------------------------------------------

/// The DMA mode select field, bits 7–5.
pub const CONTROL_DMA_MODE: u8 = 0xe0;
/// `IDI`, the intermediate disconnect interrupt enable.
pub const CONTROL_IDI: u8 = 0x04;

// -- Source ID bits (§6.2.18) ------------------------------------------------

/// `ER`: respond to reselection — and the `r` bit of the `IDENTIFY` message a
/// `Select-And-Transfer` sends.
pub const SOURCE_ER: u8 = 0x80;

// -- Command register bits (§6.2.20) -----------------------------------------

/// `SBT`: transfer exactly one byte, whatever the Transfer Count register says.
pub const CMD_SBT: u8 = 0x80;
/// The command code, bits 6–0.
pub const CMD_CODE: u8 = 0x7f;

// -- the commands (§7.1) -----------------------------------------------------

/// `Reset`.
pub const CMD_RESET: u8 = 0x00;
/// `Abort`.
pub const CMD_ABORT: u8 = 0x01;
/// `Assert ATN`.
pub const CMD_ASSERT_ATN: u8 = 0x02;
/// `Negate ACK`.
pub const CMD_NEGATE_ACK: u8 = 0x03;
/// `Disconnect`.
pub const CMD_DISCONNECT: u8 = 0x04;
/// `Select-With-ATN`.
pub const CMD_SELECT_ATN: u8 = 0x06;
/// `Select-Without-ATN`.
pub const CMD_SELECT: u8 = 0x07;
/// `Select-With-ATN-And-Transfer`.
pub const CMD_SELECT_ATN_TRANSFER: u8 = 0x08;
/// `Select-Without-ATN-And-Transfer`.
pub const CMD_SELECT_TRANSFER: u8 = 0x09;
/// `Set IDI`.
pub const CMD_SET_IDI: u8 = 0x0f;
/// `Transfer Info`.
pub const CMD_TRANSFER_INFO: u8 = 0x20;

// -- SCSI Status groups (§6.2.19) --------------------------------------------

/// `0000 xxxx`: the chip is in a reset state.
pub const INT_RESET: u8 = 0x00;
/// `0001 xxxx`: a command completed.
pub const INT_DONE: u8 = 0x10;
/// `0010 xxxx`: a command paused, or was aborted.
pub const INT_PAUSED: u8 = 0x20;
/// `0100 xxxx`: a command terminated early.
pub const INT_TERMINATED: u8 = 0x40;
/// `1000 xxxx`: the bus needs service.
pub const INT_SERVICE: u8 = 0x80;

/// `11` Hex: a `Select` completed; the chip is connected as an initiator.
pub const INT_SELECT_DONE: u8 = 0x11;
/// `16` Hex: a `Select-And-Transfer` completed.
pub const INT_SAT_DONE: u8 = 0x16;
/// `20` Hex: a Message-In `Transfer Info` paused with `ACK` asserted.
pub const INT_MSG_IN_PAUSED: u8 = 0x20;
/// `40` Hex: an invalid command was issued.
pub const INT_INVALID: u8 = 0x40;
/// `41` Hex: the target disconnected unexpectedly.
pub const INT_UNEXPECTED_DISCONNECT: u8 = 0x41;
/// `42` Hex: a selection timed out.
pub const INT_TIMEOUT: u8 = 0x42;
/// `85` Hex: a disconnect occurred; the chip is disconnected.
pub const INT_DISCONNECTED: u8 = 0x85;

/// The Command Phase values a `Select-And-Transfer` walks through (§7.6.1).
pub mod command_phase {
    /// Nothing selected.
    pub const IDLE: u8 = 0x00;
    /// The target is selected.
    pub const SELECTED: u8 = 0x10;
    /// The `IDENTIFY` message has been sent.
    pub const IDENTIFIED: u8 = 0x20;
    /// Command phase has started, no bytes sent.
    pub const COMMAND: u8 = 0x30;
    /// The data phase moved every byte the Transfer Count asked for.
    pub const DATA_DONE: u8 = 0x46;
    /// The target has begun the status phase.
    pub const STATUS: u8 = 0x47;
    /// A status byte was received and stored in the Target LUN register.
    pub const STATUS_DONE: u8 = 0x50;
    /// A `COMMAND COMPLETE` message was received.
    pub const COMPLETE: u8 = 0x60;
}

// ---------------------------------------------------------------------------
// the DMA seam
// ---------------------------------------------------------------------------

/// Somewhere for a data phase's bytes to come from and go to.
///
/// When the Control register's DMA mode select bits are not all zero the Data
/// register is not the port — a DMA controller is (§6.2.21), and on the far
/// side of it is memory this chip knows nothing about. That is exactly one
/// call in each direction, which is what this trait is.
///
/// A board with no DMA controller simply never attaches one, and every transfer
/// goes through the Data register in Polled I/O mode, which is what the DMA
/// mode select bits being zero means.
pub trait DmaPort: Send + Sync + fmt::Debug {
    /// Fetch up to `dst.len()` bytes from memory for a `DATA OUT` phase.
    ///
    /// Short is a stalled DMA controller: the transfer stops there, which is
    /// what the chip does when `DRQ` is never answered.
    fn fetch(&self, dst: &mut [u8]) -> usize;

    /// Put `src` into memory, from a `DATA IN` phase. How many bytes landed.
    fn store(&self, src: &[u8]) -> usize;
}

/// The chip, as whatever holds it sees it.
///
/// [`ExportId::SCSI_CONTROLLER`] carries this. Cloning it is cloning the
/// handle, not the chip.
#[derive(Debug, Clone)]
pub struct ControllerPort {
    chip: Arc<Chip>,
}

impl ControllerPort {
    /// Whether `INTRQ` is asserted, without disturbing anything.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.chip.state.lock().intrq
    }

    /// Whether `INTRQ` is asserted, **asking** the chip — which lets an
    /// interrupt the chip has decided on but not yet delivered arrive.
    ///
    /// A board calls this from whatever register its driver polls while it
    /// waits.
    ///
    /// # Why a chip can have an interrupt it has not delivered
    ///
    /// Two things that are one event in this model are two events in time on a
    /// real bus. A `Select` completes when the target asserts `BSY`; the target
    /// then asserts `REQ` for the first information phase, and §7.5.6 says that
    /// first `REQ` "results in a 'service required' interrupt". A host driver
    /// is written around that gap: it reads the SCSI Status register, sees the
    /// selection succeeded, sets up for the transfer, and *then* waits for the
    /// phase interrupt. §6.2.20 forbids it from writing a command while `INT`
    /// is set, so a model that revealed the second interrupt in the same bus
    /// cycle as the status read of the first would leave the driver with
    /// nowhere to go — Commodore's `scsi.device` does exactly this and waits
    /// forever. So the first is asserted when the chip decides it and the rest
    /// arrive here, which is the same "later" a real chip has expressed as the
    /// only ordering this model has.
    pub fn poll_irq(&self) -> bool {
        self.chip.poll_pending()
    }

    /// Load the Address register — the `A0` low write.
    pub fn write_address(&self, value: u8) {
        self.chip.write_address(value);
    }

    /// The Address register, which a board that can read it back exposes.
    #[must_use]
    pub fn address(&self) -> u8 {
        self.chip.state.lock().address
    }

    /// Auxiliary Status — the `A0` low read. Never a side effect.
    #[must_use]
    pub fn read_aux(&self) -> u8 {
        self.chip.read_aux()
    }

    /// Read the register the Address register names — the `A0` high read.
    #[must_use]
    pub fn read_register(&self, debug: bool) -> u8 {
        self.chip.read_indirect(debug)
    }

    /// Write the register the Address register names — the `A0` high write.
    pub fn write_register(&self, value: u8) {
        self.chip.write_indirect(value);
    }

    /// Assert `MR-`: the hardware reset of §6.3.1.
    pub fn master_reset(&self) {
        self.chip.master_reset();
    }

    /// Give the chip somewhere for a data phase to go.
    pub fn attach_dma(&self, port: Arc<dyn DmaPort>) {
        *self.chip.dma.lock() = Some(port);
    }

    /// The DMA mode select field of the Control register, for a board that
    /// wants to know whether it is expected to pump anything.
    #[must_use]
    pub fn dma_mode(&self) -> u8 {
        (self.chip.state.lock().regs[usize::from(CONTROL)] & CONTROL_DMA_MODE) >> 5
    }
}

// ---------------------------------------------------------------------------
// the chip's state
// ---------------------------------------------------------------------------

/// An active polled transfer: what `Transfer Info` is moving through the Data
/// register while the host watches `DBR`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Polled {
    /// Whether bytes are coming *in* — `I/O` asserted.
    input: bool,
    /// Bytes still to move.
    left: u32,
    /// The phase the transfer was started in; a change ends it.
    phase: Phase,
}

#[derive(Debug)]
struct State {
    /// The twenty-six host-addressable registers, `00` through `19`.
    regs: [u8; REGS],
    /// The Address register (§6.2.2).
    address: u8,
    /// `INTRQ`.
    intrq: bool,
    /// Interrupts the chip has decided on but not yet delivered: reading the
    /// SCSI Status register pops the next (§6.2.19 — "an additional interrupt
    /// will then occur when the SCSI bus goes to the Bus Free state").
    pending: VecDeque<u8>,
    /// `LCI`: the last command arrived while an interrupt was pending.
    lci: bool,
    /// Connected as an initiator.
    connected: bool,
    /// Who we are connected to.
    selected_id: u8,
    /// `ATN` asserted, waiting for the target to ask for a message.
    atn: bool,
    /// `ACK` left asserted after a Message-In transfer (§7.4.5).
    ack_held: bool,
    /// The polled transfer in progress, if any.
    polled: Option<Polled>,
    /// The byte the Data register will hand over next, in an input transfer.
    data_in: Option<u8>,
}

impl State {
    fn new() -> State {
        State {
            regs: [0; REGS],
            address: 0,
            intrq: false,
            pending: VecDeque::new(),
            lci: false,
            connected: false,
            selected_id: 0,
            atn: false,
            ack_held: false,
            polled: None,
            data_in: None,
        }
    }

    fn count(&self) -> u32 {
        u32::from_be_bytes([
            0,
            self.regs[usize::from(COUNT_MSB)],
            self.regs[usize::from(COUNT_MSB) + 1],
            self.regs[usize::from(COUNT_MSB) + 2],
        ])
    }

    fn set_count(&mut self, value: u32) {
        let bytes = value.to_be_bytes();
        self.regs[usize::from(COUNT_MSB)] = bytes[1];
        self.regs[usize::from(COUNT_MSB) + 1] = bytes[2];
        self.regs[usize::from(COUNT_MSB) + 2] = bytes[3];
    }

    /// Assert `INTRQ` with the first of `codes` and queue the rest.
    ///
    /// The first is raised **now**, because that is what a chip does: `INTRQ`
    /// goes high when the chip decides something, not when a host gets round
    /// to asking. The rest wait for [`Chip::poll_pending`], and the reasoning
    /// for that is there.
    fn raise(&mut self, codes: &[u8]) {
        let mut it = codes.iter().copied();
        if let Some(first) = it.next() {
            if self.intrq {
                // One interrupt is already waiting to be read; this one goes
                // behind it rather than overwriting the status the host has
                // not seen yet.
                self.pending.push_back(first);
            } else {
                self.regs[usize::from(SCSI_STATUS)] = first;
                self.intrq = true;
            }
        }
        for code in it {
            self.pending.push_back(code);
        }
    }

    /// The chip is no longer on the bus.
    fn disconnect(&mut self) {
        self.connected = false;
        self.atn = false;
        self.ack_held = false;
        self.polled = None;
        self.data_in = None;
    }
}

// ---------------------------------------------------------------------------
// the chip
// ---------------------------------------------------------------------------

/// The shared half of the device: the registers, the cable and the pin.
struct Chip {
    state: Mutex<State>,
    bus: Arc<Bus>,
    bus_name: String,
    intrq: Mutex<Option<WireSource>>,
    dma: Mutex<Option<Arc<dyn DmaPort>>>,
}

impl fmt::Debug for Chip {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Chip")
            .field("bus", &self.bus_name)
            .field("state", &*self.state.lock())
            .finish_non_exhaustive()
    }
}

impl Chip {
    // -- the host interface --------------------------------------------------

    /// `A0` low, read: Auxiliary Status (§6.2.1). No side effect, ever, which
    /// is what makes it the register a debugger may read.
    fn read_aux(&self) -> u8 {
        let state = self.state.lock();
        let mut aux = 0;
        if state.intrq {
            aux |= AUX_INT;
        }
        if state.lci {
            aux |= AUX_LCI;
        }
        // `BSY` and `CIP` are both about a command *in flight*. Every command
        // here completes inside the write that issued it, so neither is ever
        // set when the host can look — which is the truthful report of a chip
        // that is never busy rather than a simplification.
        if let Some(p) = state.polled {
            // §6.2.1: during a Receive or Transfer that brings bytes in, `DBR`
            // is set when a byte has arrived; during one that sends them out it
            // is set when the chip can take another.
            if p.left > 0 && (!p.input || state.data_in.is_some()) {
                aux |= AUX_DBR;
            }
        }
        aux
    }

    /// `A0` low, write: the Address register (§6.2.2).
    fn write_address(&self, value: u8) {
        self.state.lock().address = value & 0x1f;
    }

    /// `A0` high, read.
    fn read_indirect(&self, debug: bool) -> u8 {
        let address = self.state.lock().address;
        match address {
            AUX_STATUS => self.read_aux(),
            DATA => self.read_data(debug),
            SCSI_STATUS => {
                let mut state = self.state.lock();
                let value = state.regs[usize::from(SCSI_STATUS)];
                if !debug {
                    // §6.2.19: "the host should read the SCSI Status register
                    // to clear Intrq prior to issuing any commands". Whatever
                    // is queued behind it stays queued: see
                    // [`Chip::poll_pending`] for why the next one does not
                    // arrive in this very bus cycle.
                    state.intrq = false;
                    state.lci = false;
                }
                // §6.2.2: only Auxiliary Status, Data and Command do not
                // auto-increment, so this one does.
                if !debug {
                    Chip::advance(&mut state);
                }
                drop(state);
                if !debug {
                    self.refresh();
                }
                value
            }
            reg if usize::from(reg) < REGS => {
                let mut state = self.state.lock();
                let value = state.regs[usize::from(reg)];
                if !debug {
                    Chip::advance(&mut state);
                }
                value
            }
            // §6.1 note 2: "Reading an undefined or unavailable register
            // results in an all-ones data bus output."
            _ => 0xff,
        }
    }

    /// `A0` high, write.
    fn write_indirect(&self, value: u8) {
        let address = self.state.lock().address;
        match address {
            DATA => self.write_data(value),
            COMMAND => self.command(value),
            // §6.2.19: the SCSI Status register is read-only. A write lands
            // nowhere and still advances the Address register, because the
            // increment is in the bus cycle rather than in the register.
            SCSI_STATUS => {}
            reg if usize::from(reg) < REGS => {
                let mut state = self.state.lock();
                state.regs[usize::from(reg)] = value;
                Chip::advance(&mut state);
            }
            _ => {}
        }
    }

    /// §6.2.2: "following every access with A0 = 1, the Address register will
    /// automatically increment to point at the next register, with the
    /// exception of the following locations: Auxiliary Status register, Data
    /// register, and the Command register."
    fn advance(state: &mut State) {
        if matches!(state.address, AUX_STATUS | DATA | COMMAND) {
            return;
        }
        state.address = (state.address + 1) & 0x1f;
    }

    // -- the Data register ---------------------------------------------------

    /// Read one byte out of the FIFO, pulling the next off the bus.
    ///
    /// `debug` returns whatever is already staged without moving the transfer
    /// on, which is the whole content of [`MemAttrs::debug`] for this chip:
    /// a monitor that read the Data register and advanced a `READ(10)` by a
    /// byte would corrupt the guest's buffer.
    fn read_data(&self, debug: bool) -> u8 {
        if debug {
            return self.state.lock().data_in.unwrap_or(0);
        }
        // What is staged, and what the transfer wants next — decided under the
        // lock, fetched with it released.
        let (byte, want_more) = {
            let mut state = self.state.lock();
            let byte = state.data_in.take().unwrap_or(0);
            let Some(p) = state.polled.as_mut() else {
                return byte;
            };
            if !p.input {
                return byte;
            }
            p.left = p.left.saturating_sub(1);
            let left = p.left;
            state.set_count(left);
            (byte, left > 0)
        };
        if want_more {
            self.stage_input();
        } else {
            self.finish_polled();
        }
        byte
    }

    /// Write one byte into the FIFO and push it onto the bus.
    fn write_data(&self, value: u8) {
        let (target, done) = {
            let mut state = self.state.lock();
            let Some(p) = state.polled.as_mut() else {
                return;
            };
            if p.input {
                return;
            }
            p.left = p.left.saturating_sub(1);
            let left = p.left;
            state.set_count(left);
            (state.selected_id, left == 0)
        };
        if let Some(target) = self.bus.target(target) {
            target.write(&[value]);
        }
        if done {
            self.finish_polled();
        }
    }

    /// Pull the next input byte off the target into the staging slot.
    fn stage_input(&self) {
        let id = self.state.lock().selected_id;
        let Some(target) = self.bus.target(id) else {
            return;
        };
        let mut byte = [0u8; 1];
        let got = target.read(&mut byte);
        let mut state = self.state.lock();
        state.data_in = (got == 1).then_some(byte[0]);
    }

    /// A polled `Transfer Info` has moved its last byte: work out the
    /// interrupt (§7.5.6) with nothing held, then raise it.
    fn finish_polled(&self) {
        let (was, id) = {
            let mut state = self.state.lock();
            let Some(p) = state.polled.take() else {
                return;
            };
            (p, state.selected_id)
        };
        let phase = self.bus.target(id).map_or(Phase::BusFree, |t| t.phase());
        let mut state = self.state.lock();
        if was.phase == Phase::MessageIn {
            // §7.5.6: a Message-In transfer pauses with `ACK` asserted so the
            // host can look at the message before accepting it, and does *not*
            // wait for another `REQ`.
            state.ack_held = true;
            state.raise(&[INT_MSG_IN_PAUSED]);
        } else {
            match phase.mci() {
                // §7.5.6: for every other phase the completion interrupt comes
                // with the *new* phase the target is asking for.
                Some(mci) => state.raise(&[INT_DONE | mci]),
                None => {
                    state.disconnect();
                    state.raise(&[INT_DISCONNECTED]);
                }
            }
        }
        drop(state);
        self.refresh();
    }

    // -- resets --------------------------------------------------------------

    /// §6.3.1, the `MR-` pin.
    fn master_reset(&self) {
        self.release_bus();
        let mut state = self.state.lock();
        let keep = state.regs;
        *state = State::new();
        // "The following host accessible registers are NOT affected by the MR-
        // signal: Registers 01 Hex through 15 Hex; Source ID (16 Hex) register
        // bits 0-3; Command register (18 Hex)."
        state.regs[usize::from(CONTROL)..=usize::from(DEST_ID)]
            .copy_from_slice(&keep[usize::from(CONTROL)..=usize::from(DEST_ID)]);
        state.regs[usize::from(SOURCE_ID)] = keep[usize::from(SOURCE_ID)] & 0x0f;
        state.regs[usize::from(COMMAND)] = keep[usize::from(COMMAND)];
        // "The Own ID register is reset to zero" and "the INT bit (and INTRQ
        // pin) is set to one when the hardware reset is complete".
        state.raise(&[INT_RESET]);
        drop(state);
        self.bus.reset();
        self.refresh();
    }

    /// §7.4.1 and §6.3.2, the `Reset` command.
    fn soft_reset(&self) {
        // "All SCSI bus signals are reset to the negated state" (§6.3.2), so
        // whatever this chip was connected to sees the bus go free. Done first,
        // with nothing of this chip's held.
        self.release_bus();
        let mut state = self.state.lock();
        let own = state.regs[usize::from(OWN_ID)];
        let command = state.regs[usize::from(COMMAND)];
        *state = State::new();
        state.regs[usize::from(OWN_ID)] = own;
        state.regs[usize::from(COMMAND)] = command;
        // "The SCSI Status register is set as commanded by the EAF bit in the
        // Own ID register": 00 with advanced features off, 01 with them on.
        let advanced = own & 0x08 != 0;
        state.raise(&[if advanced { 0x01 } else { INT_RESET }]);
        drop(state);
        self.refresh();
    }

    // -- the command register ------------------------------------------------

    /// §6.2.20: a command written while an interrupt is pending is ignored and
    /// `LCI` is set.
    fn command(&self, value: u8) {
        {
            let mut state = self.state.lock();
            state.regs[usize::from(COMMAND)] = value;
            if state.intrq {
                state.lci = true;
                return;
            }
            state.lci = false;
        }
        let code = value & CMD_CODE;
        let sbt = value & CMD_SBT != 0;
        match code {
            CMD_RESET => self.soft_reset(),
            CMD_ABORT => self.abort(),
            CMD_ASSERT_ATN => {
                self.state.lock().atn = true;
            }
            CMD_NEGATE_ACK => self.negate_ack(),
            CMD_DISCONNECT => self.disconnect_command(),
            CMD_SELECT_ATN => self.select(true),
            CMD_SELECT => self.select(false),
            CMD_SELECT_ATN_TRANSFER => self.select_and_transfer(true),
            CMD_SELECT_TRANSFER => self.select_and_transfer(false),
            CMD_SET_IDI => {
                let mut state = self.state.lock();
                state.regs[usize::from(CONTROL)] |= CONTROL_IDI;
            }
            CMD_TRANSFER_INFO => self.transfer_info(sbt),
            // §6.2.19: "An invalid command was issued." Every target-role
            // command lands here, and so does anything the datasheet does not
            // define.
            _ => {
                let mut state = self.state.lock();
                state.raise(&[INT_INVALID]);
                drop(state);
                self.refresh();
            }
        }
    }

    /// Let go of whatever target this chip was connected to, if any.
    ///
    /// Called with nothing of this chip's held: the target takes its own lock.
    fn release_bus(&self) {
        let (connected, id) = {
            let state = self.state.lock();
            (state.connected, state.selected_id)
        };
        if !connected {
            return;
        }
        if let Some(target) = self.bus.target(id) {
            target.release();
        }
    }

    /// §7.4.2, in the initiator and disconnected states.
    fn abort(&self) {
        let connected = {
            let mut state = self.state.lock();
            state.polled = None;
            state.data_in = None;
            state.connected
        };
        let mut state = self.state.lock();
        if connected {
            // "A Transfer command was aborted" — the chip stays connected.
            state.raise(&[INT_PAUSED | 0x08]);
        } else {
            // "A Select or Reselect command was aborted."
            state.raise(&[INT_PAUSED | 0x02]);
        }
        drop(state);
        self.refresh();
    }

    /// §7.4.5: let go of `ACK`, which lets the target move on.
    fn negate_ack(&self) {
        let (id, connected) = {
            let mut state = self.state.lock();
            state.ack_held = false;
            (state.selected_id, state.connected)
        };
        if !connected {
            return;
        }
        // The target's phase may now have changed; the host will ask for the
        // next `Transfer Info` when it is ready, so nothing is raised here.
        let _ = self.bus.target(id);
    }

    /// §7.4.3: release the bus.
    fn disconnect_command(&self) {
        let id = self.state.lock().selected_id;
        if let Some(target) = self.bus.target(id) {
            target.release();
        }
        let mut state = self.state.lock();
        state.disconnect();
        state.regs[usize::from(COMMAND_PHASE)] = command_phase::IDLE;
        // §7.4.3: "The Disconnect command causes the immediate release of all
        // bus signals" and generates no interrupt.
        drop(state);
        self.refresh();
    }

    /// §7.5.1 and §7.5.2.
    fn select(&self, atn: bool) {
        let id = self.state.lock().regs[usize::from(DEST_ID)] & 0x07;
        let Some(target) = self.bus.target(id) else {
            let mut state = self.state.lock();
            state.regs[usize::from(COMMAND_PHASE)] = command_phase::IDLE;
            state.raise(&[INT_TIMEOUT]);
            drop(state);
            self.refresh();
            return;
        };
        let answered = target.select(atn);
        let phase = target.phase();
        let mut state = self.state.lock();
        if !answered {
            state.raise(&[INT_TIMEOUT]);
            drop(state);
            self.refresh();
            return;
        }
        state.connected = true;
        state.selected_id = id;
        state.atn = atn;
        // §7.5.1 then §7.5.6: the select completes, and the first `REQ` after
        // connection is a "service required" interrupt naming the phase.
        // §7.5.1 then §7.5.6: the select completes, and the first `REQ` after
        // connection is a "service required" interrupt naming the phase. The
        // second is *queued* rather than raised: a host that has just read the
        // SCSI Status register to find out the selection succeeded must find
        // `INT` clear afterwards, or it cannot issue the command that answers
        // the phase (§6.2.20). It arrives on the host's next poll — see
        // [`Chip::poll_pending`].
        // §7.5.1 then §7.5.6: the select completes, and the first `REQ` after
        // connection is a "service required" interrupt naming the phase. The
        // second is *queued* rather than raised: a host that has just read the
        // SCSI Status register to find out the selection succeeded must find
        // `INT` clear afterwards, or it cannot issue the command that answers
        // the phase (§6.2.20). It arrives on the host's next poll — see
        // [`Chip::poll_pending`].
        match phase.mci() {
            Some(mci) => state.raise(&[INT_SELECT_DONE, INT_SERVICE | mci]),
            None => state.raise(&[INT_SELECT_DONE]),
        }
        drop(state);
        self.refresh();
    }

    /// §7.5.6, `Transfer Info`.
    ///
    /// Two shapes: with the DMA mode select bits zero the whole transfer runs
    /// through the Data register a byte at a time and this only sets it up;
    /// with them non-zero the DMA port moves the lot here and now.
    fn transfer_info(&self, sbt: bool) {
        let (id, connected, count, dma) = {
            let state = self.state.lock();
            (
                state.selected_id,
                state.connected,
                if sbt { 1 } else { state.count() },
                state.regs[usize::from(CONTROL)] & CONTROL_DMA_MODE,
            )
        };
        if !connected {
            let mut state = self.state.lock();
            state.raise(&[INT_INVALID]);
            drop(state);
            self.refresh();
            return;
        }
        let Some(target) = self.bus.target(id) else {
            let mut state = self.state.lock();
            state.disconnect();
            state.raise(&[INT_UNEXPECTED_DISCONNECT]);
            drop(state);
            self.refresh();
            return;
        };
        let phase = target.phase();
        let Some(_) = phase.mci() else {
            let mut state = self.state.lock();
            state.disconnect();
            state.raise(&[INT_UNEXPECTED_DISCONNECT]);
            drop(state);
            self.refresh();
            return;
        };
        // A count of zero disables the counter (§6.2.16): one byte moves.
        let count = if count == 0 { 1 } else { count };
        let input = phase.is_input();
        if dma != 0 && matches!(phase, Phase::DataIn | Phase::DataOut) {
            let moved = self.pump(&target, input, count);
            let mut state = self.state.lock();
            state.set_count(count - moved);
            drop(state);
            self.after_transfer(&target, phase);
            return;
        }
        // Polled: stage the first byte if one is coming in, and let the host
        // pull the rest through the Data register.
        {
            let mut state = self.state.lock();
            state.polled = Some(Polled {
                input,
                left: count,
                phase,
            });
            state.set_count(count);
            state.data_in = None;
        }
        if input {
            self.stage_input();
        }
    }

    /// Move `count` bytes between the target and the DMA port.
    ///
    /// **No lock of this chip's is held**: the port writes guest memory and the
    /// target takes its own lock, and holding ours across either would nest two
    /// device locks for no reason. Returns how many bytes moved.
    fn pump(&self, target: &Arc<dyn Target>, input: bool, count: u32) -> u32 {
        /// How many bytes cross at a time. A `READ(10)` of 32 MiB should not
        /// become a 32 MiB allocation, and a `DRQ` burst was never the whole
        /// transfer anyway.
        const BURST: usize = 4096;
        let Some(port) = self.dma.lock().clone() else {
            return 0;
        };
        let mut moved = 0u32;
        while moved < count {
            let want = (count - moved).min(BURST as u32) as usize;
            let mut buf = alloc::vec![0u8; want];
            let n = if input {
                let got = target.read(&mut buf);
                if got == 0 {
                    break;
                }
                port.store(&buf[..got])
            } else {
                let got = port.fetch(&mut buf);
                if got == 0 {
                    break;
                }
                target.write(&buf[..got])
            };
            if n == 0 {
                break;
            }
            moved += n as u32;
        }
        moved
    }

    /// The interrupt a completed DMA `Transfer Info` raises (§7.5.6).
    fn after_transfer(&self, target: &Arc<dyn Target>, was: Phase) {
        let phase = target.phase();
        let mut state = self.state.lock();
        if was == Phase::MessageIn {
            state.ack_held = true;
            state.raise(&[INT_MSG_IN_PAUSED]);
        } else {
            match phase.mci() {
                Some(mci) => state.raise(&[INT_DONE | mci]),
                None => {
                    state.disconnect();
                    state.raise(&[INT_DISCONNECTED]);
                }
            }
        }
        drop(state);
        self.refresh();
    }

    /// §7.6.1, `Select-And-Transfer`: the whole SCSI operation, one interrupt.
    ///
    /// Written as the datasheet writes it — selection, message out, command,
    /// data, status, message in — with the Command Phase register set at each
    /// step so that a termination tells the host where it stopped. Every
    /// outward call is made with this chip's lock released.
    fn select_and_transfer(&self, atn: bool) {
        let (id, lun, er, count, dma) = {
            let state = self.state.lock();
            (
                state.regs[usize::from(DEST_ID)] & 0x07,
                state.regs[usize::from(TARGET_LUN)] & message::LUN_MASK,
                state.regs[usize::from(SOURCE_ID)] & SOURCE_ER != 0,
                state.count(),
                state.regs[usize::from(CONTROL)] & CONTROL_DMA_MODE,
            )
        };

        // -- selection (§7.6.1: "Failure to complete the Selection phase is
        // also indicated by the fact that the Command Phase register contains
        // all zeroes")
        let Some(target) = self.bus.target(id) else {
            let mut state = self.state.lock();
            state.regs[usize::from(COMMAND_PHASE)] = command_phase::IDLE;
            state.raise(&[INT_TIMEOUT]);
            drop(state);
            self.refresh();
            return;
        };
        if !target.select(atn) {
            let mut state = self.state.lock();
            state.regs[usize::from(COMMAND_PHASE)] = command_phase::IDLE;
            state.raise(&[INT_TIMEOUT]);
            drop(state);
            self.refresh();
            return;
        }
        {
            let mut state = self.state.lock();
            state.connected = true;
            state.selected_id = id;
            state.regs[usize::from(COMMAND_PHASE)] = command_phase::SELECTED;
        }

        // -- message out (§7.6.1: "1r000ttt, where r = 1 if the Enable Reselect
        // bit in the Source ID register is equal to 1")
        if atn {
            if target.phase() != Phase::MessageOut {
                return self.sat_terminated(&target);
            }
            let identify =
                message::IDENTIFY | if er { message::DISC_PRIV } else { 0 } | (lun & 0x07);
            target.write(&[identify]);
            self.state.lock().regs[usize::from(COMMAND_PHASE)] = command_phase::IDENTIFIED;
        }

        // -- command (§7.6.1: the Command Phase register "is set to Hex 30
        // before the first Command byte is sent and then increments with each
        // byte transferred")
        if target.phase() != Phase::Command {
            return self.sat_terminated(&target);
        }
        let cdb = {
            let state = self.state.lock();
            let first = state.regs[usize::from(CDB1)];
            // §7.6.1: "6, 10, or 12 bytes of command information as determined
            // by its evaluation of the SCSI command code in the CDB1 register".
            // An unknown group takes the Own ID register's CDB size, which is
            // what advanced mode uses it for (§6.2.3).
            let len = cdb_len(first).unwrap_or_else(|| {
                let size = usize::from(state.regs[usize::from(OWN_ID)] & 0x0f);
                if (1..=12).contains(&size) { size } else { 6 }
            });
            let at = usize::from(CDB1);
            state.regs[at..at + len].to_vec()
        };
        self.state.lock().regs[usize::from(COMMAND_PHASE)] = command_phase::COMMAND;
        for (i, byte) in cdb.iter().enumerate() {
            target.write(&[*byte]);
            self.state.lock().regs[usize::from(COMMAND_PHASE)] =
                command_phase::COMMAND | ((i as u8 + 1) & 0x0f);
        }

        // -- data (§7.6.1: "If the Transfer Count register contains any
        // non-zero value, then the WD33C93A will expect a Data Transfer phase")
        let phase = target.phase();
        if count > 0 && matches!(phase, Phase::DataIn | Phase::DataOut) {
            let moved = if dma == 0 {
                // Polled I/O inside a combination command is not something the
                // datasheet describes a host doing — §7.6.1 says "all host-side
                // Data register accesses will be accomplished via the method
                // selected by the DMA mode select bits", and with them zero the
                // chip would stall waiting for a host that has been told the
                // command runs by itself. Nothing is moved, and the unexpected
                // phase that follows is reported.
                0
            } else {
                self.pump(&target, phase.is_input(), count)
            };
            let mut state = self.state.lock();
            state.set_count(count - moved);
            if moved == count {
                state.regs[usize::from(COMMAND_PHASE)] = command_phase::DATA_DONE;
            } else {
                drop(state);
                return self.sat_terminated(&target);
            }
        } else if count > 0 {
            // A count was loaded and the target does not want a data phase.
            // §7.6.1: that is an unexpected information phase.
            return self.sat_terminated(&target);
        }

        // -- status (§7.6.1: "At the start of the Status phase, the Command
        // Phase register is loaded with Hex 47")
        if target.phase() != Phase::Status {
            return self.sat_terminated(&target);
        }
        self.state.lock().regs[usize::from(COMMAND_PHASE)] = command_phase::STATUS;
        let mut status = [0u8; 1];
        if target.read(&mut status) != 1 {
            return self.sat_terminated(&target);
        }
        {
            let mut state = self.state.lock();
            // §7.6.1: "the received status byte is stored in the Target Lun
            // register where it can be read upon completion of the command".
            state.regs[usize::from(TARGET_LUN)] = status[0];
            state.regs[usize::from(COMMAND_PHASE)] = command_phase::STATUS_DONE;
        }

        // -- message in (§7.6.1: a `COMMAND COMPLETE`, and then the Command
        // Phase register advances to Hex 60)
        if target.phase() != Phase::MessageIn {
            return self.sat_terminated(&target);
        }
        let mut msg = [0u8; 1];
        if target.read(&mut msg) != 1 || msg[0] != message::COMMAND_COMPLETE {
            return self.sat_terminated(&target);
        }
        target.release();
        let mut state = self.state.lock();
        state.regs[usize::from(COMMAND_PHASE)] = command_phase::COMPLETE;
        state.disconnect();
        // §7.6.1: the completion interrupt, and then "an additional interrupt
        // will then occur when the SCSI bus goes to the Bus Free state" — which
        // on this bus it already has.
        state.raise(&[INT_SAT_DONE, INT_DISCONNECTED]);
        drop(state);
        self.refresh();
    }

    /// §7.6.1's "terminated" interrupt: the target asked for a phase the
    /// command did not expect, and the Command Phase register says where.
    fn sat_terminated(&self, target: &Arc<dyn Target>) {
        let phase = target.phase();
        let mut state = self.state.lock();
        match phase.mci() {
            Some(mci) => {
                state.connected = true;
                state.raise(&[INT_TERMINATED | mci]);
            }
            None => {
                state.disconnect();
                state.raise(&[INT_UNEXPECTED_DISCONNECT]);
            }
        }
        drop(state);
        self.refresh();
    }

    // -- the pin -------------------------------------------------------------

    /// Deliver the next queued interrupt, if `INTRQ` is clear and one is
    /// waiting, and report whether `INTRQ` is now asserted.
    ///
    /// # Why an interrupt can be *queued* at all
    ///
    /// Two things that are one event in this model are two events in time on a
    /// real bus. A `Select` completes when the target asserts `BSY`; the target
    /// then asserts `REQ` for the first information phase, and §7.5.6 says that
    /// first `REQ` "results in a 'service required' interrupt". A host driver
    /// is written around that gap: it reads the SCSI Status register, sees the
    /// selection succeeded, sets up for the transfer, and *then* waits for the
    /// phase interrupt. §6.2.20 forbids it from writing a command while `INT`
    /// is set, so a model that raises both at once — the status read clearing
    /// one interrupt and revealing the next in the same bus cycle — leaves the
    /// driver with nowhere to go. Commodore's `scsi.device` does exactly this
    /// and waits forever.
    ///
    /// So the second interrupt is delivered here, when the host **asks** —
    /// which a board does by reading its own interrupt status register, and
    /// which `amiga.sdmac`'s `ISTR` read does. That is the same "later" a real
    /// chip has, expressed as the only ordering this model has.
    fn poll_pending(&self) -> bool {
        let raised = {
            let mut state = self.state.lock();
            if !state.intrq
                && let Some(next) = state.pending.pop_front()
            {
                state.regs[usize::from(SCSI_STATUS)] = next;
                state.intrq = true;
            }
            state.intrq
        };
        self.refresh();
        raised
    }

    /// Drive `INTRQ` from the registers, with nothing held while the net
    /// delivers.
    fn refresh(&self) {
        let high = self.state.lock().intrq;
        let source = self.intrq.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(high));
        }
    }
}

// ---------------------------------------------------------------------------
// the register window
// ---------------------------------------------------------------------------

/// The chip's own two addresses, for a board that maps it directly.
#[derive(Debug)]
struct RegsWindow(Arc<Chip>);

impl MemOps for RegsWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 1 {
            return Err(BusError::BadAccess);
        }
        dst[0] = if offset & 1 == 0 {
            self.0.read_aux()
        } else {
            self.0.read_indirect(attrs.debug)
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 1 {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A write to the Command register runs a SCSI operation and one to
            // the Data register pushes a byte onto the bus; neither can be made
            // harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        if offset & 1 == 0 {
            self.0.write_address(src[0]);
        } else {
            self.0.write_indirect(src[0]);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints {
            min: Width::U8,
            natural_alignment: false,
            ..AccessConstraints::word(Width::U8, Endian::Big)
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// A WD33C93A on a named SCSI bus.
#[derive(Debug)]
pub struct Wd33c93 {
    chip: Arc<Chip>,
    regs: RegionRef,
}

impl Wd33c93 {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or unknown.
    pub fn new(props: &Props) -> Result<Wd33c93> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", scsi::DEFAULT_BUS)?.to_string();
        // §6.2.3: the Own ID register holds the initiator's own bus address,
        // and is sampled by the `Reset` command rather than at power-on. The
        // property is what the *board* strapped, which a driver overwrites.
        let own = r.or_range("id", 7u64, 0..=7)? as u8;
        r.finish()?;
        let bus = buses::attach(props, &bus_name)?;
        Ok(Wd33c93::with_bus(bus, bus_name, own))
    }

    /// Build one around a bus the caller already has.
    #[must_use]
    pub fn with_bus(bus: Arc<Bus>, bus_name: String, own: u8) -> Wd33c93 {
        let mut state = State::new();
        state.regs[usize::from(OWN_ID)] = own & 0x07;
        let chip = Arc::new(Chip {
            state: Mutex::with_rank(CHIP_RANK, state),
            bus,
            bus_name,
            intrq: Mutex::with_rank(LockRank::LEAF, None),
            dma: Mutex::with_rank(LockRank::LEAF, None),
        });
        Wd33c93 {
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

    /// Whether `INTRQ` is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.chip.state.lock().intrq
    }

    /// The SCSI Status register without clearing the interrupt — what a
    /// debugger and a test both want.
    #[must_use]
    pub fn scsi_status(&self) -> u8 {
        self.chip.state.lock().regs[usize::from(SCSI_STATUS)]
    }

    /// The Command Phase register: how far the last combination command got.
    #[must_use]
    pub fn command_phase(&self) -> u8 {
        self.chip.state.lock().regs[usize::from(COMMAND_PHASE)]
    }
}

/// Where the controller's lock sits in the ranked order.
///
/// Below [`scsi::BUS_RANK`], so a command may hold it while it looks a target
/// up; the target's own lock is above both, so the ladder ascends the whole
/// way. In practice almost nothing here holds it across an outward call —
/// `select_and_transfer` takes and releases it at each step of the phase
/// sequence — but the rank has to permit the one case that does.
pub const CHIP_RANK: LockRank = LockRank::new(0x4c00);

/// The `wd.33c93` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a Western Digital WD33C93A SCSI Bus Interface Controller: the register file, the \
              command set and the initiator's half of the phase sequence",
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
    ],
    construct: |props| Ok(Box::new(Wd33c93::new(props)?)),
};

impl Device for Wd33c93 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` places the window and the wire graph brings
        // the pin.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: `MR-` is a pin, and a board reset pulls it.
        self.chip.master_reset();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | REGS_REGION => Some(Arc::clone(&self.regs)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != INTRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a WD33C93A drives one pin: `intrq`"),
            });
        }
        *self.chip.intrq.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == INTRQ_PIN {
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
        w.write_u8(state.address)?;
        w.write_bool(state.intrq)?;
        w.write_seq_len(state.pending.len() as u64)?;
        for code in &state.pending {
            w.write_u8(*code)?;
        }
        w.write_bool(state.lci)?;
        w.write_bool(state.connected)?;
        w.write_u8(state.selected_id)?;
        w.write_bool(state.atn)?;
        w.write_bool(state.ack_held)?;
        match state.polled {
            None => w.write_bool(false)?,
            Some(p) => {
                w.write_bool(true)?;
                w.write_bool(p.input)?;
                w.write_u32(p.left)?;
                w.write_u8(phase_code(p.phase))?;
            }
        }
        match state.data_in {
            None => w.write_bool(false)?,
            Some(byte) => {
                w.write_bool(true)?;
                w.write_u8(byte)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = [0u8; REGS];
        regs.copy_from_slice(r.take(REGS)?);
        let address = r.read_u8()? & 0x1f;
        let intrq = r.read_bool()?;
        let queued = r.read_seq_len(1)?;
        if queued > 8 {
            return Err(Error::State(format!(
                "{queued} queued interrupts is more than this chip can hold"
            )));
        }
        let mut pending = VecDeque::new();
        for _ in 0..queued {
            pending.push_back(r.read_u8()?);
        }
        let lci = r.read_bool()?;
        let connected = r.read_bool()?;
        let selected_id = r.read_u8()? & 0x07;
        let atn = r.read_bool()?;
        let ack_held = r.read_bool()?;
        let polled = if r.read_bool()? {
            Some(Polled {
                input: r.read_bool()?,
                left: r.read_u32()?,
                phase: phase_from(r.read_u8()?)?,
            })
        } else {
            None
        };
        let data_in = if r.read_bool()? {
            Some(r.read_u8()?)
        } else {
            None
        };
        let mut state = self.chip.state.lock();
        *state = State {
            regs,
            address,
            intrq,
            pending,
            lci,
            connected,
            selected_id,
            atn,
            ack_held,
            polled,
            data_in,
        };
        drop(state);
        // A restore does not re-run the wire graph; put the pin where the
        // registers say.
        self.chip.refresh();
        Ok(())
    }
}

impl Instance for Wd33c93 {}

/// A phase as one snapshot byte.
const fn phase_code(phase: Phase) -> u8 {
    match phase {
        Phase::BusFree => 0,
        Phase::DataOut => 1,
        Phase::DataIn => 2,
        Phase::Command => 3,
        Phase::Status => 4,
        Phase::MessageOut => 5,
        Phase::MessageIn => 6,
    }
}

fn phase_from(code: u8) -> Result<Phase> {
    match code {
        0 => Ok(Phase::BusFree),
        1 => Ok(Phase::DataOut),
        2 => Ok(Phase::DataIn),
        3 => Ok(Phase::Command),
        4 => Ok(Phase::Status),
        5 => Ok(Phase::MessageOut),
        6 => Ok(Phase::MessageIn),
        other => Err(Error::State(format!("{other} is not a SCSI bus phase"))),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Wd33c93::new(props)?)))
}

/// What the validator should know about `wd.33c93`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("id", ValueKind::Uint))
        .region("")
        .region(REGS_REGION)
        .port(INTRQ_PIN, PortDir::Out)
}
