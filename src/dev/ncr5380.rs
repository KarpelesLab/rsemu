//! NCR 5380: a SCSI Interface Device — eight registers and a latch per bus
//! signal.
//!
//! The third initiator in `dev/scsi`'s split, beside
//! [`wd33c93`](crate::dev::wd33c93) and [`ncr53c710`](crate::dev::ncr53c710),
//! and by far the simplest of them: it is a **transceiver with a register
//! file**, not a sequencer. There is no command register, no phase sequence
//! walked by the chip, no transfer counter. The host drives `SEL`, `ATN`,
//! `ACK` and the data bus by writing bits, and reads `BSY`, `REQ`, `MSG`,
//! `C/D` and `I/O` back:
//!
//! > "The NCR 5380 SCSI Interface Device appears as a set of eight registers
//! > to the controlling CPU. By reading and writing the appropriate registers,
//! > the CPU may initiate any SCSI bus activity or may sample and assert any
//! > signal on the SCSI bus. This allows the user to implement all or portions
//! > of the SCSI protocol in software." — §6.0
//!
//! So this file contains **no SCSI command opcode** — no `INQUIRY`, no
//! `READ(10)`, no sense key — for the reason [`crate::dev::scsi`] gives: what
//! a target does with a command descriptor block is the target's business.
//! The grep that falsifies the split is there.
//!
//! # The register file
//!
//! Three address lines and a read/write strobe, which is the whole host
//! interface (§6.0, "Register Summary"):
//!
//! | `A2 A1 A0` | Read | Write |
//! | --- | --- | --- |
//! | `000` | Current SCSI Data | Output Data |
//! | `001` | Initiator Command | Initiator Command |
//! | `010` | Mode | Mode |
//! | `011` | Target Command | Target Command |
//! | `100` | Current SCSI Bus Status | Select Enable |
//! | `101` | Bus and Status | Start DMA Send |
//! | `110` | Input Data | Start DMA Target Receive |
//! | `111` | Reset Parity/Interrupts | Start DMA Initiator Receive |
//!
//! A board decides where those three lines come from, and boards disagree:
//! a Macintosh Plus puts them on `A6`–`A4`, so its register file is sixteen
//! bytes apart. The `stride` property describes that without naming a board —
//! see `RegsWindow`.
//!
//! # What is modelled
//!
//! The **initiator** role, which is what every machine in this tree uses a
//! 5380 for, in all three of the transfer shapes §10 describes:
//!
//! * **Programmed I/O** (§10.1) — the host asserts and releases `ACK` itself
//!   and moves each byte through the Output Data or Current SCSI Data
//!   register.
//! * **Pseudo-DMA** (§10.4) — the host sets `DMA MODE`, writes a Start DMA
//!   register, and then each access to the data register moves one byte with
//!   the chip driving `REQ`/`ACK` itself. `Chip::pseudo_dma` has what that is
//!   an inference from, and what was measured to pin it down.
//! * **Arbitration** (§7) — `ARBITRATE` sets `AIP`; nothing else on this bus
//!   arbitrates, so `LA` never sets.
//!
//! # No clock, and therefore no scheduler event
//!
//! Every other device in this tree takes a `clock` property; this one does not,
//! and that is the part rather than a shortcut:
//!
//! > "The NCR 5380 is a clockless device. Delays such as bus free delay, bus
//! > set delay and bus settle delay are implemented using gate delays. These
//! > delays may differ between devices because of inherent process
//! > variations, but are well within the proposed ANSI X3T9.2 specification."
//! > — §7
//!
//! So there is nothing here for a scheduler to tick: every state change is
//! caused by a host access and happens inside it. The delays the host has to
//! honour are the host's — §7 is explicit that the 2.2 µs arbitration delay
//! "must be implemented in the controlling software driver" — and a driver
//! that spends them in a timing loop spends them in guest cycles, which is
//! where they belong.
//!
//! # What is not
//!
//! * **The target role.** `TARGETMODE`, the Select Enable interrupt and
//!   `ASSERT REQ` are stored and reported but assert nothing: a 5380
//!   pretending to be a disk is a different job, and `scsi.disk` is the disk.
//!   Start DMA Target Receive is accepted and moves nothing.
//! * **Parity.** `ENABLE PARITY CHECKING` is stored and `PARITY ERROR` never
//!   sets, because no byte on this bus was ever carried by a wire. `DBP` in
//!   the Current SCSI Bus Status register is reported as the parity of
//!   whatever is on the data bus, which is what a good byte looks like.
//! * **`EOP`.** The chip has no DMA controller behind it here, so `END OF DMA`
//!   never sets. A Macintosh Plus ties the pin high through 1 kΩ (*Guide to
//!   the Macintosh Family Hardware*, Figure 11-4), which is the same
//!   statement made in hardware.
//! * **Reselection**, because the targets in this tree never disconnect —
//!   [`crate::dev::scsi`] says so and says why.
//!
//! # Sources
//!
//! *NCR 5380 SCSI Interface Chip Design Manual*, NCR Microelectronics, May
//! 1985 — §6 the eight registers bit by bit, §7 the on-chip hardware support,
//! §8 the six interrupt conditions, §9 the three resets, §10 the four transfer
//! modes. X3.131 for what the phases mean; see [`crate::dev::scsi`]. Every
//! non-obvious bit below carries the sentence it came from.
//!
//! **No emulator source of any licence was consulted, and no operating
//! system's SCSI driver was opened** — the Linux `NCR5380.c` is GPL-2.0 and
//! off limits (`CLAUDE.md`, provenance).

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::format;
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
use crate::core::wire::{Level, WireSource};
use crate::dev::scsi::{self, Bus, Phase, buses};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "ncr.5380";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The `IRQ` output pin (§8: "The NCR 5380 provides an interrupt output (IRQ)
/// to indicate a task completion or an abnormal bus occurrence").
pub const IRQ_PIN: &str = "irq";

/// The register window a board maps.
pub const REGS_REGION: &str = "regs";

/// How many registers the chip has (§6.0).
pub const REGISTERS: u64 = 8;

// -- the register numbers (§6.0, Register Summary) ---------------------------

/// Read: Current SCSI Data. Write: Output Data.
pub const DATA: u8 = 0;
/// Initiator Command, read/write.
pub const INITIATOR_COMMAND: u8 = 1;
/// Mode, read/write.
pub const MODE: u8 = 2;
/// Target Command, read/write.
pub const TARGET_COMMAND: u8 = 3;
/// Read: Current SCSI Bus Status. Write: Select Enable.
pub const CURRENT_STATUS: u8 = 4;
/// Read: Bus and Status. Write: Start DMA Send.
pub const BUS_AND_STATUS: u8 = 5;
/// Read: Input Data. Write: Start DMA Target Receive.
pub const INPUT_DATA: u8 = 6;
/// Read: Reset Parity/Interrupts. Write: Start DMA Initiator Receive.
pub const RESET_PARITY_IRQ: u8 = 7;

// -- Initiator Command Register (§6.2) ---------------------------------------

/// `ASSERT RST`, bit 7: "the RST signal (pin 16) is asserted on the SCSI bus".
pub const ICR_ASSERT_RST: u8 = 0x80;
/// `AIP`, bit 6 **read**: arbitration in progress.
pub const ICR_AIP: u8 = 0x40;
/// `TEST MODE`, bit 6 **write**: "disable all output drivers".
pub const ICR_TEST_MODE: u8 = 0x40;
/// `LA`, bit 5 **read**: lost arbitration.
pub const ICR_LA: u8 = 0x20;
/// `DIFF ENBL`, bit 5 **write**: "not used in the NCR 5380".
pub const ICR_DIFF_ENBL: u8 = 0x20;
/// `ASSERT ACK`, bit 4.
pub const ICR_ASSERT_ACK: u8 = 0x10;
/// `ASSERT BSY`, bit 3.
pub const ICR_ASSERT_BSY: u8 = 0x08;
/// `ASSERT SEL`, bit 2.
pub const ICR_ASSERT_SEL: u8 = 0x04;
/// `ASSERT ATN`, bit 1.
pub const ICR_ASSERT_ATN: u8 = 0x02;
/// `ASSERT DATA BUS`, bit 0: "allows the contents of the Output Data Register
/// to be enabled as chip outputs on the signals DB 0-DB7".
pub const ICR_ASSERT_DATA: u8 = 0x01;

// -- Mode Register (§6.3) ----------------------------------------------------

/// `BLOCK MODE DMA`, bit 7.
pub const MODE_BLOCK_DMA: u8 = 0x80;
/// `TARGETMODE`, bit 6: "operate as either an SCSI bus initiator, bit reset
/// (0), or as an SCSI bus target device, bit set (1)".
pub const MODE_TARGET: u8 = 0x40;
/// `ENABLE PARITY CHECKING`, bit 5.
pub const MODE_PARITY_CHECK: u8 = 0x20;
/// `ENABLE PARITY INTERRUPT`, bit 4.
pub const MODE_PARITY_IRQ: u8 = 0x10;
/// `ENABLE EOP INTERRUPT`, bit 3.
pub const MODE_EOP_IRQ: u8 = 0x08;
/// `MONITOR BUSY`, bit 2: "causes an interrupt to be generated for an
/// unexpected loss of BSY".
pub const MODE_MONITOR_BUSY: u8 = 0x04;
/// `DMA MODE`, bit 1: "must be set (1) prior to writing ports 5 through 7".
pub const MODE_DMA: u8 = 0x02;
/// `ARBITRATE`, bit 0: "set (1) to start the arbitration process".
pub const MODE_ARBITRATE: u8 = 0x01;

// -- Target Command Register (§6.4) ------------------------------------------

/// `ASSERT REQ`, bit 3 — target role only: "has no meaning when operating as
/// an Initiator".
pub const TCR_ASSERT_REQ: u8 = 0x08;
/// The phase field, `ASSERT MSG` (bit 2), `ASSERT C/D` (bit 1), `ASSERT I/O`
/// (bit 0) — the same three-bit encoding [`Phase::mci`] produces.
pub const TCR_PHASE: u8 = 0x07;

// -- Current SCSI Bus Status Register (§6.5) ---------------------------------

/// `RST`, bit 7.
pub const CSR_RST: u8 = 0x80;
/// `BSY`, bit 6.
pub const CSR_BSY: u8 = 0x40;
/// `REQ`, bit 5.
pub const CSR_REQ: u8 = 0x20;
/// `MSG`, bit 4.
pub const CSR_MSG: u8 = 0x10;
/// `C/D`, bit 3.
pub const CSR_CD: u8 = 0x08;
/// `I/O`, bit 2.
pub const CSR_IO: u8 = 0x04;
/// `SEL`, bit 1.
pub const CSR_SEL: u8 = 0x02;
/// `DBP`, bit 0 — the data bus parity bit.
pub const CSR_DBP: u8 = 0x01;

// -- Bus and Status Register (§6.7) ------------------------------------------

/// `END OF DMA TRANSFER`, bit 7.
pub const BSR_END_DMA: u8 = 0x80;
/// `DMA REQUEST`, bit 6: "allows the MPU to sample the output pin DRQ".
pub const BSR_DRQ: u8 = 0x40;
/// `PARITY ERROR`, bit 5.
pub const BSR_PARITY_ERROR: u8 = 0x20;
/// `INTERRUPT REQUEST ACTIVE`, bit 4: "reflects the current state of the IRQ
/// (pin 23) output".
pub const BSR_IRQ: u8 = 0x10;
/// `PHASE MATCH`, bit 3: "indicates whether the current SCSI bus phase matches
/// the lower 3 bits of the Target Command Register".
pub const BSR_PHASE_MATCH: u8 = 0x08;
/// `BUSY ERROR`, bit 2: "active if an unexpected loss of the BSY signal (pin
/// 13) has occurred".
pub const BSR_BUSY_ERROR: u8 = 0x04;
/// `ATN`, bit 1.
pub const BSR_ATN: u8 = 0x02;
/// `ACK`, bit 0.
pub const BSR_ACK: u8 = 0x01;

/// Where the chip's lock sits in the ranked order.
///
/// Below [`scsi::BUS_RANK`], for the reason
/// [`wd33c93::CHIP_RANK`](crate::dev::wd33c93::CHIP_RANK) gives: a register
/// write may hold this while it looks a target up, and the target's own lock is
/// above both, so the ladder ascends the whole way. A distinct number from the
/// WD33C93A's and the 53C710's because a board that someday holds two of them
/// deserves a deterministic order rather than a deadlock.
pub const CHIP_RANK: LockRank = LockRank::new(0x4c20);

/// Which way a started pseudo-DMA transfer runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaDir {
    /// Start DMA Send (§6.8.1): initiator → target.
    Send,
    /// Start DMA Initiator Receive (§6.8.3): target → initiator.
    Receive,
}

impl DmaDir {
    /// Whether this direction moves bytes into the host.
    const fn is_input(self) -> bool {
        matches!(self, DmaDir::Receive)
    }
}

// ---------------------------------------------------------------------------
// the chip's state
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct State {
    /// Output Data Register (§6.1.2).
    odr: u8,
    /// Initiator Command Register's writable bits (§6.2).
    icr: u8,
    /// Mode Register (§6.3).
    mode: u8,
    /// Target Command Register (§6.4).
    tcr: u8,
    /// Select Enable Register (§6.6).
    select_enable: u8,
    /// Input Data Register (§6.1.3): "used to read latched data from the SCSI
    /// bus".
    idr: u8,
    /// The byte the target is presenting for the `REQ` now outstanding, in an
    /// input phase. This is the *bus*, not a register: the Current SCSI Data
    /// register reads it (§6.1.1, "allows the microprocessor to read the
    /// active SCSI data bus") and `ACK` latches it into [`State::idr`].
    bus_data: Option<u8>,
    /// Our own bus address, which selection needs in order to know which of
    /// the two IDs in the Output Data Register is the *other* one.
    own_id: u8,
    /// The target we are connected to, if any.
    connected: Option<u8>,
    /// `AIP`: arbitration in progress (§6.2, bit 6).
    aip: bool,
    /// `LA`: arbitration lost (§6.2, bit 5). Never true — nothing else here
    /// arbitrates.
    la: bool,
    /// The phase of the `REQ` now outstanding, if one is.
    ///
    /// **The phase signals belong to the byte, not to the target's idea of
    /// what it is doing next.** A target asserts `REQ` *with* `MSG`, `C/D` and
    /// `I/O` for the byte it is asking about, and moves them only after the
    /// `ACK` for the last byte of that phase (X3.131 §5.1.5) — so this is what
    /// the Current SCSI Bus Status register reports and what `PHASE MATCH`
    /// compares. Asking [`scsi::Target::phase`] instead would run a byte ahead:
    /// [`scsi::Target::read`] hands over the status byte *and* moves the target to
    /// `MESSAGE IN` in one call, and a host that saw `MESSAGE IN` while the
    /// status byte was still on the bus would file the status as a message.
    req_phase: Option<Phase>,
    /// The interrupt latch behind `IRQ` (§8).
    irq: bool,
    /// `BUSY ERROR` (§6.7, bit 2).
    busy_error: bool,
    /// The pseudo-DMA transfer a Start DMA register write began (§6.8).
    dma: Option<DmaDir>,
}

impl State {
    fn new(own_id: u8) -> State {
        State {
            odr: 0,
            icr: 0,
            mode: 0,
            tcr: 0,
            select_enable: 0,
            idr: 0,
            bus_data: None,
            own_id: own_id & 0x07,
            connected: None,
            aip: false,
            la: false,
            req_phase: None,
            irq: false,
            busy_error: false,
            dma: None,
        }
    }

    /// §9.1: "all internal logic and control registers are cleared". The own-ID
    /// property is the board's strapping rather than a register, so it stays.
    fn clear(&mut self) {
        let own = self.own_id;
        *self = State::new(own);
    }

    /// The phase the Target Command Register is asking for (§6.4).
    const fn wanted_phase(&self) -> u8 {
        self.tcr & TCR_PHASE
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
    irq: Mutex<Option<WireSource>>,
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
    // -- what the bus looks like ---------------------------------------------

    /// The phase of whatever we are connected to, or [`Phase::BusFree`].
    ///
    /// **Takes no lock of this chip's**: the target takes its own.
    fn target_phase(&self) -> Phase {
        let id = self.state.lock().connected;
        let Some(id) = id else {
            return Phase::BusFree;
        };
        self.bus
            .target(id)
            .map_or(Phase::BusFree, |target| target.phase())
    }

    /// Bring the outstanding `REQ`, the byte on the data bus and the
    /// connection into line with what the target is now asking for, and raise
    /// whatever interrupt that implies.
    ///
    /// The one place the signal-level model and the phase-level
    /// [`scsi::Target`] trait meet, and the reason it is a function rather
    /// than a line in each caller: every register access that could change the
    /// bus ends here.
    ///
    /// Called with **nothing of this chip's held**.
    fn settle(&self) {
        // A `REQ` the host has not answered, or an `ACK` it is still holding,
        // means the bus is mid-handshake and there is nothing to sample
        // (§10.1: "The REQ bit is sampled until it becomes false and the MPU
        // resets the ASSERT ACK bit to complete the transfer").
        //
        // The loop is for the one case that takes two turns: a target asked
        // for an input phase and has nothing left in it, which it discovers as
        // the empty read is made, so the next turn finds the phase it really
        // wants. Four is a bound rather than a count — `DATA IN` → `STATUS` →
        // `MESSAGE IN` → `BUS FREE` is the longest run there is.
        for _ in 0..4 {
            let connected = {
                let state = self.state.lock();
                if state.req_phase.is_some() || state.icr & ICR_ASSERT_ACK != 0 {
                    return;
                }
                state.connected.is_some()
            };
            if !connected {
                // The host may have just completed a selection; see
                // [`Chip::select`], which decides whether it did.
                self.select();
                if self.state.lock().connected.is_none() {
                    return;
                }
            }
            let phase = self.target_phase();
            if phase == Phase::BusFree {
                self.lost_bsy();
                return;
            }
            // An input phase puts its byte on the data bus when it asserts
            // `REQ`, and the host reads it *before* it answers with `ACK`
            // (§10.1). So the byte is fetched here, with no lock held, rather
            // than when the host gets round to looking.
            let byte = if phase.is_input() {
                let id = self.state.lock().connected;
                let mut buf = [0u8; 1];
                match id.and_then(|id| self.bus.target(id)) {
                    Some(target) if target.read(&mut buf) == 1 => Some(buf[0]),
                    // Nothing to present, so no `REQ`: another turn of the
                    // loop finds the phase the target moved to instead.
                    _ => continue,
                }
            } else {
                None
            };
            let mismatch = {
                let mut state = self.state.lock();
                if state.req_phase.is_some() {
                    return;
                }
                state.bus_data = byte;
                state.req_phase = Some(phase);
                // §8.5: "If the DMA MODE bit (port 2, bit 1) is active and a
                // phase mismatch occurs when REQ (pin 20) transitions from
                // false to true, an interrupt (IRQ) is generated." That is how
                // a blind transfer learns the data phase ended, and on a
                // Macintosh Plus it is the only interrupt software can see.
                let mismatch =
                    state.mode & MODE_DMA != 0 && phase.mci() != Some(state.wanted_phase());
                if mismatch {
                    state.irq = true;
                }
                mismatch
            };
            if mismatch {
                self.refresh();
            }
            return;
        }
    }

    /// The target has released the cable: X3.131 §5.1.1's bus free, however it
    /// came about.
    fn lost_bsy(&self) {
        let monitor = {
            let mut state = self.state.lock();
            state.connected = None;
            state.req_phase = None;
            state.bus_data = None;
            state.dma = None;
            state.mode & MODE_MONITOR_BUSY != 0
        };
        if !monitor {
            return;
        }
        // §8.6: "If the MONITOR BUSY bit (bit 2) in the Mode Register (port 2)
        // is active, an interrupt will be generated if the BSY signal (pin 13)
        // goes false for at least a bus settle delay", and §6.7 bit 2: an
        // unexpected loss of `BSY` "will disable any SCSI outputs and will
        // reset the DMA MODE bit".
        let mut state = self.state.lock();
        state.busy_error = true;
        state.mode &= !MODE_DMA;
        state.icr &= !0x3f;
        state.irq = true;
        drop(state);
        self.refresh();
    }

    /// One byte across the bus in whichever direction the phase runs: the
    /// `REQ`/`ACK` handshake's effect, however the host spelled it.
    ///
    /// Returns the byte an input phase moved. Called with **nothing held**.
    fn handshake(&self) -> Option<u8> {
        let (phase, id, odr) = {
            let mut state = self.state.lock();
            let phase = state.req_phase.take()?;
            (phase, state.connected, state.odr)
        };
        id?;
        if phase.is_input() {
            let mut state = self.state.lock();
            let byte = state.bus_data.take();
            if let Some(byte) = byte {
                // §6.1.3: the Input Data Register is where a receive latches
                // what `REQ` presented.
                state.idr = byte;
            }
            byte
        } else {
            if let Some(target) = id.and_then(|id| self.bus.target(id)) {
                target.write(&[odr]);
            }
            None
        }
    }

    // -- the eight registers, read ------------------------------------------

    /// Read register `reg`. `debug` promises no side effect of any kind.
    fn read_register(&self, reg: u8, debug: bool) -> u8 {
        match reg & 0x07 {
            // §6.1.1: "a read-only register which allows the microprocessor to
            // read the active SCSI data bus". In programmed I/O that is a
            // passive look at the bus — the byte is consumed by `ACK`, not by
            // reading it, which is what makes it safe for a debugger. Under a
            // started pseudo-DMA transfer the same access *is* the transfer;
            // see [`Chip::pseudo_dma`].
            DATA | INPUT_DATA => {
                if !debug && let Some(byte) = self.pseudo_dma(None) {
                    return byte;
                }
                let state = self.state.lock();
                if reg & 0x07 == INPUT_DATA {
                    // §6.1.3: the Input Data Register is a latch, and a
                    // debugger reading it is the same read as the guest's.
                    return state.idr;
                }
                state
                    .bus_data
                    .unwrap_or(if state.icr & ICR_ASSERT_DATA != 0 || state.aip {
                        state.odr
                    } else {
                        0
                    })
            }
            INITIATOR_COMMAND => {
                let state = self.state.lock();
                // §6.2: the read image keeps every asserted signal and swaps
                // bits 6 and 5 for `AIP` and `LA`.
                let mut value = state.icr & !(ICR_AIP | ICR_LA);
                if state.aip {
                    value |= ICR_AIP;
                }
                if state.la {
                    value |= ICR_LA;
                }
                value
            }
            MODE => self.state.lock().mode,
            TARGET_COMMAND => self.state.lock().tcr & (TCR_ASSERT_REQ | TCR_PHASE),
            CURRENT_STATUS => self.current_status(),
            BUS_AND_STATUS => self.bus_and_status(),
            RESET_PARITY_IRQ => {
                if !debug {
                    // §6.9: "Reading this register resets the PARITY ERROR bit
                    // (bit 5), the INTERRUPT REQUEST bit (bit 4) and the BUSY
                    // ERROR bit (bit 2) in the Bus and Status Register".
                    let mut state = self.state.lock();
                    state.irq = false;
                    state.busy_error = false;
                    drop(state);
                    self.refresh();
                }
                // The data sheet gives no value for this read — the register is
                // a strobe. **Inference**: nothing drives the bus, and zero is
                // as good an answer as any for a byte every driver discards.
                0
            }
            _ => unreachable!("three bits select one of eight registers"),
        }
    }

    /// §6.5, the Current SCSI Bus Status register.
    fn current_status(&self) -> u8 {
        let state = self.state.lock();
        // The phase signals of the outstanding `REQ` — see [`State::req_phase`].
        // Between bytes nothing is asserted, which is the honest report of a
        // bus whose target is not asking for anything.
        let phase = state.req_phase.unwrap_or(Phase::BusFree);
        let mut value = 0;
        if state.icr & ICR_ASSERT_RST != 0 {
            value |= CSR_RST;
        }
        // `BSY` is the wire, not our latch: ours while we assert it or while we
        // are arbitrating (§6.2 bit 6: arbitration "has asserted BSY"), the
        // target's once it has answered a selection.
        if state.icr & ICR_ASSERT_BSY != 0 || state.aip || state.connected.is_some() {
            value |= CSR_BSY;
        }
        if state.req_phase.is_some() {
            value |= CSR_REQ;
        }
        if let Some(mci) = phase.mci() {
            if mci & 0b100 != 0 {
                value |= CSR_MSG;
            }
            if mci & 0b010 != 0 {
                value |= CSR_CD;
            }
            if mci & 0b001 != 0 {
                value |= CSR_IO;
            }
        }
        if state.icr & ICR_ASSERT_SEL != 0 {
            value |= CSR_SEL;
        }
        // Parity is generated, never checked (see the module docs), so the bit
        // always reports a good byte: odd parity over whatever is on the bus.
        let data = state.bus_data.unwrap_or(state.odr);
        if data.count_ones().is_multiple_of(2) {
            value |= CSR_DBP;
        }
        value
    }

    /// §6.7, the Bus and Status register.
    fn bus_and_status(&self) -> u8 {
        let state = self.state.lock();
        let phase = state.req_phase.unwrap_or(Phase::BusFree);
        let mut value = 0;
        // §6.7 bit 3: "PHASE MATCH is continuously updated and is only
        // significant when operating as a bus initiator."
        if phase.mci() == Some(state.wanted_phase()) {
            value |= BSR_PHASE_MATCH;
        }
        // §10.4: the host polls this bit instead of the `DRQ` pin. It is the
        // chip having a byte to move under a started transfer, which needs the
        // phase to match — §8.5: "a phase mismatch prevents the recognition of
        // REQ".
        if state.dma.is_some() && state.req_phase.is_some() && value & BSR_PHASE_MATCH != 0 {
            value |= BSR_DRQ;
        }
        if state.irq {
            value |= BSR_IRQ;
        }
        if state.busy_error {
            value |= BSR_BUSY_ERROR;
        }
        if state.icr & ICR_ASSERT_ATN != 0 {
            value |= BSR_ATN;
        }
        if state.icr & ICR_ASSERT_ACK != 0 {
            value |= BSR_ACK;
        }
        value
    }

    // -- the eight registers, written ---------------------------------------

    /// Write register `reg`.
    fn write_register(&self, reg: u8, value: u8) {
        match reg & 0x07 {
            DATA => {
                self.state.lock().odr = value;
                // Under a started pseudo-DMA send the write *is* the transfer.
                self.pseudo_dma(Some(value));
            }
            INITIATOR_COMMAND => self.write_icr(value),
            MODE => self.write_mode(value),
            TARGET_COMMAND => {
                self.state.lock().tcr = value & (TCR_ASSERT_REQ | TCR_PHASE);
                // A new Target Command Register can turn a match into a
                // mismatch, which §8.5 only reports on the next `REQ` edge, so
                // nothing is raised here.
            }
            CURRENT_STATUS => self.state.lock().select_enable = value,
            // §6.8: "Simply writing these registers starts the DMA transfers.
            // Data presented to the NCR 5380 on signals D0-D7 during the
            // register write is meaningless and has no effect".
            BUS_AND_STATUS => self.start_dma(DmaDir::Send),
            // Target role: accepted, and moves nothing. See the module docs.
            INPUT_DATA => {}
            RESET_PARITY_IRQ => self.start_dma(DmaDir::Receive),
            _ => unreachable!("three bits select one of eight registers"),
        }
        self.settle();
    }

    /// §6.2, and the four transitions that do something.
    fn write_icr(&self, value: u8) {
        let (before, rst_rose) = {
            let mut state = self.state.lock();
            let before = state.icr;
            state.icr = value;
            (
                before,
                before & ICR_ASSERT_RST == 0 && value & ICR_ASSERT_RST != 0,
            )
        };
        if rst_rose {
            self.assert_rst();
            return;
        }
        let rose = |bit: u8| before & bit == 0 && value & bit != 0;
        let fell = |bit: u8| before & bit != 0 && value & bit == 0;

        if rose(ICR_ASSERT_ACK) {
            // §10.1: the host has answered the outstanding `REQ`.
            self.handshake();
        }
        // §6.2 bit 3 says of `ASSERT BSY` that "resetting this bit creates a
        // bus disconnect condition", and that is deliberately **not** acted on
        // here. `BSY` is open collector, and X3.131's selection phase (§5.1.3)
        // has the initiator release it as soon as the target answers — the
        // target holds it for the whole connection — so an initiator dropping
        // its own `BSY` is the ordinary end of a selection, not a disconnect.
        // A connection here ends where the bus says it ends: the target
        // releasing the cable after `COMMAND COMPLETE`, which
        // [`Chip::settle`] notices, or one of the two resets of §9.
        let _ = fell(ICR_ASSERT_BSY);
    }

    /// §6.3, and the one transition that does something.
    fn write_mode(&self, value: u8) {
        let arbitrate = {
            let mut state = self.state.lock();
            let before = state.mode;
            state.mode = value;
            if value & MODE_DMA == 0 {
                // §10.5.3: "A DMA operation may be halted at any time simply by
                // resetting the DMA MODE bit."
                state.dma = None;
            }
            if value & MODE_ARBITRATE == 0 {
                // §6.2 bit 6: "AIP will remain active until the ARBITRATE bit
                // is reset."
                state.aip = false;
                state.la = false;
            }
            before & MODE_ARBITRATE == 0 && value & MODE_ARBITRATE != 0
        };
        if arbitrate {
            self.arbitrate();
        }
    }

    /// §7: "Arbitration will begin if the bus is free, SEL is inactive and the
    /// ARBITRATION bit (port 2, bit 0) is active."
    ///
    /// §6.3 bit 0: "Prior to setting this bit the Output Data Register should
    /// contain the proper SCSI device ID value. Only one data bit should be
    /// active for SCSI bus arbitration." So this is also where the chip learns
    /// which address is ours, which selection needs.
    fn arbitrate(&self) {
        let mut state = self.state.lock();
        if state.connected.is_some() || state.icr & ICR_ASSERT_SEL != 0 {
            return;
        }
        state.aip = true;
        // Nothing else on this cable arbitrates, so we always win (§6.3 bit 0:
        // the result is read back through `LA` and `AIP`).
        state.la = false;
        if state.odr.count_ones() == 1 {
            state.own_id = state.odr.trailing_zeros() as u8;
        }
    }

    /// The selection phase, if the bus signals now say one is happening.
    ///
    /// **A condition, not an edge**, and that distinction is the whole of it.
    /// The data sheet says what a selection looks like from the far end, in
    /// §8.1, where it is this chip being selected as a target:
    ///
    /// > "The NCR 5380 can generate a select interrupt if SEL (pin 12) is true
    /// > (1), its device ID is true (1) and BSY (pin 13) is false for at least
    /// > a bus settle delay (400 ns)."
    ///
    /// Three signals, sampled together — which is what X3.131's selection
    /// phase (§5.1.3) requires of every target. So four things have to be true
    /// at once here: `SEL` asserted, the data bus driven (§6.2 bit 0, `ASSERT
    /// DATA BUS`) with an ID that is not ours on it, our own `BSY` released,
    /// and arbitration over — and a host reaches that state one register write
    /// at a time, in whatever order it likes.
    ///
    /// Apple's SCSI Manager, measured through this chip's own window, asserts
    /// `SEL` *before* it puts the target's ID on the data bus and *before* it
    /// releases `BSY`:
    ///
    /// ```text
    ///   ODR := $80            our own ID, for arbitration
    ///   Mode := $01           ARBITRATE
    ///   ICR  := $04           ASSERT SEL, with nothing on the data bus yet
    ///   ODR  := $81           now both IDs
    ///   ICR  := $0d           ASSERT BSY + SEL + DATA BUS
    ///   Mode := $00           arbitration over
    ///   ICR  := $05           BSY released — *this* is the selection
    /// ```
    ///
    /// A model that selected on the `SEL` write would have looked for a target
    /// while the data bus still held only our own ID, found none, and never
    /// tried again: `tests/mac_plus_scsi.rs` is where that showed up.
    ///
    /// A missing target is simply nobody answering: `BSY` never appears and the
    /// host times its own selection out, which is what §8.1's interrupt would
    /// otherwise have told a target.
    fn select(&self) {
        let (id, atn) = {
            let state = self.state.lock();
            if state.connected.is_some() || state.aip {
                return;
            }
            if state.icr & ICR_ASSERT_SEL == 0
                || state.icr & ICR_ASSERT_DATA == 0
                || state.icr & ICR_ASSERT_BSY != 0
            {
                return;
            }
            let others = state.odr & !(1u8 << state.own_id);
            if others == 0 {
                return;
            }
            (
                others.trailing_zeros() as u8,
                state.icr & ICR_ASSERT_ATN != 0,
            )
        };
        let Some(target) = self.bus.target(id) else {
            return;
        };
        if !target.select(atn) {
            return;
        }
        self.state.lock().connected = Some(id);
    }

    /// §6.8: a Start DMA register has been written.
    fn start_dma(&self, dir: DmaDir) {
        let mut state = self.state.lock();
        // §6.8.1 and §6.8.3: "The DMA MODE bit (port 2, bit 1) must be set
        // prior to writing this register", and for an initiator receive
        // "the TARGETMODE bit (bit 6) must be false (0)".
        if state.mode & MODE_DMA == 0 {
            return;
        }
        if dir == DmaDir::Receive && state.mode & MODE_TARGET != 0 {
            return;
        }
        state.dma = Some(dir);
    }

    /// One byte of a started pseudo-DMA transfer, if one is started and the
    /// target is asking for a byte. `out` is the byte a send is offering.
    ///
    /// Returns the byte a receive moved, so the register read that caused it
    /// can hand it straight over.
    ///
    /// # What is measured and what is inferred
    ///
    /// **Inference**, and the most load-bearing one in this file. The data
    /// sheet has the host read a DMA byte out of the Input Data Register "under
    /// DMA control using IOR and DACK" (§6.1.3) — two pins this chip has and
    /// this model does not, because nothing here is wired to a DMA controller.
    /// §10.4 is what makes that a board's choice rather than the chip's:
    ///
    /// > "This mode is implemented by programming the NCR 5380 to operate in
    /// > the DMA mode, but using the MPU to emulate the DMA handshake. … Once
    /// > DRQ is detected, the MPU can perform a DMA port read or write data
    /// > transfer. This MPU read/write is externally decoded to generate the
    /// > appropriate DACK and IOR or IOW signals."
    ///
    /// So *which* access a board turns into a `/DACK` cycle is the board's
    /// decode, and on a Macintosh Plus it is a read or write of the **data
    /// register** — which is measured, not guessed. Apple's own SCSI Manager,
    /// traced through this chip's window while it read `INQUIRY` data off a
    /// disk (`tests/mac_plus_scsi.rs`), does exactly this and nothing else:
    ///
    /// ```text
    ///   TCR  := $01           the DATA IN phase
    ///   Mode := $02           DMA MODE
    ///   $580071 := $00        Start DMA Initiator Receive
    ///   loop:  read $580050   Bus and Status, until DRQ
    ///          read $580000   one byte
    /// ```
    ///
    /// There is no second address in that loop, so on this board the byte has
    /// to come out of the data register. A model in which the read were
    /// passive hands the driver the same byte thirty-six times, which is what
    /// it did before this was measured.
    ///
    /// The other half of the inference is *why an access moves anything*.
    /// §6.3 bit 1: "In the DMA mode, REQ (pin 20) and ACK (pin 14) are
    /// automatically controlled" — the chip drives the handshake rather than
    /// the host, so the access is the only thing left that can pace it. That
    /// is also what makes a blind transfer work, where the host does not poll
    /// `DRQ` at all.
    fn pseudo_dma(&self, out: Option<u8>) -> Option<u8> {
        {
            let state = self.state.lock();
            let dir = state.dma?;
            let phase = state.req_phase?;
            // §8.5: "A phase mismatch prevents the recognition of REQ…" — a
            // transfer whose phase has ended moves nothing, which is how the
            // host finds out it has ended.
            if phase.mci() != Some(state.wanted_phase()) || phase.is_input() != dir.is_input() {
                return None;
            }
            // A send offers a byte and a receive does not; the other way round
            // is the caller having confused its own direction.
            if out.is_some() == dir.is_input() {
                return None;
            }
        }
        let byte = self.handshake();
        self.settle();
        // A receive hands over what it moved; a send has nothing to give back,
        // and the caller ignores it.
        byte.or_else(|| out.map(|_| 0))
    }

    // -- the resets ----------------------------------------------------------

    /// §9.3: the host has set `ASSERT RST`.
    ///
    /// "The RST signal (pin 16) goes active on the SCSI bus and an internal
    /// reset is performed. Again, all internal logic and registers are cleared
    /// except for the IRQ interrupt latch and the ASSERT RST bit (bit 7) in the
    /// Initiator Command Register (port 1)."
    fn assert_rst(&self) {
        {
            let mut state = self.state.lock();
            state.clear();
            state.icr = ICR_ASSERT_RST;
            // §8.3: "The NCR 5380 generates an interrupt when the RST signal
            // (pin 16) transitions to true… This interrupt also occurs after
            // setting the ASSERT RST bit (port 1, bit 7). This interrupt cannot
            // be disabled."
            state.irq = true;
        }
        // X3.131 §5.2.2 through [`Bus::reset`], with nothing of ours held.
        self.bus.reset();
        self.refresh();
    }

    /// §9.1, the `RESET/` pin: "the NCR 5380 device is re-initialized and all
    /// internal logic and control registers are cleared. This is a chip reset
    /// only and does not create an SCSI bus reset condition."
    fn hardware_reset(&self) {
        let id = {
            let mut state = self.state.lock();
            let id = state.connected;
            state.clear();
            id
        };
        if let Some(target) = id.and_then(|id| self.bus.target(id)) {
            // The chip has stopped driving `BSY`, which the target sees as the
            // initiator letting go — X3.131 §5.1.1's unexpected bus free.
            target.release();
        }
        self.refresh();
    }

    // -- the pin -------------------------------------------------------------

    /// Drive `IRQ` from the latch, with nothing held while the net delivers.
    fn refresh(&self) {
        let high = self.state.lock().irq;
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(high));
        }
    }
}

// ---------------------------------------------------------------------------
// the register window
// ---------------------------------------------------------------------------

/// The chip's registers, decoded the way one board wired them.
///
/// The 5380 has three address lines and a read/write strobe (§6.0), and a board
/// chooses where those come from. One number describes every decode this tree
/// needs: `stride`, the bytes between consecutive register selects. `1` is the
/// chip's own `A2`–`A0`; a Macintosh Plus puts them on `A6`–`A4`, which is
/// `16`, so its register file is 128 bytes long and repeats above that.
///
/// **Inference, and marked as one**: the direction comes from the *access*,
/// not from the address. *Guide to the Macintosh Family Hardware* (ch. 1,
/// "device address space") says "In the Macintosh SE, for example, `$580000`
/// selects a SCSI read, whereas `$580001` selects a SCSI write", and a real
/// Macintosh ROM writes this chip at odd addresses — measured: `$580011`,
/// `$580021`. But a decoder that took the strobe from `A0` alone would drive
/// `/IOR` during a processor *write* to an even address, which is nonsense, so
/// the model treats `A0` as a don't-care that software sets to match and lets
/// the bus cycle say which strobe fires. Nothing in this tree reads a write
/// address or writes a read address, so the two readings cannot be told apart
/// from outside.
///
/// There is deliberately **no `/DACK` address**. The chip has the pin and §10.4
/// has a board decode an address to it, but no document in hand says which
/// address a Macintosh Plus uses, and Apple's own driver does its pseudo-DMA
/// through the data register — see [`Chip::pseudo_dma`]. An address bit
/// invented for the purpose is exactly the mistake `docs/platforms/mac-plus.md`
/// records under "the drive's register file was invented".
#[derive(Debug)]
struct RegsWindow {
    chip: Arc<Chip>,
    stride: u64,
}

impl RegsWindow {
    /// The register an offset selects.
    fn decode(&self, offset: u64) -> u8 {
        ((offset / self.stride) & 0x07) as u8
    }

    /// How long the window is: the whole span the board's decode covers, so a
    /// machine file can mirror it through the chip select's address range.
    const fn len(stride: u64) -> u64 {
        REGISTERS * stride
    }
}

impl MemOps for RegsWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.is_empty() {
            return Err(BusError::BadAccess);
        }
        // One register access however wide the cycle was — see
        // [`RegsWindow::constraints`].
        let byte = self.chip.read_register(self.decode(offset), attrs.debug);
        dst.fill(byte);
        if !attrs.debug {
            self.chip.settle();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(&value) = src.last() else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A write here asserts `RST`, selects a target or pushes a byte
            // onto the bus; none of it can be made harmless (invariant 5).
            return Err(BusError::BadAccess);
        }
        self.chip.write_register(self.decode(offset), value);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // An eight-bit part on a word bus, and **a wider access does not
        // fault**. A Macintosh Plus has no bus-error timeout anywhere in its
        // map — `/DTACK` comes from the address decoder and the MC68000 user's
        // manual (§5.4) leaves `/BERR` to circuitry this board omits — so a
        // word access to a byte-wide part is answered rather than refused, and
        // `tests/mac_plus_board.rs::an_address_nothing_claims_floats` is the
        // assertion that says so.
        //
        // **Inference**, because it is not observable: `A0` is not in this
        // chip's register decode, so both data strobes of a word cycle select
        // the *same* register, and the chip drives its own byte lane while the
        // other floats. Which lane that is cannot be told from the documents
        // (see the type docs), so a read answers with the register's byte in
        // both halves and a write takes the low half — the lane a `move.b` to
        // an odd address uses, which is the one a real ROM was measured
        // writing. Nothing in this tree makes a word access to a SCSI
        // register, so the two readings cannot be told apart.
        AccessConstraints {
            min: Width::U8,
            max: Width::U32,
            natural_alignment: false,
            ..AccessConstraints::word(Width::U8, Endian::Big)
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An NCR 5380 on a named SCSI bus.
#[derive(Debug)]
pub struct Ncr5380 {
    chip: Arc<Chip>,
    regs: RegionRef,
}

impl Ncr5380 {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or unknown, and
    /// [`Error::Config`] if the two decode numbers do not describe a window.
    pub fn new(props: &Props) -> Result<Ncr5380> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", scsi::DEFAULT_BUS)?.to_string();
        // *Guide to the Macintosh Family Hardware*, ch. 2: "The main logic
        // board has ID number 7, giving it the highest priority in case of
        // contention for the bus with another initiator." The chip itself has
        // no own-ID register — §6.3 bit 0 has the host put its ID in the Output
        // Data Register before arbitrating — so this is the board's strapping
        // and a driver that arbitrates overwrites it.
        let own = r.or_range("id", 7u64, 0..=7)? as u8;
        let stride = r.or_addr("stride", 1)?;
        r.finish()?;
        if stride == 0 || !stride.is_power_of_two() {
            return Err(Error::Config {
                at: CLASS_NAME.to_string(),
                message: format!(
                    "`stride` is how far apart the board put the chip's three register selects, \
                     so it is a power of two, not {stride}"
                ),
            });
        }
        let bus = buses::attach(props, &bus_name)?;
        Ok(Ncr5380::with_bus(bus, bus_name, own, stride))
    }

    /// Build one around a bus the caller already has.
    #[must_use]
    pub fn with_bus(bus: Arc<Bus>, bus_name: String, own: u8, stride: u64) -> Ncr5380 {
        let chip = Arc::new(Chip {
            state: Mutex::with_rank(CHIP_RANK, State::new(own)),
            bus,
            bus_name,
            irq: Mutex::with_rank(LockRank::LEAF, None),
        });
        let window = RegsWindow {
            chip: Arc::clone(&chip),
            stride,
        };
        Ncr5380 {
            regs: Arc::new(Region::io(
                format!("{CLASS_NAME}.{REGS_REGION}"),
                RegsWindow::len(stride),
                Arc::new(window),
            )),
            chip,
        }
    }

    /// The bus it is on.
    #[must_use]
    pub fn bus(&self) -> Arc<Bus> {
        Arc::clone(&self.chip.bus)
    }

    /// Whether `IRQ` is asserted, without disturbing anything.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.chip.state.lock().irq
    }

    /// The Bus and Status register, without the read of port 7 that would
    /// clear its latches — what a debugger and a test both want.
    #[must_use]
    pub fn bus_and_status(&self) -> u8 {
        self.chip.bus_and_status()
    }

    /// The Current SCSI Bus Status register, likewise.
    #[must_use]
    pub fn current_status(&self) -> u8 {
        self.chip.current_status()
    }

    /// The target this chip is connected to, if any.
    #[must_use]
    pub fn connected(&self) -> Option<u8> {
        self.chip.state.lock().connected
    }
}

/// The `ncr.5380` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an NCR 5380 SCSI Interface Device: the eight registers, the signal latches, \
              arbitration, selection and both of the initiator's transfer modes",
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
            summary: "the initiator's own SCSI address as the board straps it, 0 to 7 (default 7)",
        },
        PropertySpec {
            name: "stride",
            kind: ValueKind::Uint,
            required: false,
            summary: "bytes between consecutive register selects, as the board wired A2-A0 \
                      (default 1; a Macintosh Plus is 16)",
        },
    ],
    construct: |props| Ok(Box::new(Ncr5380::new(props)?)),
};

impl Device for Ncr5380 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: `map` places the window and the wire graph brings
        // the pin.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: `RESET/` is a pin and a board reset pulls it (§9.1).
        self.chip.hardware_reset();
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
                message: String::from("an NCR 5380 drives one pin: `irq`"),
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

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.chip.state.lock();
        w.write_u8(state.odr)?;
        w.write_u8(state.icr)?;
        w.write_u8(state.mode)?;
        w.write_u8(state.tcr)?;
        w.write_u8(state.select_enable)?;
        w.write_u8(state.idr)?;
        match state.bus_data {
            None => w.write_bool(false)?,
            Some(byte) => {
                w.write_bool(true)?;
                w.write_u8(byte)?;
            }
        }
        w.write_u8(state.own_id)?;
        match state.connected {
            None => w.write_bool(false)?,
            Some(id) => {
                w.write_bool(true)?;
                w.write_u8(id)?;
            }
        }
        w.write_bool(state.aip)?;
        w.write_bool(state.la)?;
        w.write_u8(phase_code(state.req_phase))?;
        w.write_bool(state.irq)?;
        w.write_bool(state.busy_error)?;
        w.write_u8(match state.dma {
            None => 0,
            Some(DmaDir::Send) => 1,
            Some(DmaDir::Receive) => 2,
        })
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let odr = r.read_u8()?;
        let icr = r.read_u8()?;
        let mode = r.read_u8()?;
        let tcr = r.read_u8()?;
        let select_enable = r.read_u8()?;
        let idr = r.read_u8()?;
        let bus_data = if r.read_bool()? {
            Some(r.read_u8()?)
        } else {
            None
        };
        let own_id = r.read_u8()? & 0x07;
        let connected = if r.read_bool()? {
            Some(r.read_u8()? & 0x07)
        } else {
            None
        };
        let aip = r.read_bool()?;
        let la = r.read_bool()?;
        let req_phase = phase_from(r.read_u8()?)?;
        let irq = r.read_bool()?;
        let busy_error = r.read_bool()?;
        let dma = match r.read_u8()? {
            0 => None,
            1 => Some(DmaDir::Send),
            2 => Some(DmaDir::Receive),
            other => {
                return Err(Error::State(format!(
                    "{other} is not a pseudo-DMA direction"
                )));
            }
        };
        let mut state = self.chip.state.lock();
        *state = State {
            odr,
            icr,
            mode,
            tcr,
            select_enable,
            idr,
            bus_data,
            own_id,
            connected,
            aip,
            la,
            req_phase,
            irq,
            busy_error,
            dma,
        };
        drop(state);
        // A restore does not re-run the wire graph; put the pin where the latch
        // says.
        self.chip.refresh();
        Ok(())
    }
}

impl Instance for Ncr5380 {}

/// An outstanding `REQ`'s phase as one snapshot byte; zero is no `REQ`.
const fn phase_code(phase: Option<Phase>) -> u8 {
    match phase {
        None | Some(Phase::BusFree) => 0,
        Some(Phase::DataOut) => 1,
        Some(Phase::DataIn) => 2,
        Some(Phase::Command) => 3,
        Some(Phase::Status) => 4,
        Some(Phase::MessageOut) => 5,
        Some(Phase::MessageIn) => 6,
    }
}

/// The inverse, rejecting a byte that names no phase.
fn phase_from(code: u8) -> Result<Option<Phase>> {
    match code {
        0 => Ok(None),
        1 => Ok(Some(Phase::DataOut)),
        2 => Ok(Some(Phase::DataIn)),
        3 => Ok(Some(Phase::Command)),
        4 => Ok(Some(Phase::Status)),
        5 => Ok(Some(Phase::MessageOut)),
        6 => Ok(Some(Phase::MessageIn)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ncr5380::new(props)?)))
}

/// What the validator should know about `ncr.5380`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("id", ValueKind::Uint))
        .prop(PropSchema::new("stride", ValueKind::Uint))
        .region("")
        .region(REGS_REGION)
        .port(IRQ_PIN, PortDir::Out)
}
