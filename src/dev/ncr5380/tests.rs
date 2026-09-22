//! The 5380 driven the way a host driver drives it: bits in, bits out, and the
//! protocol in the test rather than in the chip.
//!
//! That is the point of the part, so it is the point of this suite. The target
//! is a real [`ScsiDisk`] on a real [`Bus`], and every byte reaches it through
//! the register window at the addresses a Macintosh Plus decodes — register
//! selects sixteen bytes apart, reads at even addresses and writes at odd ones
//! — so the decode is under test as well.
//!
//! The command set is [`crate::dev::scsi::disk`]'s and its own suite is
//! `src/dev/scsi/tests.rs`. What is asserted here is the *phase sequence*: that
//! a driver writing 5380 registers gets an `INQUIRY` answered, and that the
//! chip's status bits say what the data sheet says they say.

use super::*;
use crate::core::device::Device;
use crate::core::props::{Media, Props, Value};
use crate::core::space::{AddressSpace, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use crate::dev::scsi::disk::{DiskDevice, ScsiDisk};
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, Ordering};

/// The drive's SCSI address, and the initiator's.
const TARGET: u8 = 0;
const OWN: u8 = 7;

/// How many blocks the drive under test holds.
const BLOCKS: u64 = 64;
const BLOCK: usize = 512;

/// The Macintosh Plus's decode: register selects on `A6`-`A4` (measured — see
/// `docs/platforms/mac-plus.md`).
const STRIDE: u64 = 16;

/// A wire sink that remembers the last level it was given.
#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
}

impl Probe {
    fn high(&self) -> bool {
        self.level.load(Ordering::Relaxed) != 0
    }
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

struct Rig {
    chip: Arc<Ncr5380>,
    /// The decode this rig's board wired, so a helper can address the chip the
    /// way that board does.
    stride: u64,
    /// The drive, kept alive: a bus holds `Arc<dyn Target>`, and the device
    /// wrapper is what owns the medium.
    _disk: DiskDevice,
    drive: Arc<ScsiDisk>,
    irq: Arc<Probe>,
    /// The chip's window, mapped at zero in a space of its own, so the
    /// registers are reached the way a board reaches them.
    window: Arc<AddressSpace>,
}

impl Rig {
    /// Read register `reg` at the address a Macintosh reads it (`A0` low).
    fn rd(&self, reg: u8) -> u8 {
        self.window
            .read(u64::from(reg) * self.stride, Width::U8, MemAttrs::DEFAULT)
            .expect("the window answers") as u8
    }

    /// The same, the way a debugger reads it.
    fn peek(&self, reg: u8) -> u8 {
        self.window
            .read(u64::from(reg) * self.stride, Width::U8, MemAttrs::DEBUG)
            .expect("the window answers") as u8
    }

    /// Write register `reg` at the address the board writes it: a Macintosh
    /// puts `A0` high for a write, and a board with the chip's own pinout has
    /// nowhere to put it.
    fn wr(&self, reg: u8, value: u8) {
        let at = u64::from(reg) * self.stride + u64::from(self.stride > 1);
        self.window
            .write(at, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("the window answers");
    }

    /// The phase the bus is in, as the Current SCSI Bus Status register
    /// reports it — `MSG`, `C/D`, `I/O`, which is [`Phase::mci`]'s field.
    fn phase(&self) -> Option<u8> {
        let csr = self.rd(CURRENT_STATUS);
        if csr & CSR_BSY == 0 {
            return None;
        }
        let mut mci = 0;
        if csr & CSR_MSG != 0 {
            mci |= 0b100;
        }
        if csr & CSR_CD != 0 {
            mci |= 0b010;
        }
        if csr & CSR_IO != 0 {
            mci |= 0b001;
        }
        Some(mci)
    }
}

fn rig() -> Rig {
    rig_with(STRIDE)
}

fn rig_with(stride: u64) -> Rig {
    // Block *n* of the drive is filled with the byte *n*.
    let mut image = vec![0u8; BLOCKS as usize * BLOCK];
    for (n, block) in image.chunks_mut(BLOCK).enumerate() {
        block.fill(n as u8);
    }
    let hosts = Arc::new(crate::core::hosts::HostObjects::new());
    let disk = DiskDevice::new(
        &Props::new()
            .with("image", Value::Media(Media::new("hd0", image)))
            .with("bus", Value::Str(String::from("scsi0")))
            .with("id", Value::Uint(u64::from(TARGET)))
            .with_hosts(Arc::clone(&hosts)),
    )
    .expect("a drive");
    let drive = disk.drive().expect("occupied");
    let chip = Arc::new(
        Ncr5380::new(
            &Props::new()
                .with("bus", Value::Str(String::from("scsi0")))
                .with("id", Value::Uint(u64::from(OWN)))
                .with("stride", Value::Uint(stride))
                .with_hosts(Arc::clone(&hosts)),
        )
        .expect("a controller"),
    );

    let irq = Arc::new(Probe::default());
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&irq) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(chip.as_ref(), IRQ_PIN, WireSource::new(wire, id)).expect("the pin exists");
    Device::announce(chip.as_ref(), IRQ_PIN);

    let window = Arc::new(AddressSpace::new("ncr", 16).with_unassigned(UnassignedPolicy::ZEROS));
    window
        .topology()
        .map(Device::region(chip.as_ref(), "").expect("a region"), 0)
        .expect("mapped");
    Rig {
        chip,
        stride,
        _disk: disk,
        drive,
        irq,
        window,
    }
}

// ---------------------------------------------------------------------------
// a driver, written out of §10.1 and §10.4
// ---------------------------------------------------------------------------

/// Arbitrate and select `TARGET` with `ATN`, leaving the bus in `MESSAGE OUT`.
///
/// §7: "Arbitration will begin if the bus is free, SEL is inactive and the
/// ARBITRATION bit (port 2, bit 0) is active"; §6.3 bit 0: "Prior to setting
/// this bit the Output Data Register should contain the proper SCSI device ID
/// value"; §6.2 bit 2: "SEL is normally asserted after arbitration has been
/// successfully completed."
fn select(rig: &Rig) {
    rig.wr(DATA, 1 << OWN);
    rig.wr(MODE, MODE_ARBITRATE);
    assert_eq!(
        rig.rd(INITIATOR_COMMAND) & (ICR_AIP | ICR_LA),
        ICR_AIP,
        "arbitration is in progress and was not lost"
    );
    assert_eq!(
        rig.rd(DATA),
        1 << OWN,
        "§8.1: the Current SCSI Data Register shows who is arbitrating"
    );
    rig.wr(DATA, (1 << OWN) | (1 << TARGET));
    rig.wr(
        INITIATOR_COMMAND,
        ICR_ASSERT_SEL | ICR_ASSERT_ATN | ICR_ASSERT_DATA,
    );
    assert_ne!(
        rig.rd(CURRENT_STATUS) & CSR_BSY,
        0,
        "the target answered the selection by asserting BSY"
    );
    // X3.131's selection phase (§5.1.3): the initiator drops `SEL` and the
    // data bus once the target has `BSY`, and keeps `ATN` for the message it
    // is about to send.
    rig.wr(MODE, 0);
    rig.wr(INITIATOR_COMMAND, ICR_ASSERT_ATN);
}

/// Move one byte in whichever direction the current phase runs, the way §10.1
/// has the host do it: check the phase, hand `ACK` over, take it back.
fn transfer(rig: &Rig, out: Option<u8>) -> u8 {
    let phase = rig.phase().expect("still connected");
    rig.wr(TARGET_COMMAND, phase);
    assert_ne!(
        rig.rd(BUS_AND_STATUS) & BSR_PHASE_MATCH,
        0,
        "§6.7 bit 3: a phase match is required for a transfer to occur"
    );
    assert_ne!(
        rig.rd(CURRENT_STATUS) & CSR_REQ,
        0,
        "the target is asking for a byte"
    );
    let atn = rig.rd(INITIATOR_COMMAND) & ICR_ASSERT_ATN;
    let byte = match out {
        Some(value) => {
            rig.wr(DATA, value);
            rig.wr(INITIATOR_COMMAND, atn | ICR_ASSERT_DATA | ICR_ASSERT_ACK);
            0
        }
        None => {
            // §6.1.1: the byte is on the bus while `REQ` is asserted, and the
            // host reads it before it answers.
            let byte = rig.rd(DATA);
            rig.wr(INITIATOR_COMMAND, atn | ICR_ASSERT_ACK);
            byte
        }
    };
    assert_eq!(
        rig.rd(CURRENT_STATUS) & CSR_REQ,
        0,
        "§10.1: REQ goes false while the host holds ACK"
    );
    rig.wr(INITIATOR_COMMAND, atn);
    byte
}

/// A whole command in programmed I/O, recording the phase sequence it walked.
///
/// Returns the `DATA IN` bytes, the status byte, the message byte and the
/// phases, in order, as `MSG C/D I/O` triples.
fn command(rig: &Rig, cdb: &[u8]) -> (Vec<u8>, u8, u8, Vec<u8>) {
    let mut phases = Vec::new();
    let note = |rig: &Rig, phases: &mut Vec<u8>| {
        if let Some(p) = rig.phase()
            && phases.last() != Some(&p)
        {
            phases.push(p);
        }
    };
    select(rig);
    note(rig, &mut phases);
    // The `IDENTIFY` message (X3.131 §6.6.7), which is what `ATN` asked for.
    transfer(rig, Some(crate::dev::scsi::message::IDENTIFY));
    // `ATN` goes away with the last message byte.
    rig.wr(INITIATOR_COMMAND, 0);
    note(rig, &mut phases);
    for byte in cdb {
        transfer(rig, Some(*byte));
    }
    note(rig, &mut phases);
    let mut data = Vec::new();
    while rig.phase() == Some(Phase::DataIn.mci().unwrap()) {
        data.push(transfer(rig, None));
    }
    note(rig, &mut phases);
    assert_eq!(rig.phase(), Phase::Status.mci(), "the status phase follows");
    let status = transfer(rig, None);
    note(rig, &mut phases);
    assert_eq!(rig.phase(), Phase::MessageIn.mci());
    let message = transfer(rig, None);
    assert_eq!(rig.phase(), None, "the target let go of the bus");
    (data, status, message, phases)
}

// ---------------------------------------------------------------------------
// the register file
// ---------------------------------------------------------------------------

#[test]
fn the_window_decodes_the_boards_register_spacing() {
    let rig = rig();
    // A Macintosh Plus reaches the Mode register at `$580020`, which is offset
    // 2 × 16, and writes it at `$580021`.
    rig.wr(MODE, MODE_MONITOR_BUSY);
    assert_eq!(rig.rd(MODE), MODE_MONITOR_BUSY);
    // Only `A6`-`A4` reach the chip, so every address inside a stride selects
    // the same register — the fifteen bytes above `$580020` are all the Mode
    // register, and a board that put the selects somewhere else would need a
    // different `stride` rather than a different model.
    for low in 2..16 {
        assert_eq!(
            rig.window
                .read(u64::from(MODE) * STRIDE + low, Width::U8, MemAttrs::DEFAULT)
                .unwrap() as u8,
            MODE_MONITOR_BUSY,
            "offset {low} inside the stride is still register 2"
        );
    }
    // A board with the chip's own pinout gets the data sheet's eight bytes.
    let plain = rig_with(1);
    plain.wr(MODE, MODE_DMA);
    assert_eq!(plain.rd(MODE), MODE_DMA);
}

#[test]
fn the_decode_numbers_are_checked_when_the_board_is_built() {
    let bad = Ncr5380::new(&Props::new().with("stride", Value::Uint(3)));
    let message = bad.expect_err("3 is not a power of two").to_string();
    assert!(message.contains("power of two"), "{message}");
}

#[test]
fn an_unanswered_selection_leaves_bsy_clear() {
    let rig = rig();
    rig.wr(DATA, 1 << OWN);
    rig.wr(MODE, MODE_ARBITRATE);
    // Address 3 is empty. §8.1's interrupt is the *target's*; an initiator
    // learns nothing at all and times its own selection out.
    rig.wr(DATA, (1 << OWN) | (1 << 3));
    rig.wr(MODE, 0);
    rig.wr(
        INITIATOR_COMMAND,
        ICR_ASSERT_SEL | ICR_ASSERT_ATN | ICR_ASSERT_DATA,
    );
    assert_eq!(
        rig.rd(CURRENT_STATUS) & CSR_BSY,
        0,
        "nobody home: BSY never appears"
    );
    assert_eq!(rig.chip.connected(), None);
    assert!(!rig.irq.high(), "and no interrupt either");
}

// ---------------------------------------------------------------------------
// what a real Macintosh Plus ROM does, measured
// ---------------------------------------------------------------------------

/// The **only** three accesses a Macintosh Plus ROM makes to this chip in two
/// virtual minutes, in order, as `tests/mac_plus_scsi.rs` measures them:
/// `$580011 := $80`, `$580011 := $00`, `$580021 := $00`.
///
/// Asserted here as well as there so that the chip's answer to the sequence is
/// pinned even in a build with no ROM to hand: a bus reset, the interrupt latch
/// §8.3 says cannot be disabled, and a target that has forgotten everything.
#[test]
fn the_roms_three_writes_reset_the_bus() {
    let rig = rig();
    // Something in flight, so the reset has something to forget.
    select(&rig);
    assert_eq!(rig.chip.connected(), Some(TARGET));

    rig.wr(INITIATOR_COMMAND, ICR_ASSERT_RST);
    assert!(
        rig.irq.high(),
        "§8.3: the interrupt \"also occurs after setting the ASSERT RST bit\" and \"cannot be \
         disabled\""
    );
    assert_ne!(
        rig.rd(BUS_AND_STATUS) & BSR_IRQ,
        0,
        "§6.7 bit 4 says so too"
    );
    assert_ne!(
        rig.rd(CURRENT_STATUS) & CSR_RST,
        0,
        "and RST is on the cable until the bit is cleared"
    );
    assert_eq!(
        rig.rd(INITIATOR_COMMAND) & ICR_ASSERT_RST,
        ICR_ASSERT_RST,
        "§9.3: every register is cleared except the IRQ latch and this bit"
    );
    assert_eq!(rig.chip.connected(), None, "and the connection is gone");
    assert_eq!(
        crate::dev::scsi::Target::phase(rig.drive.as_ref()),
        Phase::BusFree,
        "the target saw RST and let go (X3.131 §5.2.2)"
    );

    rig.wr(INITIATOR_COMMAND, 0);
    rig.wr(MODE, 0);
    assert_eq!(rig.rd(CURRENT_STATUS) & CSR_RST, 0, "RST is released");
    assert!(
        rig.irq.high(),
        "the latch outlives the pulse: §6.9's read of port 7 is what clears it"
    );
    rig.rd(RESET_PARITY_IRQ);
    assert!(!rig.irq.high(), "§6.9 clears the INTERRUPT REQUEST bit");
    assert_eq!(rig.rd(BUS_AND_STATUS) & BSR_IRQ, 0);

    // A bus reset leaves a UNIT ATTENTION owing (X3.131 §5.2.2), which is the
    // observable half of the target having been told: `REQUEST SENSE` reports
    // sense key 6 with additional sense code 29, "power on, reset, or bus
    // device reset occurred".
    let (sense, status, _, _) = command(&rig, &[0x03, 0, 0, 0, 18, 0]);
    assert_eq!(status, crate::dev::scsi::status::GOOD);
    assert_eq!(
        sense[2] & 0x0f,
        crate::dev::scsi::sense::UNIT_ATTENTION,
        "the target was told about the reset"
    );
    assert_eq!(sense[12], crate::dev::scsi::sense::ASC_RESET);
}

// ---------------------------------------------------------------------------
// programmed I/O
// ---------------------------------------------------------------------------

#[test]
fn a_programmed_io_inquiry_walks_the_phases() {
    let rig = rig();
    // The first command after power-on owes a UNIT ATTENTION, so clear it the
    // way a driver does.
    let _ = command(&rig, &[0x03, 0, 0, 0, 18, 0]);

    let (data, status, message, phases) = command(&rig, &[0x12, 0, 0, 0, 36, 0]);
    assert_eq!(status, crate::dev::scsi::status::GOOD);
    assert_eq!(message, crate::dev::scsi::message::COMMAND_COMPLETE);
    assert_eq!(data.len(), 36, "36 bytes of standard INQUIRY data");
    assert_eq!(data[0], 0x00, "peripheral device type: direct access");
    assert_eq!(&data[8..13], b"RSEMU", "the vendor field");
    // The whole point of the part: the phase sequence is the *driver's*, and
    // this is it — MESSAGE OUT, COMMAND, DATA IN, STATUS, MESSAGE IN.
    assert_eq!(
        phases,
        vec![
            Phase::MessageOut.mci().unwrap(),
            Phase::Command.mci().unwrap(),
            Phase::DataIn.mci().unwrap(),
            Phase::Status.mci().unwrap(),
            Phase::MessageIn.mci().unwrap(),
        ]
    );
}

#[test]
fn test_unit_ready_and_read_capacity_answer() {
    let rig = rig();
    let _ = command(&rig, &[0x03, 0, 0, 0, 18, 0]);
    let (data, status, _, _) = command(&rig, &[0x00, 0, 0, 0, 0, 0]);
    assert_eq!(status, crate::dev::scsi::status::GOOD);
    assert!(data.is_empty(), "TEST UNIT READY has no data phase");

    let (data, status, _, _) = command(&rig, &[0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(status, crate::dev::scsi::status::GOOD);
    assert_eq!(data.len(), 8, "READ CAPACITY returns eight bytes");
    let last = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
    let size = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);
    assert_eq!(u64::from(last), BLOCKS - 1, "the last block's address");
    assert_eq!(size, BLOCK as u32);
}

#[test]
fn a_read_6_moves_a_block_through_the_data_register() {
    let rig = rig();
    let _ = command(&rig, &[0x03, 0, 0, 0, 18, 0]);
    let (data, status, _, _) = command(&rig, &[0x08, 0, 0, 7, 1, 0]);
    assert_eq!(status, crate::dev::scsi::status::GOOD);
    assert_eq!(data.len(), BLOCK);
    assert!(data.iter().all(|&b| b == 7), "block 7 is full of sevens");
}

// ---------------------------------------------------------------------------
// pseudo-DMA
// ---------------------------------------------------------------------------

/// §10.4: the host sets `DMA MODE`, writes a Start DMA register and then
/// answers `DRQ` by reading the data register, once per byte, with the chip
/// driving `REQ`/`ACK` itself. That loop is Apple's SCSI Manager's, measured —
/// [`Chip::pseudo_dma`] has the trace and the argument.
#[test]
fn pseudo_dma_moves_a_block_a_byte_at_a_time() {
    let rig = rig();
    let _ = command(&rig, &[0x03, 0, 0, 0, 18, 0]);

    select(&rig);
    transfer(&rig, Some(crate::dev::scsi::message::IDENTIFY));
    rig.wr(INITIATOR_COMMAND, 0);
    for byte in [0x28u8, 0, 0, 0, 0, 3, 0, 0, 2, 0] {
        transfer(&rig, Some(byte));
    }
    // A `READ(10)` of two blocks from block 3.
    assert_eq!(rig.phase(), Phase::DataIn.mci());
    rig.wr(TARGET_COMMAND, Phase::DataIn.mci().unwrap());
    rig.wr(MODE, MODE_DMA);
    rig.wr(RESET_PARITY_IRQ, 0);

    let mut data = Vec::new();
    for _ in 0..2 * BLOCK {
        assert_ne!(
            rig.rd(BUS_AND_STATUS) & BSR_DRQ,
            0,
            "§6.7 bit 6: DRQ says the chip has a byte to move"
        );
        data.push(rig.rd(DATA));
    }
    assert_eq!(&data[..BLOCK], &vec![3u8; BLOCK][..]);
    assert_eq!(&data[BLOCK..], &vec![4u8; BLOCK][..]);

    // §8.5: the transfer ends where the phase does, and with `DMA MODE` set
    // that is an interrupt — the only one a Macintosh Plus can see, because
    // its `IRQ` pin goes nowhere.
    assert!(
        rig.irq.high(),
        "the target moved to STATUS and the phase no longer matches"
    );
    assert_ne!(rig.rd(BUS_AND_STATUS) & BSR_IRQ, 0);
    assert_eq!(
        rig.rd(BUS_AND_STATUS) & BSR_PHASE_MATCH,
        0,
        "which is what the host reads to find out why"
    );
    assert_eq!(rig.rd(BUS_AND_STATUS) & BSR_DRQ, 0, "and DRQ is gone");

    // §10.5.3: "It is recommended that the DMA MODE bit be reset after
    // receiving an EOP or bus phase mismatch interrupt."
    rig.wr(MODE, 0);
    rig.rd(RESET_PARITY_IRQ);
    assert!(!rig.irq.high());
    assert_eq!(rig.phase(), Phase::Status.mci());
    assert_eq!(transfer(&rig, None), crate::dev::scsi::status::GOOD);
    assert_eq!(
        transfer(&rig, None),
        crate::dev::scsi::message::COMMAND_COMPLETE
    );
    assert_eq!(rig.phase(), None);
}

#[test]
fn pseudo_dma_sends_a_command_block_too() {
    let rig = rig();
    let _ = command(&rig, &[0x03, 0, 0, 0, 18, 0]);
    select(&rig);
    transfer(&rig, Some(crate::dev::scsi::message::IDENTIFY));
    rig.wr(INITIATOR_COMMAND, ICR_ASSERT_DATA);
    assert_eq!(rig.phase(), Phase::Command.mci());
    rig.wr(TARGET_COMMAND, Phase::Command.mci().unwrap());
    rig.wr(MODE, MODE_DMA);
    // §6.8.1, Start DMA Send.
    rig.wr(BUS_AND_STATUS, 0);
    for byte in [0x12u8, 0, 0, 0, 5, 0] {
        assert_ne!(rig.rd(BUS_AND_STATUS) & BSR_DRQ, 0);
        rig.wr(DATA, byte);
    }
    // The command block is complete, so the target is in `DATA IN` and the
    // phase no longer matches what the Target Command Register asked for.
    rig.wr(MODE, 0);
    rig.rd(RESET_PARITY_IRQ);
    assert_eq!(rig.phase(), Phase::DataIn.mci());
    let mut data = Vec::new();
    while rig.phase() == Phase::DataIn.mci() {
        data.push(transfer(&rig, None));
    }
    assert_eq!(
        data.len(),
        5,
        "five bytes is what the allocation length said"
    );
    assert_eq!(data[0], 0x00, "and it is INQUIRY data");
}

// ---------------------------------------------------------------------------
// the debugger, and the snapshot
// ---------------------------------------------------------------------------

#[test]
fn a_debug_access_moves_nothing() {
    let rig = rig();
    select(&rig);
    // A `MESSAGE OUT` phase with `REQ` outstanding, and an interrupt latched.
    rig.wr(INITIATOR_COMMAND, ICR_ASSERT_RST);
    rig.wr(INITIATOR_COMMAND, 0);
    assert!(rig.irq.high());
    assert_ne!(rig.peek(BUS_AND_STATUS) & BSR_IRQ, 0);
    // §6.9's read clears three latches, so a debugger must not perform it.
    assert_eq!(rig.peek(RESET_PARITY_IRQ), 0);
    assert!(rig.irq.high(), "a debug read of port 7 clears nothing");
    rig.rd(RESET_PARITY_IRQ);
    assert!(!rig.irq.high(), "a real one does");

    // A data-register read under a started transfer advances it by a byte, so
    // a debug read hands over the latch and moves nothing.
    let _ = command(&rig, &[0x03, 0, 0, 0, 18, 0]);
    select(&rig);
    transfer(&rig, Some(crate::dev::scsi::message::IDENTIFY));
    rig.wr(INITIATOR_COMMAND, 0);
    for byte in [0x08u8, 0, 0, 9, 1, 0] {
        transfer(&rig, Some(byte));
    }
    rig.wr(TARGET_COMMAND, Phase::DataIn.mci().unwrap());
    rig.wr(MODE, MODE_DMA);
    rig.wr(RESET_PARITY_IRQ, 0);
    let first = rig.rd(DATA);
    let before = rig.peek(DATA);
    let after = rig.peek(DATA);
    assert_eq!(before, after, "a debug read does not advance the transfer");
    assert_eq!(first, 9, "block 9 is full of nines");
    assert_eq!(rig.rd(DATA), 9, "and the real read still gets the next");

    // Every write here does something to the bus, so a debug write faults
    // rather than pretending (`ROADMAP.md` §15, invariant 5).
    let err = rig
        .window
        .write(u64::from(MODE) * STRIDE + 1, Width::U8, 0, MemAttrs::DEBUG)
        .expect_err("a debug write is refused");
    assert_eq!(err, crate::core::error::BusError::BadAccess);
}

fn snapshot(chip: &Ncr5380) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("ncr0", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ncr0", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(chip, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = rig();
    // Stopped in the middle of a `DATA IN` phase, with a byte on the bus, a
    // pseudo-DMA transfer started and an interrupt latched.
    let _ = command(&saved, &[0x03, 0, 0, 0, 18, 0]);
    select(&saved);
    transfer(&saved, Some(crate::dev::scsi::message::IDENTIFY));
    saved.wr(INITIATOR_COMMAND, 0);
    for byte in [0x08u8, 0, 0, 5, 1, 0] {
        transfer(&saved, Some(byte));
    }
    saved.wr(TARGET_COMMAND, Phase::DataIn.mci().unwrap());
    saved.wr(MODE, MODE_DMA | MODE_MONITOR_BUSY);
    saved.wr(RESET_PARITY_IRQ, 0);
    assert_eq!(saved.rd(DATA), 5);
    let bytes = snapshot(&saved.chip);

    let restored = rig();
    assert_ne!(snapshot(&restored.chip), bytes, "a fresh chip differs");
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("ncr0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(restored.chip.as_ref(), &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&restored.chip), bytes, "identical state");
    assert_eq!(restored.chip.connected(), Some(TARGET));
}

#[test]
fn a_snapshot_with_a_nonsense_dma_direction_is_refused() {
    let rig = rig();
    let mut bytes = snapshot(&rig.chip);
    // The pseudo-DMA direction is the last byte this chip writes, and the
    // writer's end-of-snapshot tag is the byte after it.
    let at = bytes.len() - 2;
    assert!(bytes[at] <= 2, "the direction byte is where it is expected");
    bytes[at] = 9;
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("ncr0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    let err = Device::load(rig.chip.as_ref(), &mut chunk.reader()).expect_err("refused");
    assert!(err.to_string().contains("pseudo-DMA direction"), "{err}");
}

#[test]
fn a_board_reset_clears_the_chip_but_not_the_cable() {
    let rig = rig();
    select(&rig);
    Device::reset(rig.chip.as_ref(), crate::core::device::ResetKind::Cold);
    // §9.1: "This is a chip reset only and does not create an SCSI bus reset
    // condition", so no interrupt — but the chip has let go of `BSY`, which
    // the target sees as the initiator abandoning the connection.
    assert!(!rig.irq.high());
    assert_eq!(rig.chip.connected(), None);
    assert_eq!(rig.rd(INITIATOR_COMMAND), 0);
    assert_eq!(
        crate::dev::scsi::Target::phase(rig.drive.as_ref()),
        Phase::BusFree
    );
}
