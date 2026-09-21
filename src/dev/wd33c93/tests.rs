//! The WD33C93A through its two addresses, the way a board reaches it.
//!
//! The target's own behaviour is `src/dev/scsi/tests.rs`'s. What is asserted
//! here is what happens between a register write and a SCSI bus: the address
//! register and its auto-increment, the interrupt codes of datasheet §6.2.19,
//! the two ways of running a whole SCSI operation — phase by phase with
//! `Transfer Info`, and in one go with `Select-And-Transfer` — the DMA seam,
//! `debug` reads, and a snapshot.

use super::*;
use crate::core::props::{Media, Props, Value};
use crate::core::space::{AddressSpace, RamStore, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink, WireSource};
use crate::dev::scsi::{DiskDevice, status};
use alloc::vec;
use alloc::vec::Vec;

// ---------------------------------------------------------------------------
// rig
// ---------------------------------------------------------------------------

/// The drive's SCSI address, and the initiator's.
const TARGET: u8 = 0;
const OWN: u8 = 7;

/// How many blocks the drive under test holds.
const BLOCKS: u64 = 64;
const BLOCK: usize = 512;

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
    fn set_level(&self, _src: WireId, _line: u32, level: crate::core::wire::Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

struct Rig {
    chip: Arc<Wd33c93>,
    /// The drive, kept alive: a bus holds `Arc<dyn Target>`, and the device
    /// wrapper is what owns the medium.
    _disk: DiskDevice,
    intrq: Arc<Probe>,
    /// The memory a DMA transfer moves through.
    space: Arc<AddressSpace>,
    /// The chip's own two addresses, mapped at 0 and 1.
    window: Arc<AddressSpace>,
}

/// A DMA port over one address space, with a cursor — the least a board has
/// to provide, and enough to show the chip uses it.
#[derive(Debug)]
struct Dma {
    space: Arc<AddressSpace>,
    at: AtomicU32,
}

impl DmaPort for Dma {
    fn fetch(&self, dst: &mut [u8]) -> usize {
        let at = u64::from(self.at.load(Ordering::Relaxed));
        if self.space.read_bytes(at, dst, MemAttrs::DEFAULT).is_err() {
            return 0;
        }
        self.at
            .store(at as u32 + dst.len() as u32, Ordering::Relaxed);
        dst.len()
    }

    fn store(&self, src: &[u8]) -> usize {
        let at = u64::from(self.at.load(Ordering::Relaxed));
        if self.space.write_bytes(at, src, MemAttrs::DEFAULT).is_err() {
            return 0;
        }
        self.at
            .store(at as u32 + src.len() as u32, Ordering::Relaxed);
        src.len()
    }
}

/// Where a DMA transfer starts in the rig's space.
const DMA_AT: u32 = 0x1000;

fn rig() -> Rig {
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
    let chip = Arc::new(
        Wd33c93::new(
            &Props::new()
                .with("bus", Value::Str(String::from("scsi0")))
                .with("id", Value::Uint(u64::from(OWN)))
                .with_hosts(Arc::clone(&hosts)),
        )
        .expect("a controller"),
    );

    let space = Arc::new(AddressSpace::new("dma", 32).with_unassigned(UnassignedPolicy::ZEROS));
    space
        .topology()
        .map(
            Arc::new(Region::ram("ram", Arc::new(RamStore::new(0x10000)))),
            0,
        )
        .expect("mapped");
    chip.port().attach_dma(Arc::new(Dma {
        space: Arc::clone(&space),
        at: AtomicU32::new(DMA_AT),
    }));

    let intrq = Arc::new(Probe::default());
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&intrq) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(chip.as_ref(), INTRQ_PIN, WireSource::new(wire, id)).expect("the pin exists");
    Device::announce(chip.as_ref(), INTRQ_PIN);

    // The chip's own window, in a little space of its own, so the two
    // addresses can be reached the way a board reaches them.
    let window = Arc::new(AddressSpace::new("wd", 8).with_unassigned(UnassignedPolicy::ZEROS));
    window
        .topology()
        .map(Device::region(chip.as_ref(), "").expect("a region"), 0)
        .expect("it maps");

    Rig {
        chip,
        _disk: disk,
        intrq,
        space,
        window,
    }
}

impl Rig {
    /// `A0` low, write: the Address register.
    fn sasr(&self, value: u8) {
        self.chip.port().write_address(value);
    }

    /// `A0` low, read: Auxiliary Status.
    fn aux(&self) -> u8 {
        self.chip.port().read_aux()
    }

    /// `A0` high, read.
    fn get(&self) -> u8 {
        self.chip.port().read_register(false)
    }

    /// `A0` high, write.
    fn put(&self, value: u8) {
        self.chip.port().write_register(value);
    }

    /// Read one register by number, leaving the address register on it.
    fn reg(&self, n: u8) -> u8 {
        self.sasr(n);
        self.get()
    }

    /// Write one register by number.
    fn set(&self, n: u8, value: u8) {
        self.sasr(n);
        self.put(value);
    }

    /// Poll the way a board's interrupt status register does, and return the
    /// SCSI Status byte once one is there. `None` if nothing arrives.
    fn interrupt(&self) -> Option<u8> {
        for _ in 0..8 {
            if self.chip.port().poll_irq() {
                assert!(self.intrq.high(), "the pin follows the interrupt");
                assert_eq!(self.aux() & AUX_INT, AUX_INT);
                let status = self.reg(SCSI_STATUS);
                assert_eq!(self.aux() & AUX_INT, 0, "reading it clears INTRQ");
                return Some(status);
            }
        }
        None
    }

    /// Issue `command` in the Command register.
    fn command(&self, command: u8) {
        self.set(COMMAND, command);
    }

    /// The transfer count, as three registers.
    fn set_count(&self, count: u32) {
        let b = count.to_be_bytes();
        self.set(COUNT_MSB, b[1]);
        self.put(b[2]);
        self.put(b[3]);
    }

    /// A debugger's read of one of the chip's two addresses.
    fn peek(&self, at: u64) -> u8 {
        self.window.read(at, Width::U8, MemAttrs::DEBUG).unwrap() as u8
    }

    /// An ordinary write to one of the chip's two addresses.
    fn poke(&self, at: u64, value: u8) {
        self.window
            .write(at, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn dma_bytes(&self, len: usize) -> Vec<u8> {
        let mut out = vec![0u8; len];
        self.space
            .read_bytes(u64::from(DMA_AT), &mut out, MemAttrs::DEBUG)
            .expect("mapped");
        out
    }
}

// ---------------------------------------------------------------------------
// the register file
// ---------------------------------------------------------------------------

#[test]
fn the_address_register_auto_increments_except_on_three_registers() {
    let r = rig();
    // §6.2.2: "following every access with A0 = 1, the Address register will
    // automatically increment … with the exception of the following
    // locations: Auxiliary Status register, Data register, and the Command
    // register."
    r.sasr(CONTROL);
    r.put(0x00);
    assert_eq!(r.chip.port().address(), CONTROL + 1);
    r.sasr(CDB1);
    let _ = r.get();
    assert_eq!(r.chip.port().address(), CDB1 + 1);

    for stay in [AUX_STATUS, DATA, COMMAND] {
        r.sasr(stay);
        let _ = r.get();
        assert_eq!(r.chip.port().address(), stay, "{stay:#04x} does not move");
    }
    // And it wraps within the five bits the register has.
    r.sasr(0x1f);
    let _ = r.get();
    assert_eq!(r.chip.port().address(), 0x1f);
}

#[test]
fn a_register_that_is_not_there_reads_all_ones() {
    let r = rig();
    // §6.1 note 2: "Reading an undefined or unavailable register results in an
    // all-ones data bus output." `1A`–`1E` are between the Data register and
    // the direct-addressing Auxiliary Status.
    for n in 0x1a..=0x1e {
        assert_eq!(r.reg(n), 0xff, "register {n:#04x}");
    }
}

#[test]
fn a_hardware_reset_keeps_what_the_datasheet_says_it_keeps() {
    let r = rig();
    r.set(OWN_ID, 0x47);
    r.set(CONTROL, 0x20);
    r.set(TIMEOUT, 0x2c);
    r.set(DEST_ID, 0x03);
    r.set(SOURCE_ID, 0x8f);
    let _ = r.interrupt();

    r.chip.port().master_reset();
    // §6.3.1: "The Own ID register is reset to zero"; "the following host
    // accessible registers are NOT affected …: Registers 01 Hex through 15
    // Hex; Source ID (16 Hex) register bits 0-3".
    assert_eq!(r.reg(OWN_ID), 0);
    assert_eq!(r.reg(CONTROL), 0x20);
    assert_eq!(r.reg(TIMEOUT), 0x2c);
    assert_eq!(r.reg(DEST_ID), 0x03);
    assert_eq!(r.reg(SOURCE_ID), 0x0f, "bits 0-3 only");
    // "the INT bit (and INTRQ pin) is set to one when the hardware reset is
    // complete."
    assert_eq!(r.interrupt(), Some(INT_RESET));
}

#[test]
fn the_reset_command_reads_own_id_and_reports_which_mode_it_is_in() {
    let r = rig();
    let _ = r.interrupt();
    // §6.3.2 and §6.2.19: `00` with advanced features off, `01` with them on
    // (`EAF`, bit 3 of Own ID).
    r.set(OWN_ID, 0x47);
    r.set(CONTROL, 0x11);
    r.command(CMD_RESET);
    assert_eq!(r.interrupt(), Some(0x00));
    assert_eq!(r.reg(OWN_ID), 0x47, "Own ID survives a Reset command");
    assert_eq!(r.reg(CONTROL), 0x00, "01-16 Hex do not");

    r.set(OWN_ID, 0x4f);
    r.command(CMD_RESET);
    assert_eq!(r.interrupt(), Some(0x01));
}

#[test]
fn a_command_issued_while_an_interrupt_is_pending_is_ignored() {
    let r = rig();
    // A board pulls `MR-` on reset, and §6.3.1 leaves `INTRQ` asserted.
    Device::reset(r.chip.as_ref(), ResetKind::Cold);
    assert!(r.chip.port().poll_irq());
    assert_eq!(r.aux() & AUX_INT, AUX_INT);
    r.set(DEST_ID, TARGET);
    r.command(CMD_SELECT_ATN);
    // §6.2.20: "this register should never be loaded when the CIP or INT bits
    // … are set to one" — and the chip says so through `LCI`.
    assert_eq!(r.aux() & AUX_LCI, AUX_LCI);
    assert_eq!(r.reg(SCSI_STATUS), INT_RESET, "still the reset interrupt");
}

#[test]
fn a_target_role_command_is_an_invalid_command() {
    let r = rig();
    let _ = r.interrupt();
    // §7.1's Target-only commands: Reselect, Wait-For-Select-And-Receive,
    // Receive Data, Send Status, Translate Address.
    for code in [0x05u8, 0x0c, 0x11, 0x14, 0x18] {
        r.command(code);
        assert_eq!(r.interrupt(), Some(INT_INVALID), "command {code:#04x}");
    }
}

// ---------------------------------------------------------------------------
// selection
// ---------------------------------------------------------------------------

#[test]
fn selecting_an_empty_address_times_out() {
    let r = rig();
    let _ = r.interrupt();
    r.set(DEST_ID, 4);
    r.command(CMD_SELECT_ATN);
    // §6.2.19: "0100 0010 — A timeout occurred during a Select or Reselect
    // command."
    assert_eq!(r.interrupt(), Some(INT_TIMEOUT));
    assert_eq!(r.reg(COMMAND_PHASE), command_phase::IDLE);
}

#[test]
fn selecting_a_target_completes_and_then_asks_for_service() {
    let r = rig();
    let _ = r.interrupt();
    r.set(DEST_ID, TARGET);
    r.command(CMD_SELECT_ATN);
    // §7.5.1's completion, then §7.5.6's first `REQ` — in that order, and the
    // second only once the host has read the first.
    assert_eq!(r.interrupt(), Some(INT_SELECT_DONE));
    assert_eq!(
        r.interrupt(),
        Some(INT_SERVICE | Phase::MessageOut.mci().unwrap()),
        "ATN was asserted, so the target wants a message"
    );

    // Without ATN the target asks for the command instead.
    let r = rig();
    let _ = r.interrupt();
    r.set(DEST_ID, TARGET);
    r.command(CMD_SELECT);
    assert_eq!(r.interrupt(), Some(INT_SELECT_DONE));
    assert_eq!(
        r.interrupt(),
        Some(INT_SERVICE | Phase::Command.mci().unwrap())
    );
}

// ---------------------------------------------------------------------------
// the simple way: Transfer Info, phase by phase
// ---------------------------------------------------------------------------

/// Move `bytes` out through the Data register with `Transfer Info`, polling
/// `DBR` the way §6.2.1 says a host must.
fn send(r: &Rig, bytes: &[u8]) {
    r.set_count(bytes.len() as u32);
    r.command(CMD_TRANSFER_INFO);
    for byte in bytes {
        assert_eq!(r.aux() & AUX_DBR, AUX_DBR, "the chip can take a byte");
        r.set(DATA, *byte);
    }
}

/// Take `len` bytes in through the Data register.
fn receive(r: &Rig, len: usize) -> Vec<u8> {
    r.set_count(len as u32);
    r.command(CMD_TRANSFER_INFO);
    let mut out = Vec::with_capacity(len);
    for _ in 0..len {
        assert_eq!(r.aux() & AUX_DBR, AUX_DBR, "a byte has arrived");
        out.push(r.reg(DATA));
    }
    out
}

#[test]
fn an_inquiry_driven_phase_by_phase_returns_the_targets_identity() {
    let r = rig();
    let _ = r.interrupt();
    r.set(CONTROL, 0x00); // Polled I/O: the Data register is the port.
    r.set(DEST_ID, TARGET);
    r.command(CMD_SELECT_ATN);
    assert_eq!(r.interrupt(), Some(INT_SELECT_DONE));
    assert_eq!(
        r.interrupt(),
        Some(INT_SERVICE | Phase::MessageOut.mci().unwrap())
    );

    // Message out: one `IDENTIFY` byte, and the target then wants the command.
    send(&r, &[0x80]);
    assert_eq!(
        r.interrupt(),
        Some(INT_DONE | Phase::Command.mci().unwrap())
    );

    // Command out: six bytes, and the target then has data.
    send(&r, &[0x12, 0, 0, 0, 36, 0]);
    assert_eq!(r.interrupt(), Some(INT_DONE | Phase::DataIn.mci().unwrap()));

    let data = receive(&r, 36);
    assert_eq!(&data[8..13], b"RSEMU");
    assert_eq!(r.interrupt(), Some(INT_DONE | Phase::Status.mci().unwrap()));

    let status = receive(&r, 1);
    assert_eq!(status[0], status::GOOD);
    assert_eq!(
        r.interrupt(),
        Some(INT_DONE | Phase::MessageIn.mci().unwrap())
    );

    // §7.5.6: a Message-In transfer *pauses* with `ACK` asserted rather than
    // completing, so the host can look at the message first.
    let msg = receive(&r, 1);
    assert_eq!(msg[0], 0x00, "COMMAND COMPLETE");
    assert_eq!(r.interrupt(), Some(INT_MSG_IN_PAUSED));
    r.command(CMD_NEGATE_ACK);
}

// ---------------------------------------------------------------------------
// the combination command
// ---------------------------------------------------------------------------

/// Set up and run a `Select-With-ATN-And-Transfer` for `cdb`, moving `count`
/// bytes through the DMA port.
fn select_and_transfer(r: &Rig, cdb: &[u8], count: u32) -> u8 {
    // DMA Mode (§6.2.4's `DM` field, `100`), so the data phase uses the port.
    r.set(CONTROL, 0x80);
    r.set(TARGET_LUN, 0);
    r.set(DEST_ID, TARGET);
    r.set(SOURCE_ID, SOURCE_ER);
    r.set_count(count);
    r.sasr(CDB1);
    for byte in cdb {
        r.put(*byte);
    }
    r.command(CMD_SELECT_ATN_TRANSFER);
    let status = r.interrupt().expect("an interrupt");
    assert_eq!(
        r.reg(COMMAND_PHASE),
        command_phase::COMPLETE,
        "§7.6.1: the Command Phase register says how far it got"
    );
    status
}

#[test]
fn select_and_transfer_runs_a_read_10_and_raises_one_interrupt() {
    let r = rig();
    let _ = r.interrupt();
    let status = select_and_transfer(&r, &[0x28, 0, 0, 0, 0, 3, 0, 0, 2, 0], 2 * BLOCK as u32);
    assert_eq!(status, INT_SAT_DONE);
    // §7.6.1: "the received status byte is stored in the Target Lun register".
    assert_eq!(r.reg(TARGET_LUN), status::GOOD);
    // §6.2.16: "after the completion of any successful transfer, the Transfer
    // Count register will be zero."
    assert_eq!(r.reg(COUNT_MSB), 0);
    assert_eq!(r.get(), 0);
    assert_eq!(r.get(), 0);

    let bytes = r.dma_bytes(2 * BLOCK);
    assert!(bytes[..BLOCK].iter().all(|&b| b == 3), "block 3");
    assert!(bytes[BLOCK..].iter().all(|&b| b == 4), "then block 4");

    // §7.6.1: and then "an additional interrupt … when the SCSI bus goes to
    // the Bus Free state".
    assert_eq!(r.interrupt(), Some(INT_DISCONNECTED));
}

#[test]
fn select_and_transfer_runs_an_inquiry_with_no_count_at_all() {
    let r = rig();
    let _ = r.interrupt();
    // Transfer Count zero and a target that wants a data phase anyway is
    // §7.6.1's unexpected information phase.
    let status = select_and_transfer(&r, &[0x12, 0, 0, 0, 0, 0], 0);
    assert_eq!(status, INT_SAT_DONE, "no data phase, and no complaint");
    assert_eq!(r.reg(TARGET_LUN), status::GOOD);
}

#[test]
fn select_and_transfer_sends_the_identify_the_source_id_asks_for() {
    let r = rig();
    let _ = r.interrupt();
    // §7.6.1: "1r000ttt, where r = 1 if the Enable Reselect bit in the Source
    // ID register is equal to 1, and ttt is … the Target Logical Unit Number".
    // LUN 3 is not a unit this drive has, so `INQUIRY` answers "nothing here",
    // which is proof the byte arrived and was understood.
    r.set(CONTROL, 0x80);
    r.set(TARGET_LUN, 3);
    r.set(DEST_ID, TARGET);
    r.set(SOURCE_ID, SOURCE_ER);
    r.set_count(36);
    r.sasr(CDB1);
    for byte in [0x12u8, 0, 0, 0, 36, 0] {
        r.put(byte);
    }
    r.command(CMD_SELECT_ATN_TRANSFER);
    assert_eq!(r.interrupt(), Some(INT_SAT_DONE));
    // §8.2.5: peripheral qualifier 3, device type 1F.
    assert_eq!(r.dma_bytes(1)[0], 0x7f);
}

#[test]
fn an_unexpected_phase_terminates_the_combination_command_where_it_stands() {
    let r = rig();
    let _ = r.interrupt();
    // A command the target refuses: it goes straight to `STATUS`, and a
    // Transfer Count that expects a data phase makes that unexpected.
    r.set(CONTROL, 0x80);
    r.set(TARGET_LUN, 0);
    r.set(DEST_ID, TARGET);
    r.set(SOURCE_ID, SOURCE_ER);
    r.set_count(512);
    r.sasr(CDB1);
    for byte in [0x04u8, 0, 0, 0, 0, 0] {
        r.put(byte);
    }
    r.command(CMD_SELECT_ATN_TRANSFER);
    // §6.2.19's `0100 1MCI`, naming the phase the target actually wants.
    assert_eq!(
        r.interrupt(),
        Some(INT_TERMINATED | Phase::Status.mci().unwrap())
    );
    assert_eq!(
        r.reg(COMMAND_PHASE) & 0xf0,
        command_phase::COMMAND,
        "it got as far as the command phase"
    );
}

#[test]
fn a_write_through_the_dma_port_lands_on_the_medium() {
    let r = rig();
    let _ = r.interrupt();
    let payload: Vec<u8> = (0..BLOCK).map(|i| (i * 5) as u8).collect();
    r.space
        .write_bytes(u64::from(DMA_AT), &payload, MemAttrs::DEFAULT)
        .expect("mapped");
    // DMA direction is the board's business; this port has one buffer and a
    // cursor, so the fetch reads what was just put there.
    let status = select_and_transfer(&r, &[0x2a, 0, 0, 0, 0, 7, 0, 0, 1, 0], BLOCK as u32);
    assert_eq!(status, INT_SAT_DONE);
    assert_eq!(r.reg(TARGET_LUN), status::GOOD);
    let _ = r.interrupt();

    // Read it back the same way, into the next 512 bytes of the buffer.
    let status = select_and_transfer(&r, &[0x28, 0, 0, 0, 0, 7, 0, 0, 1, 0], BLOCK as u32);
    assert_eq!(status, INT_SAT_DONE);
    let mut got = vec![0u8; BLOCK];
    r.space
        .read_bytes(u64::from(DMA_AT) + BLOCK as u64, &mut got, MemAttrs::DEBUG)
        .expect("mapped");
    assert_eq!(got, payload);
}

#[test]
fn with_no_target_the_combination_command_times_out_before_any_phase() {
    let r = rig();
    let _ = r.interrupt();
    r.set(CONTROL, 0x80);
    r.set(DEST_ID, 5);
    r.set_count(512);
    r.sasr(CDB1);
    for byte in [0x28u8, 0, 0, 0, 0, 0, 0, 0, 1, 0] {
        r.put(byte);
    }
    r.command(CMD_SELECT_ATN_TRANSFER);
    assert_eq!(r.interrupt(), Some(INT_TIMEOUT));
    // §7.6.1: "Failure to complete the Selection phase is also indicated by
    // the fact that the Command Phase register contains all zeroes."
    assert_eq!(r.reg(COMMAND_PHASE), command_phase::IDLE);
}

#[test]
fn a_reset_lets_go_of_the_bus_so_the_next_selection_starts_clean() {
    let r = rig();
    let _ = r.interrupt();
    r.set(DEST_ID, TARGET);
    r.command(CMD_SELECT_ATN);
    assert_eq!(r.interrupt(), Some(INT_SELECT_DONE));
    // §6.3.2: "All SCSI bus signals are reset to the negated state" — the
    // target sees the bus go free and forgets the connection.
    r.command(CMD_RESET);
    assert_eq!(r.interrupt(), Some(INT_RESET));
    let _ = r.interrupt();
    let status = select_and_transfer(&r, &[0x12, 0, 0, 0, 36, 0], 36);
    assert_eq!(status, INT_SAT_DONE);
}

// ---------------------------------------------------------------------------
// the register window, and `debug`
// ---------------------------------------------------------------------------

#[test]
fn the_window_is_two_addresses_and_a_debug_read_moves_nothing() {
    let r = rig();
    let _ = r.interrupt();
    assert_eq!(
        Device::region(r.chip.as_ref(), REGS_REGION)
            .expect("a region")
            .len(),
        REGS_WINDOW_LEN
    );

    // Set a transfer going, with one byte staged in the Data register.
    r.set(CONTROL, 0x00);
    r.set(DEST_ID, TARGET);
    r.command(CMD_SELECT);
    assert_eq!(r.interrupt(), Some(INT_SELECT_DONE));
    assert_eq!(
        r.interrupt(),
        Some(INT_SERVICE | Phase::Command.mci().unwrap())
    );
    send(&r, &[0x12, 0, 0, 0, 36, 0]);
    assert_eq!(r.interrupt(), Some(INT_DONE | Phase::DataIn.mci().unwrap()));
    r.set_count(36);
    r.command(CMD_TRANSFER_INFO);

    // A debug read of the Data register hands back what is staged and leaves
    // the transfer exactly where it was; an ordinary one pops it. Here a
    // debugger looks over the guest's shoulder between every byte, which must
    // change nothing at all.
    r.sasr(DATA);
    let mut data = alloc::vec::Vec::new();
    for _ in 0..36 {
        let seen = r.peek(1);
        assert_eq!(r.peek(1), seen, "twice is the same byte");
        data.push(r.get());
        assert_eq!(*data.last().unwrap(), seen, "and so is the real read");
    }
    // §8.2.5's first five bytes are `00 00 02 02 1F`, and there are 36 of them
    // — none eaten by a debugger.
    assert_eq!(&data[..5], &[0x00, 0x00, 0x02, 0x02, 31]);
    assert_eq!(&data[8..13], b"RSEMU");
    assert!(r.chip.port().poll_irq());
    r.sasr(SCSI_STATUS);
    assert_eq!(r.peek(1), INT_DONE | Phase::Status.mci().unwrap());
    assert_eq!(r.aux() & AUX_INT, AUX_INT, "still pending");
    assert_eq!(r.chip.port().address(), SCSI_STATUS, "and it did not move");

    // And no write at all is a debugger's business.
    assert!(
        r.window
            .write(1, Width::U8, u64::from(CMD_RESET), MemAttrs::DEBUG)
            .is_err()
    );
}

#[test]
fn the_window_answers_one_byte_at_each_of_its_two_addresses() {
    let r = rig();
    // `A0` low writes the Address register and `A0` high the register it names.
    r.poke(0, TIMEOUT);
    r.poke(1, 0x2c);
    assert_eq!(r.reg(TIMEOUT), 0x2c);
    // A read at `A0` low is Auxiliary Status, whatever the Address register
    // says (§6.2.2).
    r.sasr(TIMEOUT);
    assert_eq!(
        r.window.read(0, Width::U8, MemAttrs::DEFAULT).unwrap() as u8,
        r.aux()
    );
    // Wider than a byte is not an access this chip has.
    assert!(r.window.read(0, Width::U16, MemAttrs::DEFAULT).is_err());
}

// ---------------------------------------------------------------------------
// snapshot
// ---------------------------------------------------------------------------

fn snapshot(chip: &Wd33c93) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("wd0", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("wd0", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(chip, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = rig();
    let _ = saved.interrupt();
    // Stopped in the middle of a polled transfer, with a byte staged, an
    // interrupt queued behind the one just read, and the address register
    // part way up the command block.
    saved.set(CONTROL, 0x00);
    saved.set(DEST_ID, TARGET);
    saved.command(CMD_SELECT);
    assert_eq!(saved.interrupt(), Some(INT_SELECT_DONE));
    assert_eq!(
        saved.interrupt(),
        Some(INT_SERVICE | Phase::Command.mci().unwrap())
    );
    send(&saved, &[0x12, 0, 0, 0, 36, 0]);
    assert_eq!(
        saved.interrupt(),
        Some(INT_DONE | Phase::DataIn.mci().unwrap())
    );
    saved.set_count(36);
    saved.command(CMD_TRANSFER_INFO);
    let _ = saved.get();
    saved.sasr(CDB1 + 2);
    let bytes = snapshot(&saved.chip);

    let restored = rig();
    assert_ne!(snapshot(&restored.chip), bytes);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("wd0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(restored.chip.as_ref(), &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&restored.chip), bytes, "identical state");
    assert_eq!(restored.chip.port().address(), CDB1 + 2);
    assert_eq!(restored.intrq.high(), restored.chip.irq_asserted());
}

#[test]
fn a_snapshot_with_an_impossible_phase_is_refused() {
    let mut bytes = snapshot(&rig().chip);
    // The phase byte of the polled transfer, if there were one, is the last
    // discriminant written; corrupt the trailing `data_in` flag instead, which
    // is a `bool` and rejects anything but 0 and 1.
    let last = bytes.len() - 1;
    bytes[last] = 0x7f;
    let other = rig();
    let reader = StateReader::new(&bytes);
    // A corrupt chunk may fail either at the frame or at the field; both are
    // refusals rather than a panic, which is the property under test.
    if let Ok(reader) = reader
        && let Ok(chunk) = reader.load("wd0", CLASS_NAME, STATE_VERSION, &Migrations::new())
    {
        assert!(Device::load(other.chip.as_ref(), &mut chunk.reader()).is_err());
    }
}
