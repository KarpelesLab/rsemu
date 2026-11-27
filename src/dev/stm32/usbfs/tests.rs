//! Tests for `st.usbfs`.
//!
//! The host half is driven through the bus ([`UsbBus::setup`]/`read`/`write`,
//! the transaction entry points a controller uses) and through
//! [`ControlTransfer`](crate::bus::usb::ControlTransfer), the composer that
//! sequences a whole control transfer. The device half is the register file a
//! driver programs.
//!
//! Where a test needs an endpoint armed to a particular `STAT`, it reaches into
//! the state and sets it the way the hardware would, rather than replaying the
//! guest's toggle writes — the toggle and `rc_w0` rules are exercised on their
//! own through the register window, and open-coding them in every transaction
//! test would only make the tests harder to read than the hardware.

use super::*;
use crate::bus::usb::{ControlTransfer, Progress, host};
use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::space::RegionKind;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::AtomicBool;
use crate::core::wire::{Level, Wire, WireIdAllocator};

/// `CNTR.CTRM` (15): enable the correct-transfer interrupt.
const CNTR_CTRM: u16 = 1 << 15;
/// `CNTR.RESETM` (10): enable the USB-reset interrupt.
const CNTR_RESETM: u16 = 1 << 10;

/// A peripheral on a one-port private bus, with its two windows in hand.
struct Fixture {
    dev: UsbfsPeripheral,
    bus: Arc<UsbBus>,
    regs: Arc<dyn MemOps>,
    pma: Arc<dyn MemOps>,
}

fn io(region: &RegionRef) -> Arc<dyn MemOps> {
    match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        other => panic!("expected an io region, got {other:?}"),
    }
}

fn fixture() -> Fixture {
    let bus = Arc::new(UsbBus::new(1));
    let dev = UsbfsPeripheral::with_bus(Arc::clone(&bus));
    let regs = io(&Device::region(&dev, "regs").expect("the register block"));
    let pma = io(&Device::region(&dev, "pma").expect("the packet memory"));
    Fixture {
        dev,
        bus,
        regs,
        pma,
    }
}

impl Fixture {
    fn core(&self) -> &Arc<Usbfs> {
        self.dev.core()
    }

    fn rd(&self, offset: u64) -> u16 {
        let mut b = [0u8; 2];
        self.regs
            .read(offset, &mut b, MemAttrs::DEFAULT)
            .expect("read");
        u16::from_le_bytes(b)
    }
    fn wr(&self, offset: u64, value: u16) {
        self.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("write");
    }
    fn ep(&self, n: u64) -> u16 {
        self.rd(n * 4)
    }
    fn set_ep(&self, n: u64, value: u16) {
        self.wr(n * 4, value);
    }
    fn pma_write(&self, off: u64, bytes: &[u8]) {
        self.pma
            .write(off, bytes, MemAttrs::DEFAULT)
            .expect("pma write");
    }
    fn pma_read(&self, off: u64, len: usize) -> Vec<u8> {
        let mut v = vec![0u8; len];
        self.pma
            .read(off, &mut v, MemAttrs::DEFAULT)
            .expect("pma read");
        v
    }
    fn stat_rx(&self, n: usize) -> u16 {
        self.core().state.lock().stat_rx(n)
    }
    fn stat_tx(&self, n: usize) -> u16 {
        self.core().state.lock().stat_tx(n)
    }
    fn daddr_add(&self) -> u8 {
        (self.rd(OFF_DADDR) & DADDR_ADD) as u8
    }

    /// Arm endpoint register `n` the way a reset handler leaves it: a control
    /// endpoint with address `ea`, `STAT_RX = rx`, `STAT_TX = tx`, no flags.
    fn arm(&self, n: usize, ea: u16, rx: u16, tx: u16) {
        let mut s = self.core().state.lock();
        s.ep[n] = (0b01 << 9) | (ea & EP_EA);
        s.set_stat_rx(n, rx);
        s.set_stat_tx(n, tx);
    }
    fn set_stat_rx(&self, n: usize, v: u16) {
        self.core().state.lock().set_stat_rx(n, v);
    }
    fn set_stat_tx(&self, n: usize, v: u16) {
        self.core().state.lock().set_stat_tx(n, v);
    }

    /// Program EP0's buffer-descriptor-table entry (table at offset 0):
    /// `ADDR_TX = 0x20`, `ADDR_RX = 0x40`, then arm it and attach the pull-up.
    fn enumerate_ready(&self) {
        self.pma_write(0x00, &0x20u16.to_le_bytes()); // ADDR_TX
        self.pma_write(0x02, &0u16.to_le_bytes()); // COUNT_TX
        self.pma_write(0x04, &0x40u16.to_le_bytes()); // ADDR_RX
        self.pma_write(0x06, &0x8400u16.to_le_bytes()); // COUNT_RX: block size, count 0
        self.arm(0, 0, STAT_VALID, STAT_NAK);
        self.wr(OFF_DADDR, 0x80); // function enabled, address 0
        self.wr(OFF_CNTR, CNTR_CTRM | CNTR_RESETM); // powers up (clears PDWN/FRES)
        self.wr(OFF_BCDR, BCDR_DPPU); // attach the pull-up -> on the bus
        // The host resets and enables the port, as a controller does before it
        // addresses a freshly connected device.
        self.bus.reset_port(0);
        self.bus.set_enabled(0, true);
    }
}

// ---------------------------------------------------------------------------
// Register access rules
// ---------------------------------------------------------------------------

#[test]
fn ctr_bits_are_rc_w0_and_clear_only_the_bit_written_zero() {
    let f = fixture();
    f.core().state.lock().ep[0] = EP_CTR_RX | EP_CTR_TX; // hardware set both
    // A store with 0 in CTR_RX and 1 in CTR_TX clears RX, keeps TX.
    f.set_ep(0, EP_CTR_TX);
    assert_eq!(f.ep(0) & EP_CTR_RX, 0, "CTR_RX cleared by writing zero");
    assert_eq!(f.ep(0) & EP_CTR_TX, EP_CTR_TX, "CTR_TX kept by writing one");
}

#[test]
fn stat_and_dtog_toggle_and_setup_is_read_only() {
    let f = fixture();
    // STAT_RX 00 -> 11 by writing 11; DTOG_TX flips on a written one.
    f.set_ep(1, (STAT_VALID << 12) | EP_DTOG_TX | EP_CTR_RX | EP_CTR_TX);
    assert_eq!(f.stat_rx(1), STAT_VALID);
    assert_eq!(f.ep(1) & EP_DTOG_TX, EP_DTOG_TX, "DTOG_TX toggled to one");
    // Writing the same STAT field again toggles it back.
    f.set_ep(1, (STAT_VALID << 12) | EP_CTR_RX | EP_CTR_TX);
    assert_eq!(f.stat_rx(1), STAT_DISABLED, "11 ^ 11 = 00");
    // SETUP is read-only.
    f.set_ep(1, EP_SETUP | EP_CTR_RX | EP_CTR_TX);
    assert_eq!(f.ep(1) & EP_SETUP, 0, "SETUP is the hardware's to set");
}

#[test]
fn istr_flags_are_write_zero_to_clear_and_ctr_is_computed() {
    let f = fixture();
    {
        let mut s = f.core().state.lock();
        s.istr = ISTR_RESET | (1 << 9); // RESET and SOF latched
        s.ep[2] = EP_CTR_RX | 3; // a pending RX, EA 3, in register 2
    }
    let istr = f.rd(OFF_ISTR);
    assert_eq!(
        istr & ISTR_CTR,
        ISTR_CTR,
        "CTR is the OR over the endpoints"
    );
    assert_eq!(istr & ISTR_DIR, ISTR_DIR, "DIR: the pending one is an RX");
    assert_eq!(istr & 0x000f, 2, "EP_ID names the register, not the EA");
    // Clear RESET, keep SOF: write ones everywhere but RESET.
    f.wr(OFF_ISTR, !ISTR_RESET);
    assert_eq!(f.rd(OFF_ISTR) & ISTR_RESET, 0, "RESET cleared");
    assert_eq!(f.rd(OFF_ISTR) & (1 << 9), 1 << 9, "SOF kept");
}

#[test]
fn a_word_read_sees_the_register_in_the_low_half_and_the_high_half_is_inert() {
    let f = fixture();
    f.wr(OFF_DADDR, 0x005a);
    let mut b = [0u8; 4];
    f.regs.read(OFF_DADDR, &mut b, MemAttrs::DEFAULT).unwrap();
    assert_eq!(
        u32::from_le_bytes(b),
        0x0000_005a,
        "register low, reserved high"
    );
    assert_eq!(f.rd(OFF_DADDR + 2), 0, "the reserved half reads zero");
    f.regs
        .write(OFF_DADDR + 2, &0xffffu16.to_le_bytes(), MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(f.rd(OFF_DADDR), 0x005a, "a write to it is dropped");
}

#[test]
fn a_word_write_lands_its_low_half_and_drops_the_reserved_one() {
    let f = fixture();
    f.regs
        .write(OFF_DADDR, &0xdead_00a5u32.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("constraints admit a word, so the write does too");
    assert_eq!(f.rd(OFF_DADDR), 0x00a5, "the register is the low half");
    assert_eq!(f.rd(OFF_DADDR + 2), 0, "the reserved half took nothing");
}

#[test]
fn a_reserved_offset_and_an_offset_past_the_block_fault() {
    let f = fixture();
    let mut b = [0u8; 2];
    assert_eq!(
        f.regs.read(0x20, &mut b, MemAttrs::DEFAULT),
        Err(BusError::BadAccess),
        "the gap after the endpoint registers"
    );
    assert_eq!(
        f.regs.read(0x60, &mut b, MemAttrs::DEFAULT),
        Err(BusError::BadAccess),
        "past BCDR"
    );
}

#[test]
fn the_packet_memory_round_trips() {
    let f = fixture();
    let bytes: Vec<u8> = (0..64u8).collect();
    f.pma_write(0x80, &bytes);
    assert_eq!(f.pma_read(0x80, 64), bytes);
}

#[test]
fn a_debug_write_to_a_register_or_the_buffer_is_refused() {
    let f = fixture();
    assert_eq!(
        f.regs.write(OFF_CNTR, &0u16.to_le_bytes(), MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        f.pma.write(0, &[0u8; 2], MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );
}

// ---------------------------------------------------------------------------
// The clock gate
// ---------------------------------------------------------------------------

#[test]
fn a_gated_off_block_reads_as_zero_and_swallows_writes() {
    let f = fixture();
    f.wr(OFF_DADDR, 0x33);
    let id = WireId::new(1);
    let pin = Device::sink(&f.dev, gate::ENABLE_PIN, &[id]).expect("`enable`");
    // An `xxENR` bit resets low, so drawing the wire deasserts the clock.
    assert_eq!(f.rd(OFF_DADDR), 0, "the whole block reads zero while gated");
    assert_eq!(f.pma_read(0, 4), vec![0, 0, 0, 0], "the packet memory too");
    f.wr(OFF_DADDR, 0x44); // lost
    pin.sink.set_level(id, 0, Level::High);
    assert_eq!(f.rd(OFF_DADDR), 0x33, "the gated write never landed");
}

// ---------------------------------------------------------------------------
// On the bus
// ---------------------------------------------------------------------------

#[test]
fn the_pull_up_puts_the_device_on_the_bus_and_dropping_it_takes_it_off() {
    let f = fixture();
    assert!(!f.core().is_attached());
    f.enumerate_ready();
    assert!(f.core().is_attached(), "pull-up attached, powered up");
    assert!(f.bus.connected(0));
    assert_eq!(f.bus.speed(0), Some(Speed::Full));
    f.wr(OFF_BCDR, 0); // drop the pull-up
    assert!(!f.core().is_attached());
    assert!(!f.bus.connected(0));
}

#[test]
fn a_setup_lands_in_the_rx_buffer_and_raises_the_interrupt() {
    let f = fixture();
    f.enumerate_ready();
    let setup = host::get_descriptor(1, 0, 18);
    assert_eq!(f.bus.setup(DeviceAddress::DEFAULT, 0, setup), Status::Ack);
    assert_eq!(f.ep(0) & EP_CTR_RX, EP_CTR_RX, "CTR_RX set");
    assert_eq!(f.ep(0) & EP_SETUP, EP_SETUP, "SETUP set");
    assert_eq!(f.stat_rx(0), STAT_NAK, "RX forced to NAK");
    assert_eq!(f.stat_tx(0), STAT_NAK, "and TX too, per §30.5.3");
    assert_eq!(f.core().irq_level(), Level::High, "CTRM is enabled");
    assert_eq!(f.pma_read(0x40, 8), setup.encode(), "the eight setup bytes");
}

#[test]
fn a_control_in_moves_the_descriptor_the_firmware_staged() {
    let f = fixture();
    f.enumerate_ready();
    f.bus
        .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18));
    // Firmware stages the descriptor and arms TX.
    let descriptor: Vec<u8> = (0..18u8).collect();
    f.pma_write(0x20, &descriptor);
    f.pma_write(0x02, &18u16.to_le_bytes()); // COUNT_TX
    f.set_stat_tx(0, STAT_VALID);
    // Host IN: the whole descriptor in one packet.
    let mut buf = [0u8; 64];
    let c = f.bus.read(DeviceAddress::DEFAULT, 0, &mut buf);
    assert_eq!(c.status, Status::Ack);
    assert_eq!(c.len, 18);
    assert_eq!(&buf[..18], &descriptor[..]);
    assert_eq!(f.ep(0) & EP_CTR_TX, EP_CTR_TX, "CTR_TX set");
    assert_eq!(
        f.stat_tx(0),
        STAT_NAK,
        "TX back to NAK — firmware must re-arm"
    );
    // The debug peek shows the same without draining or re-raising.
    f.set_stat_tx(0, STAT_VALID);
    let mut peek = [0u8; 64];
    let p = f.bus.read(DeviceAddress::DEFAULT, 0, &mut peek); // real again
    assert_eq!(p.len, 18);
}

#[test]
fn an_out_lands_in_the_rx_buffer_with_its_count() {
    let f = fixture();
    f.enumerate_ready();
    let payload: Vec<u8> = (0..20u8).collect();
    let c = f.bus.write(DeviceAddress::DEFAULT, 0, &payload);
    assert_eq!(c.status, Status::Ack);
    assert_eq!(c.len, 20);
    assert_eq!(f.pma_read(0x40, 20), payload);
    let count_rx = u16::from_le_bytes(f.pma_read(0x06, 2).try_into().unwrap());
    assert_eq!(count_rx & COUNT_MASK, 20, "the byte count");
    assert_eq!(
        count_rx & !COUNT_MASK,
        0x8400 & !COUNT_MASK,
        "block size untouched"
    );
    assert_eq!(f.ep(0) & EP_CTR_RX, EP_CTR_RX);
    assert_eq!(f.ep(0) & EP_SETUP, 0, "an OUT is not a SETUP");
    assert_eq!(f.stat_rx(0), STAT_NAK);
}

#[test]
fn a_not_ready_endpoint_naks_a_stalled_one_stalls_and_an_unknown_or_disabled_one_is_silent() {
    let f = fixture();
    f.enumerate_ready();
    // TX is NAK out of enumerate_ready: an IN NAKs.
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 0, &mut [0u8; 64]).status,
        Status::Nak
    );
    f.set_stat_tx(0, STAT_STALL);
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 0, &mut [0u8; 64]).status,
        Status::Stall
    );
    // No register answers endpoint address 5: only "a token for a valid
    // function/endpoint pair" is acted on (RM0434 §40.4), so nothing answers.
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 5, &mut [0u8; 64]).status,
        Status::NoDevice
    );
    // DISABLED: "all transmission requests addressed to this endpoint are
    // ignored" (Table 261) — silence, not a STALL.
    f.set_stat_tx(0, STAT_DISABLED);
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 0, &mut [0u8; 64]).status,
        Status::NoDevice
    );
}

const STAT_STALL: u16 = 0b01;

#[test]
fn a_device_sending_past_the_hosts_packet_size_babbles() {
    let f = fixture();
    f.enumerate_ready();
    f.pma_write(0x02, &100u16.to_le_bytes()); // COUNT_TX = 100
    f.set_stat_tx(0, STAT_VALID);
    let mut buf = [0u8; 64]; // the host offers only 64
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 0, &mut buf).status,
        Status::Babble
    );
}

#[test]
fn a_full_control_transfer_through_the_composer_completes() {
    let f = fixture();
    f.enumerate_ready();
    let descriptor: Vec<u8> = (0x10..0x22u8).collect(); // 18 bytes
    f.pma_write(0x20, &descriptor);
    f.pma_write(0x02, &18u16.to_le_bytes());

    let mut xfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    let mut guard = 0;
    loop {
        // Keep the data stage armed until the device has sent it (CTR_TX), then
        // arm the receiver for the zero-length status OUT.
        if f.ep(0) & EP_CTR_TX == 0 {
            f.set_stat_tx(0, STAT_VALID);
        } else {
            f.set_stat_rx(0, STAT_VALID);
        }
        match xfer.step(&f.bus, DeviceAddress::DEFAULT, 64) {
            Progress::Done => break,
            Progress::Moved | Progress::Nak => {}
            other => panic!("unexpected {other:?}"),
        }
        guard += 1;
        assert!(guard < 12, "the transfer should finish");
    }
    assert_eq!(xfer.take_data(), descriptor);
}

#[test]
fn a_bus_reset_returns_the_device_to_address_zero_and_raises_the_flag() {
    let f = fixture();
    f.enumerate_ready();
    f.wr(OFF_DADDR, 0x80 | 0x17); // addressed
    assert_eq!(f.daddr_add(), 0x17);
    let gadget = f.core().gadget.lock().clone().unwrap();
    gadget.bus_reset();
    assert_eq!(f.daddr_add(), 0, "back to the Default state");
    assert_eq!(gadget.address(), DeviceAddress::DEFAULT);
    assert_eq!(f.rd(OFF_ISTR) & ISTR_RESET, ISTR_RESET, "RESET raised");
}

// ---------------------------------------------------------------------------
// Reception buffers: BL_SIZE / NUM_BLOCK, the overrun, the CRC
// ---------------------------------------------------------------------------

fn rd16(f: &Fixture, off: u64) -> u16 {
    u16::from_le_bytes(f.pma_read(off, 2).try_into().unwrap())
}

#[test]
fn the_allocated_size_follows_table_262() {
    let c = |bl: u16, num: u16| (bl << 15) | (num << 10);
    assert_eq!(rx_capacity(c(0, 0)), 0, "\"Not allowed\": no room");
    assert_eq!(rx_capacity(c(0, 1)), 2);
    assert_eq!(rx_capacity(c(0, 3)), 6);
    assert_eq!(rx_capacity(c(0, 31)), 62);
    assert_eq!(rx_capacity(c(1, 0)), 32);
    assert_eq!(rx_capacity(c(1, 1)), 64);
    assert_eq!(rx_capacity(c(1, 3)), 128);
    assert_eq!(rx_capacity(c(1, 14)), 480);
    assert_eq!(
        rx_capacity(c(1, 3) | 0x03ff),
        128,
        "the count bits are not the size"
    );
}

#[test]
fn the_crc_is_the_usb_crc16() {
    // The CRC-16/USB check value over "123456789".
    assert_eq!(crc16(b"123456789"), 0xb4c8);
}

#[test]
fn an_oversized_out_stalls_and_never_writes_past_the_allocated_buffer() {
    let f = fixture();
    f.enumerate_ready();
    // Eight bytes allocated (BL_SIZE = 0, NUM_BLOCK = 4), with a sentinel
    // after them.
    f.pma_write(0x06, &(4u16 << 10).to_le_bytes());
    f.pma_write(0x48, &[0xa5; 8]);
    f.wr(OFF_ISTR, 0); // the port reset's RESET flag, serviced
    let before = f.ep(0);
    let payload: Vec<u8> = (1..=12u8).collect();
    let c = f.bus.write(DeviceAddress::DEFAULT, 0, &payload);
    assert_eq!(c.status, Status::Stall, "\"a STALL handshake is sent\"");
    assert_eq!(
        f.pma_read(0x40, 8),
        payload[..8],
        "copied up to the last location"
    );
    assert_eq!(
        f.pma_read(0x48, 8),
        vec![0xa5; 8],
        "the neighbour is untouched"
    );
    assert_eq!(rd16(&f, 0x06), 4 << 10, "COUNT_RX not written back");
    assert_eq!(f.ep(0), before, "no CTR_RX, no toggle, STAT_RX still VALID");
    assert_eq!(
        f.core().irq_level(),
        Level::Low,
        "no interrupt is generated"
    );
}

#[test]
fn a_packet_that_exactly_fills_its_buffer_is_taken_and_its_crc_is_cut() {
    let f = fixture();
    f.enumerate_ready();
    f.pma_write(0x06, &(4u16 << 10).to_le_bytes()); // eight bytes
    f.pma_write(0x48, &[0xa5; 2]);
    let payload: Vec<u8> = (1..=8u8).collect();
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 0, &payload),
        Completion::ack(8)
    );
    assert_eq!(f.pma_read(0x48, 2), vec![0xa5; 2], "no room for the CRC");
    assert_eq!(rd16(&f, 0x06) & COUNT_MASK, 8);
}

#[test]
fn a_short_packet_is_followed_in_the_buffer_by_its_crc() {
    let f = fixture();
    f.enumerate_ready(); // 64 bytes allocated
    let c = f.bus.write(DeviceAddress::DEFAULT, 0, b"123456789");
    assert_eq!(c, Completion::ack(9));
    assert_eq!(
        f.pma_read(0x49, 2),
        vec![0xc8, 0xb4],
        "the CRC, low byte first"
    );
    assert_eq!(rd16(&f, 0x06) & COUNT_MASK, 9, "COUNT is the payload");
}

// ---------------------------------------------------------------------------
// Data toggles
// ---------------------------------------------------------------------------

#[test]
fn an_acked_out_toggles_dtog_rx_and_an_acked_in_toggles_dtog_tx() {
    let f = fixture();
    f.enumerate_ready();
    assert_eq!(f.ep(0) & EP_DTOG_RX, 0);
    f.bus.write(DeviceAddress::DEFAULT, 0, &[1, 2]);
    assert_eq!(f.ep(0) & EP_DTOG_RX, EP_DTOG_RX, "toggled on the ACK");

    f.pma_write(0x02, &1u16.to_le_bytes());
    f.set_stat_tx(0, STAT_VALID);
    let mut buf = [0u8; 8];
    // The peek changes nothing, the PID included.
    assert_eq!(f.bus.peek(DeviceAddress::DEFAULT, 0, &mut buf).len, 1);
    assert_eq!(f.ep(0) & EP_DTOG_TX, 0);
    let (c, pid) = f.bus.read_pid(DeviceAddress::DEFAULT, 0, &mut buf);
    assert_eq!(c, Completion::ack(1));
    assert_eq!(pid, Some(DataPid::Data0), "sent \"according to DTOG_TX\"");
    assert_eq!(
        f.ep(0) & EP_DTOG_TX,
        EP_DTOG_TX,
        "toggled on the host's ACK"
    );
    f.set_stat_tx(0, STAT_VALID);
    let (_, pid) = f.bus.read_pid(DeviceAddress::DEFAULT, 0, &mut buf);
    assert_eq!(pid, Some(DataPid::Data1));
    assert_eq!(f.ep(0) & EP_DTOG_TX, 0);
}

#[test]
fn a_setup_leaves_dtog_tx_set_and_dtog_rx_expecting_data1() {
    let f = fixture();
    f.enumerate_ready();
    for start in [0, EP_DTOG_RX | EP_DTOG_TX, EP_DTOG_RX, EP_DTOG_TX] {
        {
            let mut s = f.core().state.lock();
            s.ep[0] = (s.ep[0] & !(EP_DTOG_RX | EP_DTOG_TX | EP_CTR_RX)) | start;
        }
        assert_eq!(
            f.bus
                .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18)),
            Status::Ack
        );
        let ep = f.ep(0);
        assert_eq!(
            ep & EP_DTOG_TX,
            EP_DTOG_TX,
            "DTOG_TX set to 1 (from {start:#x})"
        );
        assert_eq!(
            ep & EP_DTOG_RX,
            EP_DTOG_RX,
            "DTOG_RX cleared at the PID, toggled by the ACK (from {start:#x})"
        );
        // An OUT data stage carrying DATA1 is then the one expected.
        f.set_stat_rx(0, STAT_VALID);
        f.core().state.lock().ep[0] &= !EP_CTR_RX;
        assert_eq!(
            f.bus
                .write_pid(DeviceAddress::DEFAULT, 0, &[9], DataPid::Data1),
            Completion::ack(1)
        );
        assert_eq!(f.ep(0) & EP_CTR_RX, EP_CTR_RX, "taken as new data");
        f.core().state.lock().ep[0] &= !EP_CTR_RX;
        f.set_stat_rx(0, STAT_VALID);
    }
}

#[test]
fn a_toggle_mismatch_is_acked_but_discarded() {
    let f = fixture();
    f.enumerate_ready();
    f.pma_write(0x40, &[0xee; 4]);
    let before = f.ep(0);
    // DTOG_RX is 0, so DATA0 is expected: DATA1 is a retransmission.
    let c = f
        .bus
        .write_pid(DeviceAddress::DEFAULT, 0, &[1, 2, 3, 4], DataPid::Data1);
    assert_eq!(c, Completion::ack(4), "the host is let move on");
    assert_eq!(f.ep(0), before, "no CTR_RX, no toggle, STAT_RX untouched");
    assert_eq!(f.pma_read(0x40, 4), vec![0xee; 4], "no data transferred");
    // The matching PID is taken.
    let c = f
        .bus
        .write_pid(DeviceAddress::DEFAULT, 0, &[1, 2, 3, 4], DataPid::Data0);
    assert_eq!(c, Completion::ack(4));
    assert_eq!(f.pma_read(0x40, 4), vec![1, 2, 3, 4]);
    assert_eq!(f.ep(0) & (EP_CTR_RX | EP_DTOG_RX), EP_CTR_RX | EP_DTOG_RX);
}

// ---------------------------------------------------------------------------
// SETUP rules, the SETUP bit, EF, STATUS_OUT
// ---------------------------------------------------------------------------

#[test]
fn a_setup_on_a_stalled_or_naked_control_endpoint_is_still_taken() {
    for rx in [STAT_STALL, STAT_NAK] {
        let f = fixture();
        f.enumerate_ready();
        f.set_stat_rx(0, rx);
        assert_eq!(
            f.bus
                .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18)),
            Status::Ack,
            "STAT_RX {rx:#b}"
        );
        assert_eq!(f.ep(0) & EP_CTR_RX, EP_CTR_RX);
    }
}

#[test]
fn a_setup_while_ctr_rx_is_pending_is_dropped_without_a_handshake() {
    let f = fixture();
    f.enumerate_ready();
    let first = host::get_descriptor(1, 0, 18);
    f.bus.setup(DeviceAddress::DEFAULT, 0, first);
    let second = host::get_descriptor(2, 0, 9);
    assert_eq!(
        f.bus.setup(DeviceAddress::DEFAULT, 0, second),
        Status::Error,
        "simulating a reception error"
    );
    assert_eq!(f.pma_read(0x40, 8), first.encode(), "the first is kept");
    // Once firmware clears CTR_RX, the retry lands.
    f.wr(0, (f.ep(0) & EP_RW) | EP_CTR_TX);
    assert_eq!(f.bus.setup(DeviceAddress::DEFAULT, 0, second), Status::Ack);
    assert_eq!(f.pma_read(0x40, 8), second.encode());
}

#[test]
fn a_setup_to_a_non_control_or_disabled_endpoint_is_ignored() {
    let f = fixture();
    f.enumerate_ready();
    f.set_stat_rx(0, STAT_DISABLED);
    assert_eq!(
        f.bus
            .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18)),
        Status::NoDevice
    );
    // Register 1: a bulk endpoint, address 1, reception VALID.
    f.core().state.lock().ep[1] = (STAT_VALID << 12) | 1;
    assert_eq!(
        f.bus
            .setup(DeviceAddress::DEFAULT, 1, host::get_descriptor(1, 0, 18)),
        Status::NoDevice,
        "only control endpoints handle SETUP"
    );
    assert_eq!(f.ep(1) & EP_CTR_RX, 0);
}

#[test]
fn the_setup_bit_is_frozen_while_ctr_rx_is_set() {
    let f = fixture();
    f.enumerate_ready();
    f.bus
        .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18));
    // Firmware re-arms RX before clearing CTR_RX, and an OUT arrives.
    f.set_stat_rx(0, STAT_VALID);
    f.bus.write(DeviceAddress::DEFAULT, 0, &[]);
    assert_eq!(f.ep(0) & EP_SETUP, EP_SETUP, "frozen while CTR_RX is 1");
    f.core().state.lock().ep[0] &= !EP_CTR_RX;
    f.set_stat_rx(0, STAT_VALID);
    f.bus.write(DeviceAddress::DEFAULT, 0, &[]);
    assert_eq!(f.ep(0) & EP_SETUP, 0, "an OUT, once it may change");
}

#[test]
fn with_the_function_disabled_nothing_is_handled() {
    let f = fixture();
    f.enumerate_ready();
    f.pma_write(0x02, &1u16.to_le_bytes());
    f.set_stat_tx(0, STAT_VALID);
    f.wr(OFF_DADDR, 0); // EF = 0
    let before = f.ep(0);
    let pma = f.pma_read(0x40, 16);
    assert_eq!(
        f.bus
            .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18)),
        Status::NoDevice
    );
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 0, &[1]),
        Completion::absent()
    );
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 0, &mut [0u8; 8]),
        Completion::absent()
    );
    assert_eq!(f.ep(0), before);
    assert_eq!(f.pma_read(0x40, 16), pma);
    f.wr(OFF_DADDR, 0x80);
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 0, &mut [0u8; 8]),
        Completion::ack(1),
        "EF set again: handled"
    );
}

#[test]
fn status_out_stalls_a_non_empty_out_and_takes_a_zero_length_one() {
    let f = fixture();
    f.enumerate_ready();
    f.core().state.lock().ep[0] |= EP_KIND; // STATUS_OUT on a control endpoint
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 0, &[1]),
        Completion::stall()
    );
    assert_eq!(f.ep(0) & EP_CTR_RX, 0, "a STALL sets no CTR_RX");
    assert_eq!(f.stat_rx(0), STAT_VALID);
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 0, &[]),
        Completion::ack(0)
    );
    assert_eq!(f.ep(0) & EP_CTR_RX, EP_CTR_RX);
    // On a bulk endpoint the same bit is DBL_BUF, not STATUS_OUT.
}

// ---------------------------------------------------------------------------
// Double-buffered bulk, isochronous
// ---------------------------------------------------------------------------

/// Register 1, endpoint address 1, its descriptor entry at 0x08: slot 0 at
/// 0x100, slot 1 at 0x140, each with 64 bytes allocated.
fn two_buffers(f: &Fixture, reg: u16) {
    f.pma_write(0x08, &0x100u16.to_le_bytes());
    f.pma_write(0x0a, &0x8400u16.to_le_bytes());
    f.pma_write(0x0c, &0x140u16.to_le_bytes());
    f.pma_write(0x0e, &0x8400u16.to_le_bytes());
    f.core().state.lock().ep[1] = reg | 1;
}

const DBL_OUT: u16 = EP_KIND | (STAT_VALID << 12); // bulk, DBL_BUF, RX only

#[test]
fn a_double_buffered_out_alternates_buffers_and_naks_on_a_conflict() {
    let f = fixture();
    f.enumerate_ready();
    // DTOG_RX = 0 (USB on buffer 0), SW_BUF (bit 6) = 1 (application on 1).
    two_buffers(&f, DBL_OUT | EP_DTOG_TX);
    // The first transaction after DBL_BUF is set ends the ordinary way.
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 1, &[1, 1]),
        Completion::ack(2)
    );
    assert_eq!(f.pma_read(0x100, 2), vec![1, 1]);
    assert_eq!(rd16(&f, 0x0a) & COUNT_MASK, 2, "COUNTn_RX_0");
    assert_eq!(f.ep(1) & EP_DTOG_RX, EP_DTOG_RX, "DTOG swapped the buffer");
    assert_eq!(f.stat_rx(1), STAT_NAK, "the first one NAKs STAT_RX");
    // Firmware: re-arm, clear CTR_RX, release buffer 0 by toggling SW_BUF.
    f.set_stat_rx(1, STAT_VALID);
    f.wr(4, (f.ep(1) & EP_RW) | EP_CTR_TX | EP_DTOG_TX);
    assert_eq!(f.ep(1) & EP_DTOG_TX, 0, "SW_BUF now 0");
    // DTOG_RX = 1, SW_BUF = 0: USB on buffer 1.
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 1, &[2, 2, 2]),
        Completion::ack(3)
    );
    assert_eq!(f.pma_read(0x140, 3), vec![2, 2, 2]);
    assert_eq!(rd16(&f, 0x0e) & COUNT_MASK, 3, "COUNTn_RX_1");
    assert_eq!(f.stat_rx(1), STAT_VALID, "STAT stays VALID from now on");
    assert_eq!(f.ep(1) & EP_DTOG_RX, 0);
    // DTOG_RX = 0 = SW_BUF: the application still holds buffer 0.
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 1, &[3]),
        Completion::nak()
    );
    assert_eq!(
        f.pma_read(0x100, 2),
        vec![1, 1],
        "nothing written on the NAK"
    );
    f.wr(4, (f.ep(1) & EP_RW) | EP_CTR_RX | EP_CTR_TX | EP_DTOG_TX);
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 1, &[3]),
        Completion::ack(1)
    );
    assert_eq!(f.pma_read(0x100, 1), vec![3]);
    // An explicit non-VALID status overrides the special flow control.
    f.set_stat_rx(1, STAT_STALL);
    assert_eq!(
        f.bus.write(DeviceAddress::DEFAULT, 1, &[4]),
        Completion::stall()
    );
}

#[test]
fn clearing_dbl_buf_rearms_the_first_transaction_rule() {
    let f = fixture();
    f.enumerate_ready();
    two_buffers(&f, DBL_OUT | EP_DTOG_TX);
    f.bus.write(DeviceAddress::DEFAULT, 1, &[1]);
    assert_eq!(f.core().state.lock().dbl_live, 1 << 1);
    // A guest write that drops DBL_BUF, then one that sets it again.
    f.wr(4, f.ep(1) & EP_RW & !EP_KIND);
    assert_eq!(f.core().state.lock().dbl_live, 0);
    f.wr(4, (f.ep(1) & EP_RW) | EP_KIND);
    f.set_stat_rx(1, STAT_VALID);
    f.bus.write(DeviceAddress::DEFAULT, 1, &[1]);
    assert_eq!(f.stat_rx(1), STAT_NAK, "a first transaction again");
}

#[test]
fn a_double_buffered_in_sends_from_the_dtog_tx_buffer() {
    let f = fixture();
    f.enumerate_ready();
    // TX only; DTOG_TX = 0 (USB on buffer 0), SW_BUF (bit 14) = 1.
    two_buffers(&f, EP_KIND | (STAT_VALID << 4) | EP_DTOG_RX);
    f.pma_write(0x100, &[0xa0, 0xa1]);
    f.pma_write(0x0a, &2u16.to_le_bytes()); // COUNTn_TX_0
    f.pma_write(0x140, &[0xb0, 0xb1, 0xb2]);
    f.pma_write(0x0e, &3u16.to_le_bytes()); // COUNTn_TX_1
    let mut buf = [0u8; 64];
    let (c, pid) = f.bus.read_pid(DeviceAddress::DEFAULT, 1, &mut buf);
    assert_eq!((c, pid), (Completion::ack(2), Some(DataPid::Data0)));
    assert_eq!(&buf[..2], &[0xa0, 0xa1]);
    assert_eq!(f.ep(1) & EP_CTR_TX, EP_CTR_TX);
    assert_eq!(
        f.stat_tx(1),
        STAT_NAK,
        "the first one ends the ordinary way"
    );
    f.set_stat_tx(1, STAT_VALID);
    // DTOG_TX = 1, SW_BUF = 1: conflict.
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 1, &mut buf),
        Completion::nak()
    );
    f.wr(4, (f.ep(1) & EP_RW) | EP_CTR_RX | EP_DTOG_RX); // SW_BUF -> 0
    let (c, pid) = f.bus.read_pid(DeviceAddress::DEFAULT, 1, &mut buf);
    assert_eq!((c, pid), (Completion::ack(3), Some(DataPid::Data1)));
    assert_eq!(&buf[..3], &[0xb0, 0xb1, 0xb2]);
    assert_eq!(f.stat_tx(1), STAT_VALID);
}

#[test]
fn an_isochronous_out_swaps_buffers_keeps_valid_and_takes_an_overrun() {
    let f = fixture();
    f.enumerate_ready();
    two_buffers(&f, (TYPE_ISO << 9) | (STAT_VALID << 12));
    f.pma_write(0x0e, &(1u16 << 10).to_le_bytes()); // RX_1: two bytes only
    // No toggle sequencing: any PID is taken.
    assert_eq!(
        f.bus
            .write_pid(DeviceAddress::DEFAULT, 1, &[1, 2, 3], DataPid::Data1),
        Completion::ack(3)
    );
    assert_eq!(f.pma_read(0x100, 3), vec![1, 2, 3], "DTOG_RX 0: buffer 0");
    assert_eq!(f.ep(1) & EP_CTR_RX, EP_CTR_RX);
    assert_eq!(f.stat_rx(1), STAT_VALID, "STAT not affected");
    f.core().state.lock().ep[1] &= !EP_CTR_RX;
    f.pma_write(0x142, &[0x5a; 4]);
    let c = f.bus.write(DeviceAddress::DEFAULT, 1, &[7, 7, 7, 7]);
    assert_eq!(c, Completion::ack(4));
    assert_eq!(f.pma_read(0x140, 2), vec![7, 7], "buffer 1, up to its end");
    assert_eq!(f.pma_read(0x142, 4), vec![0x5a; 4], "nothing past it");
    assert_eq!(f.ep(1) & EP_CTR_RX, EP_CTR_RX, "an overrun still completes");
    assert_eq!(rd16(&f, 0x0e) & COUNT_MASK, 2);
    assert_eq!(f.stat_rx(1), STAT_VALID);
}

#[test]
fn an_isochronous_in_is_always_data0_and_keeps_valid() {
    let f = fixture();
    f.enumerate_ready();
    two_buffers(&f, (TYPE_ISO << 9) | (STAT_VALID << 4) | EP_DTOG_TX);
    f.pma_write(0x140, &[0xcc]);
    f.pma_write(0x0e, &1u16.to_le_bytes()); // COUNTn_TX_1
    let mut buf = [0u8; 8];
    let (c, pid) = f.bus.read_pid(DeviceAddress::DEFAULT, 1, &mut buf);
    assert_eq!((c, pid), (Completion::ack(1), Some(DataPid::Data0)));
    assert_eq!(buf[0], 0xcc, "DTOG_TX 1: buffer 1");
    assert_eq!(f.ep(1) & EP_DTOG_TX, 0, "toggled after the transmission");
    assert_eq!(f.stat_tx(1), STAT_VALID);
    assert_eq!(f.ep(1) & EP_CTR_TX, EP_CTR_TX);
}

// ---------------------------------------------------------------------------
// Interrupt priority and the high-priority line
// ---------------------------------------------------------------------------

/// Follows the level of one output pin.
#[derive(Debug, Default)]
struct Probe {
    high: AtomicBool,
}

impl crate::core::wire::WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.high.store(level.is_high(), Ordering::Relaxed);
    }
}

fn watch(f: &Fixture, port: &str) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(
            Arc::clone(&probe) as Arc<dyn crate::core::wire::WireSink>,
            0,
        )
        .build_shared();
    Device::connect(&f.dev, port, WireSource::new(wire, id)).expect("an output pin");
    probe
}

#[test]
fn ep_id_names_an_iso_or_double_buffered_endpoint_before_the_others() {
    let f = fixture();
    {
        let mut s = f.core().state.lock();
        s.ep[0] = EP_CTR_RX | (TYPE_CONTROL << 9);
        s.ep[3] = EP_CTR_TX | EP_KIND | 3; // bulk, DBL_BUF
        s.ep[5] = EP_CTR_RX | (TYPE_ISO << 9) | 5;
    }
    let istr = f.rd(OFF_ISTR);
    assert_eq!(istr & 0x000f, 3, "the lowest of the priority set");
    assert_eq!(istr & ISTR_DIR, 0, "and it is a TX");
    f.core().state.lock().ep[3] &= !EP_CTR_TX;
    assert_eq!(f.rd(OFF_ISTR) & 0x001f, ISTR_DIR | 5);
    f.core().state.lock().ep[5] &= !EP_CTR_RX;
    assert_eq!(f.rd(OFF_ISTR) & 0x001f, ISTR_DIR, "then the others: EP0R");
}

#[test]
fn the_high_priority_line_follows_iso_and_double_buffered_ctr_whatever_ctrm_says() {
    let f = fixture();
    f.enumerate_ready();
    f.wr(OFF_ISTR, 0); // the port reset's RESET flag, serviced
    f.wr(OFF_CNTR, CNTR_RESETM); // CTRM off
    let lp = watch(&f, pin::IRQ);
    let hp = watch(&f, pin::IRQ_HP);
    // An ordinary endpoint's CTR is not high-priority.
    f.bus.write(DeviceAddress::DEFAULT, 0, &[1]);
    assert!(!hp.high.load(Ordering::Relaxed));
    assert!(
        !lp.high.load(Ordering::Relaxed),
        "CTRM masks the generic one"
    );
    // An isochronous one is, CTRM or not.
    two_buffers(&f, (TYPE_ISO << 9) | (STAT_VALID << 12));
    f.bus.write(DeviceAddress::DEFAULT, 1, &[1]);
    assert!(hp.high.load(Ordering::Relaxed));
    assert_eq!(f.core().irq_hp_level(), Level::High);
    assert!(!lp.high.load(Ordering::Relaxed));
    // It falls when software clears the endpoint's CTR_RX.
    f.wr(4, (f.ep(1) & EP_RW) | EP_CTR_TX);
    assert!(!hp.high.load(Ordering::Relaxed));
    // With CTRM set the generic line carries it too.
    f.wr(OFF_CNTR, CNTR_CTRM);
    f.bus.write(DeviceAddress::DEFAULT, 1, &[1]);
    assert!(hp.high.load(Ordering::Relaxed));
    assert!(lp.high.load(Ordering::Relaxed));
}

#[test]
fn only_the_two_interrupt_pins_connect() {
    let f = fixture();
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let wire = Wire::builder().source(id).build_shared();
    assert!(Device::connect(&f.dev, "irq_lp", WireSource::new(wire, id)).is_err());
}

// ---------------------------------------------------------------------------
// Snapshot, schema, registry
// ---------------------------------------------------------------------------

#[test]
fn a_snapshot_round_trips() {
    let f = fixture();
    f.enumerate_ready();
    f.bus
        .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18));
    let marker: Vec<u8> = (0..32u8).collect();
    f.pma_write(0x80, &marker);
    // A double-buffered endpoint past its first transaction: hidden state.
    two_buffers(&f, DBL_OUT | EP_DTOG_TX);
    f.bus.write(DeviceAddress::DEFAULT, 1, &[1]);
    assert_eq!(f.core().state.lock().dbl_live, 1 << 1);

    let mut shape = MachineShape::new();
    shape.add_device("usb", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("usb", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&f.dev, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = fixture();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("usb", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.dev, &mut chunk.reader()).unwrap();

    for off in [0x00u64, OFF_CNTR, OFF_ISTR, OFF_DADDR, OFF_BTABLE, OFF_BCDR] {
        assert_eq!(restored.rd(off), f.rd(off), "register {off:#x}");
    }
    assert_eq!(restored.pma_read(0x80, 32), marker);
    assert_eq!(restored.ep(1), f.ep(1));
    assert_eq!(
        restored.core().state.lock().dbl_live,
        1 << 1,
        "the double-buffer latch"
    );
    assert_eq!(
        restored.pma_read(0x40, 8),
        f.pma_read(0x40, 8),
        "the setup buffer"
    );
}

#[test]
fn the_class_registers_and_constructs() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    assert!(
        reg.create(CLASS_NAME, &Props::new()).is_err(),
        "`bus` is required"
    );
    let props = Props::new().with("bus", Value::from("usb0"));
    let device = reg.create(CLASS_NAME, &props).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);
}

#[test]
fn the_schema_and_the_device_agree_about_regions_and_pins() {
    let f = fixture();
    let schema = schema();
    for region in ["", "regs", "pma", "sram"] {
        assert!(
            Device::region(&f.dev, region).is_some(),
            "region {region:?}"
        );
    }
    assert!(Device::region(&f.dev, "nope").is_none());
    assert!(schema.port_named(pin::IRQ).is_some());
    assert!(schema.port_named(gate::ENABLE_PIN).is_some());
    assert!(schema.port_named(gate::RESET_PIN).is_some());
}

// ---------------------------------------------------------------------------
// The register map (RM0434 §40.6.1/§40.6.3, RM0091 §30.6.1/§30.6.3)
// ---------------------------------------------------------------------------

#[test]
fn every_register_comes_out_of_reset_at_the_tables_value() {
    // The register map tables: `CNTR` resets to 0x0003, every other register
    // to zero (`FNR`'s undefined bits read zero here before any SOF).
    let f = fixture();
    assert_eq!(f.rd(OFF_CNTR), 0x0003, "PDWN and FRES");
    for off in [OFF_ISTR, OFF_DADDR, OFF_BTABLE, OFF_LPMCSR, OFF_BCDR] {
        assert_eq!(f.rd(off), 0, "register {off:#x}");
    }
    for n in 0..8 {
        assert_eq!(f.ep(n), 0, "EP{n}R");
    }
}

#[test]
fn reserved_and_read_only_bits_do_not_take_a_write() {
    let f = fixture();
    // CNTR bit 6: "Reserved, must be kept at reset value".
    f.wr(OFF_CNTR, 0xffff);
    assert_eq!(f.rd(OFF_CNTR), 0xffbf);
    // DADDR 15:8 reserved; EF and ADD[6:0] are rw.
    f.wr(OFF_DADDR, 0xffff);
    assert_eq!(f.rd(OFF_DADDR), 0x00ff);
    // BTABLE 2:0: "Reserved, forced by hardware to 0".
    f.wr(OFF_BTABLE, 0xffff);
    assert_eq!(f.rd(OFF_BTABLE), 0xfff8);
    // LPMCSR: only LPMACK/LPMEN are rw; BESL and REMWAKE are the hardware's.
    f.core().state.lock().lpmcsr = 0x00f8;
    f.wr(OFF_LPMCSR, 0x0000);
    assert_eq!(
        f.rd(OFF_LPMCSR),
        0x00f8,
        "a write leaves BESL/REMWAKE alone"
    );
    f.wr(OFF_LPMCSR, 0xffff);
    assert_eq!(f.rd(OFF_LPMCSR), 0x00fb, "and sets only bits 1:0");
    // BCDR: DPPU and the four enables; the results are `r`. With every mode
    // selected at once no result is defined, so none reads back.
    f.wr(OFF_BCDR, 0xffff);
    assert_eq!(f.rd(OFF_BCDR), 0x800f);
    assert_eq!(
        f.core().state.lock().bcdr,
        0x800f,
        "no result bit is stored"
    );
    // FNR is read-only throughout: the write decodes and changes nothing.
    f.regs
        .write(OFF_FNR, &0xffffu16.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a write to a read-only register is not a bus fault");
    assert_eq!(f.rd(OFF_FNR), 0);
}

#[test]
fn the_packet_memory_takes_bytes_and_half_words_but_not_words() {
    // "The packet memory should be accessed only by byte (8-bit) or half-word
    // (16-bit) accesses. Word (32-bit) accesses are not allowed."
    let f = fixture();
    let c = f.pma.constraints();
    let at = MemAttrs::DEFAULT;
    assert!(c.check(0x10, Width::U8, at).is_ok());
    assert!(c.check(0x11, Width::U8, at).is_ok());
    assert!(c.check(0x10, Width::U16, at).is_ok());
    assert!(c.check(0x11, Width::U16, at).is_err(), "half-word aligned");
    assert!(c.check(0x10, Width::U32, at).is_err(), "no word access");
    // The register file is the other way round: "can be accessed by
    // half-words (16-bit) or words (32-bit)".
    let r = f.regs.constraints();
    assert!(r.check(OFF_CNTR, Width::U16, at).is_ok());
    assert!(r.check(OFF_CNTR, Width::U32, at).is_ok());
    assert!(r.check(OFF_CNTR, Width::U8, at).is_err());
}

// ---------------------------------------------------------------------------
// Battery charging detection
// ---------------------------------------------------------------------------

fn fixture_on(port_type: PortType) -> Fixture {
    let bus = Arc::new(UsbBus::new(1));
    let dev = UsbfsPeripheral::with_bus_port_type(Arc::clone(&bus), 0, port_type);
    let regs = io(&Device::region(&dev, "regs").expect("the register block"));
    let pma = io(&Device::region(&dev, "pma").expect("the packet memory"));
    Fixture {
        dev,
        bus,
        regs,
        pma,
    }
}

/// `BCDR`'s result bits (7:4) with the detector on and `mode` selected.
fn detect(f: &Fixture, mode: u16) -> u16 {
    f.wr(OFF_BCDR, BCDR_BCDEN | mode);
    f.rd(OFF_BCDR) & 0x00f0
}

#[test]
fn each_port_type_answers_the_detection_it_should() {
    // (port, DCDET, PDET, PS2DET, SDET). The bit meanings are RM0434
    // §40.6.1's: PDET "0: ... SDP ... 1: ... ACA, CDP or DCP", SDET "0: CDP
    // detected. 1: DCP detected.", PS2DET "1: PS2 port or proprietary
    // charger detected."
    let table = [
        (PortType::Sdp, true, false, false, false),
        (PortType::Cdp, true, true, false, false),
        (PortType::Dcp, true, true, false, true),
        (PortType::Ps2, false, true, true, true),
        (PortType::Unplugged, false, false, false, false),
    ];
    for (port, dcd, pd, ps2, sd) in table {
        let f = fixture_on(port);
        let b = |set: bool, bit: u16| if set { bit } else { 0 };
        assert_eq!(detect(&f, BCDR_DCDEN), b(dcd, BCDR_DCDET), "{port:?} DCD");
        assert_eq!(
            detect(&f, BCDR_PDEN),
            b(pd, BCDR_PDET) | b(ps2, BCDR_PS2DET),
            "{port:?} PD"
        );
        assert_eq!(detect(&f, BCDR_SDEN), b(sd, BCDR_SDET), "{port:?} SD");
        assert_eq!(detect(&f, 0), 0, "{port:?}: no mode, no result");
        f.wr(OFF_BCDR, BCDR_PDEN);
        assert_eq!(
            f.rd(OFF_BCDR),
            BCDR_PDEN,
            "{port:?}: BCDEN clear, no result"
        );
    }
}

#[test]
fn the_detector_holds_the_phy_off_the_bus() {
    // "When enabled, the USB PHY is fully controlled by BCD and cannot be
    // used for normal communication."
    let f = fixture();
    f.enumerate_ready();
    assert!(f.core().is_attached());
    f.wr(OFF_BCDR, BCDR_DPPU | BCDR_BCDEN | BCDR_PDEN);
    assert!(!f.core().is_attached(), "the detector has the lines");
    assert_eq!(
        f.rd(OFF_BCDR) & BCDR_PDET,
        0,
        "and an SDP answers PD with 0"
    );
    f.wr(OFF_BCDR, BCDR_DPPU);
    assert!(f.core().is_attached(), "OFF mode hands them back");
}

#[test]
fn port_type_is_a_property_with_sdp_the_default() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    let props = Props::new()
        .with("bus", Value::from("usb-bcd"))
        .with("port-type", Value::from("dcp"));
    assert!(reg.create(CLASS_NAME, &props).is_ok());
    let bad = Props::new()
        .with("bus", Value::from("usb-bcd2"))
        .with("port-type", Value::from("aca"));
    assert!(
        reg.create(CLASS_NAME, &bad).is_err(),
        "not a port kind we model"
    );
    assert_eq!(fixture().core().port_type(), PortType::Sdp);
    assert!(schema().port_named(pin::CAN_ENABLE).is_some());
}

// ---------------------------------------------------------------------------
// The F0x2's CAN share of the packet memory
// ---------------------------------------------------------------------------

#[test]
fn an_unwired_can_enable_leaves_all_of_the_packet_memory_usbs() {
    let f = fixture();
    assert!(!f.core().can_holds_pma());
    f.pma_write(0x3fe, &[0xa5, 0x5a]);
    assert_eq!(f.pma_read(0x3fe, 2), vec![0xa5, 0x5a]);
}

#[test]
fn with_can_clocked_the_top_256_bytes_are_not_usbs() {
    let f = fixture();
    f.pma_write(0x2fe, &[1, 2]);
    f.pma_write(0x300, &[3, 4]);
    // Drive the pin the way RCC would, through the sink a board's wire gets.
    let pin = Device::sink(&f.dev, pin::CAN_ENABLE, &[WireId::new(7)]).expect("the pin");
    pin.sink.set_level(WireId::new(7), 0, Level::High);
    assert!(f.core().can_holds_pma());
    assert_eq!(
        f.pma_read(0x2fe, 2),
        vec![1, 2],
        "the first 768 bytes stay USB's"
    );
    assert_eq!(
        f.pma_read(0x300, 2),
        vec![0, 0],
        "the last 256 read nothing"
    );
    f.pma_write(0x300, &[9, 9]);
    {
        let s = f.core().state.lock();
        assert_eq!(s.pma_u16(0x300), 0, "nor does the controller see them");
        assert_eq!(&s.pma[0x300..0x302], &[3, 4], "and the write was lost");
    }
    // A reset of the block does not forget a level its RCC still drives.
    Device::reset(&f.dev, ResetKind::Cold);
    assert_eq!(f.core().state.lock().usb_pma_len, 0x300);
    pin.sink.set_level(WireId::new(7), 0, Level::Low);
    assert!(!f.core().can_holds_pma());
    assert_eq!(f.pma_read(0x300, 2), vec![0, 0], "the reset cleared them");
    f.pma_write(0x300, &[5, 6]);
    assert_eq!(f.pma_read(0x300, 2), vec![5, 6], "USB's again");
}

#[test]
fn a_reception_buffer_running_into_the_can_share_writes_nothing_there() {
    let f = fixture();
    f.enumerate_ready();
    f.pma_write(0x04, &0x2fcu16.to_le_bytes()); // ADDR_RX four bytes below it
    f.core().state.lock().pma[0x300..0x304].copy_from_slice(&[0xaa; 4]);
    let pin = Device::sink(&f.dev, pin::CAN_ENABLE, &[WireId::new(7)]).expect("the pin");
    pin.sink.set_level(WireId::new(7), 0, Level::High);
    let c = f.bus.write(DeviceAddress::DEFAULT, 0, &[1, 2, 3, 4, 5, 6]);
    assert_eq!(
        c,
        Completion::ack(6),
        "the buffer's size is the descriptor's"
    );
    let s = f.core().state.lock();
    assert_eq!(&s.pma[0x2fc..0x300], &[1, 2, 3, 4]);
    assert_eq!(
        &s.pma[0x300..0x304],
        &[0xaa; 4],
        "the CAN bytes are untouched"
    );
}
