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
use crate::core::wire::Level;

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
    Fixture { dev, bus, regs, pma }
}

impl Fixture {
    fn core(&self) -> &Arc<Usbfs> {
        self.dev.core()
    }

    fn rd(&self, offset: u64) -> u16 {
        let mut b = [0u8; 2];
        self.regs.read(offset, &mut b, MemAttrs::DEFAULT).expect("read");
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
        self.pma.write(off, bytes, MemAttrs::DEFAULT).expect("pma write");
    }
    fn pma_read(&self, off: u64, len: usize) -> Vec<u8> {
        let mut v = vec![0u8; len];
        self.pma.read(off, &mut v, MemAttrs::DEFAULT).expect("pma read");
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
    assert_eq!(istr & ISTR_CTR, ISTR_CTR, "CTR is the OR over the endpoints");
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
    assert_eq!(u32::from_le_bytes(b), 0x0000_005a, "register low, reserved high");
    assert_eq!(f.rd(OFF_DADDR + 2), 0, "the reserved half reads zero");
    f.regs
        .write(OFF_DADDR + 2, &0xffffu16.to_le_bytes(), MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(f.rd(OFF_DADDR), 0x005a, "a write to it is dropped");
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
    f.bus.setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18));
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
    assert_eq!(f.stat_tx(0), STAT_NAK, "TX back to NAK — firmware must re-arm");
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
    assert_eq!(count_rx & !COUNT_MASK, 0x8400 & !COUNT_MASK, "block size untouched");
    assert_eq!(f.ep(0) & EP_CTR_RX, EP_CTR_RX);
    assert_eq!(f.ep(0) & EP_SETUP, 0, "an OUT is not a SETUP");
    assert_eq!(f.stat_rx(0), STAT_NAK);
}

#[test]
fn a_not_ready_endpoint_naks_a_stalled_one_stalls_and_an_unknown_address_stalls() {
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
    // No register answers endpoint address 5.
    assert_eq!(
        f.bus.read(DeviceAddress::DEFAULT, 5, &mut [0u8; 64]).status,
        Status::Stall
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
// Snapshot, schema, registry
// ---------------------------------------------------------------------------

#[test]
fn a_snapshot_round_trips() {
    let f = fixture();
    f.enumerate_ready();
    f.bus.setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18));
    let marker: Vec<u8> = (0..32u8).collect();
    f.pma_write(0x80, &marker);

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
    assert_eq!(restored.pma_read(0x40, 8), f.pma_read(0x40, 8), "the setup buffer");
}

#[test]
fn the_class_registers_and_constructs() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    assert!(reg.create(CLASS_NAME, &Props::new()).is_err(), "`bus` is required");
    let props = Props::new().with("bus", Value::from("usb0"));
    let device = reg.create(CLASS_NAME, &props).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);
}

#[test]
fn the_schema_and_the_device_agree_about_regions_and_pins() {
    let f = fixture();
    let schema = schema();
    for region in ["", "regs", "pma", "sram"] {
        assert!(Device::region(&f.dev, region).is_some(), "region {region:?}");
    }
    assert!(Device::region(&f.dev, "nope").is_none());
    assert!(schema.port_named(pin::IRQ).is_some());
    assert!(schema.port_named(gate::ENABLE_PIN).is_some());
    assert!(schema.port_named(gate::RESET_PIN).is_some());
}
