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
    f.bus
        .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18));
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
