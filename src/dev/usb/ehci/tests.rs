//! The EHCI controller, driven the way a driver drives it.
//!
//! Every test here builds **real queue heads and real transfer descriptors in
//! real guest memory**, points `ASYNCLISTADDR` or `PERIODICLISTBASE` at them,
//! sets `USBCMD.RS`, and lets the controller find its own way. Nothing calls a
//! walker function directly, because the thing worth testing is that a
//! structure a driver would build is one this controller can read.
//!
//! The device on the far end is a small [`Function`] declared here rather than
//! [`crate::dev::usb::hid`]: this file has to pass with only `dev-usb-ehci`
//! enabled, and CI runs `cargo test` one feature at a time.

use super::*;

use alloc::vec::Vec;

use crate::bus::usb::{
    ConfigurationDescriptor, Descriptors, DeviceDescriptor, Direction, EndpointDescriptor,
    Function, InterfaceDescriptor, Peripheral, TransferType, UsbDevice,
};
use crate::core::space::{RamStore, RegionKind};
use crate::core::sync::Mutex;

/// Where guest RAM starts, and how much of it there is.
const RAM_BASE: u64 = 0x1000;
/// 60 KiB, which leaves the first page unmapped so a null pointer in a
/// descriptor is a bus fault rather than a plausible read.
const RAM_SIZE: u64 = 0xf000;

/// A short microframe, so a test advances time in numbers a person can read.
const MICROFRAME: u64 = 10;

// Addresses inside guest RAM the tests lay their structures out at. Chosen so
// nothing overlaps and every queue head is 32-byte aligned, as §3.6 requires.
const QH_ADDR: u32 = 0x2000;
const QTD0: u32 = 0x2100;
const QTD1: u32 = 0x2140;
const QTD2: u32 = 0x2180;
const SETUP_BUF: u32 = 0x2200;
const DATA_BUF: u32 = 0x2300;
const FRAME_LIST: u32 = 0x4000;
const INT_QH: u32 = 0x5000;
const INT_QTD: u32 = 0x5100;
const INT_BUF: u32 = 0x5200;

// ---------------------------------------------------------------------------
// A device to talk to
// ---------------------------------------------------------------------------

/// What the test device has been asked and told.
#[derive(Debug, Default)]
struct Log {
    /// Bytes handed to the `OUT` endpoint.
    written: Vec<u8>,
    /// A payload waiting on the `IN` endpoint, if any.
    pending: Option<Vec<u8>>,
    /// How many `IN` transactions have been refused.
    naks: u32,
    /// Whether the endpoint should stall.
    stall: bool,
}

/// A device with one bulk `IN` endpoint, one bulk `OUT`, and descriptors.
#[derive(Debug)]
struct Widget {
    descriptors: Descriptors,
    log: Mutex<Log>,
}

/// Its `IN` endpoint.
const EP_IN: u8 = 1;
/// Its `OUT` endpoint.
const EP_OUT: u8 = 2;

impl Widget {
    fn new() -> Widget {
        let device = DeviceDescriptor {
            vendor: 0xdead,
            product: 0xbeef,
            ..DeviceDescriptor::default()
        };
        let mut body = Vec::new();
        body.extend_from_slice(
            &InterfaceDescriptor {
                endpoints: 2,
                class: 0xff,
                ..InterfaceDescriptor::default()
            }
            .encode(),
        );
        body.extend_from_slice(
            &EndpointDescriptor {
                address: EP_IN | Direction::BIT,
                attributes: TransferType::Bulk.attribute_bits(),
                max_packet: 8,
                interval: 0,
            }
            .encode(),
        );
        body.extend_from_slice(
            &EndpointDescriptor {
                address: EP_OUT,
                attributes: TransferType::Bulk.attribute_bits(),
                max_packet: 8,
                interval: 0,
            }
            .encode(),
        );
        let mut descriptors = Descriptors::new().with_device(&device);
        descriptors.add_configuration(&ConfigurationDescriptor::default(), &body);
        Widget {
            descriptors,
            log: Mutex::with_rank(LockRank::DEVICE, Log::default()),
        }
    }
}

impl Function for Widget {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn speed(&self) -> Speed {
        Speed::High
    }

    fn reset(&self) {
        *self.log.lock() = Log::default();
    }

    fn endpoint_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != EP_IN {
            return Completion::stall();
        }
        let mut log = self.log.lock();
        if log.stall {
            return Completion::stall();
        }
        let Some(payload) = log.pending.take() else {
            log.naks += 1;
            return Completion::nak();
        };
        let n = payload.len().min(dst.len());
        dst[..n].copy_from_slice(&payload[..n]);
        if n < payload.len() {
            log.pending = Some(payload[n..].to_vec());
        }
        Completion::ack(n as u64)
    }

    fn endpoint_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        if endpoint != EP_OUT {
            return Completion::stall();
        }
        let mut log = self.log.lock();
        if log.stall {
            return Completion::stall();
        }
        log.written.extend_from_slice(src);
        Completion::ack(src.len() as u64)
    }
}

// ---------------------------------------------------------------------------
// The fixture
// ---------------------------------------------------------------------------

/// A controller, its space, its bus and the device on port 0.
struct Fixture {
    controller: EhciController,
    space: Arc<AddressSpace>,
    bus: Arc<UsbBus>,
    widget: Arc<Widget>,
    ops: Arc<dyn MemOps>,
}

/// A high-speed widget on port 0 of a one-port controller, with 60 KiB of RAM
/// under it.
fn fixture() -> Fixture {
    fixture_with(Speed::High)
}

fn fixture_with(speed: Speed) -> Fixture {
    let space = AddressSpace::new("mem", 32);
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_SIZE))),
            RAM_BASE,
        )
        .expect("the map fits");
    }
    let space = Arc::new(space);

    let bus = Arc::new(UsbBus::new(1));
    let widget = Arc::new(Widget::new());
    let device: Arc<dyn UsbDevice> = match speed {
        Speed::High => Arc::new(Peripheral::new(Arc::clone(&widget) as Arc<dyn Function>)),
        other => Arc::new(SlowDevice {
            inner: Peripheral::new(Arc::clone(&widget) as Arc<dyn Function>),
            speed: other,
        }),
    };
    bus.attach(0, device).expect("an empty port");

    let controller = EhciController::with_bus(
        Arc::clone(&bus),
        Params {
            ports: 1,
            microframe_ticks: MICROFRAME,
            caplength: DEFAULT_CAPLENGTH,
            dual_role: false,
        },
    );
    controller.hcd().attach_space(&space, RequesterId(0x1234));

    let region = controller.region("").expect("the register block");
    let ops = match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        other => panic!("expected an io region, got {other:?}"),
    };
    Fixture {
        controller,
        space,
        bus,
        widget,
        ops,
    }
}

/// The same [`Peripheral`], claiming to be slower than it is.
///
/// For the one test that matters about speed: what an EHCI does when something
/// it cannot drive is plugged in.
#[derive(Debug)]
struct SlowDevice {
    inner: Peripheral,
    speed: Speed,
}

impl UsbDevice for SlowDevice {
    fn speed(&self) -> Speed {
        self.speed
    }
    fn address(&self) -> DeviceAddress {
        self.inner.address()
    }
    fn bus_reset(&self) {
        self.inner.bus_reset();
    }
    fn setup(&self, endpoint: u8, packet: SetupPacket) -> Status {
        self.inner.setup(endpoint, packet)
    }
    fn transfer_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        self.inner.transfer_in(endpoint, dst)
    }
    fn transfer_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        self.inner.transfer_out(endpoint, src)
    }
}

impl Fixture {
    /// The offset of an operational register in the region.
    fn op(offset: u64) -> u64 {
        u64::from(DEFAULT_CAPLENGTH) + offset
    }

    fn read(&self, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        self.ops
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a register read");
        u32::from_le_bytes(bytes)
    }

    fn read_debug(&self, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        self.ops
            .read(offset, &mut bytes, MemAttrs::DEBUG)
            .expect("a debug register read");
        u32::from_le_bytes(bytes)
    }

    fn write(&self, offset: u64, value: u32) {
        self.ops
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a register write");
    }

    fn poke(&self, addr: u32, value: u32) {
        self.space
            .write(
                u64::from(addr),
                Width::U32,
                u64::from(value),
                MemAttrs::DEBUG,
            )
            .expect("guest RAM");
    }

    fn peek(&self, addr: u32) -> u32 {
        self.space
            .read(u64::from(addr), Width::U32, MemAttrs::DEBUG)
            .expect("guest RAM") as u32
    }

    fn peek_bytes(&self, addr: u32, len: usize) -> Vec<u8> {
        let mut out = alloc::vec![0u8; len];
        self.space
            .read_bytes(u64::from(addr), &mut out, MemAttrs::DEBUG)
            .expect("guest RAM");
        out
    }

    fn poke_bytes(&self, addr: u32, bytes: &[u8]) {
        self.space
            .write_bytes(u64::from(addr), bytes, MemAttrs::DEBUG)
            .expect("guest RAM");
    }

    /// Run the controller forward by `microframes`.
    fn run(&self, microframes: u64) {
        let now = self.controller.current_tick();
        self.controller
            .advance_to(now + microframes * MICROFRAME + MICROFRAME);
    }

    /// What a driver does before it can talk to anything: claim the ports,
    /// reset the one with something on it, and start the controller.
    fn bring_up(&self) {
        // CONFIGFLAG: these ports are mine, not a companion's (§4.2.2).
        self.write(Fixture::op(0x40), 1);
        let sc = self.read(Fixture::op(0x44));
        assert_ne!(sc & PORT_CCS, 0, "something is plugged in");
        assert_ne!(sc & PORT_CSC, 0, "and the change bit says so");
        // Acknowledge the change, then drive a reset.
        self.write(Fixture::op(0x44), sc | PORT_RESET);
        assert_eq!(
            self.read(Fixture::op(0x44)) & PORT_PE,
            0,
            "disabled while reset"
        );
        // Software clears the reset, and that is what enables the port (§2.3.9).
        let sc = self.read(Fixture::op(0x44));
        self.write(Fixture::op(0x44), sc & !PORT_RESET);
    }

    /// Start the controller with the asynchronous schedule enabled.
    fn start_async(&self, head: u32) {
        self.write(Fixture::op(0x18), head);
        self.write(Fixture::op(0x00), CMD_RS | CMD_ASE);
    }
}

// ---------------------------------------------------------------------------
// Building a schedule, the way a driver would
// ---------------------------------------------------------------------------

/// A queue head's twelve dwords (§3.6).
struct QueueHead {
    /// Where it lives.
    addr: u32,
    /// Horizontal link, with its type and terminate bits.
    link: u32,
    /// Endpoint characteristics.
    epchar: u32,
    /// Endpoint capabilities.
    epcap: u32,
    /// The first descriptor, in the overlay's Next qTD Pointer.
    first: u32,
}

impl QueueHead {
    /// A queue head for `endpoint` of `address`, with `mps`-byte packets, whose
    /// horizontal link points back at itself — which is what a one-entry
    /// circular asynchronous list is.
    fn control(addr: u32, address: u8, endpoint: u8, mps: u32, first: u32) -> QueueHead {
        QueueHead {
            addr,
            link: addr | (TYP_QH << LINK_TYP_SHIFT),
            epchar: u32::from(address)
                | (u32::from(endpoint) << EPCHAR_EP_SHIFT)
                // EPS = 10b: high speed.
                | (0x2 << EPCHAR_EPS_SHIFT)
                // DTC: take the toggle from the descriptor, which is what a
                // control queue head does so that SETUP can force DATA0.
                | EPCHAR_DTC
                | EPCHAR_H
                | (mps << EPCHAR_MPS_SHIFT),
            // Mult = 01b: one transaction per microframe.
            epcap: 1 << 30,
            first,
        }
    }

    fn write(&self, f: &Fixture) {
        f.poke(self.addr, self.link);
        f.poke(self.addr + 4, self.epchar);
        f.poke(self.addr + 8, self.epcap);
        // CurrentqTD: nothing yet.
        f.poke(self.addr + 12, 0);
        // The overlay: Next qTD Pointer is where the queue starts, the rest is
        // clear and, crucially, not active.
        f.poke(self.addr + 16, self.first);
        f.poke(self.addr + 20, LINK_T);
        for i in 0..7 {
            f.poke(self.addr + 24 + i * 4, 0);
        }
    }
}

/// One queue element transfer descriptor (§3.5).
struct Qtd {
    addr: u32,
    next: u32,
    alt: u32,
    pid: u32,
    bytes: u32,
    toggle: bool,
    ioc: bool,
    buffer: u32,
}

impl Qtd {
    fn write(&self, f: &Fixture) {
        f.poke(self.addr, self.next);
        f.poke(self.addr + 4, self.alt);
        let mut token = TOKEN_ACTIVE
            | (self.pid << TOKEN_PID_SHIFT)
            // CERR = 3: the retry count a driver sets.
            | (3 << 10)
            | ((self.bytes & TOKEN_BYTES_MASK) << TOKEN_BYTES_SHIFT);
        if self.toggle {
            token |= TOKEN_TOGGLE;
        }
        if self.ioc {
            token |= TOKEN_IOC;
        }
        f.poke(self.addr + 8, token);
        f.poke(self.addr + 12, self.buffer);
        for i in 1..5 {
            f.poke(self.addr + 12 + i * 4, 0);
        }
    }
}

/// The token dword of the descriptor at `addr`, as the controller left it.
fn token_of(f: &Fixture, addr: u32) -> u32 {
    f.peek(addr + 8)
}

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

#[test]
fn the_capability_registers_say_what_the_spec_says() {
    let f = fixture();
    let cap = f.read(0x00);
    assert_eq!(cap & 0xff, u32::from(DEFAULT_CAPLENGTH), "CAPLENGTH");
    assert_eq!(cap >> 16, u32::from(HCIVERSION), "HCIVERSION is 1.0");
    assert_eq!(f.read(0x04) & 0xf, 1, "HCSPARAMS.N_PORTS");
    // 32-bit addressing, programmable frame list, no extended capabilities —
    // so no BIOS handoff for a driver to negotiate.
    assert_eq!(f.read(0x08) & 1, 0, "HCCPARAMS: 32-bit addressing");
    assert_ne!(f.read(0x08) & 2, 0, "HCCPARAMS: the frame list is sizeable");
    assert_eq!((f.read(0x08) >> 8) & 0xff, 0, "no extended capabilities");
}

#[test]
fn caplength_is_readable_as_a_byte() {
    // Every driver reads it that way, because it is one.
    let f = fixture();
    let mut byte = [0u8; 1];
    f.ops
        .read(0, &mut byte, MemAttrs::DEFAULT)
        .expect("a byte read");
    assert_eq!(byte[0], DEFAULT_CAPLENGTH);
}

#[test]
fn the_controller_comes_up_halted() {
    let f = fixture();
    assert_ne!(f.read(Fixture::op(0x04)) & STS_HCHALTED, 0);
    assert_eq!(
        f.controller.next_event_tick(),
        None,
        "and schedules nothing"
    );
}

#[test]
fn usbsts_is_write_one_to_clear_and_a_debug_write_is_refused() {
    let f = fixture();
    f.bring_up();
    // Bringing the port up set Port Change Detect.
    assert_ne!(f.read(Fixture::op(0x04)) & STS_PORT_CHANGE, 0);

    // A debugger must not acknowledge it (`ROADMAP.md` §15, invariant 5).
    assert!(
        f.ops
            .write(
                Fixture::op(0x04),
                &STS_PORT_CHANGE.to_le_bytes(),
                MemAttrs::DEBUG
            )
            .is_err(),
        "a debug write to a write-1-to-clear status register must be refused"
    );
    assert_ne!(
        f.read(Fixture::op(0x04)) & STS_PORT_CHANGE,
        0,
        "and must not have cleared it"
    );

    // Writing a zero to the bit leaves it alone; writing a one clears it.
    f.write(Fixture::op(0x04), 0);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_PORT_CHANGE, 0);
    f.write(Fixture::op(0x04), STS_PORT_CHANGE);
    assert_eq!(f.read(Fixture::op(0x04)) & STS_PORT_CHANGE, 0);

    // And the read-only half is not writable at all.
    let before = f.read(Fixture::op(0x04)) & STS_HCHALTED;
    f.write(Fixture::op(0x04), STS_HCHALTED);
    assert_eq!(f.read(Fixture::op(0x04)) & STS_HCHALTED, before);
}

#[test]
fn a_debug_read_does_not_advance_the_frame_counter() {
    let f = fixture();
    f.bring_up();
    f.write(Fixture::op(0x00), CMD_RS);
    f.run(4);
    let frindex = f.read(Fixture::op(0x0c));
    assert!(frindex > 0, "the controller is counting microframes");

    // A debug read syncs with `AccessKind::Debug`, which advances nothing —
    // and reading it a hundred times must not move it either.
    for _ in 0..100 {
        assert_eq!(f.read_debug(Fixture::op(0x0c)), frindex);
    }
}

#[test]
fn the_schedule_status_bits_follow_the_enables() {
    let f = fixture();
    f.bring_up();
    assert_eq!(f.read(Fixture::op(0x04)) & (STS_PSS | STS_ASS), 0);
    f.write(Fixture::op(0x00), CMD_RS | CMD_ASE);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_ASS, 0, "async running");
    assert_eq!(f.read(Fixture::op(0x04)) & STS_PSS, 0, "periodic is not");
    f.write(Fixture::op(0x00), CMD_RS | CMD_ASE | CMD_PSE);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_PSS, 0);
    // Stopping the controller stops both, whatever `USBCMD` still says.
    f.write(Fixture::op(0x00), CMD_ASE | CMD_PSE);
    assert_eq!(f.read(Fixture::op(0x04)) & (STS_PSS | STS_ASS), 0);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_HCHALTED, 0);
}

#[test]
fn a_reset_puts_everything_back() {
    let f = fixture();
    f.bring_up();
    f.write(Fixture::op(0x14), 0x8000);
    f.write(Fixture::op(0x18), QH_ADDR);
    f.write(Fixture::op(0x08), STS_USBINT);
    f.write(Fixture::op(0x00), CMD_HCRESET);
    assert_eq!(f.read(Fixture::op(0x00)) & CMD_HCRESET, 0, "self-clearing");
    assert_eq!(f.read(Fixture::op(0x14)), 0);
    assert_eq!(f.read(Fixture::op(0x18)), 0);
    assert_eq!(f.read(Fixture::op(0x08)), 0);
    assert_eq!(f.read(Fixture::op(0x40)), 0, "CONFIGFLAG too");
    assert_ne!(f.read(Fixture::op(0x04)) & STS_HCHALTED, 0);
}

// ---------------------------------------------------------------------------
// Ports and speeds
// ---------------------------------------------------------------------------

#[test]
fn a_high_speed_device_is_kept_and_enabled() {
    let f = fixture();
    f.bring_up();
    let sc = f.read(Fixture::op(0x44));
    assert_ne!(sc & PORT_PE, 0, "the port is enabled");
    assert_eq!(sc & PORT_OWNER, 0, "and it is ours");
    assert!(f.bus.enabled(0), "so the fabric routes to it");
}

/// The rule the module docs make a point of: EHCI is high-speed only.
#[test]
fn a_full_speed_device_is_handed_to_a_companion() {
    let f = fixture_with(Speed::Full);
    f.bring_up();
    let sc = f.read(Fixture::op(0x44));
    assert_ne!(
        sc & PORT_OWNER,
        0,
        "EHCI 1.0 §4.2.2: a device it cannot drive is released to a companion"
    );
    assert_eq!(sc & PORT_PE, 0, "and the port is not enabled");
    assert!(
        !f.bus.enabled(0),
        "so nothing this controller does can reach the device — which is the \
         honest outcome on a board with no companion controller"
    );
}

#[test]
fn a_low_speed_device_is_released_before_any_reset() {
    let f = fixture_with(Speed::Low);
    // Just claiming the ports is enough: a low-speed device announces itself
    // in the line state, and §4.2.2 releases it without a reset.
    f.write(Fixture::op(0x40), 1);
    let sc = f.read(Fixture::op(0x44));
    assert_ne!(sc & PORT_OWNER, 0);
    assert_eq!((sc >> 10) & 0x3, 1, "line status K: a low-speed device");
}

#[test]
fn before_configflag_every_port_belongs_to_a_companion() {
    let f = fixture();
    assert_ne!(
        f.read(Fixture::op(0x44)) & PORT_OWNER,
        0,
        "EHCI 1.0 §4.2: with CONFIGFLAG clear the ports are the companions'"
    );
}

// ---------------------------------------------------------------------------
// The asynchronous schedule: a real control transfer
// ---------------------------------------------------------------------------

/// Lay out a three-descriptor control transfer — `SETUP`, data `IN`, status
/// `OUT` — the way a driver does, and return the number of data bytes asked
/// for.
fn build_control_in(f: &Fixture, address: u8, setup: SetupPacket, length: u32) {
    f.poke_bytes(SETUP_BUF, &setup.encode());
    QueueHead::control(QH_ADDR, address, 0, 64, QTD0).write(f);
    Qtd {
        addr: QTD0,
        next: QTD1,
        alt: LINK_T,
        pid: PID_SETUP,
        bytes: 8,
        toggle: false,
        ioc: false,
        buffer: SETUP_BUF,
    }
    .write(f);
    Qtd {
        addr: QTD1,
        next: QTD2,
        alt: LINK_T,
        pid: PID_IN,
        bytes: length,
        toggle: true,
        ioc: false,
        buffer: DATA_BUF,
    }
    .write(f);
    Qtd {
        addr: QTD2,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_OUT,
        bytes: 0,
        toggle: true,
        ioc: true,
        buffer: 0,
    }
    .write(f);
}

/// A `GET_DESCRIPTOR(DEVICE)` setup packet.
fn get_device_descriptor(length: u16) -> SetupPacket {
    SetupPacket {
        request_type: 0x80,
        request: crate::bus::usb::request::GET_DESCRIPTOR,
        value: 0x0100,
        index: 0,
        length,
    }
}

#[test]
fn the_controller_dma_walks_a_control_transfer_and_the_descriptor_lands_in_guest_ram() {
    let f = fixture();
    f.bring_up();
    build_control_in(&f, 0, get_device_descriptor(18), 18);
    f.write(Fixture::op(0x08), STS_USBINT);
    f.start_async(QH_ADDR);
    f.run(4);

    // The eighteen bytes of a device descriptor, moved by the controller out of
    // the device and into the buffer the driver named — with nothing but guest
    // memory between them.
    let bytes = f.peek_bytes(DATA_BUF, 18);
    assert_eq!(bytes[0], 18, "bLength");
    assert_eq!(bytes[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(u16::from_le_bytes([bytes[8], bytes[9]]), 0xdead, "idVendor");
    assert_eq!(u16::from_le_bytes([bytes[10], bytes[11]]), 0xbeef);

    // Every descriptor retired, with no bytes left and no error.
    for qtd in [QTD0, QTD1, QTD2] {
        let token = token_of(&f, qtd);
        assert_eq!(token & TOKEN_ACTIVE, 0, "{qtd:#x} is still active");
        assert_eq!(token & TOKEN_HALTED, 0, "{qtd:#x} halted");
    }
    assert_eq!(
        (token_of(&f, QTD1) >> TOKEN_BYTES_SHIFT) & TOKEN_BYTES_MASK,
        0,
        "the data descriptor moved every byte it asked for"
    );

    // `IOC` on the last one raised the interrupt, and the wire followed.
    assert_ne!(f.read(Fixture::op(0x04)) & STS_USBINT, 0);
    assert!(
        f.controller.hcd().irq_level().is_high(),
        "the IRQ line is up"
    );

    // Acknowledging it lowers the line, which is what a handler does.
    f.write(Fixture::op(0x04), STS_USBINT);
    assert!(f.controller.hcd().irq_level().is_low());
}

#[test]
fn a_short_packet_stops_the_transfer_and_reports_the_remainder() {
    let f = fixture();
    f.bring_up();
    // Ask for 64 bytes of an 18-byte descriptor. The device runs out, sends a
    // short packet, and the controller has to notice.
    build_control_in(&f, 0, get_device_descriptor(64), 64);
    f.start_async(QH_ADDR);
    f.run(4);

    let token = token_of(&f, QTD1);
    assert_eq!(token & TOKEN_ACTIVE, 0, "the descriptor retired");
    assert_eq!(token & TOKEN_HALTED, 0, "a short packet is not an error");
    assert_eq!(
        (token >> TOKEN_BYTES_SHIFT) & TOKEN_BYTES_MASK,
        64 - 18,
        "the driver reads the length back out of the residue"
    );
}

#[test]
fn set_address_moves_the_device_and_the_controller_follows_it() {
    let f = fixture();
    f.bring_up();

    // A `SET_ADDRESS`: a SETUP and a zero-length status IN, no data stage.
    let setup = SetupPacket {
        request_type: 0x00,
        request: crate::bus::usb::request::SET_ADDRESS,
        value: 5,
        index: 0,
        length: 0,
    };
    f.poke_bytes(SETUP_BUF, &setup.encode());
    QueueHead::control(QH_ADDR, 0, 0, 64, QTD0).write(&f);
    Qtd {
        addr: QTD0,
        next: QTD1,
        alt: LINK_T,
        pid: PID_SETUP,
        bytes: 8,
        toggle: false,
        ioc: false,
        buffer: SETUP_BUF,
    }
    .write(&f);
    Qtd {
        addr: QTD1,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_IN,
        bytes: 0,
        toggle: true,
        ioc: true,
        buffer: 0,
    }
    .write(&f);
    f.start_async(QH_ADDR);
    f.run(4);
    assert_eq!(token_of(&f, QTD1) & TOKEN_ACTIVE, 0, "the status stage ran");
    assert_eq!(
        f.bus.device(0).expect("a device").address(),
        DeviceAddress(5)
    );

    // And now the *old* address answers nothing, while a queue head addressed
    // to 5 works — which is the whole point of the exercise.
    f.write(Fixture::op(0x00), 0);
    build_control_in(&f, 5, get_device_descriptor(18), 18);
    f.write(Fixture::op(0x00), CMD_RS | CMD_ASE);
    f.run(4);
    assert_eq!(f.peek_bytes(DATA_BUF, 1)[0], 18);
}

#[test]
fn a_stall_halts_the_queue_head_and_raises_the_error_interrupt() {
    let f = fixture();
    f.bring_up();
    // `GET_DESCRIPTOR(STRING)` — this device has no string table.
    let setup = SetupPacket {
        request_type: 0x80,
        request: crate::bus::usb::request::GET_DESCRIPTOR,
        value: 0x0300,
        index: 0,
        length: 8,
    };
    build_control_in(&f, 0, setup, 8);
    f.start_async(QH_ADDR);
    f.run(4);

    let token = token_of(&f, QTD1);
    assert_ne!(token & TOKEN_HALTED, 0, "the data stage halted");
    assert_eq!(token & TOKEN_ACTIVE, 0);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_USBERRINT, 0);
    // And the status descriptor was never reached: a halted queue stops.
    assert_ne!(token_of(&f, QTD2) & TOKEN_ACTIVE, 0);
}

#[test]
fn an_out_transfer_carries_bytes_from_guest_ram_to_the_device() {
    let f = fixture();
    f.bring_up();
    let payload: Vec<u8> = (0u8..20).collect();
    f.poke_bytes(DATA_BUF, &payload);
    // A bulk queue head on the OUT endpoint, eight-byte packets — so twenty
    // bytes is three packets, and the last one is short.
    QueueHead::control(QH_ADDR, 0, EP_OUT, 8, QTD0).write(&f);
    Qtd {
        addr: QTD0,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_OUT,
        bytes: payload.len() as u32,
        toggle: false,
        ioc: true,
        buffer: DATA_BUF,
    }
    .write(&f);
    f.start_async(QH_ADDR);
    f.run(2);

    assert_eq!(f.widget.log.lock().written, payload);
    assert_eq!(token_of(&f, QTD0) & TOKEN_ACTIVE, 0);
}

#[test]
fn a_nak_leaves_the_descriptor_active_and_the_host_comes_back() {
    let f = fixture();
    f.bring_up();
    QueueHead::control(QH_ADDR, 0, EP_IN, 8, QTD0).write(&f);
    Qtd {
        addr: QTD0,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_IN,
        bytes: 8,
        toggle: false,
        ioc: true,
        buffer: DATA_BUF,
    }
    .write(&f);
    f.start_async(QH_ADDR);
    f.run(3);

    assert_ne!(
        token_of(&f, QTD0) & TOKEN_ACTIVE,
        0,
        "a NAK is not an error and does not retire the transfer"
    );
    assert_eq!(f.read(Fixture::op(0x04)) & STS_USBERRINT, 0);
    assert!(f.widget.log.lock().naks >= 3, "and the host kept asking");

    // Now give it something, and the same descriptor completes.
    f.widget.log.lock().pending = Some(alloc::vec![0xa5; 8]);
    f.run(2);
    assert_eq!(token_of(&f, QTD0) & TOKEN_ACTIVE, 0);
    assert_eq!(f.peek_bytes(DATA_BUF, 8), alloc::vec![0xa5; 8]);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_USBINT, 0);
}

#[test]
fn a_transfer_crosses_a_page_boundary() {
    let f = fixture();
    f.bring_up();
    // Start six bytes below a page boundary, so an eight-byte packet straddles
    // it and the controller has to move to the next buffer pointer.
    let first = 0x2ffa_u32;
    let second = 0x3000_u32;
    f.widget.log.lock().pending = Some((0u8..16).collect());
    QueueHead::control(QH_ADDR, 0, EP_IN, 8, QTD0).write(&f);
    f.poke(QTD0, LINK_T);
    f.poke(QTD0 + 4, LINK_T);
    f.poke(
        QTD0 + 8,
        TOKEN_ACTIVE
            | (PID_IN << TOKEN_PID_SHIFT)
            | (3 << 10)
            | (16 << TOKEN_BYTES_SHIFT)
            | TOKEN_IOC,
    );
    f.poke(QTD0 + 12, first);
    f.poke(QTD0 + 16, second);
    for i in 2..5 {
        f.poke(QTD0 + 12 + i * 4, 0);
    }
    f.start_async(QH_ADDR);
    f.run(3);

    assert_eq!(token_of(&f, QTD0) & TOKEN_ACTIVE, 0, "it completed");
    let bytes = f.peek_bytes(first, 16);
    assert_eq!(
        bytes,
        (0u8..16).collect::<Vec<u8>>(),
        "across the page seam"
    );
    assert_eq!(
        (token_of(&f, QTD0) >> TOKEN_CPAGE_SHIFT) & 0x7,
        1,
        "and C_Page moved on, as §3.5.4 says it must"
    );
}

#[test]
fn the_async_advance_doorbell_is_answered() {
    let f = fixture();
    f.bring_up();
    QueueHead::control(QH_ADDR, 0, 0, 64, LINK_T).write(&f);
    f.start_async(QH_ADDR);
    f.write(Fixture::op(0x00), CMD_RS | CMD_ASE | CMD_IAAD);
    f.run(2);
    assert_eq!(
        f.read(Fixture::op(0x00)) & CMD_IAAD,
        0,
        "the doorbell self-clears once the list has been traversed"
    );
    assert_ne!(f.read(Fixture::op(0x04)) & STS_IAA, 0);
}

// ---------------------------------------------------------------------------
// The periodic schedule
// ---------------------------------------------------------------------------

#[test]
fn an_interrupt_endpoint_is_serviced_from_the_periodic_schedule() {
    let f = fixture();
    f.bring_up();

    // A frame list whose every entry points at one interrupt queue head, which
    // is what a driver polling once a frame builds.
    for i in 0..1024u32 {
        f.poke(FRAME_LIST + i * 4, INT_QH | (TYP_QH << LINK_TYP_SHIFT));
    }
    let mut qh = QueueHead::control(INT_QH, 0, EP_IN, 8, INT_QTD);
    // An interrupt queue head terminates rather than looping, and its S-mask
    // says which microframes it is serviced in (§3.6.3). `0x01` is the first
    // of the eight.
    qh.link = LINK_T;
    qh.epchar &= !EPCHAR_H;
    qh.epcap = (1 << 30) | 0x01;
    qh.write(&f);
    Qtd {
        addr: INT_QTD,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_IN,
        bytes: 4,
        toggle: false,
        ioc: true,
        buffer: INT_BUF,
    }
    .write(&f);

    f.widget.log.lock().pending = Some(alloc::vec![1, 2, 3, 4]);
    f.write(Fixture::op(0x14), FRAME_LIST);
    f.write(Fixture::op(0x00), CMD_RS | CMD_PSE);
    // Eight microframes is one frame, so this is bound to include the
    // microframe the S-mask selects.
    f.run(20);

    assert_eq!(token_of(&f, INT_QTD) & TOKEN_ACTIVE, 0, "it was serviced");
    assert_eq!(f.peek_bytes(INT_BUF, 4), alloc::vec![1, 2, 3, 4]);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_USBINT, 0);
}

#[test]
fn a_queue_head_is_skipped_in_a_microframe_its_mask_does_not_select() {
    let f = fixture();
    f.bring_up();
    for i in 0..1024u32 {
        f.poke(FRAME_LIST + i * 4, INT_QH | (TYP_QH << LINK_TYP_SHIFT));
    }
    let mut qh = QueueHead::control(INT_QH, 0, EP_IN, 8, INT_QTD);
    qh.link = LINK_T;
    qh.epchar &= !EPCHAR_H;
    // An S-mask of zero selects no microframe at all, which is a legal thing
    // for a driver to leave behind while it rebuilds a schedule.
    qh.epcap = 1 << 30;
    qh.write(&f);
    Qtd {
        addr: INT_QTD,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_IN,
        bytes: 4,
        toggle: false,
        ioc: true,
        buffer: INT_BUF,
    }
    .write(&f);
    f.widget.log.lock().pending = Some(alloc::vec![1, 2, 3, 4]);
    f.write(Fixture::op(0x14), FRAME_LIST);
    f.write(Fixture::op(0x00), CMD_RS | CMD_PSE);
    f.run(40);
    assert_ne!(
        token_of(&f, INT_QTD) & TOKEN_ACTIVE,
        0,
        "a zero S-mask is never the current microframe"
    );
}

#[test]
fn the_frame_list_rolls_over_and_says_so() {
    let f = fixture();
    f.bring_up();
    // The smallest frame list the controller offers is 256 entries, so a
    // rollover is 2048 microframes rather than 8192 — which is the reason
    // `HCCPARAMS` advertises a programmable list at all.
    f.write(Fixture::op(0x00), (0x2 << CMD_FLS_SHIFT) | CMD_RS);
    f.run(2100);
    assert_ne!(f.read(Fixture::op(0x04)) & STS_FLR, 0);
}

// ---------------------------------------------------------------------------
// A guest-controlled walk is a hostile walk
// ---------------------------------------------------------------------------

#[test]
fn a_queue_head_that_links_to_itself_does_not_hang() {
    let f = fixture();
    f.bring_up();
    // A single queue head whose horizontal link is itself: the ordinary case,
    // and the one every async list is. It has to terminate after one lap.
    QueueHead::control(QH_ADDR, 0, EP_IN, 8, LINK_T).write(&f);
    f.start_async(QH_ADDR);
    f.run(4);
}

#[test]
fn a_descriptor_that_points_at_itself_does_not_hang() {
    let f = fixture();
    f.bring_up();
    QueueHead::control(QH_ADDR, 0, EP_OUT, 8, QTD0).write(&f);
    Qtd {
        addr: QTD0,
        // Its own address: retiring it advances the queue straight back to it.
        next: QTD0,
        alt: LINK_T,
        pid: PID_OUT,
        bytes: 1,
        toggle: false,
        ioc: false,
        buffer: DATA_BUF,
    }
    .write(&f);
    f.start_async(QH_ADDR);
    f.run(4);
    // Bounded, not unbounded: the guest gets a finite amount of work per
    // microframe out of a self-referential list, and the microframe ends.
    assert!(
        f.widget.log.lock().written.len() <= MAX_QTD_ADVANCE * 4 + 4,
        "the walk is bounded per microframe"
    );
}

#[test]
fn a_circular_frame_list_does_not_hang() {
    let f = fixture();
    f.bring_up();
    // Every frame-list entry points at a queue head whose horizontal link
    // points back at *itself*, which in the periodic schedule is a cycle rather
    // than the terminator a driver should have written.
    for i in 0..1024u32 {
        f.poke(FRAME_LIST + i * 4, INT_QH | (TYP_QH << LINK_TYP_SHIFT));
    }
    let mut qh = QueueHead::control(INT_QH, 0, EP_IN, 8, LINK_T);
    qh.link = INT_QH | (TYP_QH << LINK_TYP_SHIFT);
    qh.epcap = (1 << 30) | 0xff;
    qh.write(&f);
    f.write(Fixture::op(0x14), FRAME_LIST);
    f.write(Fixture::op(0x00), CMD_RS | CMD_PSE);
    f.run(4);
}

#[test]
fn a_long_async_list_is_walked_a_bounded_number_of_nodes_at_a_time() {
    let f = fixture();
    f.bring_up();
    // A hundred queue heads in a ring — more than `MAX_ASYNC_QH`. The walk has
    // to stop, and the next microframe picks up where a real controller would:
    // at the head, because that is where `ASYNCLISTADDR` points.
    let count = 100u32;
    for i in 0..count {
        let addr = QH_ADDR + i * 0x40;
        let next = QH_ADDR + ((i + 1) % count) * 0x40;
        let mut qh = QueueHead::control(addr, 0, EP_IN, 8, LINK_T);
        qh.link = next | (TYP_QH << LINK_TYP_SHIFT);
        qh.write(&f);
    }
    f.start_async(QH_ADDR);
    f.run(4);
}

#[test]
fn a_descriptor_pointing_outside_ram_faults_the_controller_rather_than_the_host() {
    let f = fixture();
    f.bring_up();
    QueueHead::control(QH_ADDR, 0, EP_OUT, 8, QTD0).write(&f);
    Qtd {
        addr: QTD0,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_OUT,
        bytes: 8,
        toggle: false,
        ioc: true,
        // Below `RAM_BASE`: unmapped, and the space faults rather than
        // inventing bytes.
        buffer: 0,
    }
    .write(&f);
    f.start_async(QH_ADDR);
    f.run(2);
    let token = token_of(&f, QTD0);
    assert_ne!(
        token & TOKEN_HALTED,
        0,
        "the transfer is retired with an error"
    );
    assert_ne!(token & TOKEN_DBE, 0, "and it is a data buffer error");
}

#[test]
fn a_queue_head_outside_ram_halts_the_controller() {
    let f = fixture();
    f.bring_up();
    // `ASYNCLISTADDR` pointing into the hole below RAM.
    f.start_async(0x20);
    f.run(2);
    assert_ne!(
        f.read(Fixture::op(0x04)) & STS_HSE,
        0,
        "EHCI 1.0 §2.3.2: a DMA fault is a host system error"
    );
    assert_ne!(
        f.read(Fixture::op(0x04)) & STS_HCHALTED,
        0,
        "and the controller stops rather than walking a list it cannot read"
    );
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn the_register_file_round_trips() {
    let f = fixture();
    f.bring_up();
    f.write(Fixture::op(0x08), STS_USBINT | STS_USBERRINT);
    f.write(Fixture::op(0x14), FRAME_LIST);
    f.start_async(QH_ADDR);
    f.run(7);

    let mut saved = Vec::new();
    f.controller.hcd().save(&mut saved).expect("it saves");

    let fresh = fixture();
    {
        let mut reader = crate::core::state::ChunkReader::new(&saved);
        fresh.controller.hcd().load(&mut reader).expect("it loads");
    }
    let mut again = Vec::new();
    fresh.controller.hcd().save(&mut again).expect("it saves");
    assert_eq!(saved, again, "the register file did not round trip");

    // The restored controller stands where the first one did, in every
    // register a driver can see.
    for offset in [0x00, 0x04, 0x08, 0x0c, 0x14, 0x18, 0x40, 0x44] {
        assert_eq!(
            fresh.read(Fixture::op(offset)),
            f.read(Fixture::op(offset)),
            "operational register {offset:#x}"
        );
    }
    // And the fabric's enable bit, which is derived state and is rebuilt from
    // `PORTSC` rather than serialized (`ROADMAP.md` §4.5).
    assert!(fresh.bus.enabled(0));
}

/// A snapshot taken while a transfer is in flight.
///
/// It is worth being precise about what "in flight" can mean here: the
/// controller executes a whole transaction inside one microframe and every
/// durable thing about a transfer — the queue head, the overlay, the token —
/// lives in **guest memory**. So the interesting case is a *queue* that is
/// part-way through, and what has to survive is that the second descriptor of
/// three still runs after the restore.
#[test]
fn a_half_finished_queue_resumes_after_a_restore() {
    let f = fixture();
    f.bring_up();
    // Two eight-byte reads from the bulk `IN` endpoint, and only eight bytes
    // to give: the first descriptor retires, the second `NAK`s and stays
    // active. That is a queue arrested part-way through, which is the only
    // shape "mid-transfer" can take here — a *transaction* never straddles a
    // microframe.
    QueueHead::control(QH_ADDR, 0, EP_IN, 8, QTD0).write(&f);
    Qtd {
        addr: QTD0,
        next: QTD1,
        alt: LINK_T,
        pid: PID_IN,
        bytes: 8,
        toggle: false,
        ioc: false,
        buffer: DATA_BUF,
    }
    .write(&f);
    Qtd {
        addr: QTD1,
        next: LINK_T,
        alt: LINK_T,
        pid: PID_IN,
        bytes: 8,
        toggle: true,
        ioc: true,
        buffer: DATA_BUF + 8,
    }
    .write(&f);
    f.widget.log.lock().pending = Some((0u8..8).collect());
    f.start_async(QH_ADDR);
    f.run(2);

    assert_eq!(
        token_of(&f, QTD0) & TOKEN_ACTIVE,
        0,
        "the first one retired"
    );
    assert_ne!(
        token_of(&f, QTD1) & TOKEN_ACTIVE,
        0,
        "and the second is waiting on the device"
    );
    assert_eq!(f.peek_bytes(DATA_BUF, 8), (0u8..8).collect::<Vec<u8>>());

    let mut saved = Vec::new();
    f.controller.hcd().save(&mut saved).expect("it saves");

    // A fresh machine with the same guest memory — which is where the queue
    // head, its overlay and both descriptors live — and the restored register
    // file.
    let fresh = fixture();
    for addr in [QH_ADDR, QTD0, QTD1] {
        for i in 0..12u32 {
            fresh.poke(addr + i * 4, f.peek(addr + i * 4));
        }
    }
    fresh.poke_bytes(DATA_BUF, &f.peek_bytes(DATA_BUF, 16));
    {
        let mut reader = crate::core::state::ChunkReader::new(&saved);
        fresh.controller.hcd().load(&mut reader).expect("it loads");
    }

    // Now give the restored machine's device the rest, and the queue picks up
    // at the descriptor the first machine was standing on.
    fresh.widget.log.lock().pending = Some((8u8..16).collect());
    fresh.run(3);
    assert_eq!(
        token_of(&fresh, QTD1) & TOKEN_ACTIVE,
        0,
        "the restored controller finished the transfer the first one started"
    );
    assert_eq!(
        fresh.peek_bytes(DATA_BUF, 16),
        (0u8..16).collect::<Vec<u8>>()
    );
    assert_ne!(fresh.read(Fixture::op(0x04)) & STS_USBINT, 0);
}
