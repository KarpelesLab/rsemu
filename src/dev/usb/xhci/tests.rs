//! Unit tests for the xHCI controller.
//!
//! Everything here drives the controller **through its register block**, with a
//! real address space behind it and a real device on the fabric: the tests build
//! a Device Context Base Address Array, a command ring, an event ring with a
//! segment table and transfer rings in "guest" memory and then ring doorbells,
//! exactly as `tests/usb_xhci.rs` makes a RISC-V program do. Nothing calls the
//! engine directly except where a test is specifically about a bound.
//!
//! The register block is **mapped into the same space** the controller masters,
//! which is what lets `a_data_buffer_aimed_at_the_doorbells_does_not_recurse`
//! exist at all: a guest can point a Normal TRB's data buffer at the doorbell
//! array, and four bytes of disk data are a doorbell write.

use super::*;

use alloc::vec;
use core::cell::Cell;

use crate::bus::usb::{
    ConfigurationDescriptor, Descriptors, DeviceDescriptor, Direction, EndpointDescriptor,
    Function, InterfaceDescriptor, Peripheral, TransferType,
};
use crate::core::space::{MemOps, RamStore, RegionKind};
use crate::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};

// ---------------------------------------------------------------------------
// The map both sides agree on
// ---------------------------------------------------------------------------

/// Where "guest" RAM starts. Not zero, so a null pointer in a context is a bus
/// fault the controller has to survive rather than a plausible read.
const RAM: u64 = 0x1_0000;
/// How much of it there is.
const RAM_SIZE: u64 = 0x8000;
/// Where the register block is mapped, in the same space.
const REGS: u64 = 0x10_0000;

const DCBAA: u64 = RAM;
const ERST: u64 = RAM + 0x040;
const DEV_CTX: u64 = RAM + 0x400;
const IN_CTX: u64 = RAM + 0x800;
/// Sixteen TRBs, the smallest an Event Ring segment may be (§6.5) — and small
/// on purpose, so `a_full_event_ring_reports_itself_and_stops_the_rings` can
/// fill it.
const EVT_RING: u64 = RAM + 0x1000;
const EVT_TRBS: u32 = 16;
/// Sixty-four, which is comfortably more than any test queues.
const CMD_RING: u64 = RAM + 0x1400;
const EP0_RING: u64 = RAM + 0x1800;
const EPIN_RING: u64 = RAM + 0x1900;
const EPOUT_RING: u64 = RAM + 0x1a00;
const BUF: u64 = RAM + 0x2000;

/// Register offsets, derived rather than repeated.
const OP: u64 = offset::OPERATIONAL;
const USBCMD: u64 = OP;
const USBSTS: u64 = OP + 0x04;
const CRCR: u64 = OP + 0x18;
const DCBAAP: u64 = OP + 0x30;
const CONFIG: u64 = OP + 0x38;
const PORTSC1: u64 = OP + offset::PORT;
const DB: u64 = offset::DOORBELL;
const IR0: u64 = offset::RUNTIME + offset::INTERRUPTER0;
const IMAN: u64 = IR0;
const IMOD: u64 = IR0 + 0x04;
const ERSTSZ: u64 = IR0 + 0x08;
const ERSTBA: u64 = IR0 + 0x10;
const ERDP: u64 = IR0 + 0x18;

/// The bulk endpoint numbers the test device answers on, and the Device Context
/// Indices they map to (§4.5.1: `DCI = number * 2 + direction`).
const EP_IN: u8 = 1;
const EP_OUT: u8 = 2;
const DCI_EP0: u32 = 1;
const DCI_IN: u32 = 3;
const DCI_OUT: u32 = 4;
/// The bulk endpoints' maximum packet size.
const MPS: u32 = 64;

// ---------------------------------------------------------------------------
// A device to talk to
// ---------------------------------------------------------------------------

/// The smallest device that exercises everything a controller does: a control
/// pipe with descriptors, a bulk IN endpoint that produces a pattern, and a
/// bulk OUT endpoint that keeps what it is given.
#[derive(Debug)]
struct Echo {
    descriptors: Descriptors,
    state: Mutex<EchoState>,
}

#[derive(Debug, Default)]
struct EchoState {
    /// What the last `OUT` delivered.
    received: alloc::vec::Vec<u8>,
    /// How many bytes the next `IN` produces, or `None` to answer `NAK`.
    available: Option<usize>,
    /// Whether the bulk IN endpoint refuses.
    stall_in: bool,
    /// How many `NAK`s to answer before producing anything.
    naks: u32,
}

impl Echo {
    fn new() -> Arc<Echo> {
        let mut body = alloc::vec::Vec::new();
        body.extend_from_slice(
            &InterfaceDescriptor {
                number: 0,
                alternate: 0,
                endpoints: 2,
                class: 0xff,
                subclass: 0,
                protocol: 0,
                name: 0,
            }
            .encode(),
        );
        body.extend_from_slice(
            &EndpointDescriptor {
                address: EP_IN | Direction::BIT,
                attributes: TransferType::Bulk.attribute_bits(),
                max_packet: MPS as u16,
                interval: 0,
            }
            .encode(),
        );
        body.extend_from_slice(
            &EndpointDescriptor {
                address: EP_OUT,
                attributes: TransferType::Bulk.attribute_bits(),
                max_packet: MPS as u16,
                interval: 0,
            }
            .encode(),
        );
        let descriptors = Descriptors::new()
            .with_device(&DeviceDescriptor {
                usb: 0x0200,
                class: 0,
                subclass: 0,
                protocol: 0,
                max_packet0: 64,
                vendor: 0x1d6b,
                product: 0x0002,
                device: 0x0100,
                manufacturer: 0,
                product_name: 0,
                serial: 0,
                configurations: 1,
            })
            .configuration(
                &ConfigurationDescriptor {
                    interfaces: 1,
                    value: 1,
                    name: 0,
                    attributes: ConfigurationDescriptor::RESERVED,
                    max_power: 50,
                },
                &body,
            );
        Arc::new(Echo {
            descriptors,
            state: Mutex::with_rank(LockRank::DEVICE, EchoState::default()),
        })
    }
}

impl Function for Echo {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn speed(&self) -> Speed {
        Speed::High
    }

    fn endpoint_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        if endpoint != EP_IN {
            return Completion::stall();
        }
        let mut state = self.state.lock();
        if state.stall_in {
            return Completion::stall();
        }
        if state.naks > 0 {
            state.naks -= 1;
            return Completion::nak();
        }
        let Some(available) = state.available else {
            return Completion::nak();
        };
        let n = available.min(dst.len());
        for (i, byte) in dst[..n].iter_mut().enumerate() {
            *byte = (i as u8).wrapping_mul(3).wrapping_add(0x40);
        }
        state.available = Some(available - n);
        Completion::ack(n as u64)
    }

    fn endpoint_out(&self, endpoint: u8, src: &[u8]) -> Completion {
        if endpoint != EP_OUT {
            return Completion::stall();
        }
        self.state.lock().received.extend_from_slice(src);
        Completion::ack(src.len() as u64)
    }
}

// ---------------------------------------------------------------------------
// The fixture
// ---------------------------------------------------------------------------

struct Fixture {
    controller: XhciController,
    space: Arc<AddressSpace>,
    ops: Arc<dyn MemOps>,
    device: Arc<Echo>,
    /// The next free slot on the command ring.
    cmd_next: Cell<u64>,
    /// The command ring's Producer Cycle State, from software's side.
    cmd_cycle: Cell<bool>,
    /// Where the test has read to on the event ring.
    evt_next: Cell<u64>,
    /// The event ring's Consumer Cycle State.
    evt_cycle: Cell<bool>,
}

fn build() -> Fixture {
    let space = AddressSpace::new("mem", 32);
    {
        let mut topo = space.topology();
        topo.map(Region::ram("ram", Arc::new(RamStore::new(RAM_SIZE))), RAM)
            .expect("the map fits");
    }
    let space = Arc::new(space);

    let bus = Arc::new(UsbBus::new(1));
    let device = Echo::new();
    bus.attach(
        0,
        Arc::new(Peripheral::new(Arc::clone(&device) as Arc<dyn Function>)),
    )
    .expect("an empty port");

    let controller = XhciController::with_bus(
        Arc::clone(&bus),
        Params {
            ports: 1,
            slots: 4,
            // Short, so a test that lets a microframe pass is cheap.
            microframe_ticks: 8,
        },
    );
    controller.xhci().attach_space(&space, RequesterId(9));

    let region = controller.region("").expect("the register block");
    {
        let mut topo = space.topology();
        topo.map(Arc::clone(&region), REGS).expect("the map fits");
    }
    let ops = match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        _ => panic!("expected an io region"),
    };

    controller.reset(ResetKind::Cold);
    Fixture {
        controller,
        space,
        ops,
        device,
        cmd_next: Cell::new(CMD_RING),
        cmd_cycle: Cell::new(true),
        evt_next: Cell::new(EVT_RING),
        evt_cycle: Cell::new(true),
    }
}

impl Fixture {
    fn wr(&self, offset: u64, value: u32) {
        self.ops
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a legal register write");
    }

    fn rd(&self, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        self.ops
            .read(offset, &mut buf, MemAttrs::DEFAULT)
            .expect("a legal register read");
        u32::from_le_bytes(buf)
    }

    fn mem_w(&self, addr: u64, value: u32) {
        self.space
            .write(addr, Width::U32, u64::from(value), MemAttrs::DEBUG)
            .expect("mapped RAM");
    }

    fn mem_r(&self, addr: u64) -> u32 {
        self.space
            .read(addr, Width::U32, MemAttrs::DEBUG)
            .expect("mapped RAM") as u32
    }

    fn mem_bytes(&self, addr: u64, len: usize) -> alloc::vec::Vec<u8> {
        let mut out = vec![0u8; len];
        self.space
            .read_bytes(addr, &mut out, MemAttrs::DEBUG)
            .expect("mapped RAM");
        out
    }

    fn put_trb(&self, addr: u64, trb: [u32; 4]) {
        for (i, word) in trb.iter().enumerate() {
            self.mem_w(addr + (i * 4) as u64, *word);
        }
    }

    fn get_trb(&self, addr: u64) -> [u32; 4] {
        [
            self.mem_r(addr),
            self.mem_r(addr + 4),
            self.mem_r(addr + 8),
            self.mem_r(addr + 12),
        ]
    }

    /// Bring the controller up the way §4.2's initialisation sequence does.
    fn init(&self) {
        // The Device Context Base Address Array: entry 0 is the scratchpad
        // pointer (none), entry 1 is slot 1's Device Context.
        self.mem_w(DCBAA, 0);
        self.mem_w(DCBAA + 4, 0);
        self.mem_w(DCBAA + 8, DEV_CTX as u32);
        self.mem_w(DCBAA + 12, 0);
        // One Event Ring Segment Table entry (§6.5).
        self.mem_w(ERST, EVT_RING as u32);
        self.mem_w(ERST + 4, 0);
        self.mem_w(ERST + 8, EVT_TRBS);
        self.mem_w(ERST + 12, 0);

        self.wr(CONFIG, 4);
        self.wr(DCBAAP, DCBAA as u32);
        self.wr(DCBAAP + 4, 0);
        // §5.4.5: the Ring Cycle State software starts the command ring with.
        self.wr(CRCR, CMD_RING as u32 | 1);
        self.wr(CRCR + 4, 0);
        self.wr(ERSTSZ, 1);
        self.wr(ERDP, EVT_RING as u32);
        self.wr(ERDP + 4, 0);
        self.wr(ERSTBA, ERST as u32);
        self.wr(ERSTBA + 4, 0);
        // §5.5.2.2: zero disables throttling, so an event interrupts at once.
        self.wr(IMOD, 0);
        self.wr(IMAN, IMAN_IE);
        self.wr(USBCMD, CMD_RS | CMD_INTE);
    }

    /// Clear the attach the port already reports, then reset it — which is what
    /// enables a USB2 port (§5.4.8, `PR`).
    fn reset_port(&self) {
        self.wr(PORTSC1, PORT_PP | PORT_CSC);
        self.wr(PORTSC1, PORT_PP | PORT_PR);
    }

    /// Put `trb` on the command ring and ring doorbell 0.
    fn command(&self, mut trb: [u32; 4]) {
        let at = self.cmd_next.get();
        trb[3] = (trb[3] & !TRB_CYCLE) | u32::from(self.cmd_cycle.get());
        self.put_trb(at, trb);
        self.cmd_next.set(at + TRB_SIZE);
        self.wr(DB, 0);
    }

    /// Take the next event off the ring, if there is one.
    fn event(&self) -> Option<[u32; 4]> {
        let at = self.evt_next.get();
        let trb = self.get_trb(at);
        if (trb[3] & TRB_CYCLE != 0) != self.evt_cycle.get() {
            return None;
        }
        let mut next = at + TRB_SIZE;
        if next >= EVT_RING + u64::from(EVT_TRBS) * TRB_SIZE {
            next = EVT_RING;
            self.evt_cycle.set(!self.evt_cycle.get());
        }
        self.evt_next.set(next);
        Some(trb)
    }

    /// Acknowledge everything on the ring, in the order §4.17 fixes:
    /// `USBSTS.EINT`, then `ERDP` with `EHB`, then `IMAN.IP`.
    fn drain(&self) -> alloc::vec::Vec<[u32; 4]> {
        let mut out = alloc::vec::Vec::new();
        while let Some(trb) = self.event() {
            out.push(trb);
        }
        self.wr(USBSTS, STS_EINT);
        self.wr(ERDP, self.evt_next.get() as u32 | ERDP_EHB as u32);
        self.wr(ERDP + 4, 0);
        self.wr(IMAN, IMAN_IP | IMAN_IE);
        out
    }

    /// Enable a slot and address the device on port 1, leaving it Addressed
    /// with a control transfer ring on `EP0_RING`.
    fn address_device(&self) -> u8 {
        self.command([0, 0, 0, trb::ENABLE_SLOT << TRB_TYPE_SHIFT]);
        let events = self.drain();
        assert_eq!(events.len(), 1, "one Command Completion Event");
        assert_eq!(events[0][2] >> 24, code::SUCCESS);
        let slot = (events[0][3] >> 24) as u8;

        // The Input Context: an Input Control Context, a Slot Context and an
        // Endpoint 0 Context (§6.2.5).
        for word in 0..(3 * CONTEXT_SIZE / 4) {
            self.mem_w(IN_CTX + word * 4, 0);
        }
        // A0 | A1 (§4.6.5).
        self.mem_w(IN_CTX + 4, 0x3);
        // Root Hub Port Number 1, Context Entries 1, speed High.
        self.mem_w(IN_CTX + CONTEXT_SIZE, (1 << SLOT_ENTRIES_SHIFT) | (3 << 20));
        self.mem_w(IN_CTX + CONTEXT_SIZE + 4, 1 << SLOT_PORT_SHIFT);
        // Endpoint 0: control, CErr = 3, Max Packet Size 64.
        self.mem_w(
            IN_CTX + 2 * CONTEXT_SIZE + 4,
            (3 << 1) | (EP_TYPE_CONTROL << EP_TYPE_SHIFT) | (64 << EP_MPS_SHIFT),
        );
        self.mem_w(IN_CTX + 2 * CONTEXT_SIZE + 8, EP0_RING as u32 | EP_DCS);

        self.command([
            IN_CTX as u32,
            0,
            0,
            (trb::ADDRESS_DEVICE << TRB_TYPE_SHIFT) | (u32::from(slot) << 24),
        ]);
        let events = self.drain();
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0][2] >> 24,
            code::SUCCESS,
            "Address Device did not succeed"
        );
        slot
    }

    /// Add the two bulk endpoints with a Configure Endpoint Command.
    fn configure_endpoints(&self, slot: u8) {
        for word in 0..(6 * CONTEXT_SIZE / 4) {
            self.mem_w(IN_CTX + word * 4, 0);
        }
        // A0, plus the two bulk endpoints' Device Context Indices.
        self.mem_w(IN_CTX + 4, 1 | (1 << DCI_IN) | (1 << DCI_OUT));
        self.mem_w(IN_CTX + CONTEXT_SIZE, DCI_OUT << SLOT_ENTRIES_SHIFT);
        self.mem_w(IN_CTX + CONTEXT_SIZE + 4, 1 << SLOT_PORT_SHIFT);
        // Bulk In (EP Type 6) at Input Context Index DCI_IN + 1.
        let in_ctx = IN_CTX + u64::from(DCI_IN + 1) * CONTEXT_SIZE;
        self.mem_w(
            in_ctx + 4,
            (3 << 1) | (6 << EP_TYPE_SHIFT) | (MPS << EP_MPS_SHIFT),
        );
        self.mem_w(in_ctx + 8, EPIN_RING as u32 | EP_DCS);
        // Bulk Out (EP Type 2).
        let out_ctx = IN_CTX + u64::from(DCI_OUT + 1) * CONTEXT_SIZE;
        self.mem_w(
            out_ctx + 4,
            (3 << 1) | (2 << EP_TYPE_SHIFT) | (MPS << EP_MPS_SHIFT),
        );
        self.mem_w(out_ctx + 8, EPOUT_RING as u32 | EP_DCS);

        self.command([
            IN_CTX as u32,
            0,
            0,
            (trb::CONFIGURE_ENDPOINT << TRB_TYPE_SHIFT) | (u32::from(slot) << 24),
        ]);
        let events = self.drain();
        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0][2] >> 24,
            code::SUCCESS,
            "Configure Endpoint failed"
        );
    }
}

/// A Normal TRB (§6.4.1.1).
fn normal(buffer: u64, len: u32, flags: u32, cycle: bool) -> [u32; 4] {
    [
        buffer as u32,
        (buffer >> 32) as u32,
        len,
        (trb::NORMAL << TRB_TYPE_SHIFT) | flags | u32::from(cycle),
    ]
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

#[test]
fn the_capability_registers_say_where_everything_else_is() {
    let f = build();
    // §5.3.1, §5.3.2: `CAPLENGTH` is a byte and `HCIVERSION` a halfword, and a
    // driver reads them at those widths.
    let mut byte = [0u8; 1];
    f.ops.read(0, &mut byte, MemAttrs::DEFAULT).expect("a byte");
    assert_eq!(byte[0], CAPLENGTH);
    let mut half = [0u8; 2];
    f.ops
        .read(2, &mut half, MemAttrs::DEFAULT)
        .expect("a halfword");
    assert_eq!(u16::from_le_bytes(half), HCIVERSION);

    // §5.3.3: MaxSlots, MaxIntrs and MaxPorts.
    let hcs1 = f.rd(0x04);
    assert_eq!(hcs1 & 0xff, 4, "MaxSlots");
    assert_eq!((hcs1 >> 8) & 0x7ff, 1, "MaxIntrs");
    assert_eq!(hcs1 >> 24, 1, "MaxPorts");

    // §5.3.6: 64-bit pointers, 32-byte contexts, and an extended capability
    // list at a dword offset.
    let hcc1 = f.rd(0x10);
    assert_eq!(hcc1 & 1, 1, "AC64");
    assert_eq!(hcc1 & 0x4, 0, "CSZ must be zero: 32-byte contexts");
    assert_eq!(u64::from(hcc1 >> 16) * 4, offset::XECP);

    assert_eq!(u64::from(f.rd(0x14)), offset::DOORBELL, "DBOFF");
    assert_eq!(u64::from(f.rd(0x18)), offset::RUNTIME, "RTSOFF");
}

#[test]
fn the_extended_capability_declares_usb_2_ports() {
    let f = build();
    let dw0 = f.rd(offset::XECP);
    // §7, Table 7-2: capability ID 2 is Supported Protocol, and this is the
    // only one, so the next pointer is zero.
    assert_eq!(dw0 & 0xff, 2);
    assert_eq!((dw0 >> 8) & 0xff, 0);
    // §7.2.2 Table 7-11: `USB ` major 2 minor 0.
    assert_eq!(dw0 >> 24, 0x02, "Major Revision");
    assert_eq!((dw0 >> 16) & 0xff, 0x00, "Minor Revision");
    assert_eq!(f.rd(offset::XECP + 4), 0x2042_5355, "the name string");
    let dw2 = f.rd(offset::XECP + 8);
    assert_eq!(dw2 & 0xff, 1, "Compatible Port Offset");
    assert_eq!((dw2 >> 8) & 0xff, 1, "Compatible Port Count");
    assert_eq!(dw2 >> 28, 0, "PSIC: the default Speed ID mapping applies");
}

#[test]
fn a_debug_write_is_refused_and_a_debug_read_changes_nothing() {
    let f = build();
    f.init();
    f.reset_port();

    // Every offset in the block: a debug write must never be accepted, because
    // a doorbell has no harmless version and half the block is write-1-to-clear.
    for offset in [USBCMD, USBSTS, CRCR, PORTSC1, DB, IMAN, ERDP] {
        assert!(
            f.ops
                .write(offset, &0u32.to_le_bytes(), MemAttrs::DEBUG)
                .is_err(),
            "a debug write to {offset:#x} must be refused"
        );
    }

    // And a debug read is repeatable and does not consume the event a reset has
    // just posted: the event ring's TRB and the interrupt are both still there.
    let before = f.get_trb(EVT_RING);
    for offset in [USBSTS, PORTSC1, IMAN, ERDP, offset::RUNTIME] {
        let mut a = [0u8; 4];
        let mut b = [0u8; 4];
        f.ops.read(offset, &mut a, MemAttrs::DEBUG).expect("a read");
        f.ops.read(offset, &mut b, MemAttrs::DEBUG).expect("a read");
        assert_eq!(a, b, "a debug read of {offset:#x} had a side effect");
    }
    assert_eq!(f.get_trb(EVT_RING), before, "a debug read consumed a TRB");
    assert_eq!(f.rd(IMAN) & IMAN_IP, IMAN_IP, "a debug read cleared IP");
    assert_eq!(f.controller.xhci().irq_level(), Level::High);
}

#[test]
fn a_byte_write_to_the_register_block_is_refused() {
    let f = build();
    // §5.4.4 and §5.6 both say sub-dword writes produce undefined results, so
    // they are refused rather than guessed at.
    assert!(f.ops.write(USBCMD, &[1], MemAttrs::DEFAULT).is_err());
    assert!(f.ops.write(USBCMD, &[1, 0], MemAttrs::DEFAULT).is_err());
    // A Qword write is the convention §5.1 prefers for a 64-bit register.
    f.ops
        .write(DCBAAP, &0x1_0040u64.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a qword write");
    assert_eq!(f.rd(DCBAAP), 0x1_0040);
    assert_eq!(f.rd(DCBAAP + 4), 0);
}

// ---------------------------------------------------------------------------
// Ports
// ---------------------------------------------------------------------------

#[test]
fn resetting_a_port_enables_it_and_posts_one_event() {
    let f = build();
    f.init();

    // Before the reset the port reports an attached device that is not enabled,
    // which §4.19.1.1 calls Polling.
    let sc = f.rd(PORTSC1);
    assert_eq!(sc & PORT_CCS, PORT_CCS, "something is plugged in");
    assert_eq!(sc & PORT_PED, 0, "and it is not enabled yet");
    assert_eq!((sc >> PORT_PLS_SHIFT) & PORT_PLS_MASK, PLS_POLLING);

    f.reset_port();

    let sc = f.rd(PORTSC1);
    assert_eq!(sc & PORT_PR, 0, "the xHC clears PR itself (§5.4.8)");
    assert_eq!(
        sc & PORT_PED,
        PORT_PED,
        "a successful reset enables the port"
    );
    assert_eq!(sc & PORT_PRC, PORT_PRC, "and records the change");
    // §5.4.8: the Port Speed field is invalid until after the reset, and the
    // default Speed ID mapping of Table 7-13 makes high speed a 3.
    assert_eq!((sc >> PORT_SPEED_SHIFT) & PORT_SPEED_MASK, 3);

    // §4.19.2: one event for the rising edge of PSCEG, not one per change bit.
    let events = f.drain();
    assert_eq!(events.len(), 1, "exactly one Port Status Change Event");
    assert_eq!(
        (events[0][3] >> TRB_TYPE_SHIFT) & TRB_TYPE_MASK,
        trb::PORT_STATUS_CHANGE_EVENT
    );
    // §6.4.2.3: ports are one-based in the event.
    assert_eq!(events[0][0] >> 24, 1, "Port ID");
    assert_eq!(f.rd(USBSTS) & STS_PCD, STS_PCD, "USBSTS.PCD");
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

#[test]
fn enable_slot_allocates_the_lowest_free_slot_and_runs_out_honestly() {
    let f = build();
    f.init();
    for want in 1..=4u8 {
        f.command([0, 0, 0, trb::ENABLE_SLOT << TRB_TYPE_SHIFT]);
        let events = f.drain();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0][2] >> 24, code::SUCCESS);
        assert_eq!(events[0][3] >> 24, u32::from(want), "the lowest free slot");
    }
    // §6.4.5 code 9: one more would exceed MaxSlots.
    f.command([0, 0, 0, trb::ENABLE_SLOT << TRB_TYPE_SHIFT]);
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::NO_SLOTS_AVAILABLE);
    assert_eq!(events[0][3] >> 24, 0, "and no slot is reported");
}

#[test]
fn address_device_gives_the_device_an_address_and_writes_the_output_context() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    assert_eq!(slot, 1);

    // §6.2.2 Table 6-7: the Output Slot Context carries the address the xHC
    // selected and the state it reached.
    let dw3 = f.mem_r(DEV_CTX + 12);
    assert_eq!(dw3 & 0xff, u32::from(slot), "USB Device Address");
    assert_eq!(dw3 >> SLOT_STATE_SHIFT, SLOT_STATE_ADDRESSED);
    // …and the speed the port latched, which the xHC is the authority on.
    assert_eq!((f.mem_r(DEV_CTX) >> SLOT_SPEED_SHIFT) & 0xf, 3);
    // §6.2.3: the Endpoint 0 Context is Running.
    assert_eq!(
        f.mem_r(DEV_CTX + CONTEXT_SIZE) & EP_STATE_MASK,
        EP_STATE_RUNNING
    );

    // And the device itself answers to it, which is the only claim that
    // survives the emulator's own bookkeeping being wrong.
    assert_eq!(
        f.controller
            .xhci()
            .bus()
            .device(0)
            .expect("a device")
            .address(),
        DeviceAddress(slot)
    );
}

#[test]
fn a_command_for_a_disabled_slot_is_refused_rather_than_obeyed() {
    let f = build();
    f.init();
    f.command([
        IN_CTX as u32,
        0,
        0,
        (trb::ADDRESS_DEVICE << TRB_TYPE_SHIFT) | (3 << 24),
    ]);
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::SLOT_NOT_ENABLED);
}

#[test]
fn a_command_naming_a_slot_id_larger_than_any_slot_is_refused() {
    let f = build();
    f.init();
    // **`fuzz/fuzz_targets/usb_xhci.rs` found this**, on its first seeded run:
    // a Slot ID is eight bits of a TRB the guest wrote, and the enabled-slot
    // bitmap is a `u32`, so 32 and above used to shift past the end of it —
    // a panic in a debug build and a wrap in a release one. Every Slot ID above
    // MAX_SLOTS is now the same answer as a slot that was never enabled.
    for slot in [MAX_SLOTS as u32 + 1, 32, 200, 255] {
        f.command([
            0,
            0,
            0,
            (trb::RESET_ENDPOINT << TRB_TYPE_SHIFT) | (1 << 16) | (slot << 24),
        ]);
        let events = f.drain();
        assert_eq!(events.len(), 1, "slot {slot}");
        assert_eq!(
            events[0][2] >> 24,
            code::SLOT_NOT_ENABLED,
            "slot {slot} should be refused, not shifted"
        );
    }
    // And a doorbell for one, which reaches the bitmap by a different route.
    f.wr(DB + 4 * 255, DCI_EP0);
    assert!(f.drain().is_empty());
}

#[test]
fn an_unknown_command_trb_is_a_trb_error_with_no_slot() {
    let f = build();
    f.init();
    // §6.4.6: a TRB type that is not allowed on a command ring.
    f.command([0, 0, 0, (trb::NORMAL << TRB_TYPE_SHIFT) | (1 << 24)]);
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::TRB_ERROR);
    assert_eq!(events[0][3] >> 24, 0);
}

#[test]
fn an_endpoint_context_asking_for_streams_is_a_parameter_error() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    f.command([0, 0, 0, trb::ENABLE_SLOT << TRB_TYPE_SHIFT]);
    let slot = (f.drain()[0][3] >> 24) as u8;

    for word in 0..(3 * CONTEXT_SIZE / 4) {
        self_zero(&f, IN_CTX + word * 4);
    }
    f.mem_w(IN_CTX + 4, 0x3);
    f.mem_w(IN_CTX + CONTEXT_SIZE, 1 << SLOT_ENTRIES_SHIFT);
    f.mem_w(IN_CTX + CONTEXT_SIZE + 4, 1 << SLOT_PORT_SHIFT);
    // MaxPStreams = 1, which this controller does not implement (§6.2.3).
    f.mem_w(IN_CTX + 2 * CONTEXT_SIZE, 1 << EP_MAXPSTREAMS_SHIFT);
    f.mem_w(
        IN_CTX + 2 * CONTEXT_SIZE + 4,
        (3 << 1) | (EP_TYPE_CONTROL << EP_TYPE_SHIFT) | (64 << EP_MPS_SHIFT),
    );
    f.mem_w(IN_CTX + 2 * CONTEXT_SIZE + 8, EP0_RING as u32 | EP_DCS);
    f.command([
        IN_CTX as u32,
        0,
        0,
        (trb::ADDRESS_DEVICE << TRB_TYPE_SHIFT) | (u32::from(slot) << 24),
    ]);
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::PARAMETER_ERROR);
}

fn self_zero(f: &Fixture, addr: u64) {
    f.mem_w(addr, 0);
}

// ---------------------------------------------------------------------------
// Transfers
// ---------------------------------------------------------------------------

#[test]
fn a_control_transfer_moves_a_descriptor_out_of_the_device() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();

    // GET_DESCRIPTOR(Device), as three TDs: Setup, Data, Status (§4.11.2.2).
    let setup = SetupPacket {
        request_type: Direction::BIT,
        request: crate::bus::usb::request::GET_DESCRIPTOR,
        value: 0x0100,
        index: 0,
        length: 18,
    };
    let raw = setup.encode();
    f.put_trb(
        EP0_RING,
        [
            u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]),
            u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]),
            8,
            (trb::SETUP_STAGE << TRB_TYPE_SHIFT) | TRB_IDT | (3 << 16) | 1,
        ],
    );
    f.put_trb(
        EP0_RING + TRB_SIZE,
        [
            BUF as u32,
            0,
            18,
            (trb::DATA_STAGE << TRB_TYPE_SHIFT) | TRB_DIR | 1,
        ],
    );
    f.put_trb(
        EP0_RING + 2 * TRB_SIZE,
        [0, 0, 0, (trb::STATUS_STAGE << TRB_TYPE_SHIFT) | TRB_IOC | 1],
    );
    f.wr(DB + 4 * u64::from(slot), DCI_EP0);

    let events = f.drain();
    // §6.4.1.2: only the Status Stage TRB carries IOC, so one event.
    assert_eq!(events.len(), 1, "one Transfer Event for the whole transfer");
    assert_eq!(events[0][2] >> 24, code::SUCCESS);
    assert_eq!((events[0][3] >> 16) & 0x1f, DCI_EP0, "Endpoint ID");
    assert_eq!(events[0][3] >> 24, u32::from(slot), "Slot ID");

    let got = f.mem_bytes(BUF, 18);
    assert_eq!(got[0], 18, "bLength");
    assert_eq!(got[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(u16::from_le_bytes([got[8], got[9]]), 0x1d6b, "idVendor");
}

#[test]
fn a_set_address_on_a_transfer_ring_is_refused() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();

    // §4.6.5: "The xHC shall never forward a SET_ADDRESS request on a Default
    // Control Endpoint Transfer Ring to a USB device."
    let raw = host::set_address(DeviceAddress(9)).encode();
    f.put_trb(
        EP0_RING,
        [
            u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]),
            u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]),
            8,
            (trb::SETUP_STAGE << TRB_TYPE_SHIFT) | TRB_IDT | TRB_IOC | 1,
        ],
    );
    f.wr(DB + 4 * u64::from(slot), DCI_EP0);
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::TRB_ERROR);
    assert_eq!(
        f.controller
            .xhci()
            .bus()
            .device(0)
            .expect("a device")
            .address(),
        DeviceAddress(slot),
        "the device kept the address the command gave it"
    );
}

#[test]
fn bulk_data_moves_both_ways_and_the_residual_is_reported() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // OUT: the guest hands the device 40 bytes.
    let payload: alloc::vec::Vec<u8> = (0..40u8).map(|i| i.wrapping_mul(5) ^ 0x3c).collect();
    f.space
        .write_bytes(BUF, &payload, MemAttrs::DEBUG)
        .expect("mapped RAM");
    f.put_trb(EPOUT_RING, normal(BUF, 40, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);
    let events = f.drain();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0][2] >> 24, code::SUCCESS);
    assert_eq!(
        events[0][2] & 0xff_ffff,
        0,
        "a successful OUT has no residual"
    );
    assert_eq!(f.device.state.lock().received, payload);

    // IN: the device has 40 bytes and the guest asked for 64, so the transfer
    // ends short (§6.4.1.1 bit 2) and the residual is what did not arrive.
    f.device.state.lock().available = Some(40);
    f.put_trb(EPIN_RING, normal(BUF + 0x100, 64, TRB_IOC | TRB_ISP, true));
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    let events = f.drain();
    assert_eq!(events.len(), 1, "ISP and IOC together queue one event");
    assert_eq!(events[0][2] >> 24, code::SHORT_PACKET);
    assert_eq!(events[0][2] & 0xff_ffff, 64 - 40, "the residual");
    let got = f.mem_bytes(BUF + 0x100, 40);
    for (i, byte) in got.iter().enumerate() {
        assert_eq!(*byte, (i as u8).wrapping_mul(3).wrapping_add(0x40));
    }
}

#[test]
fn a_stall_halts_the_endpoint_and_reset_endpoint_recovers_it() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    f.device.state.lock().stall_in = true;
    f.put_trb(EPIN_RING, normal(BUF, 8, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::STALL_ERROR);

    // §6.2.3 Table 6-8: a Stall detected on the endpoint forces Running to
    // Halted, and software has to issue a Reset Endpoint Command.
    let ep = DEV_CTX + u64::from(DCI_IN) * CONTEXT_SIZE;
    assert_eq!(f.mem_r(ep) & EP_STATE_MASK, EP_STATE_HALTED);

    // A doorbell on a halted endpoint does nothing at all.
    f.put_trb(EPIN_RING + TRB_SIZE, normal(BUF, 8, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    assert!(
        f.drain().is_empty(),
        "a halted endpoint ignores its doorbell"
    );

    f.device.state.lock().stall_in = false;
    f.device.state.lock().available = Some(8);
    f.command([
        0,
        0,
        0,
        (trb::RESET_ENDPOINT << TRB_TYPE_SHIFT) | (DCI_IN << 16) | (u32::from(slot) << 24),
    ]);
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::SUCCESS);
    assert_eq!(f.mem_r(ep) & EP_STATE_MASK, EP_STATE_STOPPED);

    // Stopped, so software may move the ring — and then a Configure Endpoint
    // puts it back to Running.
    f.configure_endpoints(slot);
    f.command([
        (EPIN_RING + TRB_SIZE) as u32 | 1,
        0,
        0,
        (trb::SET_TR_DEQUEUE << TRB_TYPE_SHIFT) | (DCI_IN << 16) | (u32::from(slot) << 24),
    ]);
    // Configure Endpoint has already made it Running, so Set TR Dequeue is a
    // Context State Error — which is exactly what §6.4.3.9's note says.
    let events = f.drain();
    assert_eq!(events[0][2] >> 24, code::CONTEXT_STATE_ERROR);
}

#[test]
fn a_nak_leaves_the_transfer_descriptor_where_it_is_and_the_next_microframe_retries() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    f.device.state.lock().naks = 2;
    f.device.state.lock().available = Some(16);
    f.put_trb(EPIN_RING, normal(BUF, 16, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    assert!(f.drain().is_empty(), "a NAK is not an event");

    // The endpoint's dequeue pointer has not moved: the TD is still there.
    let ep = DEV_CTX + u64::from(DCI_IN) * CONTEXT_SIZE;
    assert_eq!(u64::from(f.mem_r(ep + 8) & !0xf), EPIN_RING);

    // Two microframes, two retries, and the third goes through.
    let now = f.controller.xhci().ticks();
    f.controller.xhci().advance_to(now + 3 * 8);
    let events = f.drain();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0][2] >> 24, code::SUCCESS);
    assert_eq!(f.mem_bytes(BUF, 1)[0], 0x40);
}

#[test]
fn a_link_trb_wraps_a_transfer_ring_and_toggles_the_cycle_bit() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // A four-TRB ring: three usable entries and a Link back to the start with
    // Toggle Cycle set (§6.4.4.1).
    let link = EPOUT_RING + 3 * TRB_SIZE;
    f.put_trb(
        link,
        [
            EPOUT_RING as u32,
            0,
            0,
            (trb::LINK << TRB_TYPE_SHIFT) | TRB_TC | 1,
        ],
    );
    f.space
        .write_bytes(BUF, &[0xaa; 4], MemAttrs::DEBUG)
        .expect("mapped RAM");
    // Three transfers fill the ring; the fourth has to come back round with the
    // opposite cycle bit, which is the whole protocol.
    for i in 0..3u64 {
        f.put_trb(EPOUT_RING + i * TRB_SIZE, normal(BUF, 4, TRB_IOC, true));
    }
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);
    assert_eq!(f.drain().len(), 3);
    assert_eq!(f.device.state.lock().received.len(), 12);

    // Round the link: the controller's cycle state is now false, so a TRB with
    // the bit clear is the one it will take.
    f.put_trb(EPOUT_RING, normal(BUF, 4, TRB_IOC, false));
    // …and the link itself has to be re-armed the same way.
    f.put_trb(
        link,
        [
            EPOUT_RING as u32,
            0,
            0,
            trb::LINK << TRB_TYPE_SHIFT | TRB_TC,
        ],
    );
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);
    let events = f.drain();
    assert_eq!(events.len(), 1, "the ring wrapped");
    assert_eq!(
        events[0][0], EPOUT_RING as u32,
        "and pointed at the first TRB"
    );
    assert_eq!(f.device.state.lock().received.len(), 16);
}

#[test]
fn a_ring_of_link_trbs_terminates() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // A Link TRB pointing at itself: a ring with no end. The walk is bounded by
    // MAX_LINK_HOPS, so the doorbell returns instead of spinning.
    f.put_trb(
        EPOUT_RING,
        [EPOUT_RING as u32, 0, 0, (trb::LINK << TRB_TYPE_SHIFT) | 1],
    );
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);
    assert!(f.drain().is_empty());

    // And the controller is still usable afterwards.
    f.put_trb(EPOUT_RING, normal(BUF, 0, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);
    assert_eq!(f.drain().len(), 1);
}

#[test]
fn a_data_buffer_aimed_at_the_doorbells_does_not_recurse() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // The hazard `dev-nvme` names, spelled with a TRB: a guest points a Normal
    // TRB's data buffer at this controller's own doorbell array, so the write
    // that delivers the data re-enters the engine from inside itself. Four
    // bytes of anything are a doorbell.
    f.device.state.lock().available = Some(4);
    f.put_trb(
        EPIN_RING,
        normal(REGS + DB + 4 * u64::from(slot), 4, TRB_IOC, true),
    );
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    // It returned. That is the whole assertion: iterative, not recursive.
    let events = f.drain();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0][2] >> 24, code::SUCCESS);
}

#[test]
fn a_doorbell_for_a_slot_that_was_never_enabled_does_nothing() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    // §5.4.7: a disabled Device Slot shall not respond to doorbell references.
    f.wr(DB + 4 * 3, DCI_EP0);
    assert!(f.drain().is_empty());
}

#[test]
fn a_doorbell_for_an_endpoint_that_is_not_enabled_says_so() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    // §6.4.5 code 12: the endpoint context is all zeroes, so its type is
    // Not Valid.
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    let events = f.drain();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0][2] >> 24, code::ENDPOINT_NOT_ENABLED);
}

#[test]
fn a_zero_length_transfer_never_touches_its_data_buffer_pointer() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // §4.9.1: "If a zero-length transfer is specified, the Data Buffer Pointer
    // field is ignored by the xHC" — so a guest is entitled to leave it
    // pointing at nothing at all, and a controller that read it anyway would
    // turn a legal zero-length packet into a TRB Error.
    let nowhere = 0xdead_0000u64;
    f.put_trb(EPOUT_RING, normal(nowhere, 0, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);
    let events = f.drain();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0][2] >> 24, code::SUCCESS);
    // …and it really was a transaction: the device saw a packet.
    assert_eq!(f.device.state.lock().received.len(), 0, "an empty one");

    // The same on the IN side, where the device answers zero bytes.
    f.device.state.lock().available = Some(0);
    f.put_trb(EPIN_RING, normal(nowhere, 0, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    let events = f.drain();
    assert_eq!(events.len(), 1);
    assert_eq!(events[0][2] >> 24, code::SUCCESS);
}

// ---------------------------------------------------------------------------
// The event ring and the interrupter
// ---------------------------------------------------------------------------

#[test]
fn the_interrupt_asserts_on_an_event_and_drops_only_when_ip_is_written() {
    let f = build();
    f.init();
    assert_eq!(f.controller.xhci().irq_level(), Level::Low);

    f.reset_port();
    // §4.17.2: the event set IP, and §4.17.3 says the line follows it.
    assert_eq!(f.rd(IMAN) & IMAN_IP, IMAN_IP);
    assert_eq!(f.rd(ERDP) as u64 & ERDP_EHB, ERDP_EHB, "EHB went with it");
    assert_eq!(f.controller.xhci().irq_level(), Level::High);
    assert_eq!(f.rd(USBSTS) & STS_EINT, STS_EINT);

    // Clearing EINT and advancing the dequeue pointer does *not* drop the line.
    let _ = f.event();
    f.wr(USBSTS, STS_EINT);
    f.wr(ERDP, f.evt_next.get() as u32 | ERDP_EHB as u32);
    assert_eq!(
        f.controller.xhci().irq_level(),
        Level::High,
        "only IMAN.IP drops the line (§4.17.3)"
    );

    f.wr(IMAN, IMAN_IP | IMAN_IE);
    assert_eq!(f.controller.xhci().irq_level(), Level::Low);
    assert_eq!(f.rd(USBSTS) & STS_EINT, 0);
}

#[test]
fn the_event_handler_busy_flag_coalesces_a_second_event_into_the_same_interrupt() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // Two Transfer Descriptors, one doorbell. §4.17.2: the first sets IP and
    // EHB; the second finds EHB set and does not raise a second interrupt, so
    // one trap covers both — which is what interrupt moderation is *for*.
    f.put_trb(EPOUT_RING, normal(BUF, 0, TRB_IOC, true));
    f.put_trb(EPOUT_RING + TRB_SIZE, normal(BUF, 0, TRB_IOC, true));
    assert_eq!(f.controller.xhci().irq_level(), Level::Low);
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);
    assert_eq!(f.controller.xhci().irq_level(), Level::High);

    let events = f.drain();
    assert_eq!(events.len(), 2, "two events");
    assert_eq!(f.controller.xhci().irq_level(), Level::Low, "one interrupt");
}

#[test]
fn the_event_ring_wraps_and_the_producer_cycle_state_flips() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // Fill the ring more than once, draining as we go so it never goes full.
    // §4.9.4: the Producer Cycle State toggles every time the enqueue pointer
    // wraps to the beginning, and the test's own Consumer Cycle State has to
    // follow it or `event()` stops seeing anything.
    for round in 0..(EVT_TRBS * 2) {
        f.put_trb(EPOUT_RING, normal(BUF, 0, TRB_IOC, round % 2 == 0));
        f.put_trb(
            EPOUT_RING + TRB_SIZE,
            [
                EPOUT_RING as u32,
                0,
                0,
                (trb::LINK << TRB_TYPE_SHIFT) | TRB_TC | u32::from(round % 2 == 0),
            ],
        );
        f.wr(DB + 4 * u64::from(slot), DCI_OUT);
        let events = f.drain();
        assert_eq!(events.len(), 1, "round {round}");
        assert_eq!(events[0][2] >> 24, code::SUCCESS);
    }
    // It really did go round: the dequeue pointer is back inside the segment
    // and the cycle state has flipped twice.
    assert!(f.evt_next.get() >= EVT_RING);
    assert!(f.evt_next.get() < EVT_RING + u64::from(EVT_TRBS) * TRB_SIZE);
}

#[test]
fn a_full_event_ring_reports_itself_and_stops_the_rings() {
    let f = build();
    f.init();
    f.reset_port();
    // Deliberately do not drain: the port event stays on the ring.
    let slot = {
        f.command([0, 0, 0, trb::ENABLE_SLOT << TRB_TYPE_SHIFT]);
        // Fill the ring with No Op commands until it stops answering.
        for _ in 0..(EVT_TRBS + 8) {
            f.command([0, 0, 0, trb::NO_OP_COMMAND << TRB_TYPE_SHIFT]);
        }
        1u8
    };
    let _ = slot;

    // §4.9.4 step 13b: the last entry holds a Host Controller Event with the
    // Event Ring Full Error completion code, and the controller has stopped.
    let mut codes = alloc::vec::Vec::new();
    while let Some(trb) = f.event() {
        codes.push(((trb[3] >> TRB_TYPE_SHIFT) & TRB_TYPE_MASK, trb[2] >> 24));
    }
    assert_eq!(codes.len(), EVT_TRBS as usize);
    assert_eq!(
        *codes.last().expect("a last event"),
        (trb::HOST_CONTROLLER_EVENT, code::EVENT_RING_FULL),
        "the ring says so rather than overwriting an unread event"
    );

    // §4.9.4 step 17: writing the ERDP is what lets it go again.
    f.wr(USBSTS, STS_EINT);
    f.wr(ERDP, f.evt_next.get() as u32 | ERDP_EHB as u32);
    f.wr(IMAN, IMAN_IP | IMAN_IE);
    let events = f.drain();
    assert!(
        !events.is_empty(),
        "the commands that were held resume once there is room"
    );
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

#[test]
fn a_host_controller_reset_puts_everything_back() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    assert!(f.controller.xhci().slot_enabled(slot));

    f.wr(USBCMD, CMD_HCRST);
    // §5.4.1: self-clearing, and every operational register goes back to its
    // initial value.
    assert_eq!(f.rd(USBCMD), 0);
    assert_eq!(f.rd(USBSTS) & STS_HCH, STS_HCH);
    assert_eq!(f.rd(CONFIG), 0);
    assert_eq!(f.rd(DCBAAP), 0);
    assert!(!f.controller.xhci().slot_enabled(slot));
    assert_eq!(f.controller.xhci().irq_level(), Level::Low);
}

fn snapshot(f: &Fixture) -> alloc::vec::Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("xhci", "usb.xhci").expect("a shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("xhci", "usb.xhci", STATE_VERSION).expect("a chunk");
        f.controller.save(&mut chunk).expect("it saves");
    }
    w.to_vec().expect("bytes")
}

#[test]
fn a_snapshot_taken_mid_transfer_restores_to_the_same_state() {
    let f = build();
    f.init();
    f.reset_port();
    let _ = f.drain();
    let slot = f.address_device();
    f.configure_endpoints(slot);

    // Mid-transfer in the only sense this controller has one: a NAK'd Transfer
    // Descriptor waiting for the next microframe, and an unacknowledged event
    // on the ring.
    f.device.state.lock().naks = 4;
    f.put_trb(EPIN_RING, normal(BUF, 16, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_IN);
    f.put_trb(EPOUT_RING, normal(BUF, 0, TRB_IOC, true));
    f.wr(DB + 4 * u64::from(slot), DCI_OUT);

    let first = snapshot(&f);
    let other = build();
    let reader = StateReader::new(&first).expect("we just wrote it");
    let chunk = reader
        .load("xhci", "usb.xhci", STATE_VERSION, &Migrations::new())
        .expect("it is in there");
    other
        .controller
        .load(&mut chunk.reader())
        .expect("our own snapshot loads");
    assert_eq!(snapshot(&other), first, "the state hash must be identical");
}

#[test]
fn an_untrusted_snapshot_is_rejected_rather_than_believed() {
    let f = build();
    // A chunk decoder is a parser on bytes the machine did not write: it must
    // refuse or accept, never panic, and the device must still work afterwards.
    for junk in [&b""[..], &b"\x00"[..], &[0xff; 64][..], &[0x00; 512][..]] {
        let mut r = ChunkReader::new(junk);
        let _ = f.controller.load(&mut r);
    }
    f.init();
    f.reset_port();
    assert_eq!(f.drain().len(), 1, "still usable");
}
