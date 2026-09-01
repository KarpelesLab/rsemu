//! The dwc2 controller, driven the way a driver drives it.
//!
//! Every test here goes through the **register block**: it programs a host
//! channel, pushes the bytes into that channel's FIFO window, lets a frame
//! happen, and reads the answer back out of `GRXSTSP` and the same window.
//! Nothing calls a transaction function directly, because the thing worth
//! testing is that a sequence a driver would perform is one this controller
//! answers.
//!
//! The device on the far end is declared here rather than taken from
//! [`crate::dev::usb::hid`]: this file has to pass with only `dev-usb-dwc2`
//! enabled, and CI runs `cargo test` one feature at a time.

use super::*;

use crate::bus::usb::{
    ConfigurationDescriptor, Descriptors, DeviceDescriptor, Direction, EndpointDescriptor,
    EndpointDescriptor as Ep, Function, InterfaceDescriptor, Peripheral, UsbDevice, request,
};
use alloc::vec;

use crate::core::space::RegionKind;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

/// A frame, in domain ticks. The smallest interval this model will honour, so
/// a test advances time in numbers a person can read.
const FRAME: u64 = MIN_FRAME_PHY_CLOCKS;

/// The channel every control transfer in this file uses.
const CH: u64 = 0;

/// The widget's bulk `IN` endpoint.
const EP_IN: u8 = 1;
/// Its bulk `OUT` endpoint.
const EP_OUT: u8 = 2;
/// Its interrupt `IN` endpoint.
const EP_INT: u8 = 3;
/// Every endpoint on it but the default pipe uses this packet size.
const EP_MPS: u16 = 8;

// ---------------------------------------------------------------------------
// A device to talk to
// ---------------------------------------------------------------------------

/// What the test device has been asked and told.
#[derive(Debug, Default)]
struct Log {
    /// Bytes handed to the `OUT` endpoint.
    written: Vec<u8>,
    /// A payload waiting on the bulk `IN` endpoint, if any.
    pending: Option<Vec<u8>>,
    /// A payload waiting on the interrupt `IN` endpoint, if any.
    interrupt: Option<Vec<u8>>,
    /// How many `IN` transactions have been refused.
    naks: u32,
    /// Whether the bulk endpoints should stall.
    stall: bool,
}

/// A device with two bulk endpoints, one interrupt endpoint, and descriptors.
#[derive(Debug)]
struct Widget {
    descriptors: Descriptors,
    speed: Speed,
    log: Mutex<Log>,
}

impl Widget {
    fn new(speed: Speed) -> Widget {
        let device = DeviceDescriptor {
            vendor: 0xdead,
            product: 0xbeef,
            max_packet0: speed.max_control_packet() as u8,
            ..DeviceDescriptor::default()
        };
        let mut body = Vec::new();
        body.extend_from_slice(
            &InterfaceDescriptor {
                endpoints: 3,
                class: 0xff,
                ..InterfaceDescriptor::default()
            }
            .encode(),
        );
        body.extend_from_slice(
            &Ep {
                address: EP_IN | Direction::BIT,
                attributes: TransferType::Bulk.attribute_bits(),
                max_packet: EP_MPS,
                interval: 0,
            }
            .encode(),
        );
        body.extend_from_slice(
            &EndpointDescriptor {
                address: EP_OUT,
                attributes: TransferType::Bulk.attribute_bits(),
                max_packet: EP_MPS,
                interval: 0,
            }
            .encode(),
        );
        body.extend_from_slice(
            &EndpointDescriptor {
                address: EP_INT | Direction::BIT,
                attributes: TransferType::Interrupt.attribute_bits(),
                max_packet: EP_MPS,
                interval: 1,
            }
            .encode(),
        );
        let mut descriptors = Descriptors::new().with_device(&device);
        descriptors.add_configuration(&ConfigurationDescriptor::default(), &body);
        Widget {
            descriptors,
            speed,
            log: Mutex::with_rank(LockRank::DEVICE, Log::default()),
        }
    }
}

impl Function for Widget {
    fn descriptors(&self) -> &Descriptors {
        &self.descriptors
    }

    fn speed(&self) -> Speed {
        self.speed
    }

    fn reset(&self) {
        *self.log.lock() = Log::default();
    }

    fn endpoint_in(&self, endpoint: u8, dst: &mut [u8]) -> Completion {
        let mut log = self.log.lock();
        let slot = match endpoint {
            EP_IN => {
                if log.stall {
                    return Completion::stall();
                }
                &mut log.pending
            }
            EP_INT => &mut log.interrupt,
            _ => return Completion::stall(),
        };
        let Some(payload) = slot.take() else {
            log.naks += 1;
            return Completion::nak();
        };
        let n = payload.len().min(dst.len());
        dst[..n].copy_from_slice(&payload[..n]);
        if n < payload.len() {
            *slot = Some(payload[n..].to_vec());
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

struct Fixture {
    controller: Dwc2Controller,
    bus: Arc<UsbBus>,
    widget: Arc<Widget>,
    ops: Arc<dyn MemOps>,
}

fn fixture() -> Fixture {
    fixture_with(Speed::Full, Speed::Full)
}

/// A widget of `device_speed` on the root port of a core whose transceiver tops
/// out at `phy_speed`.
fn fixture_with(phy_speed: Speed, device_speed: Speed) -> Fixture {
    let bus = Arc::new(UsbBus::new(1));
    let widget = Arc::new(Widget::new(device_speed));
    let device: Arc<dyn UsbDevice> =
        Arc::new(Peripheral::new(Arc::clone(&widget) as Arc<dyn Function>));
    bus.attach(ROOT_PORT, device).expect("an empty port");

    let controller = Dwc2Controller::with_bus(
        Arc::clone(&bus),
        Params {
            channels: 8,
            endpoints: 4,
            fifo_words: 320,
            phy_ticks: 1,
            max_speed: phy_speed,
            cid: 0x1234,
        },
    );

    let region = controller.region("").expect("the register block");
    let ops = match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        other => panic!("expected an io region, got {other:?}"),
    };
    Fixture {
        controller,
        bus,
        widget,
        ops,
    }
}

/// Where channel `channel`'s registers are.
fn hcchar(channel: u64) -> u64 {
    HCCHAR_BASE + channel * CHANNEL_STRIDE
}
fn hcint(channel: u64) -> u64 {
    hcchar(channel) + 0x08
}
fn hcintmsk(channel: u64) -> u64 {
    hcchar(channel) + 0x0c
}
fn hctsiz(channel: u64) -> u64 {
    hcchar(channel) + 0x10
}
/// Channel `channel`'s FIFO window.
fn fifo(channel: u64) -> u64 {
    FIFO_BASE + channel * FIFO_WINDOW
}

/// One `HCCHARn` value, spelled out the way a driver assembles it.
fn channel_word(address: u8, endpoint: u8, dir_in: bool, kind: TransferType, mps: u16) -> u32 {
    u32::from(mps)
        | (u32::from(endpoint) << HCCHAR_EPNUM_SHIFT)
        | if dir_in { HCCHAR_EPDIR } else { 0 }
        | (u32::from(kind.attribute_bits()) << HCCHAR_EPTYP_SHIFT)
        | (u32::from(address) << HCCHAR_DAD_SHIFT)
        | HCCHAR_CHENA
}

/// One `HCTSIZn` value.
fn size_word(bytes: u32, packets: u32, dpid: u32) -> u32 {
    bytes | (packets << TSIZ_PKTCNT_SHIFT) | (dpid << TSIZ_DPID_SHIFT)
}

impl Fixture {
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

    fn advance(&self, frames: u64) {
        let now = self.controller.core().ticks();
        self.controller.core().advance_to(now + frames * FRAME);
    }

    /// The initialisation a driver performs: size the FIFOs, power the port,
    /// reset it, and release the reset — which is the moment the port enables.
    fn bring_up(&self) {
        self.write(GAHBCFG, AHBCFG_GINTMSK);
        // `01b`: the 48 MHz clock an FS transceiver runs at.
        self.write(HCFG, 1);
        self.write(HFIR, FRAME as u32);
        self.write(GRXFSIZ, 128);
        self.write(GNPTXFSIZ, (96 << 16) | 128);
        self.write(HPTXFSIZ, (96 << 16) | 224);
        self.write(HPRT, HPRT_PPWR);
        // The port notices what is plugged into it at the next start of frame.
        self.advance(1);
        self.write(HPRT, HPRT_PPWR | HPRT_PRST);
        self.write(HPRT, HPRT_PPWR);
    }

    /// Push `bytes` into channel `channel`'s transmit FIFO, a word at a time,
    /// as a driver does.
    fn push(&self, channel: u64, bytes: &[u8]) {
        for word in bytes.chunks(4) {
            let mut full = [0u8; 4];
            full[..word.len()].copy_from_slice(word);
            self.write(fifo(channel), u32::from_le_bytes(full));
        }
    }

    /// Run the `SETUP` stage of a control transfer on channel [`CH`].
    fn setup_stage(&self, address: u8, packet: &SetupPacket) {
        self.write(hctsiz(CH), size_word(8, 1, DPID_SETUP));
        self.write(
            hcchar(CH),
            channel_word(address, 0, false, TransferType::Control, 64),
        );
        self.push(CH, &packet.encode());
        self.advance(1);
    }

    /// Run a data or status `IN` stage on channel [`CH`], returning the bytes.
    fn in_stage(&self, address: u8, want: u32) -> Vec<u8> {
        self.write(hctsiz(CH), size_word(want, 1, DPID_DATA1));
        self.write(
            hcchar(CH),
            channel_word(address, 0, true, TransferType::Control, 64),
        );
        self.advance(1);
        self.drain()
    }

    /// Run a zero-length `OUT` status stage on channel [`CH`].
    fn out_status(&self, address: u8) {
        self.write(hctsiz(CH), size_word(0, 1, DPID_DATA1));
        self.write(
            hcchar(CH),
            channel_word(address, 0, false, TransferType::Control, 64),
        );
        self.advance(1);
    }

    /// Read every data packet the receive FIFO is holding, the way an interrupt
    /// handler does: `GRXSTSP` for the announcement, then the bytes.
    fn drain(&self) -> Vec<u8> {
        let mut out = Vec::new();
        while self.read(GINTSTS) & GINT_RXFLVL != 0 {
            let status = self.read(GRXSTSP);
            let bytes = ((status >> RXSTS_BCNT_SHIFT) & 0x7ff) as usize;
            let kind = (status >> RXSTS_PKTSTS_SHIFT) & 0xf;
            let channel = u64::from(status & 0xf);
            let mut got = Vec::new();
            for _ in 0..bytes.div_ceil(4) {
                got.extend_from_slice(&self.read(fifo(channel)).to_le_bytes());
            }
            got.truncate(bytes);
            if kind == PKTSTS_IN_DATA {
                out.extend_from_slice(&got);
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// The core comes up
// ---------------------------------------------------------------------------

#[test]
fn a_core_with_no_role_forced_is_a_host_and_says_which() {
    let f = fixture();
    assert_eq!(
        f.read(GINTSTS) & GINT_CMOD,
        GINT_CMOD,
        "an A-plug is a host without being told"
    );
    assert_eq!(
        f.read(HCFG) & HCFG_FSLSS,
        HCFG_FSLSS,
        "a full-speed transceiver reports that it is one"
    );
    assert_eq!(f.read(CID), 0x1234, "the part's user ID, not the core's");
}

#[test]
fn a_high_speed_core_does_not_claim_to_be_full_speed_only() {
    let f = fixture_with(Speed::High, Speed::High);
    assert_eq!(f.read(HCFG) & HCFG_FSLSS, 0);
}

#[test]
fn the_frame_interval_out_of_reset_is_the_one_the_register_says() {
    let f = fixture();
    assert_eq!(
        f.read(HFIR) & 0xffff,
        0xea60,
        "and it is not one millisecond at 48 MHz — the driver has to write 48000"
    );
}

#[test]
fn a_core_soft_reset_self_clears_before_the_write_returns() {
    let f = fixture();
    f.write(HFIR, 4000);
    f.write(GRSTCTL, RSTCTL_CSRST);
    assert_eq!(
        f.read(GRSTCTL) & RSTCTL_CSRST,
        0,
        "a driver spins on this bit; it has to be clear by the time it looks"
    );
    assert_eq!(
        f.read(GRSTCTL) & RSTCTL_AHBIDL,
        RSTCTL_AHBIDL,
        "and the AHB master is idle, which is the other half of that spin"
    );
    assert_eq!(
        f.read(HFIR) & 0xffff,
        0xea60,
        "the reset put everything back"
    );
}

// ---------------------------------------------------------------------------
// The root port
// ---------------------------------------------------------------------------

#[test]
fn releasing_the_reset_is_what_enables_a_full_speed_port() {
    let f = fixture();
    f.bring_up();
    let hprt = f.read(HPRT);
    assert_eq!(hprt & HPRT_PCSTS, HPRT_PCSTS, "something is plugged in");
    assert_eq!(hprt & HPRT_PENA, HPRT_PENA, "and the port enabled");
    assert_eq!(hprt & HPRT_PENCHNG, HPRT_PENCHNG, "which is a change");
    assert_eq!((hprt >> HPRT_PSPD_SHIFT) & 0x3, PSPD_FULL);
    assert!(f.bus.enabled(ROOT_PORT), "and the fabric agrees");
}

#[test]
fn a_low_speed_device_enumerates_on_a_controller_an_ehci_could_not_be() {
    let f = fixture_with(Speed::Full, Speed::Low);
    // Before any reset, the line state is how the host knows: a low-speed
    // device pulls D- up, which is the K state.
    f.write(HPRT, HPRT_PPWR);
    f.advance(1);
    assert_eq!((f.read(HPRT) >> HPRT_PLSTS_SHIFT) & 0x3, 0x1);
    f.bring_up();
    let hprt = f.read(HPRT);
    assert_eq!(hprt & HPRT_PENA, HPRT_PENA);
    assert_eq!((hprt >> HPRT_PSPD_SHIFT) & 0x3, PSPD_LOW);
}

#[test]
fn a_device_faster_than_the_transceiver_leaves_the_port_disabled() {
    let f = fixture_with(Speed::Full, Speed::High);
    f.bring_up();
    let hprt = f.read(HPRT);
    assert_eq!(hprt & HPRT_PCSTS, HPRT_PCSTS, "it is still plugged in");
    assert_eq!(
        hprt & HPRT_PENA,
        0,
        "but these pins cannot signal to it, and there is no companion \
         controller to hand it to — so the port simply does not enable"
    );
    assert!(!f.bus.enabled(ROOT_PORT));
}

#[test]
fn writing_the_enable_bit_is_how_software_disables_a_port() {
    let f = fixture();
    f.bring_up();
    assert!(f.bus.enabled(ROOT_PORT));
    // The trap every dwc2 driver has a comment about: `PENA` is
    // write-1-to-clear, so a read-modify-write that keeps it set disables the
    // port it was trying to leave alone.
    f.write(HPRT, HPRT_PPWR | HPRT_PENA);
    assert_eq!(f.read(HPRT) & HPRT_PENA, 0);
    assert!(!f.bus.enabled(ROOT_PORT));
}

#[test]
fn unplugging_reports_a_disconnect() {
    let f = fixture();
    f.bring_up();
    assert!(f.bus.detach(ROOT_PORT));
    f.advance(1);
    assert_eq!(f.read(GINTSTS) & GINT_DISCINT, GINT_DISCINT);
    assert_eq!(f.read(HPRT) & HPRT_PCSTS, 0);
    assert!(!f.bus.enabled(ROOT_PORT));
}

// ---------------------------------------------------------------------------
// Transactions
// ---------------------------------------------------------------------------

#[test]
fn a_control_transfer_through_host_channels_reads_a_device_descriptor() {
    let f = fixture();
    f.bring_up();

    f.setup_stage(
        0,
        &SetupPacket {
            request_type: Direction::BIT,
            request: request::GET_DESCRIPTOR,
            value: 0x0100,
            index: 0,
            length: 18,
        },
    );
    assert_eq!(
        f.read(hcint(CH)) & HCINT_XFRC,
        HCINT_XFRC,
        "the setup stage retired"
    );
    f.write(hcint(CH), HCINT_MASK);

    let descriptor = f.in_stage(0, 18);
    assert_eq!(
        descriptor.len(),
        18,
        "bLength bytes came back through the FIFO"
    );
    assert_eq!(descriptor[0], 18);
    assert_eq!(descriptor[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(
        u16::from_le_bytes([descriptor[8], descriptor[9]]),
        0xdead,
        "idVendor"
    );
    f.out_status(0);
}

#[test]
fn set_address_takes_effect_after_its_status_stage_and_the_device_answers_there() {
    let f = fixture();
    f.bring_up();

    f.setup_stage(
        0,
        &SetupPacket {
            request_type: 0,
            request: request::SET_ADDRESS,
            value: 5,
            index: 0,
            length: 0,
        },
    );
    assert_eq!(
        f.widget_address(),
        DeviceAddress::DEFAULT,
        "the address moves when the status stage completes, not before"
    );
    f.write(hcint(CH), HCINT_MASK);

    // The status stage is still addressed to zero (USB 2.0 §9.4.6).
    f.in_stage(0, 0);
    assert_eq!(f.widget_address(), DeviceAddress(5));

    // And now the device answers at five and nowhere else.
    f.write(hcint(CH), HCINT_MASK);
    f.setup_stage(
        5,
        &SetupPacket {
            request_type: 0,
            request: request::SET_CONFIGURATION,
            value: 1,
            index: 0,
            length: 0,
        },
    );
    assert_eq!(f.read(hcint(CH)) & HCINT_XFRC, HCINT_XFRC);
}

#[test]
fn a_bulk_out_sends_the_bytes_the_guest_pushed_into_the_fifo() {
    let f = fixture();
    f.configure();

    let payload: Vec<u8> = (0u8..8).collect();
    f.write(hctsiz(1), size_word(8, 1, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_OUT, false, TransferType::Bulk, EP_MPS),
    );
    f.push(1, &payload);
    f.advance(1);

    assert_eq!(f.read(hcint(1)) & HCINT_XFRC, HCINT_XFRC);
    assert_eq!(f.widget.log.lock().written, payload);
    assert_eq!(
        f.read(hctsiz(1)) & TSIZ_XFRSIZ_MASK,
        0,
        "the core counts down what is left, which is how a driver knows"
    );
}

#[test]
fn a_channel_waits_for_the_guest_to_finish_pushing_its_packet() {
    let f = fixture();
    f.configure();

    f.write(hctsiz(1), size_word(8, 1, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_OUT, false, TransferType::Bulk, EP_MPS),
    );
    // Only half the packet is in the FIFO.
    f.push(1, &[1, 2, 3, 4]);
    f.advance(4);
    assert_eq!(f.read(hcint(1)), 0, "nothing goes out half-written");
    assert!(f.widget.log.lock().written.is_empty());

    f.push(1, &[5, 6, 7, 8]);
    f.advance(1);
    assert_eq!(f.read(hcint(1)) & HCINT_XFRC, HCINT_XFRC);
    assert_eq!(f.widget.log.lock().written, vec![1, 2, 3, 4, 5, 6, 7, 8]);
}

#[test]
fn a_bulk_in_shorter_than_the_packet_size_ends_the_transfer() {
    let f = fixture();
    f.configure();
    f.widget.log.lock().pending = Some(alloc::vec![0xaa, 0xbb, 0xcc]);

    f.write(hctsiz(1), size_word(u32::from(EP_MPS) * 2, 2, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_IN, true, TransferType::Bulk, EP_MPS),
    );
    f.advance(1);

    assert_eq!(f.drain(), vec![0xaa, 0xbb, 0xcc]);
    assert_eq!(
        f.read(hcint(1)) & HCINT_XFRC,
        HCINT_XFRC,
        "three bytes out of an eight-byte endpoint is a short packet, and a \
         short packet is the end of the transfer"
    );
}

#[test]
fn a_stall_halts_the_channel_and_the_halt_is_announced_twice_over() {
    let f = fixture();
    f.configure();
    f.widget.log.lock().stall = true;

    f.write(hctsiz(1), size_word(8, 1, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_IN, true, TransferType::Bulk, EP_MPS),
    );
    f.advance(1);

    let status = f.read(hcint(1));
    assert_eq!(status & HCINT_STALL, HCINT_STALL);
    assert_eq!(status & HCINT_CHH, HCINT_CHH, "and the channel halted");
    assert_eq!(
        f.read(hcchar(1)) & HCCHAR_CHENA,
        0,
        "the core cleared the enable itself"
    );
    let announced = f.read(GRXSTSP);
    assert_eq!(
        (announced >> RXSTS_PKTSTS_SHIFT) & 0xf,
        PKTSTS_CHANNEL_HALTED
    );
}

#[test]
fn an_idle_interrupt_endpoint_is_polled_once_a_frame_and_no_more() {
    let f = fixture();
    f.configure();

    f.write(hctsiz(2), size_word(u32::from(EP_MPS), 1, DPID_DATA0));
    f.write(
        hcchar(2),
        channel_word(5, EP_INT, true, TransferType::Interrupt, EP_MPS),
    );
    f.advance(4);
    assert_eq!(
        f.widget.log.lock().naks,
        4,
        "a service interval is a frame: four frames is four polls, however \
         much bus time the frame had left"
    );
    assert_eq!(f.read(hcint(2)) & HCINT_XFRC, 0, "and nothing completed");

    f.widget.log.lock().interrupt = Some(alloc::vec![0x11, 0x22]);
    f.advance(1);
    assert_eq!(f.drain(), vec![0x11, 0x22]);
    assert_eq!(f.read(hcint(2)) & HCINT_XFRC, HCINT_XFRC);
}

#[test]
fn a_frame_ends_even_when_every_endpoint_refuses() {
    let f = fixture();
    f.configure();

    // A bulk `IN` that will never answer, retried for as long as the frame's
    // bandwidth lasts and then not one transaction further.
    f.write(hctsiz(1), size_word(0x1000, 512, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_IN, true, TransferType::Bulk, EP_MPS),
    );
    f.advance(1);
    let one_frame = f.widget.log.lock().naks;
    assert!(
        one_frame > 0 && one_frame < MAX_TRANSACTIONS_PER_FRAME as u32,
        "a full-speed frame carries 1500 bytes, not an unbounded number of \
         retries; got {one_frame}"
    );
    f.advance(1);
    assert_eq!(
        f.widget.log.lock().naks,
        one_frame * 2,
        "and the next frame gets the same budget, not a carried-over one"
    );
}

#[test]
fn a_setup_stage_programmed_with_fewer_than_eight_bytes_is_a_transaction_error() {
    let f = fixture();
    f.bring_up();
    // Seven bytes is not a setup packet, and inventing the eighth would hand
    // the device a request nobody wrote.
    f.write(hctsiz(CH), size_word(7, 1, DPID_SETUP));
    f.write(
        hcchar(CH),
        channel_word(0, 0, false, TransferType::Control, 64),
    );
    f.push(CH, &[0; 8]);
    f.advance(1);
    let status = f.read(hcint(CH));
    assert_eq!(status & HCINT_TXERR, HCINT_TXERR);
    assert_eq!(status & HCINT_CHH, HCINT_CHH);
}

#[test]
fn a_channel_that_does_not_exist_reads_zero_and_takes_no_writes() {
    let f = fixture();
    // Eight channels, so channel 9 is not one of them.
    f.write(hcchar(9), 0xffff_ffff);
    assert_eq!(f.read(hcchar(9)), 0);
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

#[test]
fn the_interrupt_output_is_the_and_of_three_things() {
    let f = fixture();
    f.configure();
    f.write(GINTMSK, GINT_HCINT);
    f.write(hcintmsk(2), HCINT_XFRC);
    f.write(HAINTMSK, 1 << 2);

    f.widget.log.lock().interrupt = Some(alloc::vec![1, 2, 3]);
    f.write(hctsiz(2), size_word(u32::from(EP_MPS), 1, DPID_DATA0));
    f.write(
        hcchar(2),
        channel_word(5, EP_INT, true, TransferType::Interrupt, EP_MPS),
    );
    f.advance(1);

    assert_eq!(f.read(HAINT) & (1 << 2), 1 << 2);
    assert_eq!(f.read(GINTSTS) & GINT_HCINT, GINT_HCINT);
    assert_eq!(
        f.controller.core().irq_level(),
        Level::High,
        "GAHBCFG.GINTMSK is set, GINTMSK has HCINT, HAINTMSK has the channel, \
         and HCINTMSK has XFRC"
    );

    // Clearing the channel's own bit walks the whole tree back down.
    f.write(hcint(2), HCINT_MASK);
    assert_eq!(f.read(GINTSTS) & GINT_HCINT, 0);
    assert_eq!(f.controller.core().irq_level(), Level::Low);
}

#[test]
fn the_global_enable_gates_the_pin_however_loud_the_status_register_is() {
    let f = fixture();
    f.configure();
    f.write(GAHBCFG, 0);
    f.write(GINTMSK, GINT_SOF);
    f.advance(1);
    assert_eq!(f.read(GINTSTS) & GINT_SOF, GINT_SOF);
    assert_eq!(f.controller.core().irq_level(), Level::Low);
    f.write(GAHBCFG, AHBCFG_GINTMSK);
    assert_eq!(f.controller.core().irq_level(), Level::High);
}

// ---------------------------------------------------------------------------
// Roles
// ---------------------------------------------------------------------------

#[test]
fn selecting_device_mode_stops_the_host_rather_than_half_being_a_device() {
    let f = fixture();
    f.configure();
    f.write(GUSBCFG, f.read(GUSBCFG) | USBCFG_FDMOD);

    assert_eq!(f.read(GINTSTS) & GINT_CMOD, 0, "no longer a host");
    assert!(!f.bus.enabled(ROOT_PORT), "and the port went with it");
    assert_eq!(
        f.controller.core().next_event_tick(),
        None,
        "and frames stop, rather than a host schedule running with no host"
    );

    // The device register block is not modelled and reads zero, and touching it
    // from host mode is a mode mismatch — which is the register that tells a
    // driver its role change has not landed.
    f.write(GUSBCFG, f.read(GUSBCFG) & !USBCFG_FDMOD);
    f.write(GINTSTS, GINT_MMIS);
    assert_eq!(f.read(DEVICE_BASE), 0);
    assert_eq!(f.read(GINTSTS) & GINT_MMIS, GINT_MMIS);
}

// ---------------------------------------------------------------------------
// `MemAttrs::debug`
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_of_grxstsp_does_not_pop_the_fifo() {
    let f = fixture();
    f.configure();
    f.widget.log.lock().pending = Some(alloc::vec![0xde, 0xad, 0xbe, 0xef]);
    f.write(hctsiz(1), size_word(u32::from(EP_MPS), 1, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_IN, true, TransferType::Bulk, EP_MPS),
    );
    f.advance(1);

    let peeked = f.read_debug(GRXSTSP);
    assert_eq!(
        f.read_debug(GRXSTSP),
        peeked,
        "a monitor may look at the receive queue as often as it likes"
    );
    assert_eq!(
        f.read(GRXSTSR),
        peeked,
        "and it is the same word GRXSTSR shows"
    );
    // The data is still there for the guest.
    assert_eq!(f.drain(), vec![0xde, 0xad, 0xbe, 0xef]);
}

#[test]
fn a_debug_read_of_a_fifo_window_does_not_consume_the_packet() {
    let f = fixture();
    f.configure();
    f.widget.log.lock().pending = Some(alloc::vec![1, 2, 3, 4, 5, 6, 7, 8]);
    f.write(hctsiz(1), size_word(u32::from(EP_MPS), 1, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_IN, true, TransferType::Bulk, EP_MPS),
    );
    f.advance(1);

    let _ = f.read(GRXSTSP);
    let first = f.read_debug(fifo(1));
    assert_eq!(f.read_debug(fifo(1)), first, "still the same word");
    assert_eq!(f.read(fifo(1)), first, "and the guest gets it");
    assert_ne!(f.read(fifo(1)), first, "and then the next one");
}

#[test]
fn a_debug_write_is_refused_outright() {
    let f = fixture();
    for offset in [GRSTCTL, GINTSTS, HPRT, hcchar(0), hcint(0), fifo(0)] {
        assert!(
            f.ops
                .write(offset, &0xffff_ffffu32.to_le_bytes(), MemAttrs::DEBUG)
                .is_err(),
            "a debug write at {offset:#x} was accepted"
        );
    }
}

#[test]
fn a_debug_read_does_not_move_the_frame_counter() {
    let f = fixture();
    f.bring_up();
    f.advance(3);
    let before = f.read_debug(HFNUM) & 0x3fff;
    for _ in 0..10 {
        let _ = f.read_debug(HFNUM);
    }
    assert_eq!(f.read_debug(HFNUM) & 0x3fff, before);
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

#[test]
fn the_frame_counter_advances_one_frame_at_a_time() {
    let f = fixture();
    f.bring_up();
    let start = f.read(HFNUM) & 0x3fff;
    f.advance(5);
    assert_eq!((f.read(HFNUM) & 0x3fff).wrapping_sub(start) & 0x3fff, 5);
}

#[test]
fn hfir_is_what_a_frame_is_worth_in_phy_clocks() {
    let f = fixture();
    f.bring_up();
    f.write(HFIR, 4000);
    let before = f.read(HFNUM) & 0x3fff;
    let now = f.controller.core().ticks();
    f.controller.core().advance_to(now + 4000 * 3);
    assert_eq!(
        (f.read(HFNUM) & 0x3fff).wrapping_sub(before) & 0x3fff,
        3,
        "three frames of four thousand ticks, and no float anywhere"
    );
}

#[test]
fn a_reset_does_not_rewind_the_tick() {
    let f = fixture();
    f.bring_up();
    f.advance(4);
    let ticks = f.controller.core().ticks();
    assert!(ticks > 0);
    f.controller.reset(ResetKind::Cold);
    assert_eq!(
        f.controller.core().ticks(),
        ticks,
        "`Machine::reset` does not rewind clock domains, so neither may this"
    );
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_snapshot_taken_with_a_half_drained_fifo_restores_the_rest_of_it() {
    let f = fixture();
    f.configure();
    f.widget.log.lock().pending = Some(alloc::vec![1, 2, 3, 4, 5, 6, 7, 8]);
    f.write(hctsiz(1), size_word(u32::from(EP_MPS), 1, DPID_DATA0));
    f.write(
        hcchar(1),
        channel_word(5, EP_IN, true, TransferType::Bulk, EP_MPS),
    );
    f.advance(1);

    // Announce the packet and read exactly half of it, then save.
    let _ = f.read(GRXSTSP);
    let first = f.read(fifo(1));
    let saved = snapshot(&f);

    // And a channel with bytes staged but not yet sent, which is the other half
    // of "a transfer in flight is state".
    f.write(hctsiz(2), size_word(8, 1, DPID_DATA0));
    f.write(
        hcchar(2),
        channel_word(5, EP_OUT, false, TransferType::Bulk, EP_MPS),
    );
    // Whole words, because a FIFO window is a word port: half a word pushed is
    // half a word of padding, on hardware as here.
    f.push(2, &[9, 9, 9, 9]);
    let saved_two = snapshot(&f);

    let g = fixture();
    restore(&g, &saved);
    assert_eq!(
        g.read(fifo(1)),
        u32::from_le_bytes([5, 6, 7, 8]),
        "the second half of the packet came back, and so did how much of the \
         first half had already been read"
    );
    assert_eq!(first, u32::from_le_bytes([1, 2, 3, 4]));

    // The same device, taken to the same address: a snapshot restores a
    // controller, not the bus it was talking to.
    let h = fixture();
    h.configure();
    restore(&h, &saved_two);
    h.push(2, &[8, 8, 8, 8]);
    h.advance(1);
    assert_eq!(
        h.widget.log.lock().written,
        vec![9, 9, 9, 9, 8, 8, 8, 8],
        "the word that was already in the transmit FIFO was saved with it"
    );
}

#[test]
fn a_snapshot_round_trips_to_an_identical_state() {
    let f = fixture();
    f.configure();
    f.advance(7);
    let saved = snapshot(&f);
    let again = {
        let g = fixture();
        restore(&g, &saved);
        snapshot(&g)
    };
    assert_eq!(
        saved, again,
        "the encoding is not stable under a round trip"
    );
}

#[test]
fn a_truncated_snapshot_is_refused_rather_than_believed() {
    let f = fixture();
    f.configure();
    let saved = snapshot(&f);
    for cut in [4usize, 32, 64] {
        if cut >= saved.len() {
            continue;
        }
        let g = fixture();
        let mut short = saved.clone();
        short.truncate(saved.len() - cut);
        // Either the reader refuses the chunk or the loader does; what must not
        // happen is that it is believed.
        let refused = match StateReader::new(&short) {
            Ok(reader) => {
                match reader.load("dwc2", CLASS_NAME, STATE_VERSION, &Migrations::new()) {
                    Ok(chunk) => g.controller.load(&mut chunk.reader()).is_err(),
                    Err(_) => true,
                }
            }
            Err(_) => true,
        };
        assert!(refused, "a snapshot short by {cut} bytes was accepted");
    }
}

// ---------------------------------------------------------------------------
// Fixture helpers that need the tests above to have been read first
// ---------------------------------------------------------------------------

impl Fixture {
    /// The address the widget currently answers to.
    fn widget_address(&self) -> DeviceAddress {
        self.bus
            .device(ROOT_PORT)
            .expect("something is plugged in")
            .address()
    }

    /// Bring the port up and take the device all the way to *configured*, so a
    /// test about bulk transfers is about bulk transfers.
    fn configure(&self) {
        self.bring_up();
        self.setup_stage(
            0,
            &SetupPacket {
                request_type: 0,
                request: request::SET_ADDRESS,
                value: 5,
                index: 0,
                length: 0,
            },
        );
        self.write(hcint(CH), HCINT_MASK);
        self.in_stage(0, 0);
        self.write(hcint(CH), HCINT_MASK);
        self.setup_stage(
            5,
            &SetupPacket {
                request_type: 0,
                request: request::SET_CONFIGURATION,
                value: 1,
                index: 0,
                length: 0,
            },
        );
        self.write(hcint(CH), HCINT_MASK);
        self.in_stage(5, 0);
        self.write(hcint(CH), HCINT_MASK);
        assert_eq!(self.widget_address(), DeviceAddress(5));
    }
}

fn snapshot(f: &Fixture) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("dwc2", CLASS_NAME).expect("a shape");
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer
            .chunk("dwc2", CLASS_NAME, STATE_VERSION)
            .expect("a chunk");
        f.controller.save(&mut chunk).expect("it saves");
    }
    writer.to_vec().expect("it encodes")
}

fn restore(f: &Fixture, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("it decodes");
    let chunk = reader
        .load("dwc2", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    f.controller.load(&mut chunk.reader()).expect("it loads");
}
