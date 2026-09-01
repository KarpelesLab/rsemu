//! The dwc2 in device mode, driven the way a gadget driver drives it.
//!
//! Every test here goes through the **register block** on the guest side and
//! through the **fabric** on the host side, and nothing calls a transaction
//! function directly on either. That is the whole claim: a host puts a token on
//! the bus, and what answers it is a register file a guest programmed.
//!
//! The host half of these tests is [`crate::bus::usb::host::ControlTransfer`],
//! which is a transfer composer and not a controller — there is no schedule
//! here, only "issue one transaction and tell me what happened", which is
//! exactly what a test harness and a `usbfs` bridge both want.

use super::*;

use alloc::sync::Arc;
use alloc::vec;

use crate::bus::usb::{ControlTransfer, Progress, UsbBus, host};
use crate::core::device::{Device, ResetKind};
use crate::core::space::{MemAttrs, MemOps, RegionKind};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::Level;

use super::super::{
    AHBCFG_GINTMSK, CHANNEL_STRIDE, CLASS_NAME, DPID_DATA1, DPID_SETUP, Dwc2Controller, FIFO_BASE,
    FIFO_WINDOW, GAHBCFG, GINT_RXFLVL, GINT_SOF, GINTMSK, GINTSTS, GNPTXFSIZ, GRXFSIZ, GRXSTSP,
    GRXSTSR, GUSBCFG, HCCHAR_BASE, HCCHAR_CHENA, HCCHAR_DAD_SHIFT, HCCHAR_EPDIR,
    HCCHAR_EPNUM_SHIFT, HCCHAR_EPTYP_SHIFT, HCFG, HCINT_MASK, HFIR, HPRT, HPRT_PENA, HPRT_PPWR,
    HPRT_PRST, HPTXFSIZ, MAX_CHANNELS, MIN_FRAME_PHY_CLOCKS, Params, ROOT_PORT, STATE_VERSION,
    TSIZ_DPID_SHIFT, TSIZ_PKTCNT_SHIFT, USBCFG_FDMOD, USBCFG_RESET_VALUE,
};
use crate::bus::usb::TransferType;

/// A frame, in domain ticks: the smallest interval this model honours, so a
/// test advances time in numbers a person can read.
const FRAME: u64 = MIN_FRAME_PHY_CLOCKS;

/// The eighteen bytes of a device descriptor, as *the guest* would build them.
/// Nothing in the emulator knows these numbers: the firmware pushes them into a
/// FIFO and the host reads what comes back.
const DESCRIPTOR: [u8; 18] = [
    18, 0x01, 0x00, 0x02, 0xff, 0x00, 0x00, 64, 0x83, 0x04, 0x40, 0x57, 0x00, 0x02, 1, 2, 3, 1,
];

// ---------------------------------------------------------------------------
// The fixture
// ---------------------------------------------------------------------------

/// One core, its bus, and the register block as an address space sees it.
///
/// Used for both roles: the loopback test at the bottom builds two of these on
/// one bus, one a host and one a device.
struct Fixture {
    controller: Dwc2Controller,
    bus: Arc<UsbBus>,
    ops: Arc<dyn MemOps>,
}

fn gadget() -> Fixture {
    gadget_with(Speed::Full)
}

fn gadget_with(max_speed: Speed) -> Fixture {
    let bus = Arc::new(UsbBus::new(1));
    let controller = Dwc2Controller::with_bus(
        Arc::clone(&bus),
        Params {
            channels: MAX_CHANNELS as u8,
            endpoints: 4,
            fifo_words: 320,
            phy_ticks: 1,
            max_speed,
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
        ops,
    }
}

/// Where endpoint `endpoint`'s `IN` registers are.
fn diep(endpoint: u64) -> u64 {
    DIEP_BASE + endpoint * EP_STRIDE
}
/// Where its `OUT` registers are.
fn doep(endpoint: u64) -> u64 {
    DOEP_BASE + endpoint * EP_STRIDE
}
/// Its FIFO access window.
fn fifo(endpoint: u64) -> u64 {
    FIFO_BASE + endpoint * FIFO_WINDOW
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

    /// Push bytes into endpoint `endpoint`'s transmit FIFO, a word at a time,
    /// as a driver does.
    fn push(&self, endpoint: u64, bytes: &[u8]) {
        for word in bytes.chunks(4) {
            let mut full = [0u8; 4];
            full[..word.len()].copy_from_slice(word);
            self.write(fifo(endpoint), u32::from_le_bytes(full));
        }
    }

    /// What a gadget driver does out of reset: force device mode, choose a
    /// speed, partition the FIFO RAM, and let go of soft disconnect — which is
    /// the moment the device appears on the bus.
    fn bring_up(&self) {
        self.write(GAHBCFG, AHBCFG_GINTMSK);
        self.write(GUSBCFG, USBCFG_RESET_VALUE | USBCFG_FDMOD);
        self.write(DCFG, DSPD_FULL_FS_PHY);
        self.write(GRXFSIZ, 128);
        self.write(GNPTXFSIZ, (64 << 16) | 128);
        self.write(DIEPTXF_BASE, (64 << 16) | 192);
        self.write(DCTL, 0);
    }

    /// What the *host* on the other end of the cable does: reset the port, then
    /// enable it. In that order — a reset disables the port on the way in.
    fn plug_in(&self) {
        assert!(
            self.bus.connected(ROOT_PORT),
            "soft connect should have put the device on the bus"
        );
        self.bus.reset_port(ROOT_PORT);
        self.bus.set_enabled(ROOT_PORT, true);
        // The firmware acknowledges the reset, as its handler would.
        self.write(GINTSTS, GINT_USBRST | GINT_ENUMDNE);
    }

    /// Arm endpoint zero's `OUT` side for one packet: a setup packet, or the
    /// status stage of a device-to-host transfer.
    fn arm_ep0_out(&self) {
        self.write(doep(0) + 0x10, (1 << 29) | (1 << 19) | 64);
        self.write(doep(0), EPCTL_EPENA | EPCTL_CNAK);
    }

    /// Arm endpoint zero's `IN` side with `bytes`: program the size, arm, and
    /// push the packet into the FIFO.
    fn arm_ep0_in(&self, bytes: &[u8]) {
        self.write(diep(0) + 0x10, (1 << 19) | bytes.len() as u32);
        self.write(diep(0), EPCTL_EPENA | EPCTL_CNAK);
        self.push(0, bytes);
    }

    /// Take one announcement out of the receive FIFO the way an interrupt
    /// handler does: `GRXSTSP`, then the bytes behind it.
    fn pop(&self) -> Option<(u32, u32, Vec<u8>)> {
        if self.read(GINTSTS) & GINT_RXFLVL == 0 {
            return None;
        }
        let status = self.read(GRXSTSP);
        let bytes = ((status >> RXSTS_BCNT_SHIFT) & 0x7ff) as usize;
        let kind = (status >> RXSTS_PKTSTS_SHIFT) & 0xf;
        let endpoint = status & 0xf;
        let mut got = Vec::new();
        for _ in 0..bytes.div_ceil(4) {
            got.extend_from_slice(&self.read(fifo(u64::from(endpoint))).to_le_bytes());
        }
        got.truncate(bytes);
        Some((kind, endpoint, got))
    }
}

fn snapshot(g: &Fixture) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("dwc2", CLASS_NAME).expect("a shape");
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer
            .chunk("dwc2", CLASS_NAME, STATE_VERSION)
            .expect("a chunk");
        g.controller.save(&mut chunk).expect("it saves");
    }
    writer.to_vec().expect("it encodes")
}

fn restore(g: &Fixture, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("it decodes");
    let chunk = reader
        .load("dwc2", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    g.controller.load(&mut chunk.reader()).expect("it loads");
}

// ---------------------------------------------------------------------------
// Getting onto the bus
// ---------------------------------------------------------------------------

#[test]
fn a_core_out_of_reset_is_soft_disconnected_and_is_not_on_the_bus() {
    let g = gadget();
    assert_ne!(
        g.read(DCTL) & DCTL_SDIS,
        0,
        "DCTL resets with soft disconnect set (RM0090), so nothing is on the bus yet"
    );
    assert!(!g.bus.connected(ROOT_PORT));
    assert!(!g.controller.core().is_attached());
}

#[test]
fn clearing_soft_disconnect_is_what_puts_the_device_on_the_bus() {
    let g = gadget();
    g.write(GUSBCFG, USBCFG_RESET_VALUE | USBCFG_FDMOD);
    assert!(
        !g.bus.connected(ROOT_PORT),
        "selecting device mode is not the same as pulling up D+"
    );
    g.write(DCTL, 0);
    assert!(g.bus.connected(ROOT_PORT), "soft connect is the pull-up");
    assert!(g.bus.any_change(), "and a host sees it as a connect");

    g.write(DCTL, DCTL_SDIS);
    assert!(!g.bus.connected(ROOT_PORT), "and setting it again unplugs");
}

#[test]
fn selecting_host_mode_takes_the_device_off_the_bus() {
    let g = gadget();
    g.bring_up();
    assert!(g.bus.connected(ROOT_PORT));
    // One core cannot be a peripheral on the same wire it is the host of.
    g.write(GUSBCFG, USBCFG_RESET_VALUE);
    assert!(!g.bus.connected(ROOT_PORT));
}

#[test]
fn a_core_reset_unplugs_the_device() {
    let g = gadget();
    g.bring_up();
    assert!(g.bus.connected(ROOT_PORT));
    g.controller.reset(ResetKind::Cold);
    assert!(
        !g.bus.connected(ROOT_PORT),
        "a reset puts `DCTL.SDIS` back, and that is a disconnect a host sees"
    );
}

#[test]
fn the_speed_is_what_dcfg_says_but_never_faster_than_the_transceiver() {
    let g = gadget_with(Speed::Full);
    g.bring_up();
    assert_eq!(g.bus.speed(ROOT_PORT), Some(Speed::Full));
    assert_eq!(
        (g.read(DSTS) >> DSTS_ENUMSPD_SHIFT) & 0x3,
        DSPD_FULL_FS_PHY,
        "an OTG_FS enumerates at full speed on its internal transceiver"
    );

    // A firmware that asks for high speed on a full-speed transceiver does not
    // get it: the pins are the pins.
    g.write(DCFG, DSPD_HIGH);
    assert_eq!(g.bus.speed(ROOT_PORT), Some(Speed::Full));

    // The same core with a high-speed PHY does.
    let h = gadget_with(Speed::High);
    h.bring_up();
    h.write(DCFG, DSPD_HIGH);
    assert_eq!(h.bus.speed(ROOT_PORT), Some(Speed::High));
}

#[test]
fn a_bus_reset_raises_usbrst_and_enumdne_and_forgets_the_address() {
    let g = gadget();
    g.bring_up();
    g.write(DCFG, DSPD_FULL_FS_PHY | (9 << DCFG_DAD_SHIFT));
    assert_eq!(g.bus.device(ROOT_PORT).expect("plugged in").address().0, 9);

    g.bus.reset_port(ROOT_PORT);
    let status = g.read(GINTSTS);
    assert_ne!(status & GINT_USBRST, 0, "GINTSTS.USBRST");
    assert_ne!(
        status & GINT_ENUMDNE,
        0,
        "GINTSTS.ENUMDNE, so DSTS.ENUMSPD is valid"
    );
    assert_eq!(
        g.bus.device(ROOT_PORT).expect("plugged in").address().0,
        0,
        "a reset returns the device to the Default state (USB 2.0 §9.1.1.3)"
    );
    assert_ne!(
        g.read(DCTL) & DCTL_SDIS,
        DCTL_SDIS,
        "a bus reset is not a disconnect: soft connect is the application's"
    );
}

#[test]
fn the_address_the_firmware_wrote_is_the_one_the_bus_routes_to() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.write(DCFG, DSPD_FULL_FS_PHY | (7 << DCFG_DAD_SHIFT));

    assert!(g.bus.find(DeviceAddress(7)).is_some());
    assert!(
        g.bus.find(DeviceAddress::DEFAULT).is_none(),
        "and it stops answering address zero, which is what `SET_ADDRESS` means"
    );
}

// ---------------------------------------------------------------------------
// The claim: the guest answers an enumeration
// ---------------------------------------------------------------------------

#[test]
fn the_guest_answers_a_get_descriptor_out_of_its_own_endpoint_fifo() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.arm_ep0_out();

    let mut xfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));

    // 1. The host puts a `SETUP` on the wire.
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 64),
        Progress::Moved
    );

    // 2. It arrives in the guest's receive FIFO as two announcements — the
    //    eight bytes, then the transaction-complete marker — and raises
    //    `DOEPINT0.STUP`, which is where a gadget driver's handler starts.
    assert_ne!(g.read(GINTSTS) & GINT_RXFLVL, 0);
    let (kind, endpoint, bytes) = g.pop().expect("the setup packet");
    assert_eq!(kind, PKTSTS_SETUP_DATA);
    assert_eq!(endpoint, 0);
    assert_eq!(
        bytes,
        vec![0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 18, 0x00],
        "the guest sees the exact eight bytes the host sent"
    );
    let (kind, _, _) = g.pop().expect("the completion marker");
    assert_eq!(kind, PKTSTS_SETUP_COMPLETE);
    assert!(g.pop().is_none(), "and nothing else");
    assert_ne!(g.read(doep(0) + 0x08) & DOEPINT_STUP, 0, "DOEPINT0.STUP");

    // 3. Before the firmware has built its reply, the `IN` is NAKed. That is
    //    the whole synchronisation story: the guest has not run yet.
    assert_eq!(xfer.step(&g.bus, DeviceAddress::DEFAULT, 64), Progress::Nak);

    // 4. The firmware builds the answer and pushes it into the FIFO.
    g.arm_ep0_in(&DESCRIPTOR);
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 64),
        Progress::Moved
    );
    assert_ne!(
        g.read(diep(0) + 0x08) & DIEPINT_XFRC,
        0,
        "DIEPINT0.XFRC: the transfer finished"
    );

    // 5. The status stage, and the transfer is done.
    g.arm_ep0_out();
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 64),
        Progress::Done
    );
    assert!(xfer.is_finished());
    assert_eq!(
        xfer.data(),
        &DESCRIPTOR,
        "the bytes the host collected are the bytes the guest built"
    );
}

#[test]
fn a_reply_longer_than_a_packet_goes_out_a_packet_at_a_time() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.arm_ep0_out();

    // Endpoint zero at eight bytes a packet — the smallest a control endpoint
    // may be — so eighteen bytes is three packets, two full and one short.
    g.write(diep(0), 0x3);
    let mut xfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 8),
        Progress::Moved
    );
    while g.pop().is_some() {}

    // **`DIEPTSIZ0.PKTCNT` is one bit**, so the default pipe cannot be
    // programmed for a multi-packet transfer the way endpoints one and up can:
    // the firmware re-arms per packet. That is a real constraint of this
    // register and the reason a gadget driver's `ep0` path looks different from
    // the rest of it.
    for chunk in DESCRIPTOR.chunks(8) {
        g.write(diep(0) + 0x10, (1 << 19) | chunk.len() as u32);
        g.write(diep(0), 0x3 | EPCTL_EPENA | EPCTL_CNAK);
        g.push(0, chunk);
        assert_eq!(
            xfer.step(&g.bus, DeviceAddress::DEFAULT, 8),
            Progress::Moved
        );
    }
    g.arm_ep0_out();
    assert_eq!(xfer.step(&g.bus, DeviceAddress::DEFAULT, 8), Progress::Done);
    assert_eq!(xfer.data(), &DESCRIPTOR);
}

#[test]
fn a_host_to_device_transfer_reaches_the_guest_through_the_receive_fifo() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.arm_ep0_out();

    let payload = [0xde, 0xad, 0xbe, 0xef, 0x55];
    let setup = SetupPacket {
        request_type: 0x40,
        request: 0x77,
        value: 0,
        index: 0,
        length: payload.len() as u16,
    };
    let mut xfer = ControlTransfer::host_to_device(setup, &payload);
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 64),
        Progress::Moved
    );
    while g.pop().is_some() {}

    // The data stage. The firmware re-arms the `OUT` side to take it.
    g.arm_ep0_out();
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 64),
        Progress::Moved
    );
    let (kind, endpoint, bytes) = g.pop().expect("the data packet");
    assert_eq!(kind, PKTSTS_OUT_DATA);
    assert_eq!(endpoint, 0);
    assert_eq!(bytes, payload);
    let (kind, _, _) = g.pop().expect("the completion");
    assert_eq!(
        kind, PKTSTS_OUT_COMPLETE,
        "a short packet ends the transfer, and the FIFO says so"
    );

    // The status stage goes the other way: a zero-length `IN`.
    g.arm_ep0_in(&[]);
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 64),
        Progress::Done
    );
}

#[test]
fn an_unarmed_out_endpoint_naks_rather_than_swallowing_the_packet() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    // Nothing armed at all.
    assert_eq!(
        g.bus.write(DeviceAddress::DEFAULT, 1, &[1, 2, 3]).status,
        Status::Nak
    );
    assert_ne!(
        g.read(doep(1) + 0x08) & DOEPINT_OTEPDIS,
        0,
        "DOEPINT.OTEPDIS: a token arrived while the endpoint was disabled"
    );
}

#[test]
fn an_in_endpoint_with_nothing_staged_naks_and_says_why() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    // Armed for eight bytes, but the guest pushed none of them.
    g.write(diep(1) + 0x10, (1 << 19) | 8);
    g.write(diep(1), 8 | EPCTL_EPENA | EPCTL_CNAK);

    let mut buf = [0u8; 8];
    assert_eq!(
        g.bus.read(DeviceAddress::DEFAULT, 1, &mut buf).status,
        Status::Nak
    );
    assert_ne!(
        g.read(diep(1) + 0x08) & DIEPINT_ITTXFE,
        0,
        "DIEPINT.ITTXFE: an IN token arrived with the transmit FIFO empty"
    );
}

#[test]
fn a_stalled_endpoint_stalls_and_the_next_setup_clears_the_condition() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.write(diep(0), EPCTL_STALL);

    let mut buf = [0u8; 8];
    assert_eq!(
        g.bus.read(DeviceAddress::DEFAULT, 0, &mut buf).status,
        Status::Stall
    );

    // §9.2.7: the stall condition on the default pipe is cleared by the next
    // setup packet, so a host that stalls out of one request can issue the
    // next one without a `CLEAR_FEATURE`.
    assert_eq!(
        g.bus
            .setup(DeviceAddress::DEFAULT, 0, host::set_configuration(1)),
        Status::Ack
    );
    assert_eq!(g.read(diep(0)) & EPCTL_STALL, 0);
}

#[test]
fn an_endpoint_past_the_configured_count_reads_zero_and_answers_nothing() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    // Four endpoints were configured, so endpoint five is not there.
    assert_eq!(g.read(diep(5)), 0);
    g.write(diep(5), EPCTL_EPENA | 64);
    assert_eq!(g.read(diep(5)), 0, "and a write to it lands nowhere");

    let mut buf = [0u8; 8];
    assert_eq!(
        g.bus.read(DeviceAddress::DEFAULT, 5, &mut buf).status,
        Status::Stall
    );
}

#[test]
fn a_host_asking_for_less_than_the_endpoint_will_send_is_a_babble() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.write(diep(1) + 0x10, (1 << 19) | 8);
    g.write(diep(1), 8 | EPCTL_EPENA | EPCTL_CNAK);
    g.push(1, &[1, 2, 3, 4, 5, 6, 7, 8]);

    let mut small = [0u8; 4];
    assert_eq!(
        g.bus.read(DeviceAddress::DEFAULT, 1, &mut small).status,
        Status::Babble,
        "USB 2.0 §8.7.4: more than the host reserved is a babble, not a short read"
    );
}

// ---------------------------------------------------------------------------
// Time, interrupts, and the debug rule
// ---------------------------------------------------------------------------

#[test]
fn a_start_of_frame_reaches_the_device_and_moves_dsts_fnsof() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    assert_eq!((g.read(DSTS) >> DSTS_FNSOF_SHIFT) & 0x3fff, 0);

    g.bus.start_of_frame(0x2a);
    assert_eq!(
        (g.read(DSTS) >> DSTS_FNSOF_SHIFT) & 0x3fff,
        0x2a,
        "the frame number of the last SOF, which is the one thing on the wire \
         that is not a transaction"
    );
    assert_ne!(g.read(GINTSTS) & GINT_SOF, 0);
}

#[test]
fn the_device_interrupt_is_doepint_then_daint_then_gintsts_then_the_pin() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    assert_eq!(g.controller.core().irq_level(), Level::Low);

    // Every gate open but `DAINTMSK`, which is the rung that is easy to forget.
    g.write(GINTMSK, GINT_OEPINT);
    g.write(DOEPMSK, DOEPINT_STUP);
    assert_eq!(
        g.controller.core().irq_level(),
        Level::Low,
        "nothing has happened yet"
    );

    assert_eq!(
        g.bus
            .setup(DeviceAddress::DEFAULT, 0, host::set_configuration(1)),
        Status::Ack
    );
    assert_ne!(g.read(doep(0) + 0x08) & DOEPINT_STUP, 0);
    assert_eq!(
        g.read(DAINT) & (1 << 16),
        1 << 16,
        "DAINT reports the OUT endpoint in its high half"
    );
    assert_eq!(
        g.controller.core().irq_level(),
        Level::Low,
        "DAINTMSK is closed, so the pin stays down"
    );

    g.write(DAINTMSK, 1 << 16);
    assert_eq!(
        g.controller.core().irq_level(),
        Level::High,
        "and now it rises"
    );

    // Clearing the endpoint's own latch walks the whole tree back down, which
    // is what a handler does on the way out.
    g.write(doep(0) + 0x08, DOEPINT_STUP);
    assert_eq!(g.read(GINTSTS) & GINT_OEPINT, 0);
    assert_eq!(g.controller.core().irq_level(), Level::Low);
}

#[test]
fn a_debug_read_of_the_device_block_changes_nothing() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.arm_ep0_out();
    assert_eq!(
        g.bus
            .setup(DeviceAddress::DEFAULT, 0, host::get_descriptor(1, 0, 18)),
        Status::Ack
    );

    // The trap this block has: `GRXSTSP` pops. A debug read must answer what
    // `GRXSTSR` would and leave the queue where it is.
    let peeked = g.read_debug(GRXSTSP);
    assert_eq!(peeked, g.read(GRXSTSR));
    assert_eq!(peeked, g.read_debug(GRXSTSP), "twice, identically");

    for offset in [DCFG, DCTL, DSTS, DAINT, diep(0), doep(0), diep(0) + 0x08] {
        assert_eq!(
            g.read_debug(offset),
            g.read_debug(offset),
            "a debug read of {offset:#x} had a side effect"
        );
    }

    let bytes = [0u8; 4];
    assert!(
        g.ops.write(DCTL, &bytes, MemAttrs::DEBUG).is_err(),
        "a debug write is refused outright: clearing `DCTL.SDIS` puts a device \
         on somebody's bus"
    );
}

#[test]
fn a_debug_peek_at_an_in_endpoint_does_not_take_the_packet() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.write(diep(1) + 0x10, (1 << 19) | 4);
    g.write(diep(1), 4 | EPCTL_EPENA | EPCTL_CNAK);
    g.push(1, &[9, 8, 7, 6]);

    let device = g.bus.device(ROOT_PORT).expect("plugged in");
    let mut a = [0u8; 4];
    let mut b = [0u8; 4];
    assert_eq!(device.peek_in(1, &mut a).len, 4);
    assert_eq!(device.peek_in(1, &mut b).len, 4);
    assert_eq!(a, b, "peeking twice must give the same bytes");
    assert_eq!(
        g.read(diep(1)) & EPCTL_EPENA,
        EPCTL_EPENA,
        "and must not retire the transfer"
    );

    let mut real = [0u8; 4];
    assert_eq!(device.transfer_in(1, &mut real).len, 4);
    assert_eq!(real, a);
    assert_eq!(g.read(diep(1)) & EPCTL_EPENA, 0, "the real one does");
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_snapshot_taken_mid_control_transfer_restores_the_staged_reply() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.arm_ep0_out();

    let mut xfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    assert_eq!(
        xfer.step(&g.bus, DeviceAddress::DEFAULT, 64),
        Progress::Moved
    );
    while g.pop().is_some() {}
    // The reply is staged in the transmit FIFO and has not gone out yet: the
    // most fragile moment there is, and the one a snapshot has to carry.
    g.arm_ep0_in(&DESCRIPTOR);

    let saved = snapshot(&g);

    let fresh = gadget();
    restore(&fresh, &saved);
    assert_eq!(
        snapshot(&fresh),
        saved,
        "the register file did not round trip"
    );
    assert!(
        fresh.bus.connected(ROOT_PORT),
        "and the fabric agrees with `DCTL.SDIS` again, without it being in the chunk"
    );

    fresh.bus.set_enabled(ROOT_PORT, true);
    assert_eq!(
        xfer.step(&fresh.bus, DeviceAddress::DEFAULT, 64),
        Progress::Moved
    );
    fresh.arm_ep0_out();
    assert_eq!(
        xfer.step(&fresh.bus, DeviceAddress::DEFAULT, 64),
        Progress::Done
    );
    assert_eq!(
        xfer.data(),
        &DESCRIPTOR,
        "the transfer finished on the other side of a snapshot"
    );
}

#[test]
fn a_truncated_device_mode_snapshot_is_refused_rather_than_believed() {
    let g = gadget();
    g.bring_up();
    g.plug_in();
    g.arm_ep0_in(&DESCRIPTOR);
    let saved = snapshot(&g);

    for cut in 1..40 {
        if cut >= saved.len() {
            break;
        }
        let short = &saved[..saved.len() - cut];
        let fresh = gadget();
        let refused = match StateReader::new(short) {
            Ok(reader) => {
                match reader.load("dwc2", CLASS_NAME, STATE_VERSION, &Migrations::new()) {
                    Ok(chunk) => fresh.controller.load(&mut chunk.reader()).is_err(),
                    Err(_) => true,
                }
            }
            Err(_) => true,
        };
        assert!(refused, "a snapshot short by {cut} bytes was accepted");
    }
}

// ---------------------------------------------------------------------------
// The FIFO is finite, and says so
// ---------------------------------------------------------------------------

#[test]
fn no_register_write_can_make_this_device_allocate() {
    let g = gadget();
    g.bring_up();
    g.plug_in();

    // The guest declares a 64-word transmit FIFO for endpoint one and then
    // pushes a megabyte into it.
    g.write(DIEPTXF_BASE, (64 << 16) | 192);
    for _ in 0..0x4_0000 {
        g.write(fifo(1), 0xa5a5_a5a5);
    }
    let staged = {
        let state = g.controller.core().state.lock();
        state.dev.din[1].tx.len()
    };
    assert!(
        staged <= 64 * 4,
        "the staging grew to {staged} bytes past a 64-word FIFO"
    );
    assert_eq!(g.read(diep(1) + 0x18), 0, "DTXFSTS reports it full");

    // …and a receive FIFO with no room NAKs rather than growing.
    g.write(GRXFSIZ, 4);
    let mut naked = false;
    for _ in 0..64 {
        g.write(doep(1) + 0x10, (1 << 19) | 64);
        g.write(doep(1), 64 | EPCTL_EPENA | EPCTL_CNAK);
        if g.bus.write(DeviceAddress::DEFAULT, 1, &[0u8; 32]).status == Status::Nak {
            naked = true;
            break;
        }
    }
    assert!(naked, "a full receive FIFO must NAK");
}

// ---------------------------------------------------------------------------
// Two of them, one bus: a host core enumerating a device core
// ---------------------------------------------------------------------------

/// Where host channel `channel`'s registers are.
fn hcchar(channel: u64) -> u64 {
    HCCHAR_BASE + channel * CHANNEL_STRIDE
}

/// One `HCCHARn` value, spelled out the way a host driver assembles it.
fn channel_word(address: u8, endpoint: u8, dir_in: bool, mps: u16) -> u32 {
    u32::from(mps)
        | (u32::from(endpoint) << HCCHAR_EPNUM_SHIFT)
        | if dir_in { HCCHAR_EPDIR } else { 0 }
        | (u32::from(TransferType::Control.attribute_bits()) << HCCHAR_EPTYP_SHIFT)
        | (u32::from(address) << HCCHAR_DAD_SHIFT)
        | HCCHAR_CHENA
}

/// One `HCTSIZn` value.
fn size_word(bytes: u32, packets: u32, dpid: u32) -> u32 {
    bytes | (packets << TSIZ_PKTCNT_SHIFT) | (dpid << TSIZ_DPID_SHIFT)
}

impl Fixture {
    fn advance(&self, frames: u64) {
        let now = self.controller.core().ticks();
        self.controller.core().advance_to(now + frames * FRAME);
    }

    /// The host driver's bring-up: partition the FIFOs, power the port, reset
    /// it, and release the reset — which is the moment the port enables.
    fn bring_up_host(&self) {
        self.write(GAHBCFG, AHBCFG_GINTMSK);
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

    /// The `SETUP` stage of a control transfer on host channel zero.
    fn host_setup(&self, address: u8, packet: &SetupPacket) {
        self.write(hcchar(0) + 0x08, HCINT_MASK);
        self.write(hcchar(0) + 0x10, size_word(8, 1, DPID_SETUP));
        self.write(hcchar(0), channel_word(address, 0, false, 64));
        self.push(0, &packet.encode());
        self.advance(1);
    }

    /// A data or status `IN` stage on host channel zero, returning the bytes
    /// the receive FIFO announced.
    fn host_in(&self, address: u8, want: u32) -> Vec<u8> {
        self.write(hcchar(0) + 0x08, HCINT_MASK);
        self.write(hcchar(0) + 0x10, size_word(want, 1, DPID_DATA1));
        self.write(hcchar(0), channel_word(address, 0, true, 64));
        self.advance(1);
        let mut out = Vec::new();
        while let Some((kind, _, bytes)) = self.pop() {
            if kind == PKTSTS_IN_DATA {
                out.extend_from_slice(&bytes);
            }
        }
        out
    }

    /// A zero-length `OUT` status stage on host channel zero.
    fn host_out_status(&self, address: u8) {
        self.write(hcchar(0) + 0x08, HCINT_MASK);
        self.write(hcchar(0) + 0x10, size_word(0, 1, DPID_DATA1));
        self.write(hcchar(0), channel_word(address, 0, false, 64));
        self.advance(1);
    }
}

/// Two cores on one bus: a host and a device, wired as a loopback cable.
fn loopback() -> (Fixture, Fixture) {
    let bus = Arc::new(UsbBus::new(1));
    let params = Params {
        channels: 8,
        endpoints: 4,
        fifo_words: 320,
        phy_ticks: 1,
        max_speed: Speed::Full,
        cid: 0x1234,
    };
    let build = |bus: &Arc<UsbBus>| {
        let controller = Dwc2Controller::with_bus(Arc::clone(bus), params);
        let region = controller.region("").expect("the register block");
        let ops = match region.kind() {
            RegionKind::Io(ops) => Arc::clone(ops),
            other => panic!("expected an io region, got {other:?}"),
        };
        Fixture {
            controller,
            bus: Arc::clone(bus),
            ops,
        }
    };
    (build(&bus), build(&bus))
}

#[test]
fn a_dwc2_host_enumerates_a_dwc2_device_over_one_bus() {
    let (host_core, device_core) = loopback();

    // The device end first: it has to be on the wire before the host's port
    // can find anything on it.
    device_core.bring_up();
    host_core.bring_up_host();
    assert_ne!(
        host_core.read(HPRT) & HPRT_PENA,
        0,
        "the host port enabled, so the two cores agree on a full-speed link"
    );
    assert_ne!(
        device_core.read(GINTSTS) & GINT_USBRST,
        0,
        "and the host's port reset reached the device as a bus reset"
    );
    device_core.write(GINTSTS, GINT_USBRST | GINT_ENUMDNE);
    device_core.arm_ep0_out();

    // GET_DESCRIPTOR(DEVICE, 18) — issued out of host channel registers,
    // answered out of device endpoint registers, and nothing in between knows
    // which is which.
    host_core.host_setup(
        0,
        &SetupPacket {
            request_type: 0x80,
            request: 6,
            value: 0x0100,
            index: 0,
            length: 18,
        },
    );
    let (kind, endpoint, bytes) = device_core.pop().expect("the setup packet");
    assert_eq!(kind, PKTSTS_SETUP_DATA);
    assert_eq!(endpoint, 0);
    assert_eq!(bytes, vec![0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 18, 0x00]);
    while device_core.pop().is_some() {}

    device_core.arm_ep0_in(&DESCRIPTOR);
    let got = host_core.host_in(0, 18);
    assert_eq!(
        got,
        DESCRIPTOR.to_vec(),
        "the eighteen bytes the device's firmware pushed into its own FIFO came \
         out of the host's"
    );

    device_core.arm_ep0_out();
    host_core.host_out_status(0);
    assert_ne!(
        device_core.read(doep(0) + 0x08) & DOEPINT_XFRC,
        0,
        "the status stage completed at the device end too"
    );

    // And the host's start-of-frame token reached the device: the one thing on
    // the wire that is not a transaction.
    let fnsof = (device_core.read(DSTS) >> DSTS_FNSOF_SHIFT) & 0x3fff;
    assert_ne!(fnsof, 0, "DSTS.FNSOF followed the host's frame counter");
}

#[test]
fn a_device_core_that_soft_disconnects_is_a_disconnect_the_host_sees() {
    let (host_core, device_core) = loopback();
    device_core.bring_up();
    host_core.bring_up_host();
    assert_ne!(host_core.read(HPRT) & HPRT_PENA, 0);

    device_core.write(DCTL, DCTL_SDIS);
    host_core.advance(1);
    assert_eq!(
        host_core.read(HPRT) & HPRT_PENA,
        0,
        "the port disables when the device stops pulling up D+"
    );
}
