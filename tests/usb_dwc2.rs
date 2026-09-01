//! USB, end to end, through a controller that is nothing like an EHCI.
//!
//! `tests/usb_ehci.rs` makes the claim that a guest can build queue heads and
//! transfer descriptors in its own RAM and have a controller DMA-walk them.
//! This file makes the *same* claim about the same fabric and the same device
//! model through a controller with none of that:
//!
//! > A program running on an emulated CPU programs **host channels** — an
//! > address, an endpoint, a direction, a packet size, a byte count — pushes the
//! > setup packet into a **FIFO window** a word at a time, and reads the reply
//! > back out of `GRXSTSP` and the same window. A USB device enumerates and
//! > moves bytes, and nothing in `src/bus/usb/` knows which controller it was.
//!
//! Nothing here calls into the controller. Everything it does, it does by
//! executing RV32 instructions.
//!
//! # The board says the interesting thing by what it leaves out
//!
//! The `usb.dwc2` object below has **no `space =`**. An EHCI must have one — it
//! reads its own work out of guest memory and is refused at bind time without a
//! space to read it from. This controller never issues a memory access at all:
//! every byte it moves was put into its FIFO by the CPU, one `sw` at a time.
//! That is the difference between the two controllers in one line of a machine
//! file.
//!
//! # What the firmware does
//!
//! ```text
//!   HCFG = 1, HFIR = 48000              a 48 MHz FS PHY, one frame a millisecond
//!   GRXFSIZ / GNPTXFSIZ / HPTXFSIZ      partition the 1.25 KiB of FIFO RAM
//!   HPRT |= PPWR                        power the port
//!   poll HPRT.PCSTS                     wait for the device
//!   HPRT |= PRST ; HPRT &= ~PRST        reset it; the port enables here
//!   poll HPRT.PENA
//!   channel 0: SETUP + status IN        SET_ADDRESS(3)
//!   channel 0: SETUP + IN(18) + OUT     GET_DESCRIPTOR(DEVICE), read from the FIFO
//!   channel 0: SETUP + status IN        SET_CONFIGURATION(1)
//!   store a magic word                  so the test knows
//!   channel 1: interrupt IN on ep 1     armed once and left armed: a NAK is not
//!   poll GINTSTS.RXFLVL                   a completion, and the core retries it
//!   read GRXSTSP, then the packet         on the next frame
//!   store a second magic word
//! ```

#![cfg(all(
    feature = "cpu-riscv",
    feature = "dev-usb-dwc2",
    feature = "dev-usb-hid"
))]

use rsemu::bus::usb::{DeviceAddress, Speed, buses};
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::usb::hid::HidMouse;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// The memory map, as the machine description below lays it out
// ---------------------------------------------------------------------------

/// The OTG register block.
const OTG: u32 = 0x5000_0000;

// Global registers.
const GINTSTS: i32 = 0x014;
const GRXSTSP: i32 = 0x020;
const GRXFSIZ: i32 = 0x024;
const GNPTXFSIZ: i32 = 0x028;
const GAHBCFG: i32 = 0x008;
const HPTXFSIZ: i32 = 0x100;
// Host registers.
const HCFG: i32 = 0x400;
const HFIR: i32 = 0x404;
const HPRT: i32 = 0x440;
// Channel registers, for channels 0 and 1.
const HCCHAR0: i32 = 0x500;
const HCINT0: i32 = 0x508;
const HCTSIZ0: i32 = 0x510;
const HCCHAR1: i32 = 0x520;
const HCINT1: i32 = 0x528;
const HCTSIZ1: i32 = 0x530;

/// `GINTSTS.RXFLVL`.
const RXFLVL: i32 = 1 << 4;
/// `HPRT.PCSTS`.
const PCSTS: i32 = 1 << 0;
/// `HPRT.PENA`.
const PENA: i32 = 1 << 2;
/// `HPRT.PPWR`.
const PPWR: u32 = 1 << 12;
/// `HPRT.PRST`.
const PRST: u32 = 1 << 8;
/// `HCINTn.XFRC`.
const XFRC: i32 = 1 << 0;
/// Every bit of `HCINTn`, for clearing it.
const HCINT_ALL: u32 = 0x7ff;
/// `HCCHARn.CHENA`.
const CHENA: u32 = 1 << 31;

/// Where RAM starts.
const RAM: u32 = 0x1000_0000;
/// Where the controller's reply to `GET_DESCRIPTOR` ends up.
const DESC_BUF: u32 = RAM + 0x1000;
/// …and the mouse report.
const REPORT_BUF: u32 = RAM + 0x1100;
/// The firmware's two progress flags.
const DONE1: u32 = RAM + 0x2000;
const DONE2: u32 = RAM + 0x2004;
const MAGIC1: u32 = 0x0d15_ea5e;
const MAGIC2: u32 = 0x600d_f00d;

/// The address the firmware gives the device.
const GUEST_ADDRESS: u8 = 3;
/// The mouse's interrupt endpoint, and the size of a boot report.
const REPORT_EP: u8 = 1;
const REPORT_BYTES: u32 = 3;

// ---------------------------------------------------------------------------
// Just enough RV32I to write the firmware
// ---------------------------------------------------------------------------

const ZERO: u32 = 0;
const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const A0: u32 = 10;
const A1: u32 = 11;
const A2: u32 = 12;
const A3: u32 = 13;

const OP_LUI: u32 = 0b011_0111;
const OP_JAL: u32 = 0b110_1111;
const OP_BRANCH: u32 = 0b110_0011;
const OP_LOAD: u32 = 0b000_0011;
const OP_STORE: u32 = 0b010_0011;
const OP_IMM: u32 = 0b001_0011;

fn i_type(imm: i32, rs1: u32, funct3: u32, rd: u32, opcode: u32) -> u32 {
    ((imm as u32) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
}

fn s_type(imm: i32, rs2: u32, rs1: u32, funct3: u32, opcode: u32) -> u32 {
    let imm = imm as u32;
    ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | ((imm & 0x1f) << 7) | opcode
}

fn b_type(imm: i32, rs2: u32, rs1: u32, funct3: u32, opcode: u32) -> u32 {
    let imm = imm as u32;
    (((imm >> 12) & 1) << 31)
        | (((imm >> 5) & 0x3f) << 25)
        | (rs2 << 20)
        | (rs1 << 15)
        | (funct3 << 12)
        | (((imm >> 1) & 0xf) << 8)
        | (((imm >> 11) & 1) << 7)
        | opcode
}

fn j_type(imm: i32, rd: u32, opcode: u32) -> u32 {
    let imm = imm as u32;
    (((imm >> 20) & 1) << 31)
        | (((imm >> 1) & 0x3ff) << 21)
        | (((imm >> 11) & 1) << 20)
        | (((imm >> 12) & 0xff) << 12)
        | (rd << 7)
        | opcode
}

fn lui(rd: u32, imm: u32) -> u32 {
    (imm << 12) | (rd << 7) | OP_LUI
}
fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm & 0xfff, rs1, 0b000, rd, OP_IMM)
}
fn andi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm & 0xfff, rs1, 0b111, rd, OP_IMM)
}
fn srli(rd: u32, rs1: u32, shamt: i32) -> u32 {
    i_type(shamt & 0xfff, rs1, 0b101, rd, OP_IMM)
}
fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm & 0xfff, rs1, 0b010, rd, OP_LOAD)
}
fn sw(rs2: u32, rs1: u32, imm: i32) -> u32 {
    s_type(imm & 0xfff, rs2, rs1, 0b010, OP_STORE)
}
fn beq(rs1: u32, rs2: u32, offset: i32) -> u32 {
    b_type(offset, rs2, rs1, 0b000, OP_BRANCH)
}
fn bne(rs1: u32, rs2: u32, offset: i32) -> u32 {
    b_type(offset, rs2, rs1, 0b001, OP_BRANCH)
}
fn jal(rd: u32, offset: i32) -> u32 {
    j_type(offset, rd, OP_JAL)
}

/// `li rd, value`, as the two instructions it really is.
fn li(rd: u32, value: u32) -> [u32; 2] {
    let hi = (value.wrapping_add(0x800)) >> 12;
    let lo = (value & 0xfff) as i32;
    let lo = if lo >= 0x800 { lo - 0x1000 } else { lo };
    [lui(rd, hi), addi(rd, rd, lo)]
}

// ---------------------------------------------------------------------------
// The firmware, assembled out of the sequences a driver performs
// ---------------------------------------------------------------------------

/// `sw value, offset(base)`, with the value materialised first.
fn store(code: &mut Vec<u32>, base: u32, offset: i32, value: u32) {
    code.extend_from_slice(&li(A0, value));
    code.push(sw(A0, base, offset));
}

/// Spin until every bit of `mask` at `offset(base)` is set.
fn wait_for(code: &mut Vec<u32>, base: u32, offset: i32, mask: i32) {
    let top = code.len();
    code.push(lw(A0, base, offset));
    code.push(andi(A0, A0, mask));
    let back = -(((code.len() - top) * 4) as i32);
    code.push(beq(A0, ZERO, back));
}

/// One `HCCHARn` word, assembled the way a driver assembles it.
fn channel_word(address: u8, endpoint: u8, dir_in: bool, kind: u32, mps: u32) -> u32 {
    mps | (u32::from(endpoint) << 11)
        | if dir_in { 1 << 15 } else { 0 }
        | (kind << 18)
        | (u32::from(address) << 22)
        | CHENA
}

/// One `HCTSIZn` word. `dpid` is `0` for `DATA0`, `2` for `DATA1` and `3` for
/// a `SETUP`.
fn size_word(bytes: u32, packets: u32, dpid: u32) -> u32 {
    bytes | (packets << 19) | (dpid << 29)
}

/// Disarm a channel and clear its interrupt, which is what a driver does
/// between the stages of a control transfer: leaving it armed while `HCTSIZn`
/// is rewritten would let the core start the next stage with the previous
/// stage's direction.
fn quiesce(code: &mut Vec<u32>, hcchar: i32, hcint: i32) {
    store(code, T0, hcchar, 0);
    store(code, T0, hcint, HCINT_ALL);
}

/// The `SETUP` stage of a control transfer on channel 0: program the channel,
/// arm it, then push the eight bytes into its FIFO window.
fn setup_stage(code: &mut Vec<u32>, address: u8, packet: [u8; 8]) {
    quiesce(code, HCCHAR0, HCINT0);
    store(code, T0, HCTSIZ0, size_word(8, 1, 3));
    store(code, T0, HCCHAR0, channel_word(address, 0, false, 0, 64));
    store(
        code,
        T1,
        0,
        u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]),
    );
    store(
        code,
        T1,
        0,
        u32::from_le_bytes([packet[4], packet[5], packet[6], packet[7]]),
    );
    wait_for(code, T0, HCINT0, XFRC);
}

/// A data or status `IN` stage on channel 0.
fn in_stage(code: &mut Vec<u32>, address: u8, bytes: u32) {
    quiesce(code, HCCHAR0, HCINT0);
    store(code, T0, HCTSIZ0, size_word(bytes, 1, 2));
    store(code, T0, HCCHAR0, channel_word(address, 0, true, 0, 64));
    wait_for(code, T0, HCINT0, XFRC);
}

/// A zero-length `OUT` status stage on channel 0.
fn out_status(code: &mut Vec<u32>, address: u8) {
    quiesce(code, HCCHAR0, HCINT0);
    store(code, T0, HCTSIZ0, size_word(0, 1, 2));
    store(code, T0, HCCHAR0, channel_word(address, 0, false, 0, 64));
    wait_for(code, T0, HCINT0, XFRC);
}

/// Copy the next *data* packet out of the receive FIFO into `dst`.
///
/// The loop skips announcements that carry no bytes — a transfer-complete or a
/// channel-halted entry — which is what an interrupt handler has to do anyway,
/// and is what makes this usable after a status stage has left one behind.
fn read_packet(code: &mut Vec<u32>, fifo: u32, dst: u32) {
    code.extend_from_slice(&li(A3, dst));
    let top = code.len();
    code.push(lw(A0, T0, GINTSTS));
    code.push(andi(A0, A0, RXFLVL));
    let back = -(((code.len() - top) * 4) as i32);
    code.push(beq(A0, ZERO, back));
    code.push(lw(A0, T0, GRXSTSP));
    // BCNT is bits 14:4.
    code.push(srli(A1, A0, 4));
    code.push(andi(A1, A1, 0x7ff));
    let back = -(((code.len() - top) * 4) as i32);
    code.push(beq(A1, ZERO, back));
    // Round the byte count up to whole words: the FIFO is word-addressed.
    code.push(addi(A1, A1, 3));
    code.push(srli(A1, A1, 2));
    let copy = code.len();
    code.push(lw(A2, fifo, 0));
    code.push(sw(A2, A3, 0));
    code.push(addi(A3, A3, 4));
    code.push(addi(A1, A1, -1));
    let back = -(((code.len() - copy) * 4) as i32);
    code.push(bne(A1, ZERO, back));
}

/// The program, assembled here so the listing in the module docs and the bytes
/// that run cannot drift apart.
fn firmware() -> Vec<u8> {
    let mut code: Vec<u32> = Vec::new();

    // Three bases, because the FIFO windows are 4 KiB apart and a store's
    // immediate is twelve bits.
    code.extend_from_slice(&li(T0, OTG));
    code.extend_from_slice(&li(T1, OTG + 0x1000));
    code.extend_from_slice(&li(T2, OTG + 0x2000));

    // The core, then the FIFO partition, then the port.
    store(&mut code, T0, GAHBCFG, 1);
    // `01b`: the 48 MHz clock a full-speed transceiver runs at.
    store(&mut code, T0, HCFG, 1);
    // One millisecond, exactly, on that clock. No float anywhere.
    store(&mut code, T0, HFIR, 48_000);
    store(&mut code, T0, GRXFSIZ, 128);
    store(&mut code, T0, GNPTXFSIZ, (96 << 16) | 128);
    store(&mut code, T0, HPTXFSIZ, (96 << 16) | 224);

    store(&mut code, T0, HPRT, PPWR);
    wait_for(&mut code, T0, HPRT, PCSTS);
    store(&mut code, T0, HPRT, PPWR | PRST);
    store(&mut code, T0, HPRT, PPWR);
    wait_for(&mut code, T0, HPRT, PENA);

    // SET_ADDRESS(3): no data stage, so a `SETUP` and a status `IN` — and the
    // status stage is still addressed to zero (USB 2.0 §9.4.6).
    setup_stage(&mut code, 0, [0x00, 0x05, GUEST_ADDRESS, 0, 0, 0, 0, 0]);
    in_stage(&mut code, 0, 0);

    // GET_DESCRIPTOR(DEVICE, 18), now at the address it was just given.
    setup_stage(
        &mut code,
        GUEST_ADDRESS,
        [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 18, 0x00],
    );
    in_stage(&mut code, GUEST_ADDRESS, 18);
    read_packet(&mut code, T1, DESC_BUF);
    out_status(&mut code, GUEST_ADDRESS);

    // SET_CONFIGURATION(1).
    setup_stage(&mut code, GUEST_ADDRESS, [0x00, 0x09, 0x01, 0, 0, 0, 0, 0]);
    in_stage(&mut code, GUEST_ADDRESS, 0);

    store(&mut code, T0, 0, 0);
    code.extend_from_slice(&li(A1, DONE1));
    code.extend_from_slice(&li(A0, MAGIC1));
    code.push(sw(A0, A1, 0));

    // The interrupt endpoint, on channel 1. Armed once and left armed: an idle
    // interrupt endpoint answers `NAK`, a `NAK` is not a completion, and the
    // core retries it on the next frame — so the firmware simply waits.
    quiesce(&mut code, HCCHAR1, HCINT1);
    store(&mut code, T0, HCTSIZ1, size_word(REPORT_BYTES, 1, 0));
    store(
        &mut code,
        T0,
        HCCHAR1,
        channel_word(GUEST_ADDRESS, REPORT_EP, true, 3, REPORT_BYTES),
    );
    read_packet(&mut code, T2, REPORT_BUF);

    code.extend_from_slice(&li(A1, DONE2));
    code.extend_from_slice(&li(A0, MAGIC2));
    code.push(sw(A0, A1, 0));

    code.push(jal(ZERO, 0));

    let mut image = Vec::new();
    for word in code {
        image.extend_from_slice(&word.to_le_bytes());
    }
    image
}

// ---------------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------------

/// A machine file carried here rather than in `machines/`, because this board
/// exists to be a test and nothing else.
const BOARD: &str = r#"
machine "usb-dwc2-test" {
  param usbbus = "usb-test"

  # Two oscillators, because a board has two. 48 MHz is what a full-speed OTG
  # transceiver runs at, and 48 MHz / 1000 frames a second is exactly 48 000
  # ticks with no remainder — which is why the controller counts frames in its
  # own domain's ticks and `HFIR` is written with that number.
  osc sysclk = 100000000 Hz
  osc usbclk = 48000000 Hz

  space mem { width = 32 }

  object cpu "cpu.riscv" {
    clock  = sysclk
    space  = mem
    engine = "interp"
    xlen   = "rv32"
    isa    = "ima"
    hartid = 0
    reset  = 0x00000000
  }

  object fw   "rom" { size = 64K, image = "firmware" }
  object dram "ram" { size = 1M }

  # No `space =`. This controller is not a bus master: every byte it moves was
  # written into its FIFO by the CPU. An EHCI in this position would be refused
  # at bind time for exactly that omission.
  object otg "usb.dwc2" {
    clock    = usbclk
    bus      = usbbus
    channels = 8
    fifo     = 320
    speed    = "full"
  }

  map mem 0x00000000 size 64K   = fw
  map mem 0x10000000 size 1M    = dram
  map mem 0x50000000 size 256K  = otg
}
"#;

/// Build the board with a **full-speed** mouse already plugged into its bus.
///
/// Full speed because this controller's transceiver is: an OTG_FS has no
/// high-speed PHY, and a device that signals faster than the pins can is a
/// device this port refuses to enable — which the unit tests assert separately.
fn boot(bus_name: &str) -> (Machine, HidMouse) {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push((String::from("usbbus"), String::from(bus_name)));

    let bus = buses::open(&options.realize.hosts, bus_name, 1).expect("a bus of this build's");
    let mouse = HidMouse::new_detached_at_speed(0x1234, 0x5678, Speed::Full);
    bus.attach(0, mouse.device()).expect("an empty port");

    let registry = catalog::registry().expect("a registry");
    let machine =
        rsemu::machine::build("usb-dwc2-test", BOARD, &registry, &options).expect("it realizes");
    (machine, mouse)
}

/// Run until `flag` in guest RAM holds `magic`, or give up.
fn run_until(machine: &mut Machine, flag: u32, magic: u32) -> bool {
    for _ in 0..400 {
        machine
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        if peek(machine, flag) == magic {
            return true;
        }
    }
    false
}

fn peek(machine: &Machine, addr: u32) -> u32 {
    machine
        .space("mem")
        .expect("the board has one")
        .read(u64::from(addr), Width::U32, MemAttrs::DEBUG)
        .expect("mapped RAM") as u32
}

fn peek_bytes(machine: &Machine, addr: u32, len: usize) -> Vec<u8> {
    let mut out = vec![0u8; len];
    machine
        .space("mem")
        .expect("the board has one")
        .read_bytes(u64::from(addr), &mut out, MemAttrs::DEBUG)
        .expect("mapped RAM");
    out
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_enumerates_a_usb_device_through_host_channels_and_a_fifo() {
    let (mut machine, mouse) = boot("dwc2-enumerate");
    assert!(
        run_until(&mut machine, DONE1, MAGIC1),
        "the firmware never got through enumeration"
    );

    // The guest gave the device an address, and the device took it — with the
    // status stage still addressed to zero, which is the part that is easy to
    // get wrong and is the fabric's job rather than the controller's.
    assert_eq!(mouse.address(), DeviceAddress(GUEST_ADDRESS));
    assert_eq!(mouse.configuration(), 1);

    // The eighteen bytes of the device descriptor, in the buffer the *guest*
    // named, pulled out of the receive FIFO by guest code.
    let descriptor = peek_bytes(&machine, DESC_BUF, 18);
    assert_eq!(descriptor[0], 18, "bLength");
    assert_eq!(descriptor[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(
        u16::from_le_bytes([descriptor[2], descriptor[3]]),
        0x0200,
        "bcdUSB"
    );
    assert_eq!(
        descriptor[7], 64,
        "bMaxPacketSize0, which a full-speed device may choose and this one does"
    );
    assert_eq!(
        u16::from_le_bytes([descriptor[8], descriptor[9]]),
        0x1234,
        "idVendor"
    );
    assert_eq!(
        u16::from_le_bytes([descriptor[10], descriptor[11]]),
        0x5678,
        "idProduct"
    );
}

#[test]
fn a_mouse_report_reaches_guest_ram_through_an_interrupt_channel() {
    let (mut machine, mouse) = boot("dwc2-report");
    assert!(run_until(&mut machine, DONE1, MAGIC1), "enumeration");

    // Nothing has moved, so the endpoint NAKs and the firmware is still waiting
    // — the second flag must not be set.
    machine
        .run_for(GlobalTime::from_nanos(5_000_000))
        .expect("it runs");
    assert_ne!(
        peek(&machine, DONE2),
        MAGIC2,
        "an idle interrupt endpoint NAKs, and a NAK is not a completion"
    );

    mouse.motion(0x12, -0x22, 0b101);
    assert!(
        run_until(&mut machine, DONE2, MAGIC2),
        "the firmware never saw the report arrive"
    );

    let report = peek_bytes(&machine, REPORT_BUF, 3);
    assert_eq!(report[0], 0b101, "buttons one and three");
    assert_eq!(report[1] as i8, 0x12, "relative X");
    assert_eq!(report[2] as i8, -0x22, "relative Y");
}

#[test]
fn the_controller_is_where_the_machine_file_put_it_and_masters_nothing() {
    // The address is a board's, never the core's. And this board gives the
    // controller no address space at all, which is the whole difference between
    // it and the EHCI next door.
    let (machine, _mouse) = boot("dwc2-placement");
    let space = machine.space("mem").expect("the board has one");
    let hcfg = space
        .read(u64::from(OTG) + HCFG as u64, Width::U32, MemAttrs::DEBUG)
        .expect("the register block is mapped") as u32;
    assert_eq!(
        hcfg & (1 << 2),
        1 << 2,
        "HCFG.FSLSS: this transceiver is full- and low-speed only, and says so"
    );
}

#[test]
fn a_snapshot_taken_after_enumeration_restores_to_the_same_state() {
    let (mut machine, _mouse) = boot("dwc2-snapshot");
    assert!(run_until(&mut machine, DONE1, MAGIC1), "enumeration");

    let saved = machine.save().expect("it saves");
    let before = machine.state_hash().expect("a hash");

    let (mut restored, _mouse2) = boot("dwc2-snapshot");
    restored.load(&saved).expect("it loads");
    assert_eq!(
        restored.state_hash().expect("a hash"),
        before,
        "the machine did not round trip"
    );
    assert_eq!(peek_bytes(&restored, DESC_BUF, 2), vec![18, 1]);
}
