//! USB the other way round: the **guest** is the device.
//!
//! `tests/usb_dwc2.rs` makes the claim that a program running on an emulated
//! CPU can drive host channels and enumerate a device the emulator provides.
//! This file makes the claim that matters for an STM32 board and for the
//! CX92755's `libusb_printer`, and it is the reverse of that one:
//!
//! > A `GET_DESCRIPTOR` arrives **at the guest**. Guest code — RV32
//! > instructions, nothing else — reads the eight setup bytes out of its own
//! > receive FIFO through `GRXSTSP`, builds a reply, pushes it into its own
//! > `IN` endpoint FIFO a word at a time, and arms `DIEPCTL0`. The host on the
//! > other end of the cable collects the eighteen bytes the guest wrote.
//!
//! Nothing here calls into the controller. Everything the *device* does, it
//! does by executing instructions.
//!
//! # What is on the other end
//!
//! [`rsemu::bus::usb::ControlTransfer`] — a host-side transfer composer, not a
//! controller. It has no schedule and no clock: each `step` issues one
//! transaction and says what happened, and a `NAK` is the caller's cue to let
//! the guest run and try again. That is exactly the shape a `usbfs` bridge or a
//! second machine's controller needs, and it is why the composer lives in
//! `bus/usb` rather than in this file.
//!
//! (`src/dev/usb/dwc2/device/tests.rs` makes the same claim with a *dwc2 host
//! core* on the far end instead, so the register-to-register path is covered
//! too. This file is the one where the device side is a guest.)
//!
//! # What the firmware does
//!
//! ```text
//!   GAHBCFG = 1                          unmask the interrupt tree
//!   GUSBCFG |= FDMOD                     force device mode
//!   DCFG = 3                             full speed, internal FS transceiver
//!   GRXFSIZ / GNPTXFSIZ                  partition the FIFO RAM
//!   DCTL = 0                             release soft disconnect: on the bus
//!   poll GINTSTS.USBRST                  the host reset the port
//!   GINTSTS = USBRST | ENUMDNE           acknowledge both
//!   DOEPTSIZ0 / DOEPCTL0                 arm the OUT side for a setup packet
//!   poll GINTSTS.RXFLVL                  something arrived
//!   GRXSTSP, then DFIFO(0)               the eight setup bytes, into RAM
//!   DIEPTSIZ0 = wLength from the request the guest just read
//!   DIEPCTL0 = EPENA | CNAK
//!   DFIFO(0) = the descriptor            five words
//!   DOEPTSIZ0 / DOEPCTL0                 re-arm for the status stage
//! ```

#![cfg(all(feature = "cpu-riscv", feature = "dev-usb-dwc2"))]

use rsemu::bus::usb::{ControlTransfer, DeviceAddress, Progress, UsbBus, buses, host};
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

use std::sync::Arc;

// ---------------------------------------------------------------------------
// The memory map, as the machine description below lays it out
// ---------------------------------------------------------------------------

/// The OTG register block.
const OTG: u32 = 0x5000_0000;
/// Endpoint zero's FIFO access window.
const FIFO0: u32 = OTG + 0x1000;
/// The device-mode register block, which a store's twelve-bit immediate cannot
/// reach from `OTG` — so the firmware keeps a second base pointer, exactly as a
/// compiler would.
const DEV: u32 = OTG + 0x800;

// Global registers, from `OTG`.
const GAHBCFG: i32 = 0x008;
const GUSBCFG: i32 = 0x00c;
const GINTSTS: i32 = 0x014;
const GRXSTSP: i32 = 0x020;
const GRXFSIZ: i32 = 0x024;
const GNPTXFSIZ: i32 = 0x028;

// Device registers, from `DEV`.
const DCFG: i32 = 0x000;
const DCTL: i32 = 0x004;
const DIEPCTL0: i32 = 0x100;
const DIEPTSIZ0: i32 = 0x110;
const DOEPCTL0: i32 = 0x300;
const DOEPTSIZ0: i32 = 0x310;

/// `GUSBCFG` with device mode forced: the reset value, plus `FDMOD`.
const GUSBCFG_DEVICE: u32 = 0x0000_0a00 | (1 << 6) | (1 << 30);
/// `DCFG.DSPD = 11b`: full speed on the internal transceiver, which is what an
/// OTG_FS is.
const DCFG_FULL_SPEED: u32 = 3;
/// `GINTSTS.RXFLVL`.
const RXFLVL_BIT: i32 = 4;
/// `GINTSTS.USBRST`.
const USBRST_BIT: i32 = 12;
/// `USBRST | ENUMDNE`, for the acknowledgement.
const RESET_ACK: u32 = (1 << 12) | (1 << 13);
/// `DOEPTSIZ0`: one setup packet, one packet, sixty-four bytes.
const DOEPTSIZ0_ARM: u32 = (1 << 29) | (1 << 19) | 64;
/// `DIEPCTLn`/`DOEPCTLn`: `EPENA | CNAK`.
const EP_ARM: u32 = (1 << 31) | (1 << 26);

/// Where RAM starts.
const RAM: u32 = 0x1000_0000;
/// Where the firmware copies the setup packet it received.
const SETUP_BUF: u32 = RAM + 0x1000;
/// The firmware's two progress flags.
const DONE1: u32 = RAM + 0x2000;
const DONE2: u32 = RAM + 0x2004;
const MAGIC1: u32 = 0x0d15_ea5e;
const MAGIC2: u32 = 0x600d_f00d;

/// The eighteen bytes of a device descriptor. **The firmware's**, not the
/// emulator's: they exist in this file only so the test knows what to expect,
/// and they reach the host as guest stores into a FIFO window.
const DESCRIPTOR: [u8; 18] = [
    18, 0x01, 0x00, 0x02, 0xff, 0x00, 0x00, 64, 0x83, 0x04, 0x40, 0x57, 0x00, 0x02, 1, 2, 3, 1,
];

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

const OP_LUI: u32 = 0b011_0111;
const OP_JAL: u32 = 0b110_1111;
const OP_BRANCH: u32 = 0b110_0011;
const OP_LOAD: u32 = 0b000_0011;
const OP_STORE: u32 = 0b010_0011;
const OP_IMM: u32 = 0b001_0011;
const OP_REG: u32 = 0b011_0011;

fn i_type(imm: i32, rs1: u32, funct3: u32, rd: u32, opcode: u32) -> u32 {
    ((imm as u32) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
}

fn r_type(rs2: u32, rs1: u32, funct3: u32, rd: u32, opcode: u32) -> u32 {
    (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
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
fn or(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r_type(rs2, rs1, 0b110, rd, OP_REG)
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
// The firmware
// ---------------------------------------------------------------------------

/// `sw value, offset(base)`, with the value materialised first.
fn store(code: &mut Vec<u32>, base: u32, offset: i32, value: u32) {
    code.extend_from_slice(&li(A0, value));
    code.push(sw(A0, base, offset));
}

/// Spin until bit `bit` of `offset(base)` is set.
///
/// One bit at a time rather than a mask, because an `andi` immediate is twelve
/// signed bits and `GINTSTS.USBRST` is bit 12 — which does not fit. A shift and
/// an `andi 1` is what a compiler emits for the same reason.
fn wait_bit(code: &mut Vec<u32>, base: u32, offset: i32, bit: i32) {
    let top = code.len();
    code.push(lw(A0, base, offset));
    code.push(srli(A0, A0, bit));
    code.push(andi(A0, A0, 1));
    let back = -(((code.len() - top) * 4) as i32);
    code.push(beq(A0, ZERO, back));
}

/// The program, assembled here so the listing in the module docs and the bytes
/// that run cannot drift apart.
fn firmware() -> Vec<u8> {
    let mut code: Vec<u32> = Vec::new();

    // Three bases: the global block, the device block, and endpoint zero's
    // FIFO window. A store's immediate is twelve bits and they are 4 KiB apart.
    code.extend_from_slice(&li(T0, OTG));
    code.extend_from_slice(&li(T1, DEV));
    code.extend_from_slice(&li(T2, FIFO0));

    // Bring the core up as a *device*.
    store(&mut code, T0, GAHBCFG, 1);
    store(&mut code, T0, GUSBCFG, GUSBCFG_DEVICE);
    store(&mut code, T1, DCFG, DCFG_FULL_SPEED);
    store(&mut code, T0, GRXFSIZ, 128);
    store(&mut code, T0, GNPTXFSIZ, (64 << 16) | 128);
    // The pull-up on D+, in one register write. Everything before this was
    // invisible to the bus.
    store(&mut code, T1, DCTL, 0);

    // The host resets the port, which is what tells a gadget the enumeration is
    // starting.
    wait_bit(&mut code, T0, GINTSTS, USBRST_BIT);
    store(&mut code, T0, GINTSTS, RESET_ACK);

    // Arm the `OUT` side of the default pipe so a setup packet has somewhere to
    // land.
    store(&mut code, T1, DOEPTSIZ0, DOEPTSIZ0_ARM);
    store(&mut code, T1, DOEPCTL0, EP_ARM);

    // Wait for a packet, and skip the announcements that carry no bytes — the
    // setup-complete marker is one, and an interrupt handler has to skip it
    // anyway.
    let poll = code.len();
    wait_bit(&mut code, T0, GINTSTS, RXFLVL_BIT);
    code.push(lw(A0, T0, GRXSTSP));
    code.push(srli(A1, A0, 4));
    code.push(andi(A1, A1, 0x7ff));
    let back = -(((code.len() - poll) * 4) as i32);
    code.push(beq(A1, ZERO, back));

    // Eight bytes, two words, straight into RAM.
    code.extend_from_slice(&li(A2, SETUP_BUF));
    code.push(lw(A0, T2, 0));
    code.push(sw(A0, A2, 0));
    code.push(lw(A0, T2, 0));
    code.push(sw(A0, A2, 4));

    code.extend_from_slice(&li(A1, DONE1));
    code.extend_from_slice(&li(A0, MAGIC1));
    code.push(sw(A0, A1, 0));

    // The reply. Its length comes out of the request the guest just read —
    // `wLength` is the high half of the word at `SETUP_BUF + 4` — so this is an
    // answer to what was asked rather than a canned one.
    code.extend_from_slice(&li(A2, SETUP_BUF));
    code.push(lw(A0, A2, 4));
    code.push(srli(A0, A0, 16));
    code.extend_from_slice(&li(A1, 1 << 19));
    code.push(or(A1, A1, A0));
    code.push(sw(A1, T1, DIEPTSIZ0));
    store(&mut code, T1, DIEPCTL0, EP_ARM);

    // …and the bytes, a word at a time, into endpoint zero's FIFO window.
    for word in DESCRIPTOR.chunks(4) {
        let mut full = [0u8; 4];
        full[..word.len()].copy_from_slice(word);
        store(&mut code, T2, 0, u32::from_le_bytes(full));
    }

    // Re-arm the `OUT` side for the status stage that ends the transfer.
    store(&mut code, T1, DOEPTSIZ0, DOEPTSIZ0_ARM);
    store(&mut code, T1, DOEPCTL0, EP_ARM);

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
///
/// The interesting line is `port = 0`: this controller is not the *root* of its
/// bus, it is the thing plugged into it. Whatever enumerates it is elsewhere.
const BOARD: &str = r#"
machine "usb-dwc2-device-test" {
  param usbbus = "usb-device-test"

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

  object otg "usb.dwc2" {
    clock     = usbclk
    bus       = usbbus
    endpoints = 4
    fifo      = 320
    speed     = "full"
    port      = 0
  }

  map mem 0x00000000 size 64K   = fw
  map mem 0x10000000 size 1M    = dram
  map mem 0x50000000 size 256K  = otg
}
"#;

fn boot(bus_name: &str) -> (Machine, Arc<UsbBus>) {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push((String::from("usbbus"), String::from(bus_name)));

    let bus = buses::open(&options.realize.hosts, bus_name, 1).expect("a bus of this build's");
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build("usb-dwc2-device-test", BOARD, &registry, &options)
        .expect("it realizes");
    (machine, bus)
}

fn tick(machine: &mut Machine) {
    machine
        .run_for(GlobalTime::from_nanos(200_000))
        .expect("it runs");
}

/// Run until the guest's firmware pulls up D+, then do what a host does: reset
/// the port and enable it.
fn plug_in(machine: &mut Machine, bus: &UsbBus) {
    for _ in 0..400 {
        tick(machine);
        if bus.connected(0) {
            bus.reset_port(0);
            bus.set_enabled(0, true);
            return;
        }
    }
    panic!("the guest never put its device on the bus");
}

/// Step `xfer` to completion, letting the guest run between transactions —
/// which is what a `NAK` means on this side of the seam.
fn drive(machine: &mut Machine, bus: &UsbBus, xfer: &mut ControlTransfer) {
    for _ in 0..400 {
        tick(machine);
        if xfer.step(bus, DeviceAddress::DEFAULT, 64).is_finished() {
            return;
        }
    }
    panic!("the transfer never finished: {xfer:?}");
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
fn the_guest_answers_a_get_descriptor_it_read_out_of_its_own_fifo() {
    let (mut machine, bus) = boot("dwc2-gadget-enumerate");
    plug_in(&mut machine, &bus);

    let mut xfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    drive(&mut machine, &bus, &mut xfer);

    assert_eq!(xfer.failure(), None, "the transfer failed");
    assert_eq!(
        xfer.data(),
        &DESCRIPTOR,
        "the bytes the host collected are the ones guest code pushed into a FIFO"
    );

    // And the guest genuinely read the request: these eight bytes were put in
    // RAM by `lw`/`sw` out of `GRXSTSP` and endpoint zero's window.
    assert_eq!(
        peek_bytes(&machine, SETUP_BUF, 8),
        vec![0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 18, 0x00],
        "the setup packet the guest saw"
    );
    assert_eq!(peek(&machine, DONE1), MAGIC1, "the guest read the request");
    assert_eq!(peek(&machine, DONE2), MAGIC2, "and built the reply");
}

#[test]
fn nothing_is_on_the_bus_until_the_guest_releases_soft_disconnect() {
    let (mut machine, bus) = boot("dwc2-gadget-connect");
    assert!(
        !bus.connected(0),
        "a core out of reset is soft-disconnected, so the port is empty"
    );

    // Before the firmware has run, an enumeration finds nothing at all — and
    // that is a *modelled* absence, not a device that answers wrongly.
    let mut early = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    assert!(matches!(
        early.step(&bus, DeviceAddress::DEFAULT, 64),
        Progress::Failed(_)
    ));

    plug_in(&mut machine, &bus);
    assert!(bus.connected(0), "and now the guest has plugged itself in");
}

#[test]
fn a_snapshot_taken_mid_enumeration_restores_to_the_same_machine() {
    let (mut machine, bus) = boot("dwc2-gadget-snapshot");
    plug_in(&mut machine, &bus);

    // Stop after the guest has read the request but before the host has
    // collected the answer: the reply is sitting in a transmit FIFO inside the
    // controller, which is the state a snapshot is most likely to drop.
    let mut xfer = ControlTransfer::device_to_host(host::get_descriptor(1, 0, 18));
    assert_eq!(xfer.step(&bus, DeviceAddress::DEFAULT, 64), Progress::Moved);
    for _ in 0..200 {
        tick(&mut machine);
        if peek(&machine, DONE2) == MAGIC2 {
            break;
        }
    }
    assert_eq!(peek(&machine, DONE2), MAGIC2);

    let saved = machine.save().expect("it saves");
    let before = machine.state_hash().expect("a hash");

    let (mut restored, restored_bus) = boot("dwc2-gadget-snapshot-b");
    restored.load(&saved).expect("it loads");
    assert_eq!(
        restored.state_hash().expect("a hash"),
        before,
        "the machine did not round trip"
    );
    assert!(
        restored_bus.connected(0),
        "and the gadget is back on the bus, derived from DCTL rather than saved"
    );

    restored_bus.set_enabled(0, true);
    drive(&mut restored, &restored_bus, &mut xfer);
    assert_eq!(
        xfer.data(),
        &DESCRIPTOR,
        "the transfer finished on the other side of a snapshot"
    );
}
