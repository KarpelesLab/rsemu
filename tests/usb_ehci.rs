//! USB, end to end, with a **guest** driving it.
//!
//! The unit tests under `src/dev/usb/` say "the register accepted the write"
//! and "the walker read the descriptor a test wrote". This says something
//! stronger, and it is the only claim in the tree that is worth making about a
//! host controller:
//!
//! > A program running on an emulated CPU builds queue heads and transfer
//! > descriptors in guest RAM, points an EHCI controller at them, starts it,
//! > and **a USB device enumerates and moves bytes back into that RAM** —
//! > through the controller's own DMA, with the guest finding out by polling
//! > the interrupt status register the controller set.
//!
//! Nothing in this file calls into the controller. Everything it does, it does
//! by executing RV32 instructions.
//!
//! # What the firmware does
//!
//! ```text
//!   CONFIGFLAG = 1                       claim the root ports from a companion
//!   PORTSC |= PORT_RESET                 drive a bus reset
//!   PORTSC &= ~PORT_RESET                release it; the port enables here
//!   copy the schedule template into RAM  two control queue heads and one
//!                                          interrupt queue head, with their
//!                                          transfer descriptors
//!   fill the periodic frame list         1024 entries, all the interrupt QH
//!   ASYNCLISTADDR = QH_A                 the asynchronous ring
//!   PERIODICLISTBASE = the frame list
//!   USBCMD = RS | ASE                    run, asynchronous schedule enabled
//!   poll USBSTS until USBINT             SET_ADDRESS(3), GET_DESCRIPTOR(18)
//!                                          and SET_CONFIGURATION(1) have run
//!   store a magic word                   so the test knows
//!   USBCMD = RS | ASE | PSE              periodic schedule too
//!   poll USBSTS until USBINT             a mouse report arrived
//!   store a second magic word
//! ```
//!
//! The two control queue heads are on the asynchronous ring at once —
//! `SET_ADDRESS` on a queue head addressed to zero, and everything after it on
//! one addressed to three. A driver would serialise those; a schedule may
//! legitimately contain both, and the controller executes them in list order,
//! which is what makes the second one's address the right one by the time it
//! runs. That it works is the point: the address really moved.

#![cfg(all(
    feature = "cpu-riscv",
    feature = "dev-usb-ehci",
    feature = "dev-usb-hid"
))]

use rsemu::bus::usb::{DeviceAddress, buses};
use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::usb::hid::HidMouse;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// The memory map, as the machine description below lays it out
// ---------------------------------------------------------------------------

/// The EHCI register block.
const EHCI: u32 = 0xf000_0000;
/// `CAPLENGTH` for the generic controller, so the operational registers start
/// here. The firmware could read it; it is a constant in a test that also
/// asserts it.
const OP: u32 = EHCI + 0x20;

const USBCMD: i32 = 0x00;
const USBSTS: i32 = 0x04;
const PERIODICLISTBASE: i32 = 0x14;
const ASYNCLISTADDR: i32 = 0x18;
const CONFIGFLAG: i32 = 0x40;
const PORTSC: i32 = 0x44;

/// Where RAM starts.
const RAM: u32 = 0x1000_0000;
/// Where the firmware copies its schedule template to.
const TPL: u32 = RAM + 0x2000;
/// Where the template lives in the ROM image.
const TPL_ROM: u32 = 0x400;

// Offsets inside the template. Queue heads are 48 bytes and descriptors are
// 32; everything is 32-byte aligned, as EHCI 1.0 §3.5 and §3.6 require.
const QH_A: u32 = TPL;
const QH_B: u32 = TPL + 0x040;
const QH_INT: u32 = TPL + 0x080;
const QTD_SA_SETUP: u32 = TPL + 0x0c0;
const QTD_SA_STATUS: u32 = TPL + 0x0e0;
const QTD_GD_SETUP: u32 = TPL + 0x100;
const QTD_GD_IN: u32 = TPL + 0x120;
const QTD_GD_STATUS: u32 = TPL + 0x140;
const QTD_SC_SETUP: u32 = TPL + 0x160;
const QTD_SC_STATUS: u32 = TPL + 0x180;
const QTD_INT: u32 = TPL + 0x1a0;
const SETUP_SA: u32 = TPL + 0x1c0;
const SETUP_GD: u32 = TPL + 0x1c8;
const SETUP_SC: u32 = TPL + 0x1d0;
/// How many bytes of template the firmware copies.
const TPL_BYTES: u32 = 0x1d8;

/// Where the controller writes the device descriptor.
const DESC_BUF: u32 = RAM + 0x3100;
/// …and the mouse report.
const REPORT_BUF: u32 = RAM + 0x3200;
/// The firmware's two progress flags.
const DONE1: u32 = RAM + 0x3000;
const DONE2: u32 = RAM + 0x3004;
const MAGIC1: u32 = 0x0d15_ea5e;
const MAGIC2: u32 = 0x600d_f00d;

/// The periodic frame list. 4 KiB aligned, as `PERIODICLISTBASE` requires.
const FRAME_LIST: u32 = RAM + 0x4000;

/// The address the firmware gives the device.
const GUEST_ADDRESS: u8 = 3;

// ---------------------------------------------------------------------------
// Just enough RV32I to write the firmware
// ---------------------------------------------------------------------------

const ZERO: u32 = 0;
const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const A0: u32 = 10;
const A2: u32 = 12;

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
fn ori(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm & 0xfff, rs1, 0b110, rd, OP_IMM)
}
fn andi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm & 0xfff, rs1, 0b111, rd, OP_IMM)
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
///
/// `addi` sign-extends its immediate, so the upper half is rounded up when the
/// lower one is going to come out negative.
fn li(rd: u32, value: u32) -> [u32; 2] {
    let hi = (value.wrapping_add(0x800)) >> 12;
    let lo = (value & 0xfff) as i32;
    let lo = if lo >= 0x800 { lo - 0x1000 } else { lo };
    [lui(rd, hi), addi(rd, rd, lo)]
}

/// The program, assembled here so that the listing in the module docs and the
/// bytes that run cannot drift apart.
fn firmware() -> Vec<u8> {
    let mut code: Vec<u32> = Vec::new();
    let push = |c: &mut Vec<u32>, words: &[u32]| c.extend_from_slice(words);

    push(&mut code, &li(T0, OP));

    // CONFIGFLAG = 1: these root ports are mine, not a companion's.
    push(&mut code, &li(A0, 1));
    code.push(sw(A0, T0, CONFIGFLAG));

    // Drive a bus reset, then release it — releasing is what enables the port.
    code.push(lw(A0, T0, PORTSC));
    code.push(ori(A0, A0, 0x100));
    code.push(sw(A0, T0, PORTSC));
    code.push(lw(A0, T0, PORTSC));
    code.push(andi(A0, A0, -257));
    code.push(sw(A0, T0, PORTSC));

    // Copy the schedule template out of ROM and into RAM.
    push(&mut code, &li(T1, TPL));
    push(&mut code, &li(T2, TPL_ROM));
    push(&mut code, &li(A2, TPL_BYTES));
    let copy = code.len();
    code.push(lw(A0, T2, 0));
    code.push(sw(A0, T1, 0));
    code.push(addi(T1, T1, 4));
    code.push(addi(T2, T2, 4));
    code.push(addi(A2, A2, -4));
    let back = -(((code.len() - copy) * 4) as i32);
    code.push(bne(A2, ZERO, back));

    // Fill the periodic frame list: every frame points at the interrupt queue
    // head, with `Typ = 01b` saying it is one.
    push(&mut code, &li(T1, FRAME_LIST));
    push(&mut code, &li(A0, QH_INT | 0x2));
    push(&mut code, &li(A2, 1024));
    let fill = code.len();
    code.push(sw(A0, T1, 0));
    code.push(addi(T1, T1, 4));
    code.push(addi(A2, A2, -1));
    let back = -(((code.len() - fill) * 4) as i32);
    code.push(bne(A2, ZERO, back));

    // Hand over both schedule roots.
    push(&mut code, &li(A0, QH_A));
    code.push(sw(A0, T0, ASYNCLISTADDR));
    push(&mut code, &li(A0, FRAME_LIST));
    code.push(sw(A0, T0, PERIODICLISTBASE));

    // Run, with the asynchronous schedule enabled.
    push(&mut code, &li(A0, 0x21));
    code.push(sw(A0, T0, USBCMD));

    // Poll `USBSTS.USBINT`. This is the load that catches the controller up
    // (`ROADMAP.md` §4.2's sync-on-access), so the answer is the one at the
    // cycle the guest asked.
    let wait1 = code.len();
    code.push(lw(A0, T0, USBSTS));
    code.push(andi(A0, A0, 1));
    let back = -(((code.len() - wait1) * 4) as i32);
    code.push(beq(A0, ZERO, back));
    push(&mut code, &li(A0, 1));
    code.push(sw(A0, T0, USBSTS));
    push(&mut code, &li(T1, DONE1));
    push(&mut code, &li(A0, MAGIC1));
    code.push(sw(A0, T1, 0));

    // Now the periodic schedule as well, and wait for a mouse report.
    push(&mut code, &li(A0, 0x31));
    code.push(sw(A0, T0, USBCMD));
    let wait2 = code.len();
    code.push(lw(A0, T0, USBSTS));
    code.push(andi(A0, A0, 1));
    let back = -(((code.len() - wait2) * 4) as i32);
    code.push(beq(A0, ZERO, back));
    push(&mut code, &li(A0, 1));
    code.push(sw(A0, T0, USBSTS));
    push(&mut code, &li(T1, DONE2));
    push(&mut code, &li(A0, MAGIC2));
    code.push(sw(A0, T1, 0));

    code.push(jal(ZERO, 0));

    let mut image = Vec::new();
    for word in code {
        image.extend_from_slice(&word.to_le_bytes());
    }
    assert!(
        image.len() <= TPL_ROM as usize,
        "the firmware grew into the template"
    );
    image.resize(TPL_ROM as usize, 0);
    image.extend_from_slice(&template());
    image
}

// ---------------------------------------------------------------------------
// The schedule the firmware copies into RAM
// ---------------------------------------------------------------------------

/// A link pointer's terminate bit.
const T: u32 = 1;
/// A queue-head link pointer: `Typ = 01b`.
fn qh_link(addr: u32) -> u32 {
    addr | 0x2
}

/// `Endpoint Characteristics` (§3.6.2) for a high-speed endpoint.
fn epchar(address: u8, endpoint: u8, mps: u32, head: bool) -> u32 {
    let mut value = u32::from(address)
        | (u32::from(endpoint) << 8)
        // EPS = 10b: high speed.
        | (0x2 << 12)
        // DTC: the toggle comes from the descriptor, which is what a control
        // queue head needs so `SETUP` can force DATA0.
        | (1 << 14)
        | (mps << 16);
    if head {
        value |= 1 << 15;
    }
    value
}

/// A qTD token (§3.5.3).
fn token(pid: u32, bytes: u32, toggle: bool, ioc: bool) -> u32 {
    let mut value = 0x80 | (pid << 8) | (3 << 10) | ((bytes & 0x7fff) << 16);
    if toggle {
        value |= 1 << 31;
    }
    if ioc {
        value |= 1 << 15;
    }
    value
}

const PID_OUT: u32 = 0;
const PID_IN: u32 = 1;
const PID_SETUP: u32 = 2;

/// Write `words` into `blob` at guest address `addr`.
fn put(blob: &mut [u8], addr: u32, words: &[u32]) {
    let offset = (addr - TPL) as usize;
    for (i, word) in words.iter().enumerate() {
        blob[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
}

/// An eight-byte setup packet.
fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> [u32; 2] {
    let bytes = [
        request_type,
        request,
        value as u8,
        (value >> 8) as u8,
        index as u8,
        (index >> 8) as u8,
        length as u8,
        (length >> 8) as u8,
    ];
    [
        u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
    ]
}

/// The whole schedule, as bytes, laid out at the addresses the firmware copies
/// it to.
fn template() -> Vec<u8> {
    let mut blob = vec![0u8; TPL_BYTES as usize];

    // The asynchronous ring: QH_A, then QH_B, then back to QH_A.
    put(
        &mut blob,
        QH_A,
        &[
            qh_link(QH_B),
            epchar(0, 0, 64, true),
            1 << 30, // Mult = 01b
            0,
            QTD_SA_SETUP,
            T,
            0,
            0,
            0,
            0,
            0,
            0,
        ],
    );
    put(
        &mut blob,
        QH_B,
        &[
            qh_link(QH_A),
            epchar(GUEST_ADDRESS, 0, 64, false),
            1 << 30,
            0,
            QTD_GD_SETUP,
            T,
            0,
            0,
            0,
            0,
            0,
            0,
        ],
    );
    // The interrupt queue head: not on the ring, reached from the frame list.
    // Its S-mask selects microframe zero of every frame (§3.6.3).
    put(
        &mut blob,
        QH_INT,
        &[
            T,
            epchar(GUEST_ADDRESS, 1, 8, false),
            (1 << 30) | 0x01,
            0,
            QTD_INT,
            T,
            0,
            0,
            0,
            0,
            0,
            0,
        ],
    );

    // SET_ADDRESS(3): a setup and a zero-length status IN.
    put(
        &mut blob,
        QTD_SA_SETUP,
        &[
            QTD_SA_STATUS,
            T,
            token(PID_SETUP, 8, false, false),
            SETUP_SA,
            0,
            0,
            0,
            0,
        ],
    );
    put(
        &mut blob,
        QTD_SA_STATUS,
        &[T, T, token(PID_IN, 0, true, false), 0, 0, 0, 0, 0],
    );

    // GET_DESCRIPTOR(DEVICE, 18): setup, data in, status out.
    put(
        &mut blob,
        QTD_GD_SETUP,
        &[
            QTD_GD_IN,
            T,
            token(PID_SETUP, 8, false, false),
            SETUP_GD,
            0,
            0,
            0,
            0,
        ],
    );
    put(
        &mut blob,
        QTD_GD_IN,
        &[
            QTD_GD_STATUS,
            T,
            token(PID_IN, 18, true, false),
            DESC_BUF,
            0,
            0,
            0,
            0,
        ],
    );
    put(
        &mut blob,
        QTD_GD_STATUS,
        &[
            QTD_SC_SETUP,
            T,
            token(PID_OUT, 0, true, false),
            0,
            0,
            0,
            0,
            0,
        ],
    );

    // SET_CONFIGURATION(1): setup and a zero-length status IN, with `IOC` on
    // the last one so the guest finds out the whole chain is done.
    put(
        &mut blob,
        QTD_SC_SETUP,
        &[
            QTD_SC_STATUS,
            T,
            token(PID_SETUP, 8, false, false),
            SETUP_SC,
            0,
            0,
            0,
            0,
        ],
    );
    put(
        &mut blob,
        QTD_SC_STATUS,
        &[T, T, token(PID_IN, 0, true, true), 0, 0, 0, 0, 0],
    );

    // The interrupt transfer: eight bytes from the mouse's endpoint 1.
    put(
        &mut blob,
        QTD_INT,
        &[T, T, token(PID_IN, 8, false, true), REPORT_BUF, 0, 0, 0, 0],
    );

    // The three setup packets.
    put(
        &mut blob,
        SETUP_SA,
        &setup(0x00, 5, u16::from(GUEST_ADDRESS), 0, 0),
    );
    put(&mut blob, SETUP_GD, &setup(0x80, 6, 0x0100, 0, 18));
    put(&mut blob, SETUP_SC, &setup(0x00, 9, 1, 0, 0));
    blob
}

// ---------------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------------

/// A machine file carried here rather than in `machines/`, because this board
/// exists to be a test and nothing else.
const BOARD: &str = r#"
machine "usb-ehci-test" {
  param usbbus = "usb-test"

  # The hart's clock, and the USB PHY's. Two oscillators because a board has
  # two: 60 MHz is what a USB 2.0 PHY runs at, and 60 MHz / 8000 microframes a
  # second is exactly 7500 ticks with no remainder — which is the whole reason
  # the controller counts microframes in its own domain's ticks.
  osc sysclk = 100000000 Hz
  osc usbclk = 60000000 Hz

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

  # A bus master: it reads its queue heads out of `mem` itself.
  object ehci "usb.ehci" {
    clock      = usbclk
    space      = mem
    bus        = usbbus
    ports      = 1
    microframe = 7500
  }

  map mem 0x00000000 size 64K = fw
  map mem 0x10000000 size 1M  = dram
  map mem 0xf0000000 size 256 = ehci
}
"#;

/// Build the board with a bus name nothing else is using, and a mouse already
/// plugged into it.
///
/// The mouse is attached from here rather than declared in the machine file so
/// that this test can hold it and move it. `bus::usb::buses` is a process-wide
/// rendezvous table until `core::bus` exists, which is what makes that
/// possible — and what makes a unique name per test necessary.
fn boot(bus_name: &str) -> (Machine, HidMouse) {
    buses::close(bus_name);
    let bus = buses::open(bus_name, 1);
    let mouse = HidMouse::new_detached(0x1234, 0x5678);
    bus.attach(0, mouse.device()).expect("an empty port");

    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options
        .resolve
        .params
        .push((String::from("usbbus"), String::from(bus_name)));
    let registry = catalog::registry().expect("a registry");
    let machine =
        rsemu::machine::build("usb-ehci-test", BOARD, &registry, &options).expect("it realizes");
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
fn a_guest_enumerates_a_usb_device_and_the_descriptor_lands_in_its_own_ram() {
    let (mut machine, mouse) = boot("usb-enumerate");
    assert!(
        run_until(&mut machine, DONE1, MAGIC1),
        "the firmware never saw the interrupt its own schedule asked for"
    );

    // The guest gave the device an address, and the device took it.
    assert_eq!(
        mouse.address(),
        DeviceAddress(GUEST_ADDRESS),
        "SET_ADDRESS was issued by guest code, through the controller's DMA walk"
    );
    // And configured it.
    assert_eq!(mouse.configuration(), 1);

    // The eighteen bytes of the device descriptor, in the buffer the *guest*
    // named, put there by the controller reading the device.
    let descriptor = peek_bytes(&machine, DESC_BUF, 18);
    assert_eq!(descriptor[0], 18, "bLength");
    assert_eq!(descriptor[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(
        u16::from_le_bytes([descriptor[2], descriptor[3]]),
        0x0200,
        "bcdUSB"
    );
    assert_eq!(descriptor[7], 64, "bMaxPacketSize0 for a high-speed device");
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

    buses::close("usb-enumerate");
}

#[test]
fn a_mouse_report_reaches_guest_ram_through_the_periodic_schedule() {
    let (mut machine, mouse) = boot("usb-report");
    assert!(run_until(&mut machine, DONE1, MAGIC1), "enumeration");

    // Nothing has moved yet, so the interrupt endpoint NAKs and the firmware
    // is still polling — the second flag must not be set.
    machine
        .run_for(GlobalTime::from_nanos(2_000_000))
        .expect("it runs");
    assert_ne!(
        peek(&machine, DONE2),
        MAGIC2,
        "an idle interrupt endpoint NAKs, and a NAK is not a completion"
    );

    mouse.motion(0x12, -0x22, 0b101);
    assert!(
        run_until(&mut machine, DONE2, MAGIC2),
        "the firmware never saw the interrupt transfer complete"
    );

    let report = peek_bytes(&machine, REPORT_BUF, 3);
    assert_eq!(report[0], 0b101, "buttons one and three");
    assert_eq!(report[1] as i8, 0x12, "relative X");
    assert_eq!(report[2] as i8, -0x22, "relative Y");

    buses::close("usb-report");
}

#[test]
fn the_controller_is_where_the_machine_file_put_it() {
    // The addresses are a board's, never the core's: this one maps the
    // register block at 0xf0000000, and a DigiColor maps a ChipIdea variant of
    // the same engine at 0xf00bc000. Nothing in `src/dev/usb` knows either
    // number.
    let (machine, _mouse) = boot("usb-placement");
    let caplength = machine
        .space("mem")
        .expect("the board has one")
        .read(u64::from(EHCI), Width::U8, MemAttrs::DEBUG)
        .expect("the register block is mapped");
    assert_eq!(caplength, 0x20, "CAPLENGTH, read as the byte it is");
    let version = machine
        .space("mem")
        .expect("the board has one")
        .read(u64::from(EHCI) + 2, Width::U16, MemAttrs::DEBUG)
        .expect("mapped");
    assert_eq!(version, 0x0100, "HCIVERSION: EHCI 1.0");
    buses::close("usb-placement");
}

#[test]
fn a_snapshot_taken_after_enumeration_restores_to_the_same_state() {
    let (mut machine, _mouse) = boot("usb-snapshot");
    assert!(run_until(&mut machine, DONE1, MAGIC1), "enumeration");

    let saved = machine.save().expect("it saves");
    let before = machine.state_hash().expect("a hash");

    let (mut restored, _mouse2) = boot("usb-snapshot-b");
    restored.load(&saved).expect("it loads");
    assert_eq!(
        restored.state_hash().expect("a hash"),
        before,
        "the machine did not round trip"
    );
    // The device descriptor came back with the RAM.
    assert_eq!(peek_bytes(&restored, DESC_BUF, 2), vec![18, 1]);

    buses::close("usb-snapshot");
    buses::close("usb-snapshot-b");
}
