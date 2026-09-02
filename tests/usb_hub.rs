//! A USB hub, end to end, with a **guest** driving it and a device *behind* it.
//!
//! `src/dev/usb/hub/tests.rs` says "the hub answered the class request the
//! fabric handed it". This says the thing that is actually worth claiming about
//! a hub:
//!
//! > A program running on an emulated CPU builds queue heads and transfer
//! > descriptors in guest RAM, points an EHCI at them, **enumerates a hub**,
//! > powers one of its downstream ports, resets it — and then issues control
//! > transfers and Bulk-Only commands to a disk that is on **no root port at
//! > all**, getting back the sector that is on the medium, which this test holds
//! > a second handle to and the guest cannot reach.
//!
//! That last step is the whole point. Before routing existed,
//! `UsbBus::find` searched a flat list of enabled *root* ports, so a device
//! behind a hub had nowhere to be: every transaction here after stage 9 would
//! have come back `NoDevice`.
//!
//! Nothing in this file calls into the hub, the disk or the controller.
//! Everything it does, it does by executing RV32 instructions on
//! `machines/hub-mini.machine`.
//!
//! # The port numbers disagree on purpose
//!
//! The hub class requests of USB 2.0 §11.24.2 number ports from **one**; the
//! fabric indexes them from **zero**. `machines/hub-mini.machine` puts the disk
//! on downstream index **1**, so the firmware has to ask about hub port **2**,
//! and hub port 1 stays empty and unpowered for the whole run. An off-by-one in
//! either direction powers and resets the empty port, the disk never enumerates,
//! and the firmware never finishes — which is why that is a *failure* here
//! rather than two mistakes cancelling.
//!
//! # What the firmware does
//!
//! ```text
//!   PLIC priority/enable/threshold, mtvec, mie.MEIE, mstatus.MIE
//!   USBINTR = USBINT, CONFIGFLAG = 1, PORTSC reset and release
//!   copy the schedule template into RAM
//!   for each of seventeen stages:
//!       USBCMD = 0; ASYNCLISTADDR = this stage's QH; USBCMD = RS | ASE
//!       poll the last qTD's token until its Active bit clears
//!   store a magic word
//! ```
//!
//! The seventeen stages, in the order USB 2.0 §11 says a host does them:
//!
//! | | Addressed to | Stage |
//! | --- | --- | --- |
//! | 1–2 | the hub, at 0 then 1 | `SET_ADDRESS`, `SET_CONFIGURATION` |
//! | 3 | the hub | `GetHubDescriptor` — how many ports there are |
//! | 4 | the hub | `SetPortFeature(PORT_POWER, 2)` |
//! | 5 | the hub | `GetPortStatus(2)` — the connection appears *because* of the power |
//! | 6–7 | the hub | `ClearPortFeature(C_PORT_CONNECTION)`, `SetPortFeature(PORT_RESET)` |
//! | 8 | the hub | `GetPortStatus(2)` — enabled now, and `C_PORT_RESET` set |
//! | 9 | **the disk, at address 0** | `SET_ADDRESS(3)` — the first transaction that has to be routed through a hub |
//! | 10–11 | the disk | `SET_CONFIGURATION`, `GET_DESCRIPTOR(device)` |
//! | 12–15 | the disk | `READ (10)` and `WRITE (10)` as CBW/data/CSW triples |
//! | 16 | the hub | `GetPortStatus(2)` again, after all of it |
//! | 17 | the hub | `GetPortStatus(1)` — the port nothing is on, which must report nothing |
//!
//! # The interrupt is not polled
//!
//! As `tests/usb_msd.rs`: every stage's last descriptor carries `IOC`, the
//! controller's level interrupt travels `ehci.irq → plic.irq1 → plic.meip0 →
//! cpu.meip`, and the handler writes one to `USBSTS.USBINT` **before**
//! completing the PLIC claim. The test counts the traps — seventeen, one per
//! stage — because the wrong acknowledgement order doubles the count rather
//! than failing outright.
//!
//! # Sources
//!
//! USB 2.0 §11.23.2.1 (the hub descriptor), §11.24.2 tables 11-16, 11-17,
//! 11-21 and 11-22 (the class requests, the feature selectors, `wPortStatus`
//! and `wPortChange`); EHCI 1.0 §2.3, §3.5 and §3.6; the Bulk-Only Transport
//! 1.0 §5.1–§5.3 and Seagate's SCSI Commands Reference Manual Rev. J §3.16 and
//! §3.60 for `READ (10)` and `WRITE (10)`. No emulator source was consulted
//! (`ROADMAP.md` §1).

#![cfg(all(
    feature = "machine-hub-mini",
    feature = "cpu-riscv",
    feature = "dev-riscv",
    feature = "dev-usb-ehci",
    feature = "dev-usb-hub",
    feature = "dev-usb-msd"
))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::{MemAttrs, RamStore};
use rsemu::core::value::Width;
use rsemu::dev::medium::Medium;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// The memory map, as `machines/hub-mini.machine` lays it out
// ---------------------------------------------------------------------------

const EHCI: u32 = 0xf000_0000;
/// `CAPLENGTH` for the generic controller, so the operational registers start
/// here.
const OP: u32 = EHCI + 0x20;

const USBCMD: i32 = 0x00;
const USBSTS: i32 = 0x04;
const USBINTR: i32 = 0x08;
const ASYNCLISTADDR: i32 = 0x18;
const CONFIGFLAG: i32 = 0x40;
const PORTSC: i32 = 0x44;

/// `USBSTS.USBINT` and `USBINTR`'s matching enable (EHCI 1.0 §2.3.2, §2.3.3).
const STS_USBINT: u32 = 0x01;

const PLIC: u32 = 0x0c00_0000;
const PLIC_PRIORITY1: u32 = PLIC + 4;
const PLIC_PENDING: u64 = PLIC as u64 + 0x1000;
const PLIC_ENABLE0: u32 = PLIC + 0x2000;
const PLIC_THRESHOLD0: u32 = PLIC + 0x20_0000;
const PLIC_CLAIM0: u32 = PLIC + 0x20_0004;

const RAM: u32 = 0x1000_0000;

/// Where the trap handler is assembled, inside the ROM image.
const HANDLER: u32 = 0x400;
/// Where the schedule template lives in the ROM image.
const TPL_ROM: u32 = 0x800;
/// …and where the firmware copies it to.
const TPL: u32 = RAM + 0x4000;

// The buffers the *controller* writes into.
const HUBD_BUF: u32 = RAM + 0x1000;
const PS_AFTER_POWER: u32 = RAM + 0x1020;
const PS_AFTER_RESET: u32 = RAM + 0x1030;
const PS_AT_END: u32 = RAM + 0x1040;
const PS_EMPTY_PORT: u32 = RAM + 0x1050;
const DEVD_BUF: u32 = RAM + 0x1080;
const CSW_BUF: u32 = RAM + 0x1100;
const RDATA_BUF: u32 = RAM + 0x1200;
/// The firmware's progress flag, and the count of traps it took.
const DONE: u32 = RAM + 0x1800;
const IRQS: u32 = RAM + 0x1804;
const MAGIC: u32 = 0x4855_4200;

/// The address the firmware gives the hub, and the one it gives the disk. The
/// hub is enumerated first, so it gets the lower one.
const HUB_ADDRESS: u8 = 1;
const DISK_ADDRESS: u8 = 3;

/// The hub port the disk is on, **one-based** as USB 2.0 §11.24.2 numbers them:
/// `machines/hub-mini.machine` puts the disk on the fabric's index 1.
const DISK_PORT: u16 = 2;
/// A port with nothing on it, which the firmware also asks about.
const EMPTY_PORT: u16 = 1;

/// Bytes in a logical block, and how many the disk holds.
const BLOCK: u64 = 512;
const BLOCKS: u64 = 1024 * 1024 / BLOCK;
const READ_LBA: u32 = 11;
const WRITE_LBA: u32 = 40;

const TAG_RD: u32 = 0x3333_3333;
const TAG_WR: u32 = 0x4444_4444;

// -- the hub's own constants, spelled here because a test that imported them
//    from the device could not catch the device spelling them wrong ----------

/// The hub class descriptor type (§11.23.2.1).
const DESC_HUB: u8 = 0x29;
/// `bmRequestType` for a class request addressed to the hub, and to one of its
/// ports (§11.24.2, table 11-16).
const TO_PORT: u8 = 0x23;
const FROM_PORT: u8 = 0xa3;
const FROM_HUB: u8 = 0xa0;

const REQ_CLEAR_FEATURE: u8 = 1;
const REQ_SET_FEATURE: u8 = 3;
const REQ_SET_ADDRESS: u8 = 5;
const REQ_GET_DESCRIPTOR: u8 = 6;
const REQ_SET_CONFIGURATION: u8 = 9;
const REQ_GET_STATUS: u8 = 0;

const PORT_RESET: u16 = 4;
const PORT_POWER: u16 = 8;
const C_PORT_CONNECTION: u16 = 16;

const PS_CONNECTION: u16 = 1 << 0;
const PS_ENABLE: u16 = 1 << 1;
const PS_POWER: u16 = 1 << 8;
const PS_HIGH_SPEED: u16 = 1 << 10;
const PC_CONNECTION: u16 = 1 << 0;
const PC_RESET: u16 = 1 << 4;

// ---------------------------------------------------------------------------
// Just enough RV32IMA to write the firmware
// ---------------------------------------------------------------------------

const ZERO: u32 = 0;
const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const S0: u32 = 8;
const A0: u32 = 10;
const A1: u32 = 11;
const A2: u32 = 12;
const A3: u32 = 13;
const T3: u32 = 28;
const T4: u32 = 29;
const T5: u32 = 30;
const T6: u32 = 31;

const OP_LUI: u32 = 0b011_0111;
const OP_JAL: u32 = 0b110_1111;
const OP_BRANCH: u32 = 0b110_0011;
const OP_LOAD: u32 = 0b000_0011;
const OP_STORE: u32 = 0b010_0011;
const OP_IMM: u32 = 0b001_0011;
const OP_SYSTEM: u32 = 0b111_0011;

const CSR_MSTATUS: i32 = 0x300;
const CSR_MIE: i32 = 0x304;
const CSR_MTVEC: i32 = 0x305;
const MEIE: u32 = 1 << 11;
const MSTATUS_MIE: i32 = 8;

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
fn csrw(csr: i32, rs1: u32) -> u32 {
    i_type(csr, rs1, 0b001, ZERO, OP_SYSTEM)
}
fn csrs(csr: i32, rs1: u32) -> u32 {
    i_type(csr, rs1, 0b010, ZERO, OP_SYSTEM)
}
fn csrsi(csr: i32, uimm: u32) -> u32 {
    i_type(csr, uimm, 0b110, ZERO, OP_SYSTEM)
}
const MRET: u32 = 0x3020_0073;

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

/// The main program: identical in shape to `tests/usb_msd.rs`'s, because the
/// difference this board is testing is in the *topology* rather than in the
/// driver. One queue head at a time, polled to completion, out of a table.
fn main_program(template_bytes: u32) -> Vec<u32> {
    let mut code: Vec<u32> = Vec::new();
    let push = |c: &mut Vec<u32>, words: &[u32]| c.extend_from_slice(words);

    push(&mut code, &li(T0, OP));

    // -- the interrupt controller ------------------------------------------
    push(&mut code, &li(A0, PLIC_PRIORITY1));
    push(&mut code, &li(A1, 1));
    code.push(sw(A1, A0, 0));
    push(&mut code, &li(A0, PLIC_ENABLE0));
    push(&mut code, &li(A1, 1 << 1));
    code.push(sw(A1, A0, 0));
    push(&mut code, &li(A0, PLIC_THRESHOLD0));
    code.push(sw(ZERO, A0, 0));

    // -- the trap handler ---------------------------------------------------
    push(&mut code, &li(A0, HANDLER));
    code.push(csrw(CSR_MTVEC, A0));
    push(&mut code, &li(A0, MEIE));
    code.push(csrs(CSR_MIE, A0));
    code.push(csrsi(CSR_MSTATUS, MSTATUS_MIE as u32));

    // -- the controller -----------------------------------------------------
    push(&mut code, &li(A0, STS_USBINT));
    code.push(sw(A0, T0, USBINTR));
    push(&mut code, &li(A0, 1));
    code.push(sw(A0, T0, CONFIGFLAG));

    // Drive a bus reset on the root port, then release it. This resets the
    // **hub**, which is what is plugged in there.
    code.push(lw(A0, T0, PORTSC));
    code.push(ori(A0, A0, 0x100));
    code.push(sw(A0, T0, PORTSC));
    code.push(lw(A0, T0, PORTSC));
    code.push(andi(A0, A0, -257));
    code.push(sw(A0, T0, PORTSC));

    // -- copy the schedule template out of ROM and into RAM ------------------
    push(&mut code, &li(T1, TPL));
    push(&mut code, &li(T2, TPL_ROM));
    push(&mut code, &li(A2, template_bytes));
    let copy = code.len();
    code.push(lw(A0, T2, 0));
    code.push(sw(A0, T1, 0));
    code.push(addi(T1, T1, 4));
    code.push(addi(T2, T2, 4));
    code.push(addi(A2, A2, -4));
    let back = -(((code.len() - copy) * 4) as i32);
    code.push(bne(A2, ZERO, back));

    // -- run the stages -----------------------------------------------------
    push(&mut code, &li(S0, TPL + STAGE_TABLE));
    let loop_top = code.len();
    code.push(lw(A0, S0, 0));
    let exit_at = code.len();
    code.push(0);
    code.push(sw(ZERO, T0, USBCMD));
    code.push(sw(A0, T0, ASYNCLISTADDR));
    push(&mut code, &li(A1, 0x21)); // RS | ASE
    code.push(sw(A1, T0, USBCMD));
    code.push(lw(A1, S0, 4));
    let wait = code.len();
    // The `USBSTS` load is what catches the controller up; the token load is
    // the one the answer comes from.
    code.push(lw(A3, T0, USBSTS));
    code.push(lw(A2, A1, 0));
    code.push(andi(A2, A2, 0x80)); // the descriptor's Active bit (§3.5.3)
    let back = -(((code.len() - wait) * 4) as i32);
    code.push(bne(A2, ZERO, back));
    code.push(addi(S0, S0, 8));
    let back = -(((code.len() - loop_top) * 4) as i32);
    code.push(jal(ZERO, back));

    let done = code.len();
    code[exit_at] = beq(A0, ZERO, ((done - exit_at) * 4) as i32);

    code.push(sw(ZERO, T0, USBCMD));
    push(&mut code, &li(T1, DONE));
    push(&mut code, &li(A0, MAGIC));
    code.push(sw(A0, T1, 0));
    code.push(jal(ZERO, 0));
    code
}

/// The machine external interrupt handler: `USBSTS` write-one-to-clear first,
/// the PLIC claim completed second. See `tests/usb_msd.rs` for why the order is
/// what the count measures.
fn trap_handler() -> Vec<u32> {
    let mut code: Vec<u32> = Vec::new();
    let push = |c: &mut Vec<u32>, words: &[u32]| c.extend_from_slice(words);

    push(&mut code, &li(T3, PLIC_CLAIM0));
    code.push(lw(T4, T3, 0));
    push(&mut code, &li(T5, OP));
    push(&mut code, &li(T6, STS_USBINT));
    code.push(sw(T6, T5, USBSTS));
    code.push(sw(T4, T3, 0));
    push(&mut code, &li(T5, IRQS));
    code.push(lw(T6, T5, 0));
    code.push(addi(T6, T6, 1));
    code.push(sw(T6, T5, 0));
    code.push(MRET);
    code
}

fn firmware() -> Vec<u8> {
    let template = template();
    let mut image = Vec::new();
    for word in main_program(template.len() as u32) {
        image.extend_from_slice(&word.to_le_bytes());
    }
    assert!(
        image.len() <= HANDLER as usize,
        "the main program grew into the trap handler"
    );
    image.resize(HANDLER as usize, 0);
    for word in trap_handler() {
        image.extend_from_slice(&word.to_le_bytes());
    }
    assert!(
        image.len() <= TPL_ROM as usize,
        "the trap handler grew into the template"
    );
    image.resize(TPL_ROM as usize, 0);
    image.extend_from_slice(&template);
    image
}

// ---------------------------------------------------------------------------
// The schedule the firmware copies into RAM
// ---------------------------------------------------------------------------

/// A link pointer's terminate bit.
const T: u32 = 1;

const PID_OUT: u32 = 0;
const PID_IN: u32 = 1;
const PID_SETUP: u32 = 2;

/// Where each kind of object lives inside the template, as an offset from
/// [`TPL`]. Regions rather than per-object constants, because a hub run has
/// seventeen stages and hand-placing every descriptor is how a test gets an
/// alignment wrong.
const QH_BASE: u32 = 0x000;
const TD_BASE: u32 = 0x480;
const SETUP_BASE: u32 = 0x980;
const CBW_BASE: u32 = 0xa00;
const WDATA_BASE: u32 = 0xa80;
const STAGE_TABLE: u32 = 0xc80;
/// How much of the template the firmware copies. A multiple of four, because
/// the copy loop moves words.
const TPL_BYTES: u32 = 0xd20;

/// A queue-head link pointer: `Typ = 01b`.
fn qh_link(addr: u32) -> u32 {
    addr | 0x2
}

/// `Endpoint Characteristics` (EHCI 1.0 §3.6.2) for a high-speed endpoint.
fn epchar(address: u8, endpoint: u8, mps: u32, dtc: bool) -> u32 {
    let mut value = u32::from(address)
        | (u32::from(endpoint) << 8)
        // EPS = 10b: high speed.
        | (0x2 << 12)
        // H: the head of the reclamation list, which each stage's single queue
        // head is.
        | (1 << 15)
        | (mps << 16);
    if dtc {
        value |= 1 << 14;
    }
    value
}

/// A qTD token (EHCI 1.0 §3.5.3).
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

/// An eight-byte setup packet (USB 2.0 §9.3).
fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0] = request_type;
    out[1] = request;
    out[2..4].copy_from_slice(&value.to_le_bytes());
    out[4..6].copy_from_slice(&index.to_le_bytes());
    out[6..8].copy_from_slice(&length.to_le_bytes());
    out
}

/// A Command Block Wrapper: thirty-one bytes, little-endian (BOT §5.1).
fn cbw(tag: u32, data_length: u32, data_in: bool, cdb: &[u8]) -> [u8; 31] {
    let mut out = [0u8; 31];
    out[0..4].copy_from_slice(&0x4342_5355u32.to_le_bytes());
    out[4..8].copy_from_slice(&tag.to_le_bytes());
    out[8..12].copy_from_slice(&data_length.to_le_bytes());
    out[12] = if data_in { 0x80 } else { 0x00 };
    out[14] = cdb.len() as u8;
    out[15..15 + cdb.len()].copy_from_slice(cdb);
    out
}

/// A `READ (10)` command block (Seagate §3.16, table 97).
fn read10(lba: u32, blocks: u16) -> [u8; 10] {
    let lba = lba.to_be_bytes();
    let blocks = blocks.to_be_bytes();
    [
        0x28, 0, lba[0], lba[1], lba[2], lba[3], 0, blocks[0], blocks[1], 0,
    ]
}

/// A `WRITE (10)` command block (Seagate §3.60).
fn write10(lba: u32, blocks: u16) -> [u8; 10] {
    let mut cdb = read10(lba, blocks);
    cdb[0] = 0x2a;
    cdb
}

/// The 512 bytes the guest writes, deliberately unlike the pattern the medium
/// was stamped with.
fn write_payload() -> Vec<u8> {
    (0..BLOCK)
        .map(|i| (i as u8).wrapping_mul(11) ^ 0x5a)
        .collect()
}

/// The template under construction: a bump allocator per region, plus the stage
/// table the firmware walks.
struct Builder {
    blob: Vec<u8>,
    qh: u32,
    td: u32,
    setup: u32,
    cbw: u32,
    stages: Vec<(u32, u32)>,
}

impl Builder {
    fn new() -> Builder {
        Builder {
            blob: vec![0u8; TPL_BYTES as usize],
            qh: QH_BASE,
            td: TD_BASE,
            setup: SETUP_BASE,
            cbw: CBW_BASE,
            stages: Vec::new(),
        }
    }

    fn put(&mut self, offset: u32, words: &[u32]) {
        let at = offset as usize;
        for (i, word) in words.iter().enumerate() {
            self.blob[at + i * 4..at + i * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
    }

    fn put_bytes(&mut self, offset: u32, bytes: &[u8]) {
        let at = offset as usize;
        self.blob[at..at + bytes.len()].copy_from_slice(bytes);
    }

    /// A queue head, 32-byte aligned as EHCI 1.0 §3.6 requires, linked to
    /// itself with `first` as the head of its overlay chain.
    fn queue_head(&mut self, epchar: u32, first: u32) -> u32 {
        let offset = self.qh;
        self.qh += 0x40;
        assert!(self.qh <= TD_BASE, "the queue heads outgrew their region");
        let address = TPL + offset;
        self.put(
            offset,
            &[
                qh_link(address),
                epchar,
                // Endpoint Capabilities: `Mult = 01b`.
                1 << 30,
                0,
                first,
                T,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
        );
        address
    }

    /// A transfer descriptor: eight dwords (§3.5). Returns its guest address.
    fn qtd(&mut self, next: u32, token: u32, buffer: u32) -> u32 {
        let offset = self.td;
        self.td += 0x20;
        assert!(
            self.td <= SETUP_BASE,
            "the descriptors outgrew their region"
        );
        self.put(
            offset,
            &[
                if next == 0 { T } else { next },
                T,
                token,
                buffer,
                0,
                0,
                0,
                0,
            ],
        );
        TPL + offset
    }

    fn setup_packet(&mut self, bytes: [u8; 8]) -> u32 {
        let offset = self.setup;
        self.setup += 8;
        assert!(self.setup <= CBW_BASE, "the setup packets outgrew theirs");
        self.put_bytes(offset, &bytes);
        TPL + offset
    }

    fn command_block(&mut self, bytes: [u8; 31]) -> u32 {
        let offset = self.cbw;
        self.cbw += 0x20;
        assert!(self.cbw <= WDATA_BASE, "the command blocks outgrew theirs");
        self.put_bytes(offset, &bytes);
        TPL + offset
    }

    /// A control transfer with no data stage: `SETUP`, then the zero-length
    /// `IN` that is the status stage (USB 2.0 §8.5.3).
    ///
    /// Descriptors are allocated **backwards** — the last one first — because
    /// each points at the next and a pointer has to exist before it is written.
    fn control_out(&mut self, address: u8, packet: [u8; 8]) {
        let bytes = self.setup_packet(packet);
        let status = self.qtd(0, token(PID_IN, 0, true, true), 0);
        let first = self.qtd(status, token(PID_SETUP, 8, false, false), bytes);
        let qh = self.queue_head(epchar(address, 0, 64, true), first);
        self.stages.push((qh, status + 8));
    }

    /// A control transfer in the device-to-host direction: `SETUP`, one data
    /// packet, then the zero-length `OUT` status stage.
    ///
    /// One packet because every reply this firmware asks for is smaller than
    /// the 64-byte control endpoint.
    fn control_in(&mut self, address: u8, packet: [u8; 8], buffer: u32, length: u32) {
        assert!(length <= 64, "one packet only");
        let bytes = self.setup_packet(packet);
        let status = self.qtd(0, token(PID_OUT, 0, true, true), 0);
        let data = self.qtd(status, token(PID_IN, length, true, false), buffer);
        let first = self.qtd(data, token(PID_SETUP, 8, false, false), bytes);
        let qh = self.queue_head(epchar(address, 0, 64, true), first);
        self.stages.push((qh, status + 8));
    }

    /// One stage on a bulk endpoint: a chain of (token, buffer, byte count)
    /// with `IOC` on the last.
    fn bulk(&mut self, address: u8, endpoint: u8, chain: &[(u32, u32, u32)]) {
        let mut next = 0u32;
        let mut last = 0u32;
        for (index, (pid, buffer, bytes)) in chain.iter().enumerate().rev() {
            let ioc = index + 1 == chain.len();
            let toggle = index != 0;
            let td = self.qtd(next, token(*pid, *bytes, toggle, ioc), *buffer);
            if ioc {
                last = td;
            }
            next = td;
        }
        let qh = self.queue_head(epchar(address, endpoint, 512, false), next);
        self.stages.push((qh, last + 8));
    }

    fn finish(mut self) -> Vec<u8> {
        let mut words = Vec::new();
        for (qh, poll) in &self.stages {
            words.push(*qh);
            words.push(*poll);
        }
        // The terminator the firmware's `beq` looks for.
        words.push(0);
        words.push(0);
        let table = STAGE_TABLE;
        assert!(
            table as usize + words.len() * 4 <= TPL_BYTES as usize,
            "the stage table outgrew the template"
        );
        self.put(table, &words);
        self.blob
    }
}

/// How many stages the firmware runs, and therefore how many machine external
/// traps a correct run takes: one `IOC` descriptor each.
const STAGES: u32 = 17;

/// The whole schedule, as bytes.
fn template() -> Vec<u8> {
    let mut b = Builder::new();

    // -- the hub, as an ordinary device first -------------------------------
    b.control_out(0, setup(0, REQ_SET_ADDRESS, u16::from(HUB_ADDRESS), 0, 0));
    b.control_out(HUB_ADDRESS, setup(0, REQ_SET_CONFIGURATION, 1, 0, 0));
    // `GetHubDescriptor` (§11.24.2.5). Four ports means a nine-byte descriptor:
    // seven fixed bytes and two one-byte bitmaps.
    b.control_in(
        HUB_ADDRESS,
        setup(FROM_HUB, REQ_GET_DESCRIPTOR, u16::from(DESC_HUB) << 8, 0, 9),
        HUBD_BUF,
        9,
    );

    // -- and now as a hub ---------------------------------------------------
    //
    // Power first: until the port is powered it is in §11.5.1.1's *Powered-off*
    // state and reports no connection at all, however much is plugged into it.
    b.control_out(
        HUB_ADDRESS,
        setup(TO_PORT, REQ_SET_FEATURE, PORT_POWER, DISK_PORT, 0),
    );
    b.control_in(
        HUB_ADDRESS,
        setup(FROM_PORT, REQ_GET_STATUS, 0, DISK_PORT, 4),
        PS_AFTER_POWER,
        4,
    );
    b.control_out(
        HUB_ADDRESS,
        setup(TO_PORT, REQ_CLEAR_FEATURE, C_PORT_CONNECTION, DISK_PORT, 0),
    );
    b.control_out(
        HUB_ADDRESS,
        setup(TO_PORT, REQ_SET_FEATURE, PORT_RESET, DISK_PORT, 0),
    );
    b.control_in(
        HUB_ADDRESS,
        setup(FROM_PORT, REQ_GET_STATUS, 0, DISK_PORT, 4),
        PS_AFTER_RESET,
        4,
    );

    // -- the device behind it ----------------------------------------------
    //
    // Addressed to **zero**, which is where a freshly reset device answers —
    // and it is on no root port, so this transaction reaches it only because
    // the fabric walks tiers.
    b.control_out(0, setup(0, REQ_SET_ADDRESS, u16::from(DISK_ADDRESS), 0, 0));
    b.control_out(DISK_ADDRESS, setup(0, REQ_SET_CONFIGURATION, 1, 0, 0));
    b.control_in(
        DISK_ADDRESS,
        setup(0x80, REQ_GET_DESCRIPTOR, 1 << 8, 0, 18),
        DEVD_BUF,
        18,
    );

    // -- and its bulk endpoints --------------------------------------------
    let read_cbw = b.command_block(cbw(TAG_RD, BLOCK as u32, true, &read10(READ_LBA, 1)));
    b.bulk(DISK_ADDRESS, 2, &[(PID_OUT, read_cbw, 31)]);
    b.bulk(
        DISK_ADDRESS,
        1,
        &[(PID_IN, RDATA_BUF, BLOCK as u32), (PID_IN, CSW_BUF, 13)],
    );

    let write_cbw = b.command_block(cbw(TAG_WR, BLOCK as u32, false, &write10(WRITE_LBA, 1)));
    b.put_bytes(WDATA_BASE, &write_payload());
    b.bulk(
        DISK_ADDRESS,
        2,
        &[
            (PID_OUT, write_cbw, 31),
            (PID_OUT, TPL + WDATA_BASE, BLOCK as u32),
        ],
    );
    b.bulk(DISK_ADDRESS, 1, &[(PID_IN, CSW_BUF + 0x20, 13)]);

    // -- the hub, once more, after all of that ------------------------------
    b.control_in(
        HUB_ADDRESS,
        setup(FROM_PORT, REQ_GET_STATUS, 0, DISK_PORT, 4),
        PS_AT_END,
        4,
    );
    // And the port nothing is on. It was never powered, so §11.5.1.1 says it
    // reports nothing — which is what makes the one-based/zero-based confusion
    // visible instead of silently symmetric.
    b.control_in(
        HUB_ADDRESS,
        setup(FROM_PORT, REQ_GET_STATUS, 0, EMPTY_PORT, 4),
        PS_EMPTY_PORT,
        4,
    );

    assert_eq!(b.stages.len() as u32, STAGES);
    b.finish()
}

// ---------------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------------

/// A recognisable block: every byte says which block it came from and where in
/// it that byte sits.
fn stamp(lba: u64) -> Vec<u8> {
    (0..BLOCK)
        .map(|i| (lba as u8).wrapping_mul(37).wrapping_add(i as u8))
        .collect()
}

fn board() -> (Machine, Arc<RamStore>) {
    let store = Arc::new(RamStore::new(BLOCKS * BLOCK));
    for lba in 0..BLOCKS {
        RamStore::write_at(&store, lba * BLOCK, &stamp(lba)).expect("the image fits");
    }

    let mut options = catalog::build_options().expect("this build's options");
    options.realize.media.insert("firmware", firmware());
    rsemu::dev::medium::install(
        &options.realize.hosts,
        "usb0",
        Arc::clone(&store) as Arc<dyn Medium>,
    )
    .expect("nothing else claimed the name");
    options.realize.media.insert("usb0", Vec::new());

    let registry = catalog::registry().expect("this build's registry");
    let entry = &catalog::HUB_MINI;
    let mut machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, store)
}

fn run_until_done(machine: &mut Machine) -> bool {
    for _ in 0..600 {
        machine
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        if peek(machine, DONE) == MAGIC {
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

/// `wPortStatus` and `wPortChange` as the guest received them.
fn port_status(machine: &Machine, addr: u32) -> (u16, u16) {
    let bytes = peek_bytes(machine, addr, 4);
    (
        u16::from_le_bytes([bytes[0], bytes[1]]),
        u16::from_le_bytes([bytes[2], bytes[3]]),
    )
}

fn on_medium(store: &RamStore, lba: u64) -> Vec<u8> {
    let mut got = vec![0u8; BLOCK as usize];
    Medium::read_at(store, lba * BLOCK, &mut got).expect("the medium reads");
    got
}

fn csw(machine: &Machine, index: u32) -> (u32, u32, u32, u8) {
    let bytes = peek_bytes(machine, CSW_BUF + index * 0x20, 13);
    (
        u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
        bytes[12],
    )
}

/// `dCSWSignature`, spelling `USBS`.
const CSW_SIGNATURE: u32 = 0x5342_5355;

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_reads_a_sector_through_a_hub_and_it_is_the_sector_on_the_medium() {
    let (mut machine, store) = board();
    assert!(
        run_until_done(&mut machine),
        "the firmware never finished its seventeen stages"
    );

    let got = peek_bytes(&machine, RDATA_BUF, BLOCK as usize);
    // Against the **medium**, which the guest cannot reach — and which it
    // reached over a bus with a tier in it.
    assert_eq!(
        got,
        on_medium(&store, u64::from(READ_LBA)),
        "the block in guest RAM is not the block on the medium"
    );
    assert_ne!(got, on_medium(&store, u64::from(READ_LBA) + 1));
    assert_ne!(got, on_medium(&store, u64::from(READ_LBA) - 1));

    let (signature, tag, residue, status) = csw(&machine, 0);
    assert_eq!(signature, CSW_SIGNATURE);
    assert_eq!(tag, TAG_RD, "the CSW must echo its own CBW's tag");
    assert_eq!(residue, 0);
    assert_eq!(status, 0, "Command Passed");
}

#[test]
fn a_guest_writes_a_sector_through_a_hub_and_it_reaches_the_medium() {
    let (mut machine, store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    assert_eq!(
        on_medium(&store, u64::from(WRITE_LBA)),
        write_payload(),
        "the write did not reach the medium"
    );
    assert_eq!(
        on_medium(&store, u64::from(WRITE_LBA) - 1),
        stamp(u64::from(WRITE_LBA) - 1)
    );
    assert_eq!(
        on_medium(&store, u64::from(WRITE_LBA) + 1),
        stamp(u64::from(WRITE_LBA) + 1)
    );

    let (signature, tag, residue, status) = csw(&machine, 1);
    assert_eq!(signature, CSW_SIGNATURE);
    assert_eq!(tag, TAG_WR);
    assert_eq!(residue, 0);
    assert_eq!(status, 0);
}

#[test]
fn the_guest_enumerated_the_hub_before_it_could_reach_anything_behind_it() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    // The hub descriptor of §11.23.2.1, table 11-13.
    let descriptor = peek_bytes(&machine, HUBD_BUF, 9);
    assert_eq!(descriptor[0], 9, "bDescLength");
    assert_eq!(descriptor[1], DESC_HUB, "bDescriptorType");
    assert_eq!(descriptor[2], 4, "bNbrPorts, as the machine file says");

    // The device descriptor that came back from **behind** the hub: this is the
    // claim, and `0x0781` is `usb.storage`'s default `idVendor` rather than the
    // hub's zero.
    let device = peek_bytes(&machine, DEVD_BUF, 18);
    assert_eq!(device[0], 18, "bLength");
    assert_eq!(device[1], 1, "bDescriptorType");
    assert_eq!(
        u16::from_le_bytes([device[8], device[9]]),
        0x0781,
        "this is the disk's descriptor and not the hub's"
    );
    assert_eq!(device[4], 0, "BOT §4.1 puts the class on the interface");
    assert_eq!(device[7], 64, "bMaxPacketSize0 at high speed");
}

#[test]
fn the_port_reported_the_connection_only_after_it_was_powered_and_enabled_only_after_the_reset() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    // Straight after `SetPortFeature(PORT_POWER)`: §11.5.1.1's *Powered-off*
    // state is over, so the connection is visible — and it is not enabled,
    // because a connection is not an enable.
    let (status, change) = port_status(&machine, PS_AFTER_POWER);
    assert_eq!(status & PS_POWER, PS_POWER);
    assert_eq!(status & PS_CONNECTION, PS_CONNECTION);
    assert_eq!(status & PS_HIGH_SPEED, PS_HIGH_SPEED);
    assert_eq!(status & PS_ENABLE, 0);
    assert_eq!(change & PC_CONNECTION, PC_CONNECTION, "C_PORT_CONNECTION");

    // After the reset: enabled, and the change bit that says the reset finished
    // (§11.24.2.7.2.5). `C_PORT_CONNECTION` is gone because the firmware
    // cleared it before resetting, which is what a host does.
    let (status, change) = port_status(&machine, PS_AFTER_RESET);
    assert_eq!(status & PS_ENABLE, PS_ENABLE);
    assert_eq!(change & PC_RESET, PC_RESET, "C_PORT_RESET");
    assert_eq!(change & PC_CONNECTION, 0);

    // And it is still enabled at the end, after a sector went each way through
    // it — a port that had quietly dropped out would have failed earlier, but
    // this is the assertion that says so.
    let (status, _) = port_status(&machine, PS_AT_END);
    assert_eq!(
        status & (PS_ENABLE | PS_CONNECTION),
        PS_ENABLE | PS_CONNECTION
    );
}

#[test]
fn the_port_nothing_is_on_reports_nothing() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    // Hub port 1 is the fabric's index 0, and the machine file left it empty.
    // The firmware never powered it, so §11.5.1.1 says every bit is zero —
    // including the connection bit, which is the point of that paragraph.
    let (status, change) = port_status(&machine, PS_EMPTY_PORT);
    assert_eq!(status, 0, "an unpowered, empty port reports nothing at all");
    assert_eq!(change, 0);
}

#[test]
fn the_completion_interrupt_travelled_the_wire_and_was_acknowledged() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    let taken = peek(&machine, IRQS);
    assert_eq!(
        taken, STAGES,
        "one trap per IOC descriptor, no more and no less"
    );

    let status = machine
        .space("mem")
        .expect("the board has one")
        .read(u64::from(OP) + 0x04, Width::U32, MemAttrs::DEBUG)
        .expect("the register block") as u32;
    assert_eq!(status & STS_USBINT, 0, "USBSTS.USBINT is still asserted");

    let pending = machine
        .space("mem")
        .expect("the board has one")
        .read(PLIC_PENDING, Width::U32, MemAttrs::DEBUG)
        .expect("the PLIC") as u32;
    assert_eq!(pending & 0b10, 0, "the PLIC still holds source 1 pending");
}

#[test]
fn a_snapshot_taken_after_the_transfers_restores_to_the_same_state() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    let first = machine.save().expect("it saves");
    let (mut other, _other_store) = board();
    other.load(&first).expect("it loads");
    let second = other.save().expect("it saves again");
    assert_eq!(
        first.len(),
        second.len(),
        "a round trip changed the snapshot's size"
    );
    assert_eq!(first, second, "the state hash must be identical");
}
