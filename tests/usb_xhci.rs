//! xHCI, end to end, with a **guest** driving it and a **medium** behind it.
//!
//! `src/dev/usb/xhci/tests.rs` says "the controller answered the registers a
//! test wrote". This says the thing that is actually worth claiming about a
//! modern USB host controller:
//!
//! > A program running on an emulated CPU builds a Device Context Base Address
//! > Array, a command ring, an event ring with a segment table and three
//! > transfer rings in guest RAM, points an xHCI at them, issues **Enable Slot**
//! > and **Address Device**, hands over endpoint contexts with a **Configure
//! > Endpoint** command, pushes a **Command Block Wrapper** out of a bulk
//! > endpoint and pulls a sector and a **Command Status Wrapper** back in — and
//! > the bytes that arrived are the bytes **on the medium**, which this test
//! > holds a second handle to and the guest cannot reach.
//!
//! Nothing here calls into the device or the controller. Everything it does, it
//! does by executing RV32 instructions on `machines/xhci-mini.machine`.
//!
//! # The firmware is a stage-table interpreter
//!
//! Everything a driver does to an xHCI is *a dword written to an address*, and
//! that includes handing a TRB over: the **Cycle bit is the ownership flag**
//! (xHCI 1.2 §4.9), so a TRB the guest built with its Cycle bit clear belongs to
//! software, and setting that bit is what gives it to the controller. So the
//! whole firmware is:
//!
//! ```text
//!   for each (address, value, expected_events) in the stage table:
//!       *address = value
//!       while events < expected_events: spin
//! ```
//!
//! plus a prologue that arms the PLIC and `mtvec` and copies the template into
//! RAM. The rings are built in ROM with every Cycle bit **clear**; a stage sets
//! one, and the next stage rings the doorbell. That is not a shortcut around
//! the protocol — it *is* the protocol.
//!
//! # What the guest does, in order
//!
//! ```text
//!   CONFIG = 1                       one device slot enabled
//!   DCBAAP, CRCR, ERSTSZ, ERDP, ERSTBA, IMOD = 0, IMAN.IE
//!   USBCMD = RS | INTE
//!   PORTSC: clear CSC, then PR       the reset is what enables a USB2 port
//!   Enable Slot                      → Command Completion Event, Slot ID
//!   Address Device                   → the xHC issues SET_ADDRESS itself
//!   SET_CONFIGURATION(1)             a control transfer on the EP0 ring
//!   GET_MAX_LUN                      …with a data stage
//!   GET_DESCRIPTOR(Device, 18)       …and a longer one
//!   Configure Endpoint               the two bulk endpoints
//!   INQUIRY, READ CAPACITY (10), READ (10), WRITE (10) as CBW/data/CSW
//! ```
//!
//! # The interrupt is not polled, and the acknowledgement is three writes
//!
//! Every completion is a TRB the controller writes to the event ring, and
//! §4.17.2 says the first one sets `IMAN.IP` and `ERDP.EHB` together. That `IP`
//! is a level on `xhci.irq`, which is a wire into a **PLIC**, whose `meip0` is a
//! wire into the hart's external-interrupt pin, and the firmware takes a real
//! machine external trap.
//!
//! The handler drains every event whose Cycle bit says it is valid and then
//! acknowledges **in the order the specification fixes**:
//!
//! 1. `USBSTS.EINT` — §5.4.2 bit 3: *"Software that uses EINT shall clear it
//!    prior to clearing any IP flags."*
//! 2. `ERDP` with `EHB` set — §5.5.2.3.3: `EHB` is RW1C and is cleared by
//!    writing this register, which is also how software says how far it has read
//!    (§4.9.4).
//! 3. `IMAN.IP` — §4.17.3: the pin *"remains asserted until the device driver
//!    clears the Interrupt Pending (IP) flag"*.
//!
//! …and only then completes the PLIC claim. Completing the claim first would
//! leave the level asserted, the PLIC would re-latch the source, and the guest
//! would take a spurious trap for every real one — measured, not assumed:
//! **fifteen** traps in the right order and **thirty** in the wrong one.
//!
//! **Traps and events are different numbers here, and that is the point.**
//! Nineteen events arrive in fifteen interrupts, because four of the doorbells
//! retire two Transfer Descriptors each and `ERDP.EHB` blocks a second
//! interrupt until the handler has drained the ring — which is exactly what
//! §4.17.2's moderation scheme exists to do. Both numbers are asserted.
//!
//! # Sources
//!
//! The xHCI specification revision 1.2c (Intel, document 868295): §4.2 the
//! initialisation sequence, §4.6 the commands, §4.9 ring operation and the
//! Cycle bit, §4.17 interrupters, §5.3-§5.6 the register files, §6.2 the
//! contexts, §6.4 the TRBs, §6.5 the Event Ring Segment Table. The USB Mass
//! Storage Class Bulk-Only Transport 1.0 §5.1, §5.2 and §5.3; Seagate's SCSI
//! Commands Reference Manual Rev. J §3.6, §3.16, §3.22 and §3.60 for the four
//! command blocks. The RISC-V privileged specification for `mtvec`, `mie` and
//! `mret`, and the RISC-V PLIC specification for the claim/complete register.
//! No emulator source and no operating system's xHCI driver was opened
//! (`ROADMAP.md` §1).

#![cfg(all(
    feature = "machine-xhci-mini",
    feature = "cpu-riscv",
    feature = "dev-riscv",
    feature = "dev-usb-xhci",
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
// The memory map, as `machines/xhci-mini.machine` lays it out
// ---------------------------------------------------------------------------

/// The xHCI register block.
const XHCI: u32 = 0xf000_0000;
/// `CAPLENGTH` (xHCI 1.2 §5.3.1) — where the operational registers start.
const CAPLENGTH: u32 = 0x40;
const OP: u32 = XHCI + CAPLENGTH;
/// `DBOFF` (§5.3.7) and `RTSOFF` (§5.3.8).
const DB: u32 = XHCI + 0x1000;
const RT: u32 = XHCI + 0x2000;
/// Interrupter 0's register set (§5.5, Table 5-35).
const IR0: u32 = RT + 0x20;

const USBCMD: u32 = 0x00;
const USBSTS: u32 = 0x04;
const CRCR: u32 = 0x18;
const DCBAAP: u32 = 0x30;
const CONFIG: u32 = 0x38;
/// The first port register set (§5.4, Table 5-18): ports are one-based.
const PORTSC1: u32 = 0x400;

const IMAN: u32 = 0x00;
const IMOD: u32 = 0x04;
const ERSTSZ: u32 = 0x08;
const ERSTBA: u32 = 0x10;
const ERDP: u32 = 0x18;

/// `USBSTS.EINT` (§5.4.2 bit 3).
const STS_EINT: u32 = 1 << 3;
/// `USBCMD.RS | USBCMD.INTE` (§5.4.1).
const CMD_RUN: u32 = (1 << 0) | (1 << 2);
/// `IMAN.IP | IMAN.IE` (§5.5.2.1).
const IMAN_IP: u32 = 1 << 0;
const IMAN_IE: u32 = 1 << 1;
/// `ERDP.EHB` (§5.5.2.3.3).
const ERDP_EHB: u32 = 1 << 3;

/// `PORTSC` bits this firmware writes (§5.4.8, Table 5-27).
const PORT_PR: u32 = 1 << 4;
const PORT_PP: u32 = 1 << 9;
const PORT_CSC: u32 = 1 << 17;
const PORT_PED: u32 = 1 << 1;
const PORT_PRC: u32 = 1 << 21;

/// The PLIC's register window.
const PLIC: u32 = 0x0c00_0000;
const PLIC_PRIORITY1: u32 = PLIC + 4;
const PLIC_PENDING: u64 = PLIC as u64 + 0x1000;
const PLIC_ENABLE0: u32 = PLIC + 0x2000;
const PLIC_THRESHOLD0: u32 = PLIC + 0x20_0000;
const PLIC_CLAIM0: u32 = PLIC + 0x20_0004;

/// Where RAM starts.
const RAM: u32 = 0x1000_0000;

/// Where the trap handler is assembled, inside the ROM image.
const HANDLER: u32 = 0x400;
/// Where the ring and context template lives in the ROM image.
const TPL_ROM: u32 = 0x1000;
/// …and where the firmware copies it to.
const TPL: u32 = RAM + 0x2000;
/// How much of it there is.
const TPL_BYTES: u32 = 0x1000;

// -- structures the controller writes, which start as zeroed RAM ------------

/// The Output Device Context for slot 1 (§6.2.1). 64-byte aligned.
const DEV_CTX: u32 = RAM + 0x0400;
/// The event ring segment (§6.5). 64-byte aligned, and sixty-four TRBs — more
/// than the run ever posts, so it never wraps and the handler needs no Consumer
/// Cycle State of its own.
const EVT_RING: u32 = RAM + 0x0800;
const EVT_TRBS: u32 = 64;

const GML_BUF: u32 = RAM + 0x0c00;
const INQ_BUF: u32 = RAM + 0x0c40;
const CAP_BUF: u32 = RAM + 0x0c80;
/// Four Command Status Wrappers, one per Bulk-Only command.
const CSW_BUF: u32 = RAM + 0x0cc0;
const RDATA_BUF: u32 = RAM + 0x0e00;
const DESC_BUF: u32 = RAM + 0x1000;

/// The firmware's counters and flags.
const EVENTS: u32 = RAM + 0x1100;
const IRQS: u32 = RAM + 0x1104;
const ERQ_PTR: u32 = RAM + 0x1108;
const SCRATCH: u32 = RAM + 0x1110;
const DONE: u32 = RAM + 0x1120;
const MAGIC: u32 = 0x7c81_0d15;

// -- the template, at its addresses in RAM ----------------------------------

/// The Device Context Base Address Array (§6.1). 64-byte aligned.
const DCBAA: u32 = TPL;
/// The Event Ring Segment Table (§6.5). 64-byte aligned.
const ERST: u32 = TPL + 0x040;
/// The command ring (§4.6.1). 64-byte aligned, as §5.4.5 requires.
const CMD_RING: u32 = TPL + 0x100;
/// The three transfer rings (§4.9.2). 16-byte aligned.
const EP0_RING: u32 = TPL + 0x200;
const EPIN_RING: u32 = TPL + 0x300;
const EPOUT_RING: u32 = TPL + 0x400;
/// The Input Contexts (§6.2.5): one for Address Device, one for Configure
/// Endpoint.
const IN_CTX_A: u32 = TPL + 0x500;
const IN_CTX_C: u32 = TPL + 0x600;
/// The four Command Block Wrappers.
const CBW_INQ: u32 = TPL + 0x700;
const CBW_CAP: u32 = TPL + 0x720;
const CBW_RD: u32 = TPL + 0x740;
const CBW_WR: u32 = TPL + 0x760;
/// The 512 bytes the guest writes to the disk.
const WDATA: u32 = TPL + 0x800;
/// The stage table the firmware walks: triples of (address, value, events to
/// wait for), terminated by a zero address.
const STAGES: u32 = TPL + 0xa00;

/// One context entry is thirty-two bytes, because `HCCPARAMS1.CSZ` is zero
/// (§6.2.2).
const CTX: u32 = 32;
/// One TRB is sixteen bytes (§6.4).
const TRB: u32 = 16;

/// The Device Context Index of each endpoint (§4.5.1: `number * 2 + direction`,
/// and the control pipe is 1). The mass storage device's bulk IN is endpoint 1
/// and its bulk OUT endpoint 2.
const DCI_EP0: u32 = 1;
const DCI_IN: u32 = 3;
const DCI_OUT: u32 = 4;

/// The device slot the firmware assumes, and the test asserts the controller
/// actually allocated. It is the lowest free one, so it is 1.
const SLOT: u32 = 1;
/// …which is also the USB address, because §4.6.5 lets the xHC pick and the
/// Slot ID is unique by construction.
const GUEST_ADDRESS: u8 = SLOT as u8;

/// The doorbell registers (§5.6): register 0 is the command ring, register
/// *n* is device slot *n*.
const DB_CMD: u32 = DB;
const DB_SLOT: u32 = DB + 4 * SLOT;

/// Bytes in a logical block, and how many the disk holds — `param capacity =
/// 1M` and `param block = 512` in the machine file.
const BLOCK: u64 = 512;
const BLOCKS: u64 = 1024 * 1024 / BLOCK;
/// The block the guest reads, and the one it writes.
const READ_LBA: u32 = 9;
const WRITE_LBA: u32 = 33;

/// The bulk endpoints' `wMaxPacketSize` at high speed, and the control pipe's.
const BULK_MPS: u32 = 512;
const EP0_MPS: u32 = 64;

/// The four `dCBWTag` values, one per command, so a CSW that came back attached
/// to the wrong command is visible rather than plausible (BOT §5.2).
const TAG_INQ: u32 = 0x1111_1111;
const TAG_CAP: u32 = 0x2222_2222;
const TAG_RD: u32 = 0x3333_3333;
const TAG_WR: u32 = 0x4444_4444;

/// How many event TRBs a correct run puts on the ring, and how many machine
/// external traps it takes.
///
/// They differ, and the difference is the specification working: four of the
/// doorbells retire two Transfer Descriptors each, and §4.17.2's `ERDP.EHB`
/// blocks a second interrupt until the handler has drained the ring. See the
/// module docs.
const EXPECTED_EVENTS: u32 = 19;
const EXPECTED_TRAPS: u32 = 15;

// ---------------------------------------------------------------------------
// Just enough RV32IMA to write the firmware
// ---------------------------------------------------------------------------

const ZERO: u32 = 0;
const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const S0: u32 = 8;
const S1: u32 = 9;
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

/// `mtvec`, `mie` and `mstatus` (the RISC-V privileged specification, §3.1).
const CSR_MSTATUS: i32 = 0x300;
const CSR_MIE: i32 = 0x304;
const CSR_MTVEC: i32 = 0x305;
/// `mie.MEIE`, machine external interrupt enable — bit 11.
const MEIE: u32 = 1 << 11;
/// `mstatus.MIE` — bit 3.
const MSTATUS_MIE: u32 = 8;

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
/// `bltu rs1, rs2, offset` — the unsigned compare, because these are counters
/// and addresses rather than signed quantities.
fn bltu(rs1: u32, rs2: u32, offset: i32) -> u32 {
    b_type(offset, rs2, rs1, 0b110, OP_BRANCH)
}
fn jal(rd: u32, offset: i32) -> u32 {
    j_type(offset, rd, OP_JAL)
}
/// `csrrw x0, csr, rs1`.
fn csrw(csr: i32, rs1: u32) -> u32 {
    i_type(csr, rs1, 0b001, ZERO, OP_SYSTEM)
}
/// `csrrs x0, csr, rs1`.
fn csrs(csr: i32, rs1: u32) -> u32 {
    i_type(csr, rs1, 0b010, ZERO, OP_SYSTEM)
}
/// `csrrsi x0, csr, uimm`.
fn csrsi(csr: i32, uimm: u32) -> u32 {
    i_type(csr, uimm, 0b110, ZERO, OP_SYSTEM)
}
/// `mret`.
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

/// The main program: arm the interrupt path, copy the template into RAM, then
/// walk the stage table.
fn main_program() -> Vec<u32> {
    let mut code: Vec<u32> = Vec::new();
    let push = |c: &mut Vec<u32>, words: &[u32]| c.extend_from_slice(words);

    // -- the interrupt controller ------------------------------------------
    //
    // A source with priority zero never interrupts, and a context whose
    // threshold is not below the source's priority never sees it; both are
    // wrong by default, which is why all three writes are here.
    push(&mut code, &li(A0, PLIC_PRIORITY1));
    push(&mut code, &li(A1, 1));
    code.push(sw(A1, A0, 0));
    push(&mut code, &li(A0, PLIC_ENABLE0));
    push(&mut code, &li(A1, 1 << 1));
    code.push(sw(A1, A0, 0));
    push(&mut code, &li(A0, PLIC_THRESHOLD0));
    code.push(sw(ZERO, A0, 0));

    // -- the trap handler ---------------------------------------------------
    //
    // `mtvec` in direct mode: the low two bits are the mode field and `HANDLER`
    // is word aligned, so they are zero. The vector is armed before the
    // enables, or an interrupt arriving in between would trap to whatever
    // `mtvec` reset to.
    push(&mut code, &li(A0, HANDLER));
    code.push(csrw(CSR_MTVEC, A0));
    push(&mut code, &li(A0, MEIE));
    code.push(csrs(CSR_MIE, A0));
    code.push(csrsi(CSR_MSTATUS, MSTATUS_MIE));

    // -- copy the template out of ROM and into RAM --------------------------
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

    // -- walk the stage table -----------------------------------------------
    //
    // `s0` is the table pointer and `s1` the address of the event counter; the
    // trap handler touches neither, because it uses `t0`-`t6` and saves the
    // three of those the interrupted program might care about.
    push(&mut code, &li(S0, STAGES));
    push(&mut code, &li(S1, EVENTS));
    let loop_top = code.len();
    code.push(lw(A0, S0, 0)); // the address
    let exit_at = code.len();
    code.push(0); // patched to `beq a0, zero, done`
    code.push(lw(A1, S0, 4)); // the value
    code.push(sw(A1, A0, 0)); // …written, which is the whole stage
    code.push(lw(A2, S0, 8)); // how many events to wait for
    let skip_at = code.len();
    code.push(0); // patched to `beq a2, zero, next`
    let wait = code.len();
    code.push(lw(A3, S1, 0));
    let back = -(((code.len() - wait) * 4) as i32);
    code.push(bltu(A3, A2, back));
    let next = code.len();
    code[skip_at] = beq(A2, ZERO, ((next - skip_at) * 4) as i32);
    code.push(addi(S0, S0, 12));
    let back = -(((code.len() - loop_top) * 4) as i32);
    code.push(jal(ZERO, back));

    let done = code.len();
    code[exit_at] = beq(A0, ZERO, ((done - exit_at) * 4) as i32);

    push(&mut code, &li(T1, DONE));
    push(&mut code, &li(A0, MAGIC));
    code.push(sw(A0, T1, 0));
    code.push(jal(ZERO, 0));
    code
}

/// The machine external interrupt handler.
///
/// Drains every event TRB whose Cycle bit says it is valid — the ring is sixty-
/// four entries and the run posts nineteen, so it never wraps and the handler
/// needs no Consumer Cycle State of its own — and then acknowledges in the
/// order xHCI 1.2 fixes: `USBSTS.EINT` (§5.4.2), `ERDP` with `EHB` (§5.5.2.3.3),
/// `IMAN.IP` (§4.17.3), and only then the PLIC claim.
fn trap_handler() -> Vec<u32> {
    let mut code: Vec<u32> = Vec::new();
    let push = |c: &mut Vec<u32>, words: &[u32]| c.extend_from_slice(words);

    // `t0`-`t2` are saved to a fixed scratch word rather than a stack: a trap
    // cannot nest, because `mstatus.MIE` is cleared on entry.
    push(&mut code, &li(T3, SCRATCH));
    code.push(sw(T0, T3, 0));
    code.push(sw(T1, T3, 4));
    code.push(sw(T2, T3, 8));

    push(&mut code, &li(T4, PLIC_CLAIM0));
    code.push(lw(T5, T4, 0)); // claim

    push(&mut code, &li(T6, ERQ_PTR));
    code.push(lw(T0, T6, 0)); // where we had read to
    push(&mut code, &li(T1, EVENTS));
    code.push(lw(T2, T1, 0)); // how many we had seen

    let drain = code.len();
    code.push(lw(T3, T0, 12)); // the event TRB's fourth dword
    code.push(andi(T3, T3, 1)); // …and its Cycle bit (§4.9.4)
    let exit_at = code.len();
    code.push(0); // patched to `beq t3, zero, done`
    code.push(addi(T2, T2, 1));
    code.push(addi(T0, T0, 16));
    let back = -(((code.len() - drain) * 4) as i32);
    code.push(jal(ZERO, back));
    let done = code.len();
    code[exit_at] = beq(T3, ZERO, ((done - exit_at) * 4) as i32);

    code.push(sw(T2, T1, 0)); // the event counter
    code.push(sw(T0, T6, 0)); // and where we have now read to

    // 1. `USBSTS.EINT` first (§5.4.2 bit 3).
    push(&mut code, &li(T1, OP));
    push(&mut code, &li(T2, STS_EINT));
    code.push(sw(T2, T1, USBSTS as i32));
    // 2. the Event Ring Dequeue Pointer, with `EHB` written to clear it
    //    (§5.5.2.3.3). The pointer is sixteen-byte aligned, so setting bit 3
    //    cannot disturb it.
    push(&mut code, &li(T1, IR0));
    code.push(ori(T2, T0, ERDP_EHB as i32));
    code.push(sw(T2, T1, ERDP as i32));
    code.push(sw(ZERO, T1, ERDP as i32 + 4));
    // 3. `IMAN.IP`, which is what actually drops the line (§4.17.3).
    push(&mut code, &li(T2, IMAN_IP | IMAN_IE));
    code.push(sw(T2, T1, IMAN as i32));
    // 4. …and only now the PLIC claim.
    code.push(sw(T5, T4, 0));

    push(&mut code, &li(T1, IRQS));
    code.push(lw(T2, T1, 0));
    code.push(addi(T2, T2, 1));
    code.push(sw(T2, T1, 0));

    push(&mut code, &li(T3, SCRATCH));
    code.push(lw(T0, T3, 0));
    code.push(lw(T1, T3, 4));
    code.push(lw(T2, T3, 8));
    code.push(MRET);
    code
}

/// The ROM image: the main program at zero, the handler at [`HANDLER`], the
/// template at [`TPL_ROM`].
fn firmware() -> Vec<u8> {
    let mut image = Vec::new();
    for word in main_program() {
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
    image.extend_from_slice(&template());
    image
}

// ---------------------------------------------------------------------------
// The rings, the contexts and the stage table
// ---------------------------------------------------------------------------

/// TRB type identifiers (§6.4.6, Table 6-91).
mod trb {
    pub(crate) const NORMAL: u32 = 1;
    pub(crate) const SETUP_STAGE: u32 = 2;
    pub(crate) const DATA_STAGE: u32 = 3;
    pub(crate) const STATUS_STAGE: u32 = 4;
    pub(crate) const ENABLE_SLOT: u32 = 9;
    pub(crate) const ADDRESS_DEVICE: u32 = 11;
    pub(crate) const CONFIGURE_ENDPOINT: u32 = 12;
    pub(crate) const TRANSFER_EVENT: u32 = 32;
    pub(crate) const COMMAND_COMPLETION_EVENT: u32 = 33;
    pub(crate) const PORT_STATUS_CHANGE_EVENT: u32 = 34;
}

/// TRB control flags (§6.4.1).
const TRB_ISP: u32 = 1 << 2;
const TRB_IOC: u32 = 1 << 5;
const TRB_IDT: u32 = 1 << 6;
const TRB_DIR: u32 = 1 << 16;

/// The dword-3 type field's position.
fn kind(t: u32) -> u32 {
    t << 10
}

/// Builds the template blob and the stage table together, so the two cannot
/// drift apart: every TRB placed on a ring is placed with its **Cycle bit
/// clear**, and the stage that hands it to the controller is emitted from the
/// same call.
struct Build {
    blob: Vec<u8>,
    stages: Vec<(u32, u32, u32)>,
    /// How many events the run has asked for so far — the number a stage waits
    /// on is a running total.
    events: u32,
}

impl Build {
    fn new() -> Build {
        Build {
            blob: vec![0u8; TPL_BYTES as usize],
            stages: Vec::new(),
            events: 0,
        }
    }

    /// Store dwords into the template at their guest address.
    fn put(&mut self, addr: u32, words: &[u32]) {
        let offset = (addr - TPL) as usize;
        for (i, word) in words.iter().enumerate() {
            self.blob[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
    }

    /// The same, for the byte-granular structures.
    fn put_bytes(&mut self, addr: u32, bytes: &[u8]) {
        let offset = (addr - TPL) as usize;
        self.blob[offset..offset + bytes.len()].copy_from_slice(bytes);
    }

    /// A plain stage: write `value` to `addr` and carry on.
    fn write(&mut self, addr: u32, value: u32) {
        let events = self.events;
        self.stages.push((addr, value, events));
    }

    /// A stage that waits: write `value`, then spin until `count` more events
    /// have arrived.
    fn write_wait(&mut self, addr: u32, value: u32, count: u32) {
        self.events += count;
        let events = self.events;
        self.stages.push((addr, value, events));
    }

    /// Place a TRB on a ring with its Cycle bit clear, and emit the stage that
    /// sets it — which is how software hands a TRB to the controller (§4.9).
    fn trb(&mut self, addr: u32, t: [u32; 4]) {
        self.put(addr, &[t[0], t[1], t[2], t[3] & !1]);
        self.write(addr + 12, t[3] | 1);
    }
}

/// The 512 bytes the guest writes to the disk. Deliberately unlike the pattern
/// the medium was stamped with, so "the write reached the medium" cannot be
/// satisfied by the block that was already there.
fn write_payload() -> Vec<u8> {
    (0..BLOCK)
        .map(|i| (i as u8).wrapping_mul(11) ^ 0x5c)
        .collect()
}

/// A Command Block Wrapper: thirty-one bytes, little-endian (BOT §5.1).
fn cbw(tag: u32, data_length: u32, data_in: bool, cdb: &[u8]) -> [u8; 31] {
    let mut out = [0u8; 31];
    out[0..4].copy_from_slice(&0x4342_5355u32.to_le_bytes());
    out[4..8].copy_from_slice(&tag.to_le_bytes());
    out[8..12].copy_from_slice(&data_length.to_le_bytes());
    out[12] = if data_in { 0x80 } else { 0x00 };
    out[13] = 0;
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

/// The whole template: the array, the tables, the rings, the contexts, the
/// command blocks — and the stage table that drives all of them.
fn template() -> Vec<u8> {
    let mut b = Build::new();

    // -- the Device Context Base Address Array (§6.1) -----------------------
    //
    // Entry 0 is the Scratchpad Buffer Array pointer, and this controller asks
    // for no scratchpad buffers, so it stays zero.
    b.put(DCBAA + 8, &[DEV_CTX, 0]);

    // -- the Event Ring Segment Table (§6.5) --------------------------------
    b.put(ERST, &[EVT_RING, 0, EVT_TRBS, 0]);

    // -- the Input Context for Address Device (§4.6.5, §6.2.5) --------------
    //
    // A0 and A1 set and nothing else; a Slot Context naming the root hub port
    // and the speed; an Endpoint 0 Context naming the control transfer ring.
    b.put(IN_CTX_A + 4, &[0x3]);
    b.put(
        IN_CTX_A + CTX,
        &[
            // Context Entries = 1, Speed = 3 (high, per Table 7-13).
            (1 << 27) | (3 << 20),
            // Root Hub Port Number 1 (§6.2.2, Table 6-5).
            1 << 16,
        ],
    );
    b.put(
        IN_CTX_A + 2 * CTX,
        &[
            0,
            // CErr = 3, EP Type = 4 (control), Max Packet Size (§6.2.3).
            (3 << 1) | (4 << 3) | (EP0_MPS << 16),
            EP0_RING | 1, // TR Dequeue Pointer, DCS = 1
            0,
            // §6.2.3: software shall set Average TRB Length to 8 for control
            // endpoints.
            8,
        ],
    );

    // -- the Input Context for Configure Endpoint (§4.6.6) ------------------
    b.put(IN_CTX_C + 4, &[1 | (1 << DCI_IN) | (1 << DCI_OUT)]);
    b.put(IN_CTX_C + CTX, &[(DCI_OUT << 27) | (3 << 20), 1 << 16]);
    b.put(
        IN_CTX_C + (DCI_IN + 1) * CTX,
        &[
            0,
            // EP Type = 6: Bulk In.
            (3 << 1) | (6 << 3) | (BULK_MPS << 16),
            EPIN_RING | 1,
            0,
            BULK_MPS,
        ],
    );
    b.put(
        IN_CTX_C + (DCI_OUT + 1) * CTX,
        &[
            0,
            // EP Type = 2: Bulk Out.
            (3 << 1) | (2 << 3) | (BULK_MPS << 16),
            EPOUT_RING | 1,
            0,
            BULK_MPS,
        ],
    );

    // -- the four Command Block Wrappers ------------------------------------
    b.put_bytes(CBW_INQ, &cbw(TAG_INQ, 36, true, &[0x12, 0, 0, 0, 36, 0]));
    b.put_bytes(
        CBW_CAP,
        &cbw(TAG_CAP, 8, true, &[0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
    );
    b.put_bytes(
        CBW_RD,
        &cbw(TAG_RD, BLOCK as u32, true, &read10(READ_LBA, 1)),
    );
    b.put_bytes(
        CBW_WR,
        &cbw(TAG_WR, BLOCK as u32, false, &write10(WRITE_LBA, 1)),
    );
    b.put_bytes(WDATA, &write_payload());

    // =======================================================================
    // The stage table
    // =======================================================================

    // -- §4.2's initialisation sequence -------------------------------------
    b.write(ERQ_PTR, EVT_RING);
    // §5.4.7: one device slot enabled, which is also how software tells the xHC
    // that a driver has loaded.
    b.write(OP + CONFIG, 1);
    b.write(OP + DCBAAP, DCBAA);
    b.write(OP + DCBAAP + 4, 0);
    // §5.4.5: the pointer and the Ring Cycle State the first fetch uses.
    b.write(OP + CRCR, CMD_RING | 1);
    b.write(OP + CRCR + 4, 0);
    // §4.9.4: ERSTSZ and ERDP before ERSTBA, because writing ERSTBA is what
    // puts the Event Ring State Machine in the Start state.
    b.write(IR0 + ERSTSZ, 1);
    b.write(IR0 + ERDP, EVT_RING);
    b.write(IR0 + ERDP + 4, 0);
    b.write(IR0 + ERSTBA, ERST);
    b.write(IR0 + ERSTBA + 4, 0);
    // §5.5.2.2: zero disables throttling, so an event interrupts at once.
    b.write(IR0 + IMOD, 0);
    b.write(IR0 + IMAN, IMAN_IE);
    b.write(OP + USBCMD, CMD_RUN);

    // -- the root port ------------------------------------------------------
    //
    // The device is already attached and `PORTSC.CSC` already says so, so the
    // first thing to do is acknowledge it: §4.19.2 makes a Port Status Change
    // Event the *rising* edge of the OR of the change bits, and leaving one set
    // would mean the reset never produced an edge. Then `PR`, which is what
    // transitions a USB2 port from Polling to Enabled (§5.4.8).
    b.write(XHCI + CAPLENGTH + PORTSC1, PORT_PP | PORT_CSC);
    b.write_wait(XHCI + CAPLENGTH + PORTSC1, PORT_PP | PORT_PR, 1);
    b.write(XHCI + CAPLENGTH + PORTSC1, PORT_PP | PORT_PRC);

    // -- Enable Slot and Address Device (§4.6.3, §4.6.5) --------------------
    b.trb(CMD_RING, [0, 0, 0, kind(trb::ENABLE_SLOT)]);
    b.write_wait(DB_CMD, 0, 1);
    b.trb(
        CMD_RING + TRB,
        [IN_CTX_A, 0, 0, kind(trb::ADDRESS_DEVICE) | (SLOT << 24)],
    );
    b.write_wait(DB_CMD, 0, 1);

    // -- three control transfers on the default pipe (§4.11.2.2) ------------
    //
    // Each is a Setup Stage TD, an optional Data Stage TD and a Status Stage
    // TD, and §6.4.1.2 says only the Status Stage TRB should carry `IOC` — so
    // each transfer is one event however many TRBs it took.
    let mut ep0 = EP0_RING;
    // SET_CONFIGURATION(1): no data stage, so TRT = 0 and the status stage is
    // an IN (§4.11.2.2).
    b.trb(ep0, setup_trb(0x00, 9, 1, 0, 0, 0));
    b.trb(
        ep0 + TRB,
        [0, 0, 0, kind(trb::STATUS_STAGE) | TRB_IOC | TRB_DIR],
    );
    ep0 += 2 * TRB;
    b.write_wait(DB_SLOT, DCI_EP0, 1);

    // GET_MAX_LUN (BOT §3.2): class, interface, device to host, one byte.
    b.trb(ep0, setup_trb(0xa1, 0xfe, 0, 0, 1, 3));
    b.trb(ep0 + TRB, [GML_BUF, 0, 1, kind(trb::DATA_STAGE) | TRB_DIR]);
    b.trb(ep0 + 2 * TRB, [0, 0, 0, kind(trb::STATUS_STAGE) | TRB_IOC]);
    ep0 += 3 * TRB;
    b.write_wait(DB_SLOT, DCI_EP0, 1);

    // GET_DESCRIPTOR(Device, 18) — the request §4.6.5 says software should make
    // first, and the one that proves a data stage of more than a packet's worth
    // of interest works.
    b.trb(ep0, setup_trb(0x80, 6, 0x0100, 0, 18, 3));
    b.trb(
        ep0 + TRB,
        [DESC_BUF, 0, 18, kind(trb::DATA_STAGE) | TRB_DIR],
    );
    b.trb(ep0 + 2 * TRB, [0, 0, 0, kind(trb::STATUS_STAGE) | TRB_IOC]);
    b.write_wait(DB_SLOT, DCI_EP0, 1);

    // -- Configure Endpoint (§4.6.6) ----------------------------------------
    b.trb(
        CMD_RING + 2 * TRB,
        [IN_CTX_C, 0, 0, kind(trb::CONFIGURE_ENDPOINT) | (SLOT << 24)],
    );
    b.write_wait(DB_CMD, 0, 1);

    // -- the four Bulk-Only commands (BOT §5.3) -----------------------------
    //
    // A CBW out on the bulk-out ring, then the data and the status wrapper in
    // on the bulk-in ring. Both of the latter are armed before the doorbell, so
    // one doorbell retires two Transfer Descriptors and posts two events — for
    // which §4.17.2's `EHB` allows exactly one interrupt.
    let mut epout = EPOUT_RING;
    let mut epin = EPIN_RING;

    for (cbw_at, buf, len, csw) in [
        (CBW_INQ, INQ_BUF, 36u32, CSW_BUF),
        (CBW_CAP, CAP_BUF, 8, CSW_BUF + 0x20),
        (CBW_RD, RDATA_BUF, BLOCK as u32, CSW_BUF + 0x40),
    ] {
        b.trb(epout, [cbw_at, 0, 31, kind(trb::NORMAL) | TRB_IOC]);
        epout += TRB;
        b.write_wait(DB_SLOT, DCI_OUT, 1);
        // `ISP` as well as `IOC`, so a device that ended the transfer early
        // would say so rather than looking like a success (§6.4.1.1 bit 2).
        b.trb(epin, [buf, 0, len, kind(trb::NORMAL) | TRB_IOC | TRB_ISP]);
        b.trb(epin + TRB, [csw, 0, 13, kind(trb::NORMAL) | TRB_IOC]);
        epin += 2 * TRB;
        b.write_wait(DB_SLOT, DCI_IN, 2);
    }

    // `WRITE (10)`: the CBW and its data both go out on the bulk-out ring, so
    // one doorbell there retires two TDs, and only the status wrapper comes
    // back in.
    b.trb(epout, [CBW_WR, 0, 31, kind(trb::NORMAL) | TRB_IOC]);
    b.trb(
        epout + TRB,
        [WDATA, 0, BLOCK as u32, kind(trb::NORMAL) | TRB_IOC],
    );
    b.write_wait(DB_SLOT, DCI_OUT, 2);
    b.trb(epin, [CSW_BUF + 0x60, 0, 13, kind(trb::NORMAL) | TRB_IOC]);
    b.write_wait(DB_SLOT, DCI_IN, 1);

    // -- the terminator -----------------------------------------------------
    assert_eq!(
        b.events, EXPECTED_EVENTS,
        "the stage table and EXPECTED_EVENTS disagree"
    );
    let mut words = Vec::new();
    for (addr, value, wait) in &b.stages {
        words.push(*addr);
        words.push(*value);
        words.push(*wait);
    }
    words.push(0);
    words.push(0);
    words.push(0);
    assert!(
        (STAGES - TPL) as usize + words.len() * 4 <= TPL_BYTES as usize,
        "the stage table does not fit in the template"
    );
    b.put(STAGES, &words);
    b.blob
}

/// A Setup Stage TRB (§6.4.1.2.1): the eight bytes of the setup packet carried
/// immediately, and the Transfer Type in bits 17:16.
fn setup_trb(
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
    trt: u32,
) -> [u32; 4] {
    [
        u32::from(request_type) | (u32::from(request) << 8) | (u32::from(value) << 16),
        u32::from(index) | (u32::from(length) << 16),
        8,
        kind(trb::SETUP_STAGE) | TRB_IDT | (trt << 16),
    ]
}

// ---------------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------------

/// A recognisable block: every byte says which block it came from and where in
/// it that byte sits, so a read of the wrong LBA and a read of the right LBA at
/// the wrong offset look different.
fn stamp(lba: u64) -> Vec<u8> {
    (0..BLOCK)
        .map(|i| (lba as u8).wrapping_mul(31).wrapping_add(i as u8))
        .collect()
}

/// The board from the catalog, and the medium the *host* installed under its
/// media slot.
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
    let entry = &catalog::XHCI_MINI;
    let mut machine = match rsemu::machine::build(entry.name, entry.source, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, store)
}

/// Run until the firmware's flag holds the magic word, or give up.
fn run_until_done(machine: &mut Machine) -> bool {
    for _ in 0..400 {
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

/// What the medium itself holds at `lba` — the only assertion that proves a
/// byte moved rather than being echoed.
fn on_medium(store: &RamStore, lba: u64) -> Vec<u8> {
    let mut got = vec![0u8; BLOCK as usize];
    Medium::read_at(store, lba * BLOCK, &mut got).expect("the medium reads");
    got
}

/// One Command Status Wrapper, as the guest received it (BOT §5.2).
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

/// The event ring, as TRBs.
fn events(machine: &Machine) -> Vec<[u32; 4]> {
    let count = peek(machine, EVENTS) as usize;
    (0..count)
        .map(|i| {
            let at = EVT_RING + (i as u32) * TRB;
            [
                peek(machine, at),
                peek(machine, at + 4),
                peek(machine, at + 8),
                peek(machine, at + 12),
            ]
        })
        .collect()
}

/// A TRB's type field (§6.4).
fn trb_kind(trb: &[u32; 4]) -> u32 {
    (trb[3] >> 10) & 0x3f
}

/// A TRB's completion code (§6.4.5).
fn trb_code(trb: &[u32; 4]) -> u32 {
    trb[2] >> 24
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_reads_a_sector_over_xhci_and_it_is_the_sector_on_the_medium() {
    let (mut machine, store) = board();
    assert!(
        run_until_done(&mut machine),
        "the firmware never finished its stage table (events so far: {}, traps: {})",
        peek(&machine, EVENTS),
        peek(&machine, IRQS)
    );

    let got = peek_bytes(&machine, RDATA_BUF, BLOCK as usize);
    // Against the **medium**, which the guest cannot reach: this is the whole
    // claim, and it is not satisfied by a device with a buffer.
    assert_eq!(
        got,
        on_medium(&store, u64::from(READ_LBA)),
        "the block in guest RAM is not the block on the medium"
    );
    // And it is the right block rather than a neighbour, which is what catches
    // a length computed in blocks and applied in bytes.
    assert_ne!(got, on_medium(&store, u64::from(READ_LBA) + 1));
    assert_ne!(got, on_medium(&store, u64::from(READ_LBA) - 1));

    let (signature, tag, residue, status) = csw(&machine, 2);
    assert_eq!(signature, CSW_SIGNATURE);
    assert_eq!(tag, TAG_RD, "the CSW must echo its own CBW's tag");
    assert_eq!(residue, 0);
    assert_eq!(status, 0, "Command Passed");
}

#[test]
fn a_guest_writes_a_sector_over_xhci_and_it_reaches_the_medium() {
    let (mut machine, store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    let want = write_payload();
    assert_eq!(
        on_medium(&store, u64::from(WRITE_LBA)),
        want,
        "the write did not reach the medium"
    );
    // The neighbours are exactly as the image left them.
    assert_eq!(
        on_medium(&store, u64::from(WRITE_LBA) - 1),
        stamp(u64::from(WRITE_LBA) - 1)
    );
    assert_eq!(
        on_medium(&store, u64::from(WRITE_LBA) + 1),
        stamp(u64::from(WRITE_LBA) + 1)
    );

    let (signature, tag, residue, status) = csw(&machine, 3);
    assert_eq!(signature, CSW_SIGNATURE);
    assert_eq!(tag, TAG_WR);
    assert_eq!(residue, 0);
    assert_eq!(status, 0);
}

#[test]
fn the_guest_enumerated_the_disk_over_a_command_ring_and_a_default_pipe() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    // The port really did come up: §5.4.8's `PED`, and the Protocol Speed ID
    // of Table 7-13 saying high speed.
    let sc = peek(&machine, XHCI + CAPLENGTH + PORTSC1);
    assert_eq!(sc & PORT_PED, PORT_PED, "the port is enabled");
    assert_eq!((sc >> 10) & 0xf, 3, "PORTSC Port Speed: high");

    // §6.2.2 Table 6-7: the controller wrote the Output Slot Context, and it
    // says the device is Addressed at the address the xHC chose.
    let dw3 = peek(&machine, DEV_CTX + 12);
    assert_eq!(dw3 & 0xff, u32::from(GUEST_ADDRESS), "USB Device Address");
    assert_eq!(dw3 >> 27, 3, "Slot State: Configured");

    // The device descriptor the guest pulled over the default pipe — which is
    // the request §4.6.5 says software should make first.
    let desc = peek_bytes(&machine, DESC_BUF, 18);
    assert_eq!(desc[0], 18, "bLength");
    assert_eq!(desc[1], 1, "bDescriptorType: DEVICE");
    assert_eq!(
        desc[7], 64,
        "bMaxPacketSize0 at high speed (USB 2.0 §5.5.3)"
    );

    // BOT §3.2: one logical unit, so the highest LUN is zero.
    assert_eq!(peek(&machine, GML_BUF) & 0xff, 0, "GET_MAX_LUN");

    // The standard INQUIRY data (Seagate §3.6.2, table 59).
    let inquiry = peek_bytes(&machine, INQ_BUF, 36);
    assert_eq!(inquiry[0], 0x00, "a direct-access block device, connected");
    assert_eq!(inquiry[3] & 0x0f, 2, "RESPONSE DATA FORMAT");
    assert_eq!(&inquiry[8..16], b"RSEMU   ");
    assert_eq!(&inquiry[16..32], b"USB DISK        ");
    let (_, tag, residue, status) = csw(&machine, 0);
    assert_eq!((tag, residue, status), (TAG_INQ, 0, 0));

    // READ CAPACITY (10) (Seagate §3.22.2, table 120): the *last* block and the
    // block length. Off by one here and every host reads one block past the end.
    let capacity = peek_bytes(&machine, CAP_BUF, 8);
    assert_eq!(
        u32::from_be_bytes([capacity[0], capacity[1], capacity[2], capacity[3]]),
        (BLOCKS - 1) as u32
    );
    assert_eq!(
        u32::from_be_bytes([capacity[4], capacity[5], capacity[6], capacity[7]]),
        BLOCK as u32
    );
    let (_, tag, residue, status) = csw(&machine, 1);
    assert_eq!((tag, residue, status), (TAG_CAP, 0, 0));
}

#[test]
fn every_event_the_controller_posted_is_the_one_the_specification_says() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    let events = events(&machine);
    assert_eq!(events.len(), EXPECTED_EVENTS as usize);
    assert!(
        events.len() < EVT_TRBS as usize,
        "the event ring must not wrap, or the handler's cycle-bit walk is wrong"
    );

    // The first is the port coming up (§6.4.2.3), and it names port 1 — ports
    // are one-based in a Port Status Change Event and zero-based on the fabric,
    // which is the classic place to be off by one.
    assert_eq!(trb_kind(&events[0]), trb::PORT_STATUS_CHANGE_EVENT);
    assert_eq!(events[0][0] >> 24, 1, "Port ID");

    // Then Enable Slot and Address Device, both Command Completion Events
    // naming the slot the controller allocated (§6.4.2.2).
    for event in &events[1..=2] {
        assert_eq!(trb_kind(event), trb::COMMAND_COMPLETION_EVENT);
        assert_eq!(event[3] >> 24, SLOT, "Slot ID");
    }
    assert_eq!(
        events[1][0], CMD_RING,
        "the event points at the command TRB that caused it"
    );
    assert_eq!(events[2][0], CMD_RING + TRB);

    // Every event succeeded, and every transfer event names the right slot and
    // Device Context Index (§6.4.2.1).
    for (i, event) in events.iter().enumerate() {
        assert_eq!(
            trb_code(event),
            1,
            "event {i} did not complete with Success: {event:08x?}"
        );
        if trb_kind(event) == trb::TRANSFER_EVENT {
            assert_eq!(event[3] >> 24, SLOT, "event {i}: Slot ID");
            let dci = (event[3] >> 16) & 0x1f;
            assert!(
                dci == DCI_EP0 || dci == DCI_IN || dci == DCI_OUT,
                "event {i}: Endpoint ID {dci}"
            );
            assert_eq!(event[2] & 0xff_ffff, 0, "event {i}: residual");
        }
    }

    // Six commands and control transfers, then twelve bulk events.
    let transfers = events
        .iter()
        .filter(|e| trb_kind(e) == trb::TRANSFER_EVENT)
        .count();
    assert_eq!(transfers, 15);
}

#[test]
fn the_completion_interrupts_travelled_the_wire_and_were_acknowledged() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    // The guest took real machine external traps: an event TRB became
    // `IMAN.IP`, which became a level on `xhci.irq`, which became a pending
    // source in the PLIC, which became `meip` on the hart.
    assert_eq!(
        peek(&machine, IRQS),
        EXPECTED_TRAPS,
        "the exact number of interrupts, no more and no less"
    );
    assert_eq!(peek(&machine, EVENTS), EXPECTED_EVENTS);
    // And the two numbers differ, which is §4.17.2's `EHB` doing its job: four
    // doorbells retired two Transfer Descriptors each and the guest saw one
    // interrupt for each pair.
    assert_eq!(EXPECTED_EVENTS - EXPECTED_TRAPS, 4);

    // The acknowledgement landed in all three places the specification names.
    let space = machine.space("mem").expect("the board has one");
    let usbsts = space
        .read(u64::from(OP + USBSTS), Width::U32, MemAttrs::DEBUG)
        .expect("the register block") as u32;
    assert_eq!(usbsts & STS_EINT, 0, "USBSTS.EINT is still asserted");
    let iman = space
        .read(u64::from(IR0 + IMAN), Width::U32, MemAttrs::DEBUG)
        .expect("the register block") as u32;
    assert_eq!(iman & IMAN_IP, 0, "IMAN.IP is still asserted");
    assert_eq!(iman & IMAN_IE, IMAN_IE, "…and IE survived clearing IP");
    let erdp = space
        .read(u64::from(IR0 + ERDP), Width::U32, MemAttrs::DEBUG)
        .expect("the register block") as u32;
    assert_eq!(erdp & ERDP_EHB, 0, "ERDP.EHB is still set");
    assert_eq!(
        erdp & !0xf,
        EVT_RING + EXPECTED_EVENTS * TRB,
        "the dequeue pointer is where software stopped reading (§4.9.4)"
    );

    // …and the interrupt controller is clear, which is what the *order* buys:
    // completing the claim before writing `IMAN.IP` would have left the level
    // asserted and re-latched the source.
    let pending = space
        .read(PLIC_PENDING, Width::U32, MemAttrs::DEBUG)
        .expect("the PLIC") as u32;
    assert_eq!(pending & 0b10, 0, "the PLIC still holds source 1 pending");
}

#[test]
fn the_controller_is_where_the_machine_file_put_it() {
    let (machine, _store) = board();
    let space = machine.space("mem").expect("the board has one");
    // §5.3.1 and §5.3.2: `CAPLENGTH` is a byte at offset zero and `HCIVERSION`
    // a halfword at two, and every driver reads them at those widths — so this
    // is also the assertion that `param xhci_at` is what decides where the
    // block lives.
    assert_eq!(
        space
            .read(u64::from(XHCI), Width::U8, MemAttrs::DEBUG)
            .expect("CAPLENGTH"),
        u64::from(CAPLENGTH)
    );
    assert_eq!(
        space
            .read(u64::from(XHCI) + 2, Width::U16, MemAttrs::DEBUG)
            .expect("HCIVERSION"),
        0x0100
    );
    // §5.3.7, §5.3.8: and the doorbell and runtime offsets a driver has to read
    // rather than assume.
    assert_eq!(
        space
            .read(u64::from(XHCI) + 0x14, Width::U32, MemAttrs::DEBUG)
            .expect("DBOFF") as u32,
        DB - XHCI
    );
    assert_eq!(
        space
            .read(u64::from(XHCI) + 0x18, Width::U32, MemAttrs::DEBUG)
            .expect("RTSOFF") as u32,
        RT - XHCI
    );
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
