//! USB mass storage, end to end, with a **guest** driving it and a **medium**
//! behind it.
//!
//! `src/dev/usb/msd/tests.rs` says "the device answered the transaction the
//! fabric handed it". This says the thing that is actually worth claiming about
//! a storage device:
//!
//! > A program running on an emulated CPU builds queue heads and transfer
//! > descriptors in guest RAM, points an EHCI at them, enumerates a USB disk,
//! > pushes a **Command Block Wrapper** out of a bulk endpoint and pulls a
//! > sector and a **Command Status Wrapper** back in — and the bytes that
//! > arrived are the bytes **on the medium**, which this test holds a second
//! > handle to and the guest cannot reach.
//!
//! Nothing here calls into the device or the controller. Everything it does, it
//! does by executing RV32 instructions on `machines/usb-mini.machine`.
//!
//! # What the firmware does
//!
//! ```text
//!   PLIC priority/enable/threshold       source 1 is the controller
//!   mtvec, mie.MEIE, mstatus.MIE         a real trap handler, not a poll
//!   USBINTR = USBINT                     let the interrupt line exist at all
//!   CONFIGFLAG = 1                       these root ports are mine
//!   PORTSC |= PORT_RESET, then release   the port enables on the release
//!   copy the schedule template into RAM
//!   for each of eleven stages:
//!       USBCMD = 0                       stop before repointing the list
//!       ASYNCLISTADDR = this stage's QH
//!       USBCMD = RS | ASE                run
//!       poll the last qTD's token until its Active bit clears
//!   store a magic word
//! ```
//!
//! The eleven stages are `SET_ADDRESS(3)`, `SET_CONFIGURATION(1)`,
//! `GET_MAX_LUN`, and then four Bulk-Only commands as their CBW/data/CSW
//! triples: `INQUIRY`, `READ CAPACITY (10)`, `READ (10)` and `WRITE (10)`.
//! Serialising them by polling a transfer descriptor's Active bit is what a
//! driver does — BOT §3.4 forbids a second CBW before the first CSW — and it is
//! also what makes a failure name the stage that failed.
//!
//! # The interrupt is not polled
//!
//! Every stage's last descriptor carries `IOC`, so the controller sets
//! `USBSTS.USBINT` and — because `USBINTR` enables it — pulls its interrupt line
//! down. That line is a **wire** into a **PLIC**, whose `meip0` is a wire into
//! the hart's external-interrupt pin, and the firmware takes a real machine
//! external trap.
//!
//! **The acknowledgement is two writes in a required order**, and the test
//! counts traps rather than merely observing one, because that is what makes the
//! order visible. EHCI's line is level triggered and is the AND of `USBSTS` with
//! `USBINTR` (§2.3.2), so the handler writes one to `USBSTS.USBINT` *first* and
//! completes the PLIC claim second. Completing the claim while the level is
//! still asserted makes the PLIC re-latch the source, and the guest takes a
//! second, spurious trap for every completion — measured, not assumed:
//! **eleven** traps in the right order and **twenty-two** in the wrong one. So
//! `the_completion_interrupt_travelled_the_wire_and_was_acknowledged` asserts
//! the count, and a handler that acknowledged in the other order fails it.
//!
//! # Sources
//!
//! EHCI 1.0 §2.3 (the operational registers), §3.5 (the queue element transfer
//! descriptor), §3.6 (the queue head); the USB Mass Storage Class Bulk-Only
//! Transport 1.0 §5.1, §5.2 and §5.3; Seagate's SCSI Commands Reference Manual
//! Rev. J §3.6, §3.16, §3.22 and §3.60 for the four command blocks. The RISC-V
//! privileged specification for `mtvec`, `mie` and `mret`, and the RISC-V PLIC
//! specification for the claim/complete register.

#![cfg(all(
    feature = "machine-usb-mini",
    feature = "cpu-riscv",
    feature = "dev-riscv",
    feature = "dev-usb-ehci",
    feature = "dev-usb-msd"
))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::{MemAttrs, RamStore};
use rsemu::core::value::Width;
use rsemu::dev::ata::Medium;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// The memory map, as `machines/usb-mini.machine` lays it out
// ---------------------------------------------------------------------------

/// The EHCI register block.
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

/// The PLIC's register window.
const PLIC: u32 = 0x0c00_0000;
/// `priority[1]`: source 1 is the controller.
const PLIC_PRIORITY1: u32 = PLIC + 4;
/// The interrupt-pending bitmap.
const PLIC_PENDING: u64 = PLIC as u64 + 0x1000;
/// Context 0's enable bits — hart 0, machine mode, this board's only context.
const PLIC_ENABLE0: u32 = PLIC + 0x2000;
/// Context 0's priority threshold.
const PLIC_THRESHOLD0: u32 = PLIC + 0x20_0000;
/// Context 0's claim/complete register.
const PLIC_CLAIM0: u32 = PLIC + 0x20_0004;

/// Where RAM starts.
const RAM: u32 = 0x1000_0000;

/// Where the trap handler is assembled, inside the ROM image.
const HANDLER: u32 = 0x400;
/// Where the schedule template lives in the ROM image.
const TPL_ROM: u32 = 0x800;
/// …and where the firmware copies it to.
const TPL: u32 = RAM + 0x2000;

// The buffers the *controller* writes into. Outside the template, because they
// hold what came back rather than what went out.
const GML_BUF: u32 = RAM + 0x1000;
const INQ_BUF: u32 = RAM + 0x1040;
const CAP_BUF: u32 = RAM + 0x1080;
/// Four Command Status Wrappers, one per Bulk-Only command.
const CSW_BUF: u32 = RAM + 0x10c0;
const RDATA_BUF: u32 = RAM + 0x1200;
/// The firmware's progress flag, and the count of traps it took.
const DONE: u32 = RAM + 0x1800;
const IRQS: u32 = RAM + 0x1804;
const MAGIC: u32 = 0x5c51_0d15;

// The template's contents, at their addresses in RAM. Queue heads are 48 bytes
// and descriptors 32; everything is 32-byte aligned, as EHCI 1.0 §3.5 and §3.6
// require.
const QH_SA: u32 = TPL;
const QH_SC: u32 = TPL + 0x040;
const QH_ML: u32 = TPL + 0x080;
const QH_O_INQ: u32 = TPL + 0x0c0;
const QH_I_INQ: u32 = TPL + 0x100;
const QH_O_CAP: u32 = TPL + 0x140;
const QH_I_CAP: u32 = TPL + 0x180;
const QH_O_RD: u32 = TPL + 0x1c0;
const QH_I_RD: u32 = TPL + 0x200;
const QH_O_WR: u32 = TPL + 0x240;
const QH_I_WR: u32 = TPL + 0x280;

const T_SA_SETUP: u32 = TPL + 0x300;
const T_SA_STS: u32 = TPL + 0x320;
const T_SC_SETUP: u32 = TPL + 0x340;
const T_SC_STS: u32 = TPL + 0x360;
const T_ML_SETUP: u32 = TPL + 0x380;
const T_ML_IN: u32 = TPL + 0x3a0;
const T_ML_STS: u32 = TPL + 0x3c0;
const T_O_INQ: u32 = TPL + 0x3e0;
const T_I_INQ_D: u32 = TPL + 0x400;
const T_I_INQ_S: u32 = TPL + 0x420;
const T_O_CAP: u32 = TPL + 0x440;
const T_I_CAP_D: u32 = TPL + 0x460;
const T_I_CAP_S: u32 = TPL + 0x480;
const T_O_RD: u32 = TPL + 0x4a0;
const T_I_RD_D: u32 = TPL + 0x4c0;
const T_I_RD_S: u32 = TPL + 0x4e0;
const T_O_WR: u32 = TPL + 0x500;
const T_O_WR_D: u32 = TPL + 0x520;
const T_I_WR_S: u32 = TPL + 0x540;

const S_SA: u32 = TPL + 0x560;
const S_SC: u32 = TPL + 0x568;
const S_ML: u32 = TPL + 0x570;

const CBW_INQ: u32 = TPL + 0x580;
const CBW_CAP: u32 = TPL + 0x5a0;
const CBW_RD: u32 = TPL + 0x5c0;
const CBW_WR: u32 = TPL + 0x5e0;
/// The 512 bytes the guest writes to the disk.
const WDATA: u32 = TPL + 0x600;
/// The stage table the firmware walks: pairs of (queue head, token to poll),
/// terminated by a zero.
const STAGES: u32 = TPL + 0x800;
/// How much of the template the firmware copies.
const TPL_BYTES: u32 = 0x860;

/// How many transfer descriptors in the template carry `IOC`, and therefore how
/// many machine external traps a correct run takes.
///
/// One per stage: the status descriptor of each of the three control transfers,
/// the CBW of each of the four Bulk-Only commands, and the last descriptor of
/// each of their four data/status chains. Asserting the exact count is what
/// makes the handler's acknowledge *order* visible — see the module docs.
const IOC_DESCRIPTORS: u32 = 11;

/// The address the firmware gives the disk.
const GUEST_ADDRESS: u8 = 3;
/// Bytes in a logical block, and how many the disk holds — `param disk = 1M`
/// and `param block = 512` in the machine file.
const BLOCK: u64 = 512;
const BLOCKS: u64 = 1024 * 1024 / BLOCK;
/// The block the guest reads, and the one it writes.
const READ_LBA: u32 = 9;
const WRITE_LBA: u32 = 33;

/// The four `dCBWTag` values, one per command, so a CSW that came back attached
/// to the wrong command is visible rather than plausible (BOT §5.2).
const TAG_INQ: u32 = 0x1111_1111;
const TAG_CAP: u32 = 0x2222_2222;
const TAG_RD: u32 = 0x3333_3333;
const TAG_WR: u32 = 0x4444_4444;

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

/// `mtvec`, `mie` and `mstatus` (the RISC-V privileged specification, §3.1).
const CSR_MSTATUS: i32 = 0x300;
const CSR_MIE: i32 = 0x304;
const CSR_MTVEC: i32 = 0x305;
/// `mie.MEIE`, machine external interrupt enable — bit 11.
const MEIE: u32 = 1 << 11;
/// `mstatus.MIE` — bit 3.
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
/// `csrrw x0, csr, rs1` — write a CSR and discard the old value.
fn csrw(csr: i32, rs1: u32) -> u32 {
    i_type(csr, rs1, 0b001, ZERO, OP_SYSTEM)
}
/// `csrrs x0, csr, rs1` — set the bits `rs1` names.
fn csrs(csr: i32, rs1: u32) -> u32 {
    i_type(csr, rs1, 0b010, ZERO, OP_SYSTEM)
}
/// `csrrsi x0, csr, uimm` — set the bits a five-bit immediate names.
fn csrsi(csr: i32, uimm: u32) -> u32 {
    i_type(csr, uimm, 0b110, ZERO, OP_SYSTEM)
}
/// `mret`.
const MRET: u32 = 0x3020_0073;

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

// ---------------------------------------------------------------------------
// The firmware
// ---------------------------------------------------------------------------

/// The main program, assembled here so that the listing in the module docs and
/// the bytes that run cannot drift apart.
fn main_program() -> Vec<u32> {
    let mut code: Vec<u32> = Vec::new();
    let push = |c: &mut Vec<u32>, words: &[u32]| c.extend_from_slice(words);

    push(&mut code, &li(T0, OP));

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
    // is word aligned, so they are zero. Order matters — the vector is armed
    // before the enables, or an interrupt arriving in between would trap to
    // whatever `mtvec` reset to.
    push(&mut code, &li(A0, HANDLER));
    code.push(csrw(CSR_MTVEC, A0));
    push(&mut code, &li(A0, MEIE));
    code.push(csrs(CSR_MIE, A0));
    code.push(csrsi(CSR_MSTATUS, MSTATUS_MIE as u32));

    // -- the controller -----------------------------------------------------
    //
    // `USBINTR` first: the interrupt line is the AND of `USBSTS` with this
    // register (§2.3.2), so without it the wire never moves however many
    // descriptors retire.
    push(&mut code, &li(A0, STS_USBINT));
    code.push(sw(A0, T0, USBINTR));
    push(&mut code, &li(A0, 1));
    code.push(sw(A0, T0, CONFIGFLAG));

    // Drive a bus reset, then release it — releasing is what enables the port.
    code.push(lw(A0, T0, PORTSC));
    code.push(ori(A0, A0, 0x100));
    code.push(sw(A0, T0, PORTSC));
    code.push(lw(A0, T0, PORTSC));
    code.push(andi(A0, A0, -257));
    code.push(sw(A0, T0, PORTSC));

    // -- copy the schedule template out of ROM and into RAM ------------------
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

    // -- run the stages -----------------------------------------------------
    //
    // One queue head at a time, because BOT §3.4 forbids a second CBW before
    // the first CSW and because a failure then names the stage rather than the
    // schedule. `s0` holds the table pointer, which the trap handler does not
    // touch: it uses `t3`-`t6` precisely so that nothing here has to be saved.
    push(&mut code, &li(S0, STAGES));
    let loop_top = code.len();
    code.push(lw(A0, S0, 0));
    // Patched below, once the end is known.
    let exit_at = code.len();
    code.push(0);
    // Stop before repointing the list: changing `ASYNCLISTADDR` while the
    // asynchronous schedule is enabled is undefined (§2.3.5).
    code.push(sw(ZERO, T0, USBCMD));
    code.push(sw(A0, T0, ASYNCLISTADDR));
    push(&mut code, &li(A1, 0x21)); // RS | ASE
    code.push(sw(A1, T0, USBCMD));
    code.push(lw(A1, S0, 4));
    let wait = code.len();
    // The `USBSTS` load is what catches the controller up (`ROADMAP.md` §4.2's
    // sync-on-access); the token load is the one the answer comes from.
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

/// The machine external interrupt handler.
///
/// **The acknowledgement is two writes in a required order.** EHCI's line is
/// level triggered and is the AND of `USBSTS` with `USBINTR` (§2.3.2), so the
/// controller holds it down until software writes a one to the status bit.
/// Completing the PLIC claim while that level is still asserted makes the PLIC
/// re-latch the source, so the guest takes a second trap it had no work for —
/// which is why `USBSTS` is cleared *before* the claim is completed and not
/// after, and why the test counts traps instead of just noticing one.
fn trap_handler() -> Vec<u32> {
    let mut code: Vec<u32> = Vec::new();
    let push = |c: &mut Vec<u32>, words: &[u32]| c.extend_from_slice(words);

    // `t3`-`t6` only: the interrupted program keeps every register it uses.
    push(&mut code, &li(T3, PLIC_CLAIM0));
    code.push(lw(T4, T3, 0)); // claim
    push(&mut code, &li(T5, OP));
    push(&mut code, &li(T6, STS_USBINT));
    code.push(sw(T6, T5, USBSTS)); // write-one-to-clear: drop the level first
    code.push(sw(T4, T3, 0)); // …and only then complete the claim
    push(&mut code, &li(T5, IRQS));
    code.push(lw(T6, T5, 0));
    code.push(addi(T6, T6, 1));
    code.push(sw(T6, T5, 0));
    code.push(MRET);
    code
}

/// The ROM image: the main program at zero, the handler at [`HANDLER`], the
/// schedule template at [`TPL_ROM`].
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
// The schedule the firmware copies into RAM
// ---------------------------------------------------------------------------

/// A link pointer's terminate bit.
const T: u32 = 1;

/// A queue-head link pointer: `Typ = 01b`.
fn qh_link(addr: u32) -> u32 {
    addr | 0x2
}

/// `Endpoint Characteristics` (EHCI 1.0 §3.6.2) for a high-speed endpoint.
///
/// `dtc` is the Data Toggle Control bit: set for a control queue head, so that
/// a `SETUP` descriptor can force `DATA0`, and clear for a bulk one, where the
/// toggle lives in the queue head and survives the descriptor it came from.
fn epchar(address: u8, endpoint: u8, mps: u32, dtc: bool) -> u32 {
    let mut value = u32::from(address)
        | (u32::from(endpoint) << 8)
        // EPS = 10b: high speed.
        | (0x2 << 12)
        // H: this queue head is the head of the reclamation list, which it is,
        // because each stage's list has exactly one.
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

const PID_OUT: u32 = 0;
const PID_IN: u32 = 1;
const PID_SETUP: u32 = 2;

/// A queue head with one chain hanging off it: twelve dwords, of which the
/// overlay's Next qTD Pointer is where the chain starts (§3.6.3).
fn queue_head(link_to_self: u32, epchar: u32, first: u32) -> [u32; 12] {
    [
        qh_link(link_to_self),
        epchar,
        // Endpoint Capabilities: `Mult = 01b`, one transaction per microframe.
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
    ]
}

/// A transfer descriptor: eight dwords (§3.5).
fn qtd(next: u32, token: u32, buffer: u32) -> [u32; 8] {
    [
        if next == 0 { T } else { next },
        T,
        token,
        buffer,
        0,
        0,
        0,
        0,
    ]
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

/// The 512 bytes the guest writes to the disk. Deliberately unlike the pattern
/// the medium was stamped with, so "the write reached the medium" cannot be
/// satisfied by the block that was already there.
fn write_payload() -> Vec<u8> {
    (0..BLOCK)
        .map(|i| (i as u8).wrapping_mul(7) ^ 0xa5)
        .collect()
}

/// Write `words` into the template blob at guest address `addr`.
fn put(blob: &mut [u8], addr: u32, words: &[u32]) {
    let offset = (addr - TPL) as usize;
    for (i, word) in words.iter().enumerate() {
        blob[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
}

/// The same, for the byte-granular structures.
fn put_bytes(blob: &mut [u8], addr: u32, bytes: &[u8]) {
    let offset = (addr - TPL) as usize;
    blob[offset..offset + bytes.len()].copy_from_slice(bytes);
}

/// The whole schedule, as bytes, laid out at the addresses the firmware copies
/// it to.
fn template() -> Vec<u8> {
    let mut blob = vec![0u8; TPL_BYTES as usize];

    // -- the three control transfers ---------------------------------------
    //
    // `SET_ADDRESS` is addressed to zero, which is where the device answers
    // until the *status stage* of that very request completes (USB 2.0 §9.4.6);
    // everything after it is addressed to three.
    put(
        &mut blob,
        QH_SA,
        &queue_head(QH_SA, epchar(0, 0, 64, true), T_SA_SETUP),
    );
    put(
        &mut blob,
        T_SA_SETUP,
        &qtd(T_SA_STS, token(PID_SETUP, 8, false, false), S_SA),
    );
    put(
        &mut blob,
        T_SA_STS,
        &qtd(0, token(PID_IN, 0, true, true), 0),
    );
    put_bytes(
        &mut blob,
        S_SA,
        &setup(0x00, 5, u16::from(GUEST_ADDRESS), 0, 0),
    );

    put(
        &mut blob,
        QH_SC,
        &queue_head(QH_SC, epchar(GUEST_ADDRESS, 0, 64, true), T_SC_SETUP),
    );
    put(
        &mut blob,
        T_SC_SETUP,
        &qtd(T_SC_STS, token(PID_SETUP, 8, false, false), S_SC),
    );
    put(
        &mut blob,
        T_SC_STS,
        &qtd(0, token(PID_IN, 0, true, true), 0),
    );
    put_bytes(&mut blob, S_SC, &setup(0x00, 9, 1, 0, 0));

    // `GET_MAX_LUN` (BOT §3.2, table 3.2): class, interface, device to host,
    // one byte of data.
    put(
        &mut blob,
        QH_ML,
        &queue_head(QH_ML, epchar(GUEST_ADDRESS, 0, 64, true), T_ML_SETUP),
    );
    put(
        &mut blob,
        T_ML_SETUP,
        &qtd(T_ML_IN, token(PID_SETUP, 8, false, false), S_ML),
    );
    put(
        &mut blob,
        T_ML_IN,
        &qtd(T_ML_STS, token(PID_IN, 1, true, false), GML_BUF),
    );
    put(
        &mut blob,
        T_ML_STS,
        &qtd(0, token(PID_OUT, 0, true, true), 0),
    );
    put_bytes(&mut blob, S_ML, &setup(0xa1, 0xfe, 0, 0, 1));

    // -- the four Bulk-Only commands ---------------------------------------
    //
    // Each is a CBW on the bulk-out queue head, then the data and the CSW on
    // the bulk-in one — except `WRITE (10)`, whose data goes out on the same
    // queue head as its CBW and whose CSW is a stage of its own.
    let bulk_out = epchar(GUEST_ADDRESS, 2, 512, false);
    let bulk_in = epchar(GUEST_ADDRESS, 1, 512, false);

    put(
        &mut blob,
        QH_O_INQ,
        &queue_head(QH_O_INQ, bulk_out, T_O_INQ),
    );
    put(
        &mut blob,
        T_O_INQ,
        &qtd(0, token(PID_OUT, 31, false, true), CBW_INQ),
    );
    put_bytes(
        &mut blob,
        CBW_INQ,
        &cbw(TAG_INQ, 36, true, &[0x12, 0, 0, 0, 36, 0]),
    );
    put(
        &mut blob,
        QH_I_INQ,
        &queue_head(QH_I_INQ, bulk_in, T_I_INQ_D),
    );
    put(
        &mut blob,
        T_I_INQ_D,
        &qtd(T_I_INQ_S, token(PID_IN, 36, false, false), INQ_BUF),
    );
    put(
        &mut blob,
        T_I_INQ_S,
        &qtd(0, token(PID_IN, 13, true, true), CSW_BUF),
    );

    put(
        &mut blob,
        QH_O_CAP,
        &queue_head(QH_O_CAP, bulk_out, T_O_CAP),
    );
    put(
        &mut blob,
        T_O_CAP,
        &qtd(0, token(PID_OUT, 31, false, true), CBW_CAP),
    );
    put_bytes(
        &mut blob,
        CBW_CAP,
        &cbw(TAG_CAP, 8, true, &[0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
    );
    put(
        &mut blob,
        QH_I_CAP,
        &queue_head(QH_I_CAP, bulk_in, T_I_CAP_D),
    );
    put(
        &mut blob,
        T_I_CAP_D,
        &qtd(T_I_CAP_S, token(PID_IN, 8, false, false), CAP_BUF),
    );
    put(
        &mut blob,
        T_I_CAP_S,
        &qtd(0, token(PID_IN, 13, true, true), CSW_BUF + 0x20),
    );

    put(&mut blob, QH_O_RD, &queue_head(QH_O_RD, bulk_out, T_O_RD));
    put(
        &mut blob,
        T_O_RD,
        &qtd(0, token(PID_OUT, 31, false, true), CBW_RD),
    );
    put_bytes(
        &mut blob,
        CBW_RD,
        &cbw(TAG_RD, BLOCK as u32, true, &read10(READ_LBA, 1)),
    );
    put(&mut blob, QH_I_RD, &queue_head(QH_I_RD, bulk_in, T_I_RD_D));
    put(
        &mut blob,
        T_I_RD_D,
        &qtd(
            T_I_RD_S,
            token(PID_IN, BLOCK as u32, false, false),
            RDATA_BUF,
        ),
    );
    put(
        &mut blob,
        T_I_RD_S,
        &qtd(0, token(PID_IN, 13, true, true), CSW_BUF + 0x40),
    );

    // The CBW and its data phase on one chain: both are `OUT` on endpoint 2,
    // and a queue head serves one endpoint.
    put(&mut blob, QH_O_WR, &queue_head(QH_O_WR, bulk_out, T_O_WR));
    put(
        &mut blob,
        T_O_WR,
        &qtd(T_O_WR_D, token(PID_OUT, 31, false, false), CBW_WR),
    );
    put(
        &mut blob,
        T_O_WR_D,
        &qtd(0, token(PID_OUT, BLOCK as u32, true, true), WDATA),
    );
    put_bytes(
        &mut blob,
        CBW_WR,
        &cbw(TAG_WR, BLOCK as u32, false, &write10(WRITE_LBA, 1)),
    );
    put_bytes(&mut blob, WDATA, &write_payload());
    put(&mut blob, QH_I_WR, &queue_head(QH_I_WR, bulk_in, T_I_WR_S));
    put(
        &mut blob,
        T_I_WR_S,
        &qtd(0, token(PID_IN, 13, false, true), CSW_BUF + 0x60),
    );

    // -- the stage table ----------------------------------------------------
    let stages: [(u32, u32); 11] = [
        (QH_SA, T_SA_STS + 8),
        (QH_SC, T_SC_STS + 8),
        (QH_ML, T_ML_STS + 8),
        (QH_O_INQ, T_O_INQ + 8),
        (QH_I_INQ, T_I_INQ_S + 8),
        (QH_O_CAP, T_O_CAP + 8),
        (QH_I_CAP, T_I_CAP_S + 8),
        (QH_O_RD, T_O_RD + 8),
        (QH_I_RD, T_I_RD_S + 8),
        (QH_O_WR, T_O_WR_D + 8),
        (QH_I_WR, T_I_WR_S + 8),
    ];
    let mut words = Vec::new();
    for (qh, poll) in stages {
        words.push(qh);
        words.push(poll);
    }
    // The terminator the firmware's `beq` looks for.
    words.push(0);
    words.push(0);
    put(&mut blob, STAGES, &words);
    blob
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
///
/// The medium is kept on this side of the seam deliberately: `--drive
/// usb0=disk.img` installs one exactly like this, and holding a second handle
/// to it is what lets every assertion below check the bytes that reached
/// storage rather than the bytes that came back over the wire.
fn board() -> (Machine, Arc<RamStore>) {
    let store = Arc::new(RamStore::new(BLOCKS * BLOCK));
    for lba in 0..BLOCKS {
        RamStore::write_at(&store, lba * BLOCK, &stamp(lba)).expect("the image fits");
    }

    let mut options = catalog::build_options().expect("this build's options");
    options.realize.media.insert("firmware", firmware());
    rsemu::dev::ata::medium::install(
        &options.realize.hosts,
        "usb0",
        Arc::clone(&store) as Arc<dyn Medium>,
    )
    .expect("nothing else claimed the name");
    // Bound to no bytes: the medium above wins, and this is only how the
    // machine file's `image = "usb0"` finds a slot at all.
    options.realize.media.insert("usb0", Vec::new());

    let registry = catalog::registry().expect("this build's registry");
    let entry = &catalog::USB_MINI;
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

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_reads_a_sector_over_usb_and_it_is_the_sector_on_the_medium() {
    let (mut machine, store) = board();
    assert!(
        run_until_done(&mut machine),
        "the firmware never finished its eleven stages"
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

    // The status wrapper says the command passed and that every byte the host
    // asked for was relevant (§5.2).
    let (signature, tag, residue, status) = csw(&machine, 2);
    assert_eq!(signature, CSW_SIGNATURE);
    assert_eq!(tag, TAG_RD, "the CSW must echo its own CBW's tag");
    assert_eq!(residue, 0);
    assert_eq!(status, 0, "Command Passed");
}

#[test]
fn a_guest_writes_a_sector_over_usb_and_it_reaches_the_medium() {
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
fn the_guest_enumerated_the_disk_and_read_its_identity_and_its_capacity() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    // BOT §3.2: one logical unit, so the highest LUN is zero.
    assert_eq!(peek(&machine, GML_BUF) & 0xff, 0, "GET_MAX_LUN");

    // The standard INQUIRY data (Seagate §3.6.2, table 59), as the machine
    // file's defaults describe the disk.
    let inquiry = peek_bytes(&machine, INQ_BUF, 36);
    assert_eq!(inquiry[0], 0x00, "a direct-access block device, connected");
    assert_eq!(inquiry[3] & 0x0f, 2, "RESPONSE DATA FORMAT");
    assert_eq!(inquiry[4], 31, "ADDITIONAL LENGTH is n - 4");
    assert_eq!(&inquiry[8..16], b"RSEMU   ");
    assert_eq!(&inquiry[16..32], b"USB DISK        ");
    let (_, tag, residue, status) = csw(&machine, 0);
    assert_eq!((tag, residue, status), (TAG_INQ, 0, 0));

    // READ CAPACITY (10) (Seagate §3.22.2, table 120): the *last* block, and
    // the block length. `param disk = 1M` at 512 bytes a block is 2048 blocks,
    // so the answer is 2047 — off by one here and every host reads one block
    // past the end of the disk.
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
fn the_completion_interrupt_travelled_the_wire_and_was_acknowledged() {
    let (mut machine, _store) = board();
    assert!(run_until_done(&mut machine), "the firmware never finished");

    // The guest took real machine external traps: `IOC` on a descriptor became
    // `USBSTS.USBINT`, which became a level on `ehci.irq`, which became a
    // pending source in the PLIC, which became `meip` on the hart.
    let taken = peek(&machine, IRQS);
    assert_eq!(
        taken, IOC_DESCRIPTORS,
        "one trap per IOC descriptor, no more and no less"
    );

    // And it was acknowledged, in both places. `USBSTS.USBINT` is
    // write-one-to-clear and the handler cleared it *before* completing the
    // PLIC claim — the other order leaves the level asserted and the guest
    // never leaves the handler, which would show up as the run never finishing.
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
fn the_controller_is_where_the_machine_file_put_it() {
    let (machine, _store) = board();
    // `CAPLENGTH` is a byte at offset zero and `HCIVERSION` a halfword at two
    // (EHCI 1.0 §2.2), and every driver reads them at those widths — so this is
    // also the assertion that the `param ehci_at` in the machine file is what
    // decides where the block lives.
    let space = machine.space("mem").expect("the board has one");
    assert_eq!(
        space
            .read(u64::from(EHCI), Width::U8, MemAttrs::DEBUG)
            .expect("CAPLENGTH"),
        0x20
    );
    assert_eq!(
        space
            .read(u64::from(EHCI) + 2, Width::U16, MemAttrs::DEBUG)
            .expect("HCIVERSION"),
        0x0100
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
