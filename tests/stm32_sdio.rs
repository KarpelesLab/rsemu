//! A guest moves blocks between an SD card and its own RAM **over DMA2**.
//!
//! `src/dev/stm32/sdio/tests.rs` drives the register block directly, and one of
//! its cases already puts a real `st.dma` behind the request line. This one runs
//! a **program**, on a board, on the scheduler's timeline: a small RV32 firmware
//! that powers the controller, walks the identification sequence, arms a DMA2
//! stream at the SDIO's FIFO, sends `CMD17`, and waits for the stream's `TCIF` —
//! having never read `SDIO_FIFO` itself. Then it does the same in reverse with
//! `CMD24`, and reads the result back to compare.
//!
//! That is the milestone GitHub issue #13 asks for and the reason the F4's SDIO
//! could not be written when the H7's SDMMC was. The H7 is a bus master and
//! moves its own block; this one raises a line and something else does the work,
//! so "SDIO transfers are DMA-driven" is either a code path under test or a
//! sentence in a comment.
//!
//! # What would fail here and nowhere else
//!
//! * A `DMAEN` bit that is stored and ignored. The firmware never touches the
//!   FIFO, so every byte in `BUF_A` arrived because the request line was raised
//!   and DMA2 answered it.
//! * A request that goes down when `DCOUNT` reaches zero. The card finishes
//!   filling the FIFO thirty-two words before the stream has emptied it, and a
//!   line that dropped there would leave the last 128 bytes of every block
//!   unwritten — with `DATAEND` set, so the SDIO would look fine.
//! * A transfer that completes inside the store that arms it. The `TCIF` wait
//!   loop below really spins: one beat costs one tick of the DMA's clock domain.
//!
//! The firmware **polls** rather than taking an interrupt, because a trap
//! handler hand-assembled here would be testing the hart.

#![cfg(all(
    feature = "dev-stm32-sdio",
    feature = "dev-stm32-dma",
    feature = "cpu-riscv"
))]

use alloc::vec::Vec;
extern crate alloc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

const BOARD: &str = include_str!("../machines/tests/stm32f4-sdio.machine");

/// Where the SDIO's register block sits: RM0090 Table 1, APB2 + 0x2c00.
const SDIO: u32 = 0x4001_2c00;
/// Where DMA2's register block sits: AHB1 + 0x6400.
const DMA2: u32 = 0x4002_6400;
/// Where the board's RAM starts.
const RAM: u32 = 0x2000_0000;

// ---------------------------------------------------------------------------
// The register maps, as the firmware uses them
// ---------------------------------------------------------------------------

const R_POWER: i32 = 0x00;
const R_CLKCR: i32 = 0x04;
const R_ARG: i32 = 0x08;
const R_CMD: i32 = 0x0c;
const R_RESP1: i32 = 0x14;
const R_DTIMER: i32 = 0x24;
const R_DLEN: i32 = 0x28;
const R_DCTRL: i32 = 0x2c;
const R_STA: i32 = 0x34;
const R_ICR: i32 = 0x38;
/// The FIFO aperture, which is where `CPAR` points and where the firmware
/// deliberately never reads.
const R_FIFO: u32 = 0x80;

/// Every bit `SDIO_ICR` clears (RM0090 §31.9.12).
const ICR_ALL: u32 = 0x00c0_07ff;
/// `SDIO_STA.DATAEND`.
const STA_DATAEND: u32 = 1 << 8;
/// `SDIO_POWER.PWRCTRL = 11b`.
const POWER_ON: u32 = 0x3;
/// `SDIO_CMD.CPSMEN` — **bit 10** on this block, not 12.
const CPSMEN: u32 = 1 << 10;
/// `WAITRESP = 01b`, the only short encoding this block has.
const SHORT: u32 = 1;
/// `WAITRESP = 11b`.
const LONG: u32 = 3;
/// `SDIO_DCTRL`: 512-byte blocks, card to controller, DMA, enabled.
const DCTRL_READ: u32 = (9 << 4) | (1 << 3) | (1 << 1) | 1;
/// …and controller to card.
const DCTRL_WRITE: u32 = (9 << 4) | (1 << 3) | 1;

/// `DMA_LISR`, and `DMA_LIFCR` which clears the same bits.
const R_LISR: i32 = 0x00;
const R_LIFCR: i32 = 0x08;
/// Stream 3's flag group starts at bit 22 (RM0090 §10.5.1), and `TCIF` is the
/// sixth bit of the group.
const TCIF3_SHIFT: u32 = 22 + 5;
/// Everything in stream 3's group, for `LIFCR`.
const S3_FLAGS: u32 = 0x3d << 22;

const fn s_cr(s: i32) -> i32 {
    0x10 + 0x18 * s
}
const fn s_ndtr(s: i32) -> i32 {
    s_cr(s) + 4
}
const fn s_par(s: i32) -> i32 {
    s_cr(s) + 8
}
const fn s_m0ar(s: i32) -> i32 {
    s_cr(s) + 0x0c
}

/// `SxCR` for a peripheral-to-memory word transfer with `MINC`, `CHSEL = 4`.
///
/// `CHSEL` is what a driver writes and what RM0090 Table 43 indexes; `st.dma`
/// stores it and the board's wiring is what actually selects the stream. It is
/// written anyway, because a driver writes it.
const CR_READ: u32 = (4 << 25) | (0b10 << 13) | (0b10 << 11) | (1 << 10) | 1;
/// The same with `DIR = 01`, memory to peripheral.
const CR_WRITE: u32 = CR_READ | (0b01 << 6);

// ---------------------------------------------------------------------------
// Where the firmware puts things
// ---------------------------------------------------------------------------

/// The block the firmware reads out of the card.
const SOURCE_BLOCK: u32 = 1;
/// …and the one it writes back to.
const TARGET_BLOCK: u32 = 300;

const BUF_A: u32 = RAM + 0x1000;
const BUF_B: u32 = RAM + 0x2000;
const FLAG: u32 = RAM + 0x3000;
const DIFF: u32 = RAM + 0x3004;
const LAST_STA: u32 = RAM + 0x3008;

const MAGIC: u32 = 0x5d_10_de_00;

// ---------------------------------------------------------------------------
// Just enough RV32I to write the firmware
// ---------------------------------------------------------------------------

const ZERO: u32 = 0;
const T0: u32 = 5;
const T1: u32 = 6;
const T2: u32 = 7;
const T3: u32 = 28;
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
const OP_REG: u32 = 0b011_0011;

fn i_type(imm: i32, rs1: u32, funct3: u32, rd: u32, opcode: u32) -> u32 {
    ((imm as u32) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
}

fn r_type(rs2: u32, rs1: u32, funct3: u32, rd: u32) -> u32 {
    (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | OP_REG
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
fn srli(rd: u32, rs1: u32, shamt: u32) -> u32 {
    i_type(shamt as i32, rs1, 0b101, rd, OP_IMM)
}
fn xor(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r_type(rs2, rs1, 0b100, rd)
}
fn or(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r_type(rs2, rs1, 0b110, rd)
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
    let hi = value.wrapping_add(0x800) >> 12;
    let lo = (value & 0xfff) as i32;
    let lo = if lo >= 0x800 { lo - 0x1000 } else { lo };
    [lui(rd, hi), addi(rd, rd, lo)]
}

/// Store `value` into the register at `base + offset`.
fn set_at(code: &mut Vec<u32>, base: u32, offset: i32, value: u32) {
    code.extend_from_slice(&li(A0, value));
    code.push(sw(A0, base, offset));
}

/// An SDIO register, through the base kept in `T0`.
fn set(code: &mut Vec<u32>, offset: i32, value: u32) {
    set_at(code, T0, offset, value);
}

/// Clear the status latch, load the argument, and start the command state
/// machine — the three writes every SD driver makes for every command.
///
/// `ICR` first is what makes `ACMD41`'s `CCRCFAIL` a non-event: on this block an
/// `R3` always reports a CRC failure, because the card drives all ones where the
/// CRC belongs and there is no `WAITRESP` encoding that says "do not check". A
/// driver clears the flag with the next command and thinks no more about it,
/// which is exactly what this does.
fn command(code: &mut Vec<u32>, index: u32, arg: u32, waitresp: u32) {
    set(code, R_ICR, ICR_ALL);
    set(code, R_ARG, arg);
    set(code, R_CMD, index | (waitresp << 6) | CPSMEN);
}

/// The same, with the argument taken from `T1` — which is where the firmware
/// keeps `RESP1` after `CMD3`.
fn command_addressed(code: &mut Vec<u32>, index: u32, waitresp: u32) {
    set(code, R_ICR, ICR_ALL);
    code.push(sw(T1, T0, R_ARG));
    set(code, R_CMD, index | (waitresp << 6) | CPSMEN);
}

/// Move one 512-byte block **by DMA2 stream 3**, and wait for the stream's own
/// transfer-complete flag.
///
/// Waiting on `TCIF` rather than on `SDIO_STA.DATAEND` is not a detail: on this
/// block `DATAEND` means the *card* has finished handing bytes to the FIFO, and
/// at that moment the last thirty-two words of the block are still in it. A
/// driver that stopped at `DATAEND` would read a buffer whose tail had not been
/// written yet, which is what makes `TCIF` the flag ST's own driver waits on.
fn transfer(code: &mut Vec<u32>, buffer: u32, cr: u32, dctrl: u32, index: u32, block: u32) {
    // Arm the stream at the FIFO. `PINC` is clear, so `CPAR` stays put: the
    // peripheral's data register does not move.
    set_at(code, T3, R_LIFCR, S3_FLAGS);
    set_at(code, T3, s_par(3), SDIO + R_FIFO);
    set_at(code, T3, s_m0ar(3), buffer);
    set_at(code, T3, s_ndtr(3), 512 / 4);
    set_at(code, T3, s_cr(3), cr);

    if dctrl & (1 << 1) == 0 {
        // A write: the command goes first, and then the data configuration
        // starts the DPSM and raises the request.
        command(code, index, block, SHORT);
        set(code, R_DTIMER, 0x00ff_ffff);
        set(code, R_DLEN, 512);
        set(code, R_DCTRL, dctrl);
    } else {
        // A read: `DTEN` first, because this block has no `CMDTRANS` and the
        // DPSM waits on DAT until the command goes out.
        set(code, R_DTIMER, 0x00ff_ffff);
        set(code, R_DLEN, 512);
        set(code, R_DCTRL, dctrl);
        command(code, index, block, SHORT);
    }

    // Spin on LISR.TCIF3. The flag is above the reach of a twelve-bit immediate,
    // so it is shifted down rather than masked in place.
    let wait = code.len();
    code.push(lw(A0, T3, R_LISR));
    code.push(srli(A0, A0, TCIF3_SHIFT));
    code.push(andi(A0, A0, 1));
    let back = -(((code.len() - wait) * 4) as i32);
    code.push(beq(A0, ZERO, back));

    // Remember the last SDIO status the firmware saw, for the harness.
    code.push(lw(A0, T0, R_STA));
    code.push(sw(A0, T2, 0));
}

/// The program.
fn firmware() -> Vec<u8> {
    let mut code: Vec<u32> = Vec::new();
    code.extend_from_slice(&li(T0, SDIO));
    code.extend_from_slice(&li(T3, DMA2));
    code.extend_from_slice(&li(T2, LAST_STA));

    // Power the card and run the bus at the identification clock: 48 MHz over
    // 118 + 2 is 400 kHz, which is what a driver programs.
    set(&mut code, R_POWER, POWER_ON);
    set(&mut code, R_CLKCR, 118);

    // The identification sequence of Physical Layer §4.2, in the v1 encoding.
    command(&mut code, 0, 0, 0); // CMD0  GO_IDLE_STATE, no response
    command(&mut code, 8, 0x1aa, SHORT); // CMD8  SEND_IF_COND, R7
    command(&mut code, 55, 0, SHORT); // CMD55 APP_CMD, R1
    command(&mut code, 41, 0x40ff_8000, SHORT); // ACMD41 with HCS, R3 → CCRCFAIL
    command(&mut code, 2, 0, LONG); // CMD2  ALL_SEND_CID, R2

    command(&mut code, 3, 0, SHORT); // CMD3 SEND_RELATIVE_ADDR, R6
    // R6's top half is the published address, and the card ignores the rest of a
    // CMD7 argument, so the whole word can be handed straight back.
    code.push(lw(T1, T0, R_RESP1));
    command_addressed(&mut code, 7, SHORT); // CMD7 SELECT_CARD
    command_addressed(&mut code, 55, SHORT); // CMD55 APP_CMD
    command(&mut code, 6, 0b10, SHORT); // ACMD6 SET_BUS_WIDTH, four bits
    command(&mut code, 16, 512, SHORT); // CMD16 SET_BLOCKLEN
    // Four-bit bus and the transfer clock, now that identification is over.
    set(&mut code, R_CLKCR, (0b01 << 11) | (1 << 8));

    // Read a block, write it somewhere else, read that back — every one of the
    // three through DMA2, and the FIFO never named by the program.
    transfer(&mut code, BUF_A, CR_READ, DCTRL_READ, 17, SOURCE_BLOCK);
    transfer(&mut code, BUF_A, CR_WRITE, DCTRL_WRITE, 24, TARGET_BLOCK);
    transfer(&mut code, BUF_B, CR_READ, DCTRL_READ, 17, TARGET_BLOCK);

    // Accumulate the difference between the two buffers rather than branching
    // out of the loop: one number the harness can read, and no forward branch to
    // patch.
    code.extend_from_slice(&li(T1, BUF_A));
    code.extend_from_slice(&li(T2, BUF_B));
    code.extend_from_slice(&li(A1, 0));
    code.extend_from_slice(&li(A2, 512));
    let cmp = code.len();
    code.push(lw(A0, T1, 0));
    code.push(lw(A3, T2, 0));
    code.push(xor(A0, A0, A3));
    code.push(or(A1, A1, A0));
    code.push(addi(T1, T1, 4));
    code.push(addi(T2, T2, 4));
    code.push(addi(A2, A2, -4));
    let back = -(((code.len() - cmp) * 4) as i32);
    code.push(bne(A2, ZERO, back));

    code.extend_from_slice(&li(T1, DIFF));
    code.push(sw(A1, T1, 0));
    code.extend_from_slice(&li(T1, FLAG));
    code.extend_from_slice(&li(A0, MAGIC));
    code.push(sw(A0, T1, 0));
    code.push(jal(ZERO, 0)); // and stop here

    let mut bytes = Vec::with_capacity(code.len() * 4);
    for word in code {
        bytes.extend_from_slice(&word.to_le_bytes());
    }
    bytes
}

/// What the card holds when the machine starts.
///
/// Generated rather than committed: a test image is not something to keep in the
/// repository.
fn card_image() -> Vec<u8> {
    (0..4096u32)
        .map(|i| (i.wrapping_mul(31).wrapping_add(7)) as u8)
        .collect()
}

// ---------------------------------------------------------------------------
// The harness
// ---------------------------------------------------------------------------

fn boot(slot_name: &str) -> (Machine, alloc::sync::Arc<rsemu::dev::sd::slots::Slot>) {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", firmware());
    options.realize.media.insert("card", card_image());
    options
        .resolve
        .params
        .push((alloc::string::String::from("slot"), slot_name.into()));
    let slot = rsemu::dev::sd::slots::open(&options.realize.hosts, slot_name)
        .expect("a socket of this build's");
    let registry = catalog::registry().expect("a registry");
    let machine = rsemu::machine::build("stm32f4-sdio-test", BOARD, &registry, &options)
        .expect("it realizes");
    (machine, slot)
}

fn peek(machine: &Machine, addr: u32) -> u32 {
    machine
        .space("mem")
        .expect("the board has one")
        .read(u64::from(addr), Width::U32, MemAttrs::DEBUG)
        .expect("mapped RAM") as u32
}

fn peek_bytes(machine: &Machine, addr: u32, len: usize) -> Vec<u8> {
    let mut out = alloc::vec![0u8; len];
    machine
        .space("mem")
        .expect("the board has one")
        .read_bytes(u64::from(addr), &mut out, MemAttrs::DEBUG)
        .expect("mapped RAM");
    out
}

fn run_until_done(machine: &mut Machine) -> bool {
    for _ in 0..400 {
        machine
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        if peek(machine, FLAG) == MAGIC {
            return true;
        }
    }
    false
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_moves_blocks_through_the_sdio_without_ever_touching_its_fifo() {
    let (mut machine, _slot) = boot("sdio-board-move-a-block");
    assert!(
        run_until_done(&mut machine),
        "the firmware never reached its end; the last SDIO_STA it saw was {:#010x}",
        peek(&machine, LAST_STA)
    );

    // The block it read is the block the image put there — all 512 bytes of it,
    // which is the assertion the tail of a dropped request would fail.
    let image = card_image();
    let want = &image[512..1024];
    assert_eq!(
        peek_bytes(&machine, BUF_A, 512),
        want,
        "DMA2 put the card's block one in the guest's own RAM"
    );

    // And the block it wrote came back identical.
    assert_eq!(peek(&machine, DIFF), 0, "the read-back differs");
    assert_eq!(peek_bytes(&machine, BUF_B, 512), want);
    assert_ne!(
        peek(&machine, LAST_STA) & STA_DATAEND,
        0,
        "and the SDIO saw its own transfer end"
    );
}

#[test]
fn the_bytes_the_guest_wrote_are_in_the_card_afterwards() {
    // Read from the *card*, not from the guest's copy of it: a write path that
    // only updated a buffer would pass the test above.
    let (mut machine, slot) = boot("sdio-board-check-the-card");
    assert!(run_until_done(&mut machine));

    let card = slot.card().expect("with a card in it");
    let mut block = alloc::vec![0u8; 512];
    card.read_media(u64::from(TARGET_BLOCK) * 512, &mut block)
        .expect("inside the card");
    let image = card_image();
    assert_eq!(block, image[512..1024], "block {TARGET_BLOCK} of the card");

    // The block it copied *from* is untouched, so nothing wrote to the wrong
    // address on the way past.
    let mut source = alloc::vec![0u8; 512];
    card.read_media(u64::from(SOURCE_BLOCK) * 512, &mut source)
        .expect("inside the card");
    assert_eq!(source, image[512..1024]);
}

#[test]
fn the_transfer_takes_bus_time_rather_than_finishing_inside_a_store() {
    // One beat is one tick of the DMA's clock domain, so a 128-word block cannot
    // possibly be over when the store that arms the stream retires. Running for
    // a single microsecond gets the firmware through identification and into the
    // first transfer and no further, which is what a device with a clock domain
    // behind it looks like from the outside.
    let (mut machine, _slot) = boot("sdio-board-timing");
    machine
        .run_for(GlobalTime::from_nanos(1_000))
        .expect("it runs");
    assert_ne!(peek(&machine, FLAG), MAGIC, "nowhere near finished");
    assert!(run_until_done(&mut machine), "and it does finish");
}
