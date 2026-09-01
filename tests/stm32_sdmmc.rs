//! A guest brings up an SD card through the STM32H7 SDMMC and moves a block.
//!
//! The device tests in `src/dev/stm32/sdmmc/tests.rs` drive the register block
//! directly. This one runs a **program**: a small RV32 firmware, assembled
//! here, that powers the controller, walks the whole identification sequence,
//! reads a block into its own RAM by internal DMA, writes that block back to a
//! different address, reads it back and compares the two — with nothing but
//! loads and stores to the register block in between.
//!
//! That is the milestone worth having. A controller that answers a test
//! harness's method calls is not the same claim as a controller a guest can
//! use, and the difference is usually a register that reads back wrong, a flag
//! that never clears, or a state machine that only advances when a test happens
//! to poke it in the right order.
//!
//! # Why a RISC-V hart on an STM32 peripheral
//!
//! Because this test is about the SDMMC rather than about a core, and because
//! the peripheral modelled here is the **H7's SDMMC**, not the F407's older
//! SDIO — so `machines/stm32f407.machine`, which describes a real F407, is the
//! wrong board to bend into shape. `machines/spi-panel.machine` makes the same
//! choice for the same reason: the board below is the smallest thing that can
//! execute instructions against the register block, and the register block does
//! not know or care what is fetching from it.
//!
//! The firmware **polls** rather than taking an interrupt, because a trap
//! handler hand-assembled here would be testing the hart. That the `irq` output
//! follows `STA & MASK` is checked in the device's own tests, where a wire and
//! a counting sink make the assertion direct.

#![cfg(all(feature = "dev-stm32-sdmmc", feature = "cpu-riscv"))]

use alloc::vec::Vec;
extern crate alloc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

// ---------------------------------------------------------------------------
// The board
// ---------------------------------------------------------------------------

/// Where the SDMMC's register block sits.
///
/// `0x5200_7000` is where SDMMC1 really is on an STM32H7 (RM0433 §2.3.2's
/// memory map). Nothing here depends on the address; it is the real one so that
/// a person reading the file recognises it.
const SDMMC: u32 = 0x5200_7000;
/// Where the board's RAM starts.
const RAM: u32 = 0x2000_0000;

const BOARD: &str = r#"
machine "stm32-sdmmc-test" {
  # Which named card socket the controller and the card meet in, so two of
  # these can run in one process without swapping cards.
  param slot = "sd0"

  osc sysclk = 100000000 Hz
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

  # An 8 MiB high-capacity card. Smaller than any card that was ever sold, and
  # exact in the CSD encoding all the same — see `src/dev/sd/card.rs`.
  object card "sd.card" {
    size          = 8M
    high-capacity = true
    slot          = slot
    image         = "card"
  }

  # A bus master: its internal DMA reads and writes guest memory itself, so it
  # declares the space that DMA traverses (ROADMAP.md §4.4).
  object sdmmc "stm32.sdmmc" {
    space = mem
    slot  = slot
  }

  map mem 0x00000000 size 64K = fw
  map mem 0x20000000 size 1M  = dram
  map mem 0x52007000 size 1K  = sdmmc
}
"#;

// ---------------------------------------------------------------------------
// The register map, as the firmware uses it
// ---------------------------------------------------------------------------

const R_POWER: i32 = 0x00;
const R_CLKCR: i32 = 0x04;
const R_ARGR: i32 = 0x08;
const R_CMDR: i32 = 0x0c;
const R_RESP1R: i32 = 0x14;
const R_DTIMER: i32 = 0x24;
const R_DLENR: i32 = 0x28;
const R_DCTRL: i32 = 0x2c;
const R_STAR: i32 = 0x34;
const R_ICR: i32 = 0x38;
const R_IDMACTRLR: i32 = 0x50;
const R_IDMABASE0R: i32 = 0x58;

/// Every bit `ICR` clears.
const ICR_ALL: u32 = 0x1fe0_0fff;
/// `STA.DATAEND`.
const STA_DATAEND: i32 = 1 << 8;

/// `POWER.PWRCTRL = 11b`.
const POWER_ON: u32 = 0x3;
/// `CMDR.CPSMEN`.
const CPSMEN: u32 = 1 << 12;
/// `DCTRL`: 512-byte blocks, card to controller, enabled.
const DCTRL_READ: u32 = (9 << 4) | (1 << 1) | 1;
/// `DCTRL`: 512-byte blocks, controller to card, enabled.
const DCTRL_WRITE: u32 = (9 << 4) | 1;

// ---------------------------------------------------------------------------
// Where the firmware puts things
// ---------------------------------------------------------------------------

/// The block the firmware reads out of the card.
const SOURCE_BLOCK: u32 = 1;
/// …and the one it writes back to.
const TARGET_BLOCK: u32 = 300;

/// Where the first read lands.
const BUF_A: u32 = RAM + 0x1000;
/// Where the read-back lands.
const BUF_B: u32 = RAM + 0x2000;
/// The firmware's "I got to the end" flag.
const FLAG: u32 = RAM + 0x3000;
/// The accumulated difference between the two buffers. Zero means identical.
const DIFF: u32 = RAM + 0x3004;
/// The last `STA` the firmware saw.
const LAST_STA: u32 = RAM + 0x3008;
/// Where the CID the card sent lands, four words, most significant first.
const CID: u32 = RAM + 0x3010;

const MAGIC: u32 = 0x5d_c0_de_00;

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

/// Store `value` into the register at `offset`, through the base in `T0`.
fn set(code: &mut Vec<u32>, offset: i32, value: u32) {
    code.extend_from_slice(&li(A0, value));
    code.push(sw(A0, T0, offset));
}

/// Clear the status latch, load the argument, and start the command state
/// machine — the three writes every SDMMC driver makes for every command.
fn command(code: &mut Vec<u32>, index: u32, arg: u32, waitresp: u32) {
    set(code, R_ICR, ICR_ALL);
    set(code, R_ARGR, arg);
    set(code, R_CMDR, index | (waitresp << 8) | CPSMEN);
}

/// The same, with the argument taken from `T1` — which is where the firmware
/// keeps `RESP1R` after `CMD3`, so the published address addresses the card
/// without the firmware having to know what it is.
fn command_addressed(code: &mut Vec<u32>, index: u32, waitresp: u32) {
    set(code, R_ICR, ICR_ALL);
    code.push(sw(T1, T0, R_ARGR));
    set(code, R_CMDR, index | (waitresp << 8) | CPSMEN);
}

/// Move one 512-byte block by internal DMA, and wait for `DATAEND`.
fn transfer(code: &mut Vec<u32>, buffer: u32, dctrl: u32, index: u32, block: u32) {
    set(code, R_IDMABASE0R, buffer);
    set(code, R_IDMACTRLR, 1);
    set(code, R_DTIMER, 0x00ff_ffff);
    set(code, R_DLENR, 512);
    set(code, R_DCTRL, dctrl);
    command(code, index, block, 1);
    // Poll STA.DATAEND. Under this model it is already set, but a driver that
    // polls is what the flag exists for, and a spin here is what a broken
    // transfer would look like — the harness gives up rather than hanging.
    let wait = code.len();
    code.push(lw(A0, T0, R_STAR));
    code.push(sw(A0, T2, 0)); // remember the last STA for the harness
    code.push(andi(A0, A0, STA_DATAEND));
    let back = -(((code.len() - wait) * 4) as i32);
    code.push(beq(A0, ZERO, back));
}

/// The program.
fn firmware() -> Vec<u8> {
    let mut code: Vec<u32> = Vec::new();
    code.extend_from_slice(&li(T0, SDMMC));
    code.extend_from_slice(&li(T2, LAST_STA));

    // Power the card and run the bus at the identification clock.
    set(&mut code, R_POWER, POWER_ON);
    set(&mut code, R_CLKCR, 250);

    // The identification sequence of Physical Layer §4.2, as a driver walks it.
    command(&mut code, 0, 0, 0); // CMD0  GO_IDLE_STATE, no response
    command(&mut code, 8, 0x1aa, 1); // CMD8  SEND_IF_COND, R7
    command(&mut code, 55, 0, 1); // CMD55 APP_CMD, R1
    command(&mut code, 41, 0x40ff_8000, 2); // ACMD41 with HCS, R3 (no CRC)
    command(&mut code, 2, 0, 3); // CMD2  ALL_SEND_CID, R2

    // Copy the CID out of the four response registers, so the harness can
    // check that a guest really read the card's identity.
    code.extend_from_slice(&li(T1, CID));
    for i in 0..4i32 {
        code.push(lw(A0, T0, R_RESP1R + i * 4));
        code.push(sw(A0, T1, i * 4));
    }

    command(&mut code, 3, 0, 1); // CMD3 SEND_RELATIVE_ADDR, R6
    // R6's top half is the published address, and the card ignores the rest of
    // a CMD7 argument, so the whole word can be handed straight back.
    code.push(lw(T1, T0, R_RESP1R));
    command_addressed(&mut code, 7, 1); // CMD7 SELECT_CARD, R1b
    command_addressed(&mut code, 55, 1); // CMD55 APP_CMD
    command(&mut code, 6, 0b10, 1); // ACMD6 SET_BUS_WIDTH, four bits
    command(&mut code, 16, 512, 1); // CMD16 SET_BLOCKLEN

    // Read a block, write it somewhere else, read that back.
    transfer(&mut code, BUF_A, DCTRL_READ, 17, SOURCE_BLOCK);
    transfer(&mut code, BUF_A, DCTRL_WRITE, 24, TARGET_BLOCK);
    transfer(&mut code, BUF_B, DCTRL_READ, 17, TARGET_BLOCK);

    // Accumulate the difference between the two buffers rather than branching
    // out of the loop: one number the harness can read, and no forward branch
    // to patch.
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
/// Generated rather than committed, exactly as `dev-flash-cfi`'s fixtures are:
/// a test image is not something to keep in the repository.
fn card_image() -> Vec<u8> {
    (0..4096u32)
        .map(|i| (i.wrapping_mul(31).wrapping_add(7)) as u8)
        .collect()
}

// ---------------------------------------------------------------------------
// The harness
// ---------------------------------------------------------------------------

/// Build the board, and hand back the card socket as well as the machine.
///
/// The socket is opened in *this build's* host objects before the build, the
/// way `tests/usb_ehci.rs` opens its bus: the rendezvous is per build, so the
/// name need not be unique across the test binary and two of these can run at
/// once.
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
    let machine =
        rsemu::machine::build("stm32-sdmmc-test", BOARD, &registry, &options).expect("it realizes");
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
    for _ in 0..200 {
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
fn a_guest_brings_up_a_card_and_moves_a_block_through_the_controller() {
    let (mut machine, _slot) = boot("board-move-a-block");
    assert!(
        run_until_done(&mut machine),
        "the firmware never reached its end; the last STA it saw was {:#010x}",
        peek(&machine, LAST_STA)
    );

    // The identification sequence really ran: this is the card's own CID,
    // CRC7 and all, read out of RESP1R..RESP4R by the guest. The response
    // registers hold the register's bits most significant first, and the guest
    // stored them as little-endian words, so putting them back in register
    // order is what the reassembly below does.
    let mut cid = [0u8; 16];
    for i in 0..4 {
        let word = peek(&machine, CID + (i as u32) * 4);
        cid[i * 4..i * 4 + 4].copy_from_slice(&word.to_be_bytes());
    }
    assert_eq!(cid[0], 0x03, "the CID's manufacturer field");
    assert_eq!(&cid[3..8], b"RSEMU", "and its product name");
    assert_eq!(cid[15] & 1, 1, "a register's end bit is always one");

    // The block it read is the block the image put there.
    let image = card_image();
    let want = &image[512..1024];
    assert_eq!(
        peek_bytes(&machine, BUF_A, 512),
        want,
        "the internal DMA put the card's block one in the guest's own RAM"
    );

    // And the block it wrote came back identical — the round trip that proves
    // the model rather than the plumbing.
    assert_eq!(
        peek(&machine, DIFF),
        0,
        "the read-back differs from the write"
    );
    assert_eq!(peek_bytes(&machine, BUF_B, 512), want);
    assert_ne!(peek(&machine, LAST_STA) & (STA_DATAEND as u32), 0);
}

#[test]
fn the_bytes_the_guest_wrote_are_in_the_card_afterwards() {
    // Read from the *card*, not from the guest's copy of it: a write path that
    // only updated a buffer would pass the test above.
    let (mut machine, slot) = boot("board-check-the-card");
    assert!(run_until_done(&mut machine));

    let card = slot.card().expect("with a card in it");
    let mut block = alloc::vec![0u8; 512];
    card.read_media(u64::from(TARGET_BLOCK) * 512, &mut block)
        .expect("inside the card");
    let image = card_image();
    assert_eq!(block, image[512..1024], "block {TARGET_BLOCK} of the card");
    // The block the firmware copied *from* is untouched, so nothing wrote to
    // the wrong address on the way past.
    let mut source = alloc::vec![0u8; 512];
    card.read_media(u64::from(SOURCE_BLOCK) * 512, &mut source)
        .expect("inside the card");
    assert_eq!(source, image[512..1024]);
}

#[test]
fn a_reset_puts_the_card_back_in_the_idle_state() {
    let (mut machine, slot) = boot("board-reset");
    assert!(run_until_done(&mut machine));
    let card = slot.card().expect("with a card in it");
    assert_eq!(card.phase(), rsemu::dev::sd::Phase::Transfer);
    assert_ne!(card.rca(), 0);

    machine.reset(rsemu::core::device::ResetKind::Cold);
    assert_eq!(card.phase(), rsemu::dev::sd::Phase::Idle);
    assert_eq!(card.rca(), 0, "the published address went with the power");
    // The contents did not: a card is not volatile.
    let mut block = alloc::vec![0u8; 512];
    card.read_media(u64::from(TARGET_BLOCK) * 512, &mut block)
        .expect("inside the card");
    assert_eq!(block, card_image()[512..1024]);
}
