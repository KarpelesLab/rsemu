//! The firmware the `spi-flash` board boots when nothing else is bound.
//!
//! An RV32I program, assembled at compile time by the `const fn` encoders
//! below, that does the whole serial-flash path end to end and finishes by
//! **executing code it wrote into the flash itself**:
//!
//! 1. It drives the [`stm32.spi`](super::spi) peripheral as a master and reads
//!    the part's JEDEC identifier out of it — `9Fh`, three bytes — leaving them
//!    in RAM where a test can see them.
//! 2. It switches to the [`stm32.octospi`](super::octospi) in **indirect
//!    write** mode and programs a short payload into the flash: `06h` write
//!    enable, then `02h` page program with the bytes pushed through `DR`.
//! 3. It puts the OCTOSPI into **memory-mapped** mode with `0Bh` fast read as
//!    the command, and jumps into the window.
//! 4. The payload — now living only in the emulated flash array, reached one
//!    SPI frame per instruction fetch — stores a sentinel into RAM.
//!
//! That last step is the claim worth making. Nothing copies the payload into
//! RAM or into a shadow buffer: every fetch is an `0Bh` frame clocked down
//! `bus::spi` to a `flash.spinor`, and the sentinel can only appear if the
//! page program landed, the flash kept the bytes, and the hart fetched them
//! back through the aperture.
//!
//! # Why it is assembled here rather than committed as bytes
//!
//! Same reason as `dev::lcd::demo`: a blob of hex is unreviewable and
//! a committed `.bin` would need a toolchain nobody has. The encoders are
//! here, the listing is beside them, and [`tests`](self) checks the label
//! indices against the program so the two cannot drift.
//!
//! The encoders duplicate a handful of the ones in `dev::lcd::demo`. They are
//! deliberately not shared yet: that module is behind another feature, and one
//! small assembler in each of two demos is a better trade than a public
//! `dev::riscv::asm` nothing else asks for. A third board is the moment to
//! merge them.
//!
//! # The program
//!
//! ```text
//!         j       main
//!
//! xfer:                           ; a1 = byte out, a0 = byte in. t0 = SPI1
//!         sw      a1, 12(t0)      ; DR: the write starts the frame
//! xwait:  lw      a0, 8(t0)       ; SR
//!         andi    a0, a0, 1       ;   RXNE
//!         beq     a0, x0, xwait
//!         lw      a0, 12(t0)      ; pop DR
//!         ret
//!
//! main:   li      t0, 0xf0000000  ; the STM32 SPI
//!         li      t1, 0xf0001000  ; the OCTOSPI
//!         li      t2, 0x20000000  ; RAM
//!
//!         li      a0, 4           ; SPI CR2 = SSOE, so NSS follows SPE and
//!         sw      a0, 4(t0)       ;   enabling the peripheral *is* the chip
//!         li      a0, 0x5c        ;   select (RM0090 §28.3.1)
//!         sw      a0, 0(t0)       ; CR1 = MSTR | SPE | BR=3
//!         li      a1, 0x9f        ; JEDEC id
//!         jal     ra, xfer
//!         li      a1, 0
//!         jal     ra, xfer
//!         sb      a0, 0(t2)       ; RAM[0] = manufacturer
//!         li      a1, 0
//!         jal     ra, xfer
//!         sb      a0, 1(t2)       ; RAM[1] = memory type
//!         li      a1, 0
//!         jal     ra, xfer
//!         sb      a0, 2(t2)       ; RAM[2] = capacity
//!         sw      x0, 0(t0)       ; CR1 = 0: NSS rises, the frame ends
//!
//!         li      a0, 0x130000    ; OCTOSPI DCR1: DEVSIZE = 19, a 1 MiB part
//!         sw      a0, 8(t1)
//!         li      a0, 1           ; CR = EN, FMODE = 00 indirect write
//!         sw      a0, 0(t1)
//!         li      a0, 1           ; CCR = IMODE 1: an opcode and nothing else
//!         sw      a0, 0x100(t1)
//!         li      a0, 6           ; IR = 06h write enable — and writing IR is
//!         sw      a0, 0x110(t1)   ;   the trigger when there is no address
//!
//!         li      a0, 0x01002101  ; CCR = IMODE 1, ADMODE 1, ADSIZE 24-bit,
//!         sw      a0, 0x100(t1)   ;   DMODE 1
//!         li      a0, 2           ; IR = 02h page program
//!         sw      a0, 0x110(t1)
//!         li      a0, 23          ; DLR = length less one
//!         sw      a0, 0x40(t1)
//!         li      a0, 0
//!         sw      a0, 0x48(t1)    ; AR = 0: this is the trigger
//!         li      a1, payload     ; where the bytes are in ROM
//!         li      a2, 24
//! copy:   lbu     a3, 0(a1)
//!         sb      a3, 0x50(t1)    ; DR: one byte into the flash
//!         addi    a1, a1, 1
//!         addi    a2, a2, -1
//!         bne     a2, x0, copy    ; the last byte raises the chip select,
//!                                 ;   which is where the flash commits it
//!
//!         li      a0, 0x01002101  ; the read command: the same shape …
//!         sw      a0, 0x100(t1)
//!         li      a0, 8           ; TCR: DCYC = 8, one dummy byte
//!         sw      a0, 0x108(t1)
//!         li      a0, 0x0b        ; … with 0Bh fast read as the instruction
//!         sw      a0, 0x110(t1)
//!         li      a0, 0x30000001  ; CR = EN, FMODE = 11 memory-mapped
//!         sw      a0, 0(t1)
//!         li      a5, 0x90000000
//!         jalr    x0, 0(a5)       ; and into the window
//!
//! payload:                        ; programmed into the flash, executed there
//!         li      t1, 0x20000000
//!         li      a0, 0x00c0ffee
//!         sw      a0, 4(t1)
//! done:   j       done
//! ```

// ---------------------------------------------------------------------------
// A very small assembler
// ---------------------------------------------------------------------------

/// `x0`.
const ZERO: u32 = 0;
/// `x1`, the return address.
const RA: u32 = 1;
/// `x5`.
const T0: u32 = 5;
/// `x6`.
const T1: u32 = 6;
/// `x7`.
const T2: u32 = 7;
/// `x10`.
const A0: u32 = 10;
/// `x11`.
const A1: u32 = 11;
/// `x12`.
const A2: u32 = 12;
/// `x13`.
const A3: u32 = 13;
/// `x15`.
const A5: u32 = 15;

const OP_LUI: u32 = 0b011_0111;
const OP_JAL: u32 = 0b110_1111;
const OP_JALR: u32 = 0b110_0111;
const OP_BRANCH: u32 = 0b110_0011;
const OP_LOAD: u32 = 0b000_0011;
const OP_STORE: u32 = 0b010_0011;
const OP_IMM: u32 = 0b001_0011;

const fn i_type(imm: i32, rs1: u32, funct3: u32, rd: u32, opcode: u32) -> u32 {
    ((imm as u32 & 0xfff) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode
}

const fn s_type(imm: i32, rs2: u32, rs1: u32, funct3: u32, opcode: u32) -> u32 {
    let imm = imm as u32;
    (((imm >> 5) & 0x7f) << 25)
        | (rs2 << 20)
        | (rs1 << 15)
        | (funct3 << 12)
        | ((imm & 0x1f) << 7)
        | opcode
}

const fn b_type(imm: i32, rs2: u32, rs1: u32, funct3: u32, opcode: u32) -> u32 {
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

const fn u_type(imm: u32, rd: u32, opcode: u32) -> u32 {
    (imm << 12) | (rd << 7) | opcode
}

const fn j_type(imm: i32, rd: u32, opcode: u32) -> u32 {
    let imm = imm as u32;
    (((imm >> 20) & 1) << 31)
        | (((imm >> 1) & 0x3ff) << 21)
        | (((imm >> 11) & 1) << 20)
        | (((imm >> 12) & 0xff) << 12)
        | (rd << 7)
        | opcode
}

const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b000, rd, OP_IMM)
}
const fn andi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b111, rd, OP_IMM)
}
const fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b010, rd, OP_LOAD)
}
const fn lbu(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b100, rd, OP_LOAD)
}
const fn sw(rs2: u32, rs1: u32, imm: i32) -> u32 {
    s_type(imm, rs2, rs1, 0b010, OP_STORE)
}
const fn sb(rs2: u32, rs1: u32, imm: i32) -> u32 {
    s_type(imm, rs2, rs1, 0b000, OP_STORE)
}
const fn beq(rs1: u32, rs2: u32, offset: i32) -> u32 {
    b_type(offset, rs2, rs1, 0b000, OP_BRANCH)
}
const fn bne(rs1: u32, rs2: u32, offset: i32) -> u32 {
    b_type(offset, rs2, rs1, 0b001, OP_BRANCH)
}
const fn jal(rd: u32, offset: i32) -> u32 {
    j_type(offset, rd, OP_JAL)
}
const fn jalr(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b000, rd, OP_JALR)
}
const fn ret() -> u32 {
    jalr(ZERO, RA, 0)
}

/// The `lui` half of a two-instruction `li`.
///
/// `addi` sign-extends its immediate, so a low half with bit 11 set borrows one
/// from the upper half. Rounding by `+0x800` before the shift is the standard
/// correction, and getting it wrong is off by exactly 4096 — which looks like a
/// memory-map bug rather than an encoding one.
const fn li_hi(rd: u32, value: u32) -> u32 {
    u_type((value.wrapping_add(0x800) >> 12) & 0xf_ffff, rd, OP_LUI)
}

/// The `addi` half.
const fn li_lo(rd: u32, value: u32) -> u32 {
    let low = (value & 0xfff) as i32;
    let low = if low >= 0x800 { low - 0x1000 } else { low };
    addi(rd, rd, low)
}

// ---------------------------------------------------------------------------
// The board's addresses
// ---------------------------------------------------------------------------

/// The STM32 SPI's register block on the `spi-flash` board.
pub const SPI1: u32 = 0xf000_0000;
/// The OCTOSPI's.
pub const OCTOSPI: u32 = 0xf000_1000;
/// Where the board's RAM starts.
pub const RAM: u32 = 0x2000_0000;
/// Where the OCTOSPI's memory-mapped window starts.
pub const WINDOW: u32 = 0x9000_0000;

/// The value the payload stores at `RAM + 4`, once it is executing out of the
/// flash. A test looks for exactly this.
pub const SENTINEL: u32 = 0x00c0_ffee;

/// `DCR1` for the board's 1 MiB part: `DEVSIZE = 19`, so `2^(19+1)` bytes.
const DCR1_1M: u32 = 19 << 16;

/// `CCR` for a single-line command with a 24-bit address and a data phase.
const CCR_SINGLE_24: u32 = 1 | (1 << 8) | (2 << 12) | (1 << 24);

/// `CR` with the peripheral enabled and `FMODE` selecting memory-mapped.
const CR_MEMORY_MAPPED: u32 = 1 | (3 << 28);

/// `CR1` for the STM32 SPI: master, enabled, `PCLK/16`.
const SPI_CR1: u32 = 0x04 | 0x40 | (3 << 3);

/// How long the payload is, in bytes.
pub const PAYLOAD_BYTES: u32 = 24;

// ---------------------------------------------------------------------------
// The program
// ---------------------------------------------------------------------------

/// Instruction index of `xfer`.
const XFER: i32 = 1;
/// Instruction index of `xwait`, inside it.
const XWAIT: i32 = 2;
/// Instruction index of `main`.
const MAIN: i32 = 7;
/// Instruction index of `copy`.
const COPY: i32 = 63;
/// Instruction index of `payload` — where the flash image starts in ROM.
const PAYLOAD: i32 = 83;
/// Instruction index of the payload's own halt loop.
const DONE: i32 = 88;

/// The byte offset from instruction `from` to instruction `to`.
const fn rel(from: i32, to: i32) -> i32 {
    (to - from) * 4
}

/// The assembled image, one `u32` per instruction.
///
/// The index constants above have to match this list exactly;
/// `the_label_indices_match_the_program` checks that they do rather than
/// trusting anybody to have counted right.
const PROGRAM: [u32; DONE as usize + 1] = [
    // 0
    jal(ZERO, rel(0, MAIN)),
    // 1: xfer
    sw(A1, T0, 0x0c),
    // 2: xwait
    lw(A0, T0, 0x08),
    andi(A0, A0, 1),
    beq(A0, ZERO, rel(4, XWAIT)),
    lw(A0, T0, 0x0c),
    ret(),
    // 7: main
    li_hi(T0, SPI1),
    li_lo(T0, SPI1),
    li_hi(T1, OCTOSPI),
    li_lo(T1, OCTOSPI),
    li_hi(T2, RAM),
    li_lo(T2, RAM),
    li_hi(A0, 4),
    li_lo(A0, 4),
    sw(A0, T0, 0x04),
    li_hi(A0, SPI_CR1),
    li_lo(A0, SPI_CR1),
    sw(A0, T0, 0x00),
    li_hi(A1, 0x9f),
    li_lo(A1, 0x9f),
    jal(RA, rel(21, XFER)),
    li_hi(A1, 0),
    li_lo(A1, 0),
    jal(RA, rel(24, XFER)),
    sb(A0, T2, 0),
    li_hi(A1, 0),
    li_lo(A1, 0),
    jal(RA, rel(28, XFER)),
    sb(A0, T2, 1),
    li_hi(A1, 0),
    li_lo(A1, 0),
    jal(RA, rel(32, XFER)),
    sb(A0, T2, 2),
    sw(ZERO, T0, 0x00),
    // 35: the OCTOSPI, in indirect write
    li_hi(A0, DCR1_1M),
    li_lo(A0, DCR1_1M),
    sw(A0, T1, 0x08),
    li_hi(A0, 1),
    li_lo(A0, 1),
    sw(A0, T1, 0x00),
    li_hi(A0, 1),
    li_lo(A0, 1),
    sw(A0, T1, 0x100),
    li_hi(A0, 6),
    li_lo(A0, 6),
    sw(A0, T1, 0x110),
    // 47: the page program
    li_hi(A0, CCR_SINGLE_24),
    li_lo(A0, CCR_SINGLE_24),
    sw(A0, T1, 0x100),
    li_hi(A0, 2),
    li_lo(A0, 2),
    sw(A0, T1, 0x110),
    li_hi(A0, PAYLOAD_BYTES - 1),
    li_lo(A0, PAYLOAD_BYTES - 1),
    sw(A0, T1, 0x40),
    li_hi(A0, 0),
    li_lo(A0, 0),
    sw(A0, T1, 0x48),
    li_hi(A1, PAYLOAD as u32 * 4),
    li_lo(A1, PAYLOAD as u32 * 4),
    li_hi(A2, PAYLOAD_BYTES),
    li_lo(A2, PAYLOAD_BYTES),
    // 63: copy
    lbu(A3, A1, 0),
    sb(A3, T1, 0x50),
    addi(A1, A1, 1),
    addi(A2, A2, -1),
    bne(A2, ZERO, rel(67, COPY)),
    // 68: memory-mapped mode
    li_hi(A0, CCR_SINGLE_24),
    li_lo(A0, CCR_SINGLE_24),
    sw(A0, T1, 0x100),
    li_hi(A0, 8),
    li_lo(A0, 8),
    sw(A0, T1, 0x108),
    li_hi(A0, 0x0b),
    li_lo(A0, 0x0b),
    sw(A0, T1, 0x110),
    li_hi(A0, CR_MEMORY_MAPPED),
    li_lo(A0, CR_MEMORY_MAPPED),
    sw(A0, T1, 0x00),
    li_hi(A5, WINDOW),
    li_lo(A5, WINDOW),
    jalr(ZERO, A5, 0),
    // 83: payload — copied into the flash, and executed from the window
    li_hi(T1, RAM),
    li_lo(T1, RAM),
    li_hi(A0, SENTINEL),
    li_lo(A0, SENTINEL),
    sw(A0, T1, 4),
    // 88: done
    jal(ZERO, 0),
];

/// The image as bytes, little-endian, ready to bind to a media slot.
pub const SPI_FLASH_DEMO: &[u8] = &{
    let mut out = [0u8; (DONE as usize + 1) * 4];
    let mut i = 0;
    while i < PROGRAM.len() {
        let word = PROGRAM[i];
        out[i * 4] = word as u8;
        out[i * 4 + 1] = (word >> 8) as u8;
        out[i * 4 + 2] = (word >> 16) as u8;
        out[i * 4 + 3] = (word >> 24) as u8;
        i += 1;
    }
    out
};

/// The bytes the firmware programs into the flash, which are the payload it
/// then executes out of the memory-mapped window.
///
/// A test uses it to check that the flash holds exactly what the guest wrote.
#[must_use]
pub fn payload() -> &'static [u8] {
    let at = PAYLOAD as usize * 4;
    &SPI_FLASH_DEMO[at..at + PAYLOAD_BYTES as usize]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_label_indices_match_the_program() {
        // Each label must land on the instruction the listing says it does,
        // and an off-by-one here is a jump into the middle of an `li` pair —
        // which executes and then goes wrong somewhere else entirely.
        assert_eq!(PROGRAM[XFER as usize], sw(A1, T0, 0x0c), "xfer");
        assert_eq!(PROGRAM[XWAIT as usize], lw(A0, T0, 0x08), "xwait");
        assert_eq!(PROGRAM[MAIN as usize], li_hi(T0, SPI1), "main");
        assert_eq!(PROGRAM[COPY as usize], lbu(A3, A1, 0), "copy");
        assert_eq!(PROGRAM[PAYLOAD as usize], li_hi(T1, RAM), "payload");
        assert_eq!(PROGRAM[DONE as usize], jal(ZERO, 0), "done");
    }

    #[test]
    fn the_payload_is_the_tail_of_the_image() {
        assert_eq!(payload().len(), PAYLOAD_BYTES as usize);
        assert_eq!(
            payload(),
            &SPI_FLASH_DEMO[SPI_FLASH_DEMO.len() - PAYLOAD_BYTES as usize..],
            "the payload is the last thing in the image, so a test can compare \
             the flash's contents with it directly"
        );
    }

    #[test]
    fn a_li_pair_reconstructs_the_value_it_names() {
        // The `+0x800` correction is the one thing in this assembler that is
        // easy to get subtly wrong, and every address on the board goes
        // through it.
        for value in [SPI1, OCTOSPI, RAM, WINDOW, SENTINEL, CCR_SINGLE_24, DCR1_1M] {
            let hi = (li_hi(A0, value) >> 12) << 12;
            let lo = (li_lo(A0, value) as i32) >> 20;
            assert_eq!(hi.wrapping_add(lo as u32), value, "{value:#x}");
        }
    }
}
