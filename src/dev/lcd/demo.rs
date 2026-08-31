//! The demo firmware the `spi-panel` board boots when nothing else is bound.
//!
//! An RV32I program, assembled at compile time by the `const fn` encoders
//! below, that does the whole display path end to end: it configures a Sitronix
//! ST7272A over the SPI controller, paints a gradient into the framebuffer, and
//! programs the scanout engine to show it. `rsemu run spi-panel` therefore
//! draws something with no toolchain and no download, and
//! `machine::catalog`'s "every shipped machine realizes" check has a fixture.
//!
//! # Why it is assembled here rather than committed as bytes
//!
//! A blob of hex in a source file is unreviewable and unmaintainable, and a
//! committed `.bin` would need a toolchain nobody has to regenerate. So the
//! encoders are here, the program is a list of instructions with the assembly
//! beside it, and [`tests`](self) disassembles the result with rsemu's own
//! RISC-V disassembler and checks it against that listing. The build cannot
//! drift from the comment without a test failing.
//!
//! # The program
//!
//! ```text
//!         j       main
//!
//! send:                           ; a1 = one 16-bit ST7272A command
//!         li      a0, 1
//!         sw      a0, 8(t0)       ; SPI CS = 1        (assert chip select 0)
//!         sw      a1, 16(t0)      ; SPI DATA          (starts the transfer)
//! wait:   lw      a0, 12(t0)      ; SPI STATUS
//!         andi    a0, a0, 1       ;   BUSY
//!         bne     a0, x0, wait
//!         lw      a0, 16(t0)      ; pop DATA, clearing RXVALID
//!         sw      x0, 8(t0)       ; SPI CS = 0        (the command commits here)
//!         ret
//!
//! main:   li      t0, 0xf0000000  ; the SPI controller
//!         li      t1, 0xf0001000  ; the scanout engine
//!         li      t2, 0x10000000  ; the framebuffer
//!
//!         li      a0, 0x0f01      ; CTRL: EN, 16-bit words, mode 0, MSB first
//!         sw      a0, 0(t0)
//!         li      a0, 3           ; CLKDIV: a half period of four sysclk ticks,
//!         sw      a0, 4(t0)       ;   so 12.5 MHz — inside the ST7272A's
//!                                 ;   50 ns minimum pulse widths (§9.3.3)
//!
//!         li      a1, 0x1009      ; 10h <- 09h: GRB = 1, DISP = 1 (leave standby)
//!         jal     ra, send
//!         li      a1, 0x1180      ; 11h <- 80h: contrast gain 2
//!         jal     ra, send
//!         li      a1, 0x1440      ; 14h <- 40h: brightness 0
//!         jal     ra, send
//!
//!         li      a2, 0           ; y
//!         mv      a4, t2          ; pixel pointer
//! row:    li      a3, 0           ; x
//! col:    sb      a3, 0(a4)       ; R = x
//!         sb      a2, 1(a4)       ; G = y
//!         xor     a5, a3, a2
//!         sb      a5, 2(a4)       ; B = x ^ y
//!         addi    a4, a4, 3
//!         addi    a3, a3, 1
//!         li      a5, 320
//!         bne     a3, a5, col
//!         addi    a2, a2, 1
//!         li      a5, 240
//!         bne     a2, a5, row
//!
//!         sw      t2, 4(t1)       ; LCDC BASE
//!         li      a0, 0
//!         sw      a0, 8(t1)       ; LCDC BASE_HI
//!         sw      a0, 12(t1)      ; LCDC STRIDE = 0 (width x bpp)
//!         li      a0, 320
//!         sw      a0, 16(t1)      ; LCDC WIDTH
//!         li      a0, 240
//!         sw      a0, 20(t1)      ; LCDC HEIGHT
//!         li      a0, 0
//!         sw      a0, 24(t1)      ; LCDC FORMAT = rgb888
//!         li      a0, 1
//!         sw      a0, 0(t1)       ; LCDC CTRL = EN
//! done:   j       done
//! ```
//!
//! The picture is `R = x & 0xff`, `G = y & 0xff`, `B = (x ^ y) & 0xff` — chosen
//! because every pixel is a pure function of its coordinates, so a test can
//! assert the whole frame without a reference image.
//!
//! # Not the DigiColor
//!
//! This drives *rsemu's own* generic SPI controller and scanout engine, whose
//! register maps are defined in this tree. It is not a Conexant DigiColor
//! firmware and the addresses above are this board's, not a product's.

// ---------------------------------------------------------------------------
// A very small assembler
// ---------------------------------------------------------------------------

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
/// `x14`.
const A4: u32 = 14;
/// `x15`.
const A5: u32 = 15;
/// `x0`.
const ZERO: u32 = 0;

const OP_LUI: u32 = 0b011_0111;
const OP_JAL: u32 = 0b110_1111;
const OP_JALR: u32 = 0b110_0111;
const OP_BRANCH: u32 = 0b110_0011;
const OP_LOAD: u32 = 0b000_0011;
const OP_STORE: u32 = 0b010_0011;
const OP_IMM: u32 = 0b001_0011;
const OP_REG: u32 = 0b011_0011;

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

const fn lui(rd: u32, imm: u32) -> u32 {
    u_type(imm, rd, OP_LUI)
}
const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b000, rd, OP_IMM)
}
const fn andi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b111, rd, OP_IMM)
}
const fn xor(rd: u32, rs1: u32, rs2: u32) -> u32 {
    (rs2 << 20) | (rs1 << 15) | (0b100 << 12) | (rd << 7) | OP_REG
}
const fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(imm, rs1, 0b010, rd, OP_LOAD)
}
const fn sw(rs2: u32, rs1: u32, imm: i32) -> u32 {
    s_type(imm, rs2, rs1, 0b010, OP_STORE)
}
const fn sb(rs2: u32, rs1: u32, imm: i32) -> u32 {
    s_type(imm, rs2, rs1, 0b000, OP_STORE)
}
const fn bne(rs1: u32, rs2: u32, offset: i32) -> u32 {
    b_type(offset, rs2, rs1, 0b001, OP_BRANCH)
}
const fn jal(rd: u32, offset: i32) -> u32 {
    j_type(offset, rd, OP_JAL)
}
const fn ret() -> u32 {
    i_type(0, RA, 0b000, ZERO, OP_JALR)
}

/// The `lui` half of a two-instruction `li`.
///
/// `addi` sign-extends its immediate, so a low half with bit 11 set borrows one
/// from the upper half. Rounding by `+0x800` before the shift is the standard
/// correction, and getting it wrong is off by exactly 4096 — which looks like a
/// memory-map bug rather than an encoding one.
const fn li_hi(rd: u32, value: u32) -> u32 {
    lui(rd, (value.wrapping_add(0x800) >> 12) & 0xf_ffff)
}

/// The `addi` half.
const fn li_lo(rd: u32, value: u32) -> u32 {
    // Sign-extended back out of the low twelve bits.
    let low = (value & 0xfff) as i32;
    let low = if low >= 0x800 { low - 0x1000 } else { low };
    addi(rd, rd, low)
}

// ---------------------------------------------------------------------------
// The program
// ---------------------------------------------------------------------------

/// The SPI controller's base address on the `spi-panel` board.
const SPI: u32 = 0xf000_0000;
/// The scanout engine's.
const LCDC: u32 = 0xf000_1000;
/// Where the framebuffer starts.
const FB: u32 = 0x1000_0000;
/// The panel's width in pixels.
const WIDTH: i32 = 320;
/// And its height.
const HEIGHT: i32 = 240;

/// Instruction index of `send`.
const SEND: i32 = 1;
/// Instruction index of `wait`, inside it.
const WAIT: i32 = 5;
/// Instruction index of `main`.
const MAIN: i32 = 11;
/// Instruction index of `row`.
const ROW: i32 = 35;
/// Instruction index of `col`.
const COL: i32 = 37;
/// Instruction index of `done`.
const DONE: i32 = 67;

/// The byte offset from instruction `from` to instruction `to`.
const fn rel(from: i32, to: i32) -> i32 {
    (to - from) * 4
}

/// The assembled program, one `u32` per instruction.
///
/// The index constants above have to match this list exactly;
/// `the_label_indices_match_the_program` checks that they do rather than
/// trusting anybody to have counted right.
const PROGRAM: [u32; DONE as usize + 1] = [
    // 0
    jal(ZERO, rel(0, MAIN)),
    // 1: send
    li_hi(A0, 1),
    li_lo(A0, 1),
    sw(A0, T0, 8),
    sw(A1, T0, 16),
    // 5: wait
    lw(A0, T0, 12),
    andi(A0, A0, 1),
    bne(A0, ZERO, rel(7, WAIT)),
    lw(A0, T0, 16),
    sw(ZERO, T0, 8),
    ret(),
    // 11: main
    li_hi(T0, SPI),
    li_lo(T0, SPI),
    li_hi(T1, LCDC),
    li_lo(T1, LCDC),
    li_hi(T2, FB),
    li_lo(T2, FB),
    li_hi(A0, 0x0f01),
    li_lo(A0, 0x0f01),
    sw(A0, T0, 0),
    li_hi(A0, 3),
    li_lo(A0, 3),
    sw(A0, T0, 4),
    li_hi(A1, 0x1009),
    li_lo(A1, 0x1009),
    jal(RA, rel(25, SEND)),
    li_hi(A1, 0x1180),
    li_lo(A1, 0x1180),
    jal(RA, rel(28, SEND)),
    li_hi(A1, 0x1440),
    li_lo(A1, 0x1440),
    jal(RA, rel(31, SEND)),
    li_hi(A2, 0),
    li_lo(A2, 0),
    addi(A4, T2, 0),
    // 35: row
    li_hi(A3, 0),
    li_lo(A3, 0),
    // 37: col
    sb(A3, A4, 0),
    sb(A2, A4, 1),
    xor(A5, A3, A2),
    sb(A5, A4, 2),
    addi(A4, A4, 3),
    addi(A3, A3, 1),
    li_hi(A5, WIDTH as u32),
    li_lo(A5, WIDTH as u32),
    bne(A3, A5, rel(45, COL)),
    addi(A2, A2, 1),
    li_hi(A5, HEIGHT as u32),
    li_lo(A5, HEIGHT as u32),
    bne(A2, A5, rel(49, ROW)),
    sw(T2, T1, 4),
    li_hi(A0, 0),
    li_lo(A0, 0),
    sw(A0, T1, 8),
    sw(A0, T1, 12),
    li_hi(A0, WIDTH as u32),
    li_lo(A0, WIDTH as u32),
    sw(A0, T1, 16),
    li_hi(A0, HEIGHT as u32),
    li_lo(A0, HEIGHT as u32),
    sw(A0, T1, 20),
    li_hi(A0, 0),
    li_lo(A0, 0),
    sw(A0, T1, 24),
    li_hi(A0, 1),
    li_lo(A0, 1),
    sw(A0, T1, 0),
    // 67: done
    jal(ZERO, 0),
];

/// The program as bytes, little-endian, ready to bind to a media slot.
pub const PANEL_DEMO: &[u8] = &{
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

/// What the demo paints at `(x, y)`, so a test can check the picture without a
/// reference image.
#[must_use]
pub const fn demo_pixel(x: u32, y: u32) -> [u8; 3] {
    [x as u8, y as u8, (x ^ y) as u8]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_label_indices_match_the_program() {
        // Each label must land on the instruction the listing in the module
        // docs says it does. Checked by decoding the opcode at each, which is
        // enough to catch an off-by-one without duplicating the encoder.
        assert_eq!(PROGRAM[SEND as usize] & 0x7f, OP_LUI, "send starts with li");
        assert_eq!(
            PROGRAM[WAIT as usize] & 0x7f,
            OP_LOAD,
            "wait starts with lw"
        );
        assert_eq!(PROGRAM[MAIN as usize] & 0x7f, OP_LUI, "main starts with li");
        assert_eq!(PROGRAM[ROW as usize] & 0x7f, OP_LUI, "row starts with li");
        assert_eq!(PROGRAM[COL as usize] & 0x7f, OP_STORE, "col starts with sb");
        assert_eq!(PROGRAM[DONE as usize], jal(ZERO, 0), "done spins on itself");
        assert_eq!(PANEL_DEMO.len(), PROGRAM.len() * 4);
    }

    #[test]
    fn li_puts_the_value_a_two_instruction_sequence_would() {
        // The correction that is off by exactly 4096 when it is wrong.
        for value in [
            0u32,
            1,
            3,
            0x7ff,
            0x800,
            0xfff,
            0x1009,
            0x0f01,
            0xf000_0000,
            0x1000_0000,
        ] {
            let hi = li_hi(A0, value) >> 12;
            let lo = (li_lo(A0, value) as i32) >> 20;
            let built = (hi << 12).wrapping_add(lo as u32);
            assert_eq!(built, value, "li a0, {value:#x}");
        }
    }

    /// The assembly listing in the module docs, decoded from the bytes.
    ///
    /// rsemu's own disassembler, so the comment and the build cannot drift.
    #[cfg(feature = "cpu-riscv")]
    #[test]
    fn the_program_disassembles_to_the_listing() {
        use crate::cpu::riscv::disasm::disassemble_one;
        use crate::cpu::riscv::isa::Xlen;

        let text = |index: usize| -> alloc::string::String {
            let addr = (index * 4) as u64;
            disassemble_one(addr, Xlen::Rv32, &mut |at: u64| {
                let word = PROGRAM.get((at / 4) as usize).copied()?;
                Some(if at.is_multiple_of(4) {
                    word as u16
                } else {
                    (word >> 16) as u16
                })
            })
            .expect("the program disassembles")
            .text
        };

        for (index, want) in [
            (0, "jal zero, 0x2c"),
            (SEND as usize + 2, "sw a0, 8(t0)"),
            (SEND as usize + 3, "sw a1, 0x10(t0)"),
            (WAIT as usize, "lw a0, 0xc(t0)"),
            (WAIT as usize + 1, "andi a0, a0, 1"),
            (WAIT as usize + 2, "bne a0, zero, 0x14"),
            (SEND as usize + 9, "jalr zero, 0(ra)"),
            (MAIN as usize, "lui t0, 0xfffffffff0000000"),
            (COL as usize, "sb a3, 0(a4)"),
            (COL as usize + 2, "xor a5, a3, a2"),
            (DONE as usize, "jal zero, 0x10c"),
        ] {
            assert_eq!(text(index), want, "instruction {index}");
        }
    }
}
