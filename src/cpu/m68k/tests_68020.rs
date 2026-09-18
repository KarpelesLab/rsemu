//! Hand-written tests for the 68020 model.
//!
//! Every expected value is computed from the manuals: the addressing modes
//! from M68000PRM §2.2 (Figure 2-2 and Table 2-2), the instructions from their
//! Section 4 pages, the frames from MC68020UM Table 6-5. The 68000 corpus run
//! through the 68020 in `conformance.rs` covers the instructions the two share.

use super::tests_68010::{Board, length_sweep};
use super::{Model, flags, vector};

/// A 68020 booted with `words` at `$400`, and a register file edited by
/// `edit`.
fn m68020(words: &[u16], edit: impl FnOnce(&mut super::Regs)) -> Board {
    let board = Board::new(Model::M68020);
    board.boot(words);
    board.with_regs(edit);
    board
}

// ---------------------------------------------------------------------------
// Addressing modes
// ---------------------------------------------------------------------------

#[test]
fn a_brief_extension_word_is_scaled() {
    // LEA $4(A0,D1.w*4),A2 — the scale is bits 10-9 (M68000PRM §2.2.3).
    let words = [0x45f0, 0x1404];
    let board = m68020(&words, |r| {
        r.a[0] = 0x1000;
        r.d[1] = 0x0000_0003;
    });
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[2], 0x1000 + 3 * 4 + 4);
    // A 68000 ignores bits 10-8 of a brief word.
    let old = Board::new(Model::M68000);
    old.boot(&words);
    old.with_regs(|r| {
        r.a[0] = 0x1000;
        r.d[1] = 3;
    });
    old.cpu.step();
    assert_eq!(old.cpu.regs().a[2], 0x1000 + 3 + 4);
}

#[test]
fn a_word_index_is_sign_extended_before_it_is_scaled() {
    // LEA $0(A0,D1.w*8),A2 with D1.w = -2.
    let board = m68020(&[0x45f0, 0x1600], |r| {
        r.a[0] = 0x1000;
        r.d[1] = 0x1234_fffe;
    });
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[2], 0x1000 - 16);
}

#[test]
fn a_full_format_word_adds_a_base_displacement() {
    // LEA ($1234,A0,D1.l*8),A2: D/A 0, D1, long, scale 8, full format, base
    // displacement a word, no indirection.
    let board = m68020(&[0x45f0, 0x1f20, 0x1234], |r| {
        r.a[0] = 0x1000;
        r.d[1] = 2;
    });
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.a[2], 0x1000 + 0x1234 + 16);
    assert_eq!(regs.pc, 0x406, "three words");
}

#[test]
fn a_base_displacement_word_is_sign_extended() {
    // LEA (-$10,A0),A2 with the index suppressed.
    let board = m68020(&[0x45f0, 0x0160, 0xfff0], |r| r.a[0] = 0x1000);
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[2], 0x0ff0);
}

#[test]
fn memory_indirect_postindexed_adds_the_index_after_the_fetch() {
    // LEA ([$10,A0],D1.w*2,$20),A2: the pointer is fetched from A0+$10, and
    // the index and the outer displacement are added to *it*.
    let board = m68020(&[0x45f0, 0x1326, 0x0010, 0x0020], |r| {
        r.a[0] = 0x1000;
        r.d[1] = 3;
    });
    board.poke_long(0x1010, 0x3000);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.a[2], 0x3000 + 3 * 2 + 0x20);
    assert_eq!(regs.pc, 0x408);
}

#[test]
fn memory_indirect_preindexed_adds_the_index_before_the_fetch() {
    // LEA ([$10,A0,D1.w*2],$20),A2.
    let board = m68020(&[0x45f0, 0x1322, 0x0010, 0x0020], |r| {
        r.a[0] = 0x1000;
        r.d[1] = 3;
    });
    board.poke_long(0x1016, 0x4000);
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[2], 0x4020);
}

#[test]
fn a_suppressed_base_and_index_leave_an_absolute_pointer() {
    // LEA ([$00001100.L]),A2: base and index suppressed, a long base
    // displacement, memory indirect with a null outer displacement.
    let board = m68020(&[0x45f0, 0x01f1, 0x0000, 0x1100], |r| r.a[0] = 0xdead_0000);
    board.poke_long(0x1100, 0x5000);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.a[2], 0x5000);
    assert_eq!(regs.pc, 0x408);
}

#[test]
fn a_long_outer_displacement_follows_the_base_displacement() {
    // LEA ([$10,A0],$00012345.L),A2: bd a word, od a long (I/IS 011).
    let board = m68020(&[0x45f0, 0x0163, 0x0010, 0x0001, 0x2345], |r| {
        r.a[0] = 0x1000
    });
    board.poke_long(0x1010, 0x0600_0000);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.a[2], 0x0601_2345);
    assert_eq!(regs.pc, 0x40a, "five words");
}

#[test]
fn the_pc_relative_base_is_the_extension_word() {
    // LEA ($100,PC,D0.w),A2 at $400: the extension word is at $402.
    let board = m68020(&[0x45fb, 0x0120, 0x0100], |r| r.d[0] = 8);
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[2], 0x402 + 0x100 + 8);
}

#[test]
fn an_operand_through_memory_indirection_is_read_and_written() {
    // MOVE.L ([$10,A0]),([$20,A1]): both ends indirect.
    let board = m68020(&[0x23b0, 0x0161, 0x0010, 0x0161, 0x0020], |r| {
        r.a[0] = 0x1000;
        r.a[1] = 0x1000;
    });
    board.poke_long(0x1010, 0x3000);
    board.poke_long(0x1020, 0x3100);
    board.poke_long(0x3000, 0xcafe_f00d);
    board.cpu.step();
    assert_eq!(board.peek_long(0x3100), 0xcafe_f00d);
    assert_eq!(board.cpu.regs().pc, 0x40a);
}

#[test]
fn a_reserved_full_format_word_is_an_illegal_instruction() {
    // BD SIZE 00 is reserved (M68000PRM Table 2-2).
    let board = m68020(&[0x45f0, 0x0100], |_| {});
    board.handler(0, vector::ILLEGAL, 0x0b00);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0xb00);
    assert_eq!(board.peek_long(u64::from(regs.a[7]) + 2), 0x400);
}

#[test]
fn jsr_through_memory_returns_after_every_extension_word() {
    // JSR ([$10,A0]) at $400, four bytes of extension.
    let board = m68020(&[0x4eb0, 0x0161, 0x0010], |r| r.a[0] = 0x1000);
    board.poke_long(0x1010, 0x0700);
    board.poke_word(0x700, 0x4e75); // RTS
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x700);
    assert_eq!(board.peek_long(u64::from(regs.a[7])), 0x406);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x406);
}

#[test]
fn a_misaligned_operand_is_not_an_address_error() {
    // MOVE.L D0,(A0) and MOVE.W (A1),D1 with odd addresses (MC68020UM
    // §6.1.3: address errors are for instruction fetches only).
    let board = m68020(&[0x2080, 0x3211], |r| {
        r.a[0] = 0x1001;
        r.a[1] = 0x1003;
        r.d[0] = 0x1122_3344;
    });
    board.cpu.step();
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x404);
    assert_eq!(board.ram.read_u8(0x1001).unwrap(), 0x11);
    assert_eq!(board.ram.read_u8(0x1004).unwrap(), 0x44);
    assert_eq!(regs.d[1] & 0xffff, 0x3344);
}

#[test]
fn the_68020_drives_all_32_address_lines_and_the_68ec020_24() {
    // MOVE.W #$1234,($01002000).L. The 68EC020 has 24 address pins, so
    // this is $002000; a 68020 reaches $01002000, which nothing answers.
    let words = [0x33fc, 0x1234, 0x0100, 0x2000];
    let ec = Board::new(Model::M68EC020);
    ec.boot(&words);
    ec.cpu.step();
    assert_eq!(ec.peek_word(0x2000), 0x1234);

    let full = Board::new(Model::M68020);
    full.boot(&words);
    full.handler(0, vector::BUS_ERROR, 0x0800);
    full.cpu.step();
    assert_eq!(full.cpu.regs().pc, 0x800, "a bus error");
    assert_eq!(full.peek_word(0x2000), 0, "nothing aliased");
}

#[test]
fn the_disassembler_prints_the_68020_modes() {
    use super::disasm::disassemble_for;
    let text = |words: &[u16]| alloc::format!("{}", disassemble_for(Model::M68020, 0x400, words));
    assert_eq!(text(&[0x45f0, 0x1404]), "LEA $4(A0,D1.w*4),A2");
    assert_eq!(text(&[0x45f0, 0x1f20, 0x1234]), "LEA ($1234,A0,D1.l*8),A2");
    assert_eq!(
        text(&[0x45f0, 0x1326, 0x0010, 0x0020]),
        "LEA ([$10,A0],D1.w*2,$20),A2"
    );
    assert_eq!(
        text(&[0x45f0, 0x1322, 0x0010, 0x0020]),
        "LEA ([$10,A0,D1.w*2],$20),A2"
    );
    assert_eq!(
        text(&[0x45f0, 0x01f1, 0x0000, 0x1100]),
        "LEA ([$1100,ZA0]),A2"
    );
    assert_eq!(text(&[0x45f0, 0x0160, 0xfff0]), "LEA (-$10,A0),A2");
    assert_eq!(text(&[0x45fb, 0x0120, 0x0100]), "LEA ($100,PC,D0.w),A2");
    assert_eq!(text(&[0x45f0, 0x0100]), "LEA <reserved $0100>,A2");
    // Lengths come from the words.
    assert_eq!(
        disassemble_for(Model::M68020, 0, &[0x45f0, 0x0163, 1, 2, 3]).len,
        10
    );
    assert_eq!(
        disassemble_for(Model::M68020, 0, &[0x23b0, 0x0161, 0x0010, 0x0161, 0x0020]).len,
        10
    );
    // A 68000 reads the same words as a brief format and stops after one.
    assert_eq!(
        super::disasm::disassemble(0, &[0x45f0, 0x0163, 1, 2, 3]).len,
        4
    );
}

#[test]
fn the_disassembler_and_the_68020_agree_on_every_full_format_word() {
    // Every one of the 65 536 possible first extension words under LEA,
    // indexed from A0 and from the PC: the length the disassembler reports is
    // the distance the program counter moved.
    use super::disasm::disassemble_for;
    let board = Board::new(Model::M68020);
    board.poke_long(0, 0x2000);
    board.poke_long(4, 0x0400);
    board.cpu.step();
    let mut checked = 0usize;
    for opcode in [0x43f0u16, 0x43fb] {
        for word in 0..=u16::MAX {
            let words = [opcode, word, 0x0010, 0x0010, 0x0010, 0x0010];
            board.load(0x400, &words);
            board.cpu.request_reset();
            board.cpu.step();
            board.with_regs(|r| {
                r.a = [0x1000; 8];
                r.a[7] = 0x2000;
                r.ssp = 0x2000;
                r.d = [4; 8];
            });
            let expected = disassemble_for(Model::M68020, 0x400, &words);
            board.cpu.step();
            if board.cpu.last_exception().is_some() {
                continue;
            }
            assert_eq!(
                board.cpu.regs().pc - 0x400,
                u32::from(expected.len),
                "{opcode:04x} {word:04x}: {expected}"
            );
            checked += 1;
        }
    }
    assert!(checked > 60_000, "only {checked} words were exercised");
}

#[test]
fn the_68ec020_masks_only_at_the_pins() {
    // An address is computed in 32 bits and then presented on 24 pins: a
    // base displacement that carries into bit 24 wraps there, not before.
    let board = Board::new(Model::M68EC020);
    // LEA ($00FFFFF0.L,A0),A1 then MOVE.W #$abcd,(A1).
    board.boot(&[0x43f0, 0x0170, 0x00ff, 0xfff0, 0x32bc, 0xabcd]);
    board.with_regs(|r| r.a[0] = 0x20);
    board.cpu.step();
    assert_eq!(
        board.cpu.regs().a[1],
        0x0100_0010,
        "the register holds 32 bits"
    );
    board.cpu.step();
    assert_eq!(board.peek_word(0x10), 0xabcd, "the pins see $000010");
}

#[test]
fn tst_and_cmpi_reach_the_68020s_extra_modes() {
    // TST.W A0 and TST.L #0, and CMPI.W #1,$2(PC).
    let board = m68020(
        &[0x4a48, 0x4abc, 0x0000, 0x0000, 0x0c7a, 0x0001, 0x0002],
        |r| {
            r.a[0] = 0x8000;
        },
    );
    board.cpu.step();
    assert!(board.cpu.regs().flag(flags::N));
    board.cpu.step();
    assert!(board.cpu.regs().flag(flags::Z));
    // The displacement's base is its own address, $40c, so the operand is
    // the word at $40e.
    board.poke_word(0x40e, 0x0001);
    board.cpu.step();
    assert!(board.cpu.regs().flag(flags::Z), "the word at $40e is 1");
}

// ---------------------------------------------------------------------------
// Instructions
// ---------------------------------------------------------------------------

/// Run one instruction on a fresh 68020 and return the registers after it.
fn one(words: &[u16], edit: impl FnOnce(&mut super::Regs)) -> (Board, super::Regs) {
    let board = m68020(words, edit);
    board.cpu.step();
    let regs = board.cpu.regs();
    (board, regs)
}

/// The condition codes as `XNZVC` bits.
fn ccr(regs: &super::Regs) -> u16 {
    regs.sr & flags::CCR
}

const N: u16 = flags::N;
const Z: u16 = flags::Z;
const V: u16 = flags::V;
const C: u16 = flags::C;
const X: u16 = flags::X;

#[test]
fn mulu_l_keeps_32_bits_and_says_when_it_lost_some() {
    // MULU.L D1,D2: $10000 × $10000 = $1_0000_0000, low long zero.
    let (_, r) = one(&[0x4c01, 0x2000], |r| {
        r.d[1] = 0x1_0000;
        r.d[2] = 0x1_0000;
        r.sr |= C | X;
    });
    assert_eq!(r.d[2], 0);
    assert_eq!(ccr(&r), X | Z | V, "Z and V; C cleared; X untouched");
}

#[test]
fn muls_l_into_a_register_pair_is_64_bits() {
    // MULS.L D1,D3:D2: -2 × $4000_0000 = -$8000_0000.
    let (_, r) = one(&[0x4c01, 0x2c03], |r| {
        r.d[1] = 0xffff_fffe;
        r.d[2] = 0x4000_0000;
    });
    assert_eq!((r.d[3], r.d[2]), (0xffff_ffff, 0x8000_0000));
    assert_eq!(ccr(&r), N, "a 64-bit product cannot overflow");
}

#[test]
fn muls_l_overflows_when_the_high_long_is_not_a_sign_extension() {
    // MULS.L #-1,D0 with D0 = $8000_0000: +2^31 does not fit.
    let (_, r) = one(&[0x4c3c, 0x0800, 0xffff, 0xffff], |r| r.d[0] = 0x8000_0000);
    assert_eq!(r.d[0], 0x8000_0000);
    assert_eq!(ccr(&r), N | V);
    assert_eq!(r.pc, 0x408, "opcode, extension word, immediate long");
}

#[test]
fn divu_l_and_divul_l_divide_32_bits() {
    // DIVU.L #7,D1: 100 / 7 = 14, remainder discarded.
    let (_, r) = one(&[0x4c7c, 0x1001, 0x0000, 0x0007], |r| r.d[1] = 100);
    assert_eq!(r.d[1], 14);
    assert_eq!(ccr(&r), 0);
    // DIVUL.L #7,D2:D1: the remainder in D2.
    let (_, r) = one(&[0x4c7c, 0x1002, 0x0000, 0x0007], |r| {
        r.d[1] = 100;
        r.d[2] = 0xdead;
    });
    assert_eq!((r.d[1], r.d[2]), (14, 2));
}

#[test]
fn divs_l_divides_64_bits_and_the_remainder_takes_the_dividends_sign() {
    // DIVS.L #-3,D2:D1 with D2:D1 = -10: quotient 3, remainder -1.
    let (_, r) = one(&[0x4c7c, 0x1c02, 0xffff, 0xfffd], |r| {
        r.d[2] = 0xffff_ffff;
        r.d[1] = 0xffff_fff6;
    });
    assert_eq!((r.d[1], r.d[2]), (3, 0xffff_ffff));
    assert_eq!(ccr(&r), 0);
}

#[test]
fn a_quotient_that_does_not_fit_sets_v_and_changes_nothing() {
    // DIVU.L #1,D2:D1 with D2:D1 = 2^32.
    let (_, r) = one(&[0x4c7c, 0x1402, 0x0000, 0x0001], |r| {
        r.d[2] = 1;
        r.d[1] = 0;
    });
    assert_eq!((r.d[1], r.d[2]), (0, 1));
    assert!(r.flag(V));
    // DIVS.L #-1,D1 with D1 = -2^31: +2^31 does not fit either.
    let (_, r) = one(&[0x4c7c, 0x1801, 0xffff, 0xffff], |r| r.d[1] = 0x8000_0000);
    assert_eq!(r.d[1], 0x8000_0000);
    assert!(r.flag(V));
}

#[test]
fn a_zero_divide_pushes_the_six_word_frame() {
    // DIVS.L D0,D1 with D0 = 0. MC68020UM Table 6-5: format $2, the next
    // instruction in the PC field and the divide's own address at +8.
    let board = m68020(&[0x4c40, 0x1801], |r| r.d[1] = 5);
    board.handler(0, vector::DIVIDE_BY_ZERO, 0x0900);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x900);
    let sp = u64::from(r.a[7]);
    assert_eq!(sp, 0x2000 - 12);
    assert_eq!(board.peek_long(sp + 2), 0x404, "+2 the next instruction");
    assert_eq!(board.peek_word(sp + 6), 0x2014, "+6 format 2, offset $014");
    assert_eq!(
        board.peek_long(sp + 8),
        0x400,
        "+8 the instruction's address"
    );
    // The word form too.
    let board = m68020(&[0x83c0], |r| r.d[1] = 5); // DIVS.W D0,D1
    board.handler(0, vector::DIVIDE_BY_ZERO, 0x0900);
    board.cpu.step();
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_long(sp + 2), 0x402);
    assert_eq!(board.peek_long(sp + 8), 0x400);
}

#[test]
fn bftst_reads_a_field_from_the_top_of_a_register() {
    // BFTST D0{4:8}: bits 27-20 of $0F00_0000 are 1111_0000.
    let (_, r) = one(&[0xe8c0, 0x0108], |r| r.d[0] = 0x0f00_0000);
    assert_eq!(ccr(&r), N);
}

#[test]
fn a_field_in_a_register_wraps_from_bit_0_to_bit_31() {
    // BFEXTU D0{28:8},D1: bits 3-0 then 31-28, so $5000_000A gives $A5.
    let (_, r) = one(&[0xe9c0, 0x1708], |r| r.d[0] = 0x5000_000a);
    assert_eq!(r.d[1], 0xa5);
    assert_eq!(ccr(&r), N);
}

#[test]
fn a_negative_register_offset_is_taken_modulo_32_in_a_register() {
    // BFEXTS D0{D2:4},D1 with D2 = -2: bits 1, 0, 31, 30 of $8000_0002 are
    // 1010, sign-extended.
    let (_, r) = one(&[0xebc0, 0x1884], |r| {
        r.d[0] = 0x8000_0002;
        r.d[2] = (-2i32) as u32;
    });
    assert_eq!(r.d[1], 0xffff_fffa);
    assert_eq!(ccr(&r), N);
}

#[test]
fn a_negative_register_offset_reaches_below_the_base_in_memory() {
    // BFEXTU (A0){D2:12},D1 with A0 = $1002 and D2 = -4: the field starts at
    // bit 4 of the byte at $1001 and runs through $1002.
    let (_, r) = one(&[0xe9d0, 0x188c], |r| {
        r.a[0] = 0x1002;
        r.d[2] = (-4i32) as u32;
    });
    // Set up memory after boot, then run again from the top.
    let _ = r;
    let board = m68020(&[0xe9d0, 0x188c], |r| {
        r.a[0] = 0x1002;
        r.d[2] = (-4i32) as u32;
    });
    board.ram.write_u8(0x1001, 0x3c).unwrap();
    board.ram.write_u8(0x1002, 0x5a).unwrap();
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.d[1], 0xc5a);
    assert_eq!(ccr(&r), N, "the field's top bit is set");
}

#[test]
fn bfins_can_straddle_five_bytes() {
    // BFINS D3,(A0){30:32}: from bit 6 of $1003 through bit 5 of $1007.
    let board = m68020(&[0xefd0, 0x3780], |r| {
        r.a[0] = 0x1000;
        r.d[3] = 0x1234_5678;
    });
    board.ram.write_u8(0x1003, 0xfc).unwrap();
    board.ram.write_u8(0x1007, 0x03).unwrap();
    board.cpu.step();
    let bytes: alloc::vec::Vec<u8> = (0x1003..0x1008)
        .map(|a| board.ram.read_u8(a).unwrap())
        .collect();
    // The bits outside the field keep what they had.
    assert_eq!(bytes, [0xfc, 0x48, 0xd1, 0x59, 0xe3]);
    assert_eq!(ccr(&board.cpu.regs()), 0, "flags from the inserted value");
}

#[test]
fn bfffo_counts_from_the_field_offset() {
    // BFFFO D0{8:16},D1 with bit 12 the first one: 8 + 11.
    let (_, r) = one(&[0xedc0, 0x1210], |r| r.d[0] = 0x0000_1000);
    assert_eq!(r.d[1], 19);
    assert_eq!(ccr(&r), 0);
    // No one at all: offset plus width, and Z.
    let (_, r) = one(&[0xedc0, 0x1210], |r| r.d[0] = 0);
    assert_eq!(r.d[1], 24);
    assert_eq!(ccr(&r), Z);
}

#[test]
fn bfset_bfclr_and_bfchg_flag_the_field_as_it_was() {
    // BFSET (A0){0:4}, BFCLR (A0){4:4}, BFCHG (A0){0:8} on $5A.
    let board = m68020(&[0xeed0, 0x0004, 0xecd0, 0x0104, 0xead0, 0x0008], |r| {
        r.a[0] = 0x1000;
    });
    board.ram.write_u8(0x1000, 0x5a).unwrap();
    board.cpu.step();
    assert_eq!(board.ram.read_u8(0x1000).unwrap(), 0xfa);
    assert_eq!(ccr(&board.cpu.regs()), 0, "the old field was 0101");
    board.cpu.step();
    assert_eq!(board.ram.read_u8(0x1000).unwrap(), 0xf0);
    assert_eq!(ccr(&board.cpu.regs()), N, "the old field was 1010");
    board.cpu.step();
    assert_eq!(board.ram.read_u8(0x1000).unwrap(), 0x0f);
    assert_eq!(ccr(&board.cpu.regs()), N);
}

#[test]
fn a_32_bit_field_in_a_register_is_the_whole_register() {
    // BFCLR D0{0:0} — width 0 means 32.
    let (_, r) = one(&[0xecc0, 0x0000], |r| r.d[0] = 0x8000_0001);
    assert_eq!(r.d[0], 0);
    assert_eq!(ccr(&r), N);
}

#[test]
fn cas_swaps_when_equal_and_loads_when_not() {
    // CAS.L D0,D1,(A0).
    let board = m68020(&[0x0ed0, 0x0040, 0x0ed0, 0x0040], |r| {
        r.a[0] = 0x1000;
        r.d[0] = 5;
        r.d[1] = 9;
    });
    board.poke_long(0x1000, 5);
    board.cpu.step();
    assert_eq!(board.peek_long(0x1000), 9);
    assert_eq!(ccr(&board.cpu.regs()), Z);
    // Again: memory now 9, D0 still 5 — not equal, D0 loaded.
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.d[0], 9);
    assert_eq!(board.peek_long(0x1000), 9);
    assert_eq!(ccr(&r), 0, "9 - 5");
}

#[test]
fn cas_b_compares_and_loads_only_a_byte() {
    // CAS.B D0,D1,(A0) with memory $80 and D0.b $01: $80 - $01 overflows.
    let board = m68020(&[0x0ad0, 0x0040], |r| {
        r.a[0] = 0x1000;
        r.d[0] = 0xaaaa_aa01;
    });
    board.ram.write_u8(0x1000, 0x80).unwrap();
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.d[0], 0xaaaa_aa80);
    assert_eq!(ccr(&r), V);
}

#[test]
fn cas2_writes_both_only_when_both_match() {
    // CAS2.L D0:D1,D2:D3,(A0):(A1).
    let words = [0x0efc, 0x8080, 0x90c1];
    let setup = |r: &mut super::Regs| {
        r.a[0] = 0x1000;
        r.a[1] = 0x1010;
        r.d[0] = 1;
        r.d[1] = 2;
        r.d[2] = 0x11;
        r.d[3] = 0x22;
    };
    let board = m68020(&words, setup);
    board.poke_long(0x1000, 1);
    board.poke_long(0x1010, 2);
    board.cpu.step();
    assert_eq!(
        (board.peek_long(0x1000), board.peek_long(0x1010)),
        (0x11, 0x22)
    );
    assert!(board.cpu.regs().flag(Z));

    let board = m68020(&words, setup);
    board.poke_long(0x1000, 1);
    board.poke_long(0x1010, 7);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(
        (board.peek_long(0x1000), board.peek_long(0x1010)),
        (1, 7),
        "nothing written"
    );
    assert_eq!((r.d[0], r.d[1]), (1, 7), "both compare registers loaded");
    assert_eq!(ccr(&r), 0, "flags from the second comparison, 7 - 2");
}

#[test]
fn cmp2_checks_a_signed_or_unsigned_bounds_pair() {
    // CMP2.W (A0),D1 against 10..20.
    let run = |value: u32| {
        let board = m68020(&[0x02d0, 0x1000], |r| {
            r.a[0] = 0x1000;
            r.d[1] = value;
        });
        board.poke_word(0x1000, 10);
        board.poke_word(0x1002, 20);
        board.cpu.step();
        ccr(&board.cpu.regs()) & (Z | C)
    };
    assert_eq!(run(15), 0);
    assert_eq!(run(10), Z);
    assert_eq!(run(20), Z);
    assert_eq!(run(21), C);
    assert_eq!(run(9), C);
    assert_eq!(run(0xffff_000f), 0, "only the low word of a data register");
    // A signed pair, -5..5, as bytes.
    let run = |value: u32| {
        let board = m68020(&[0x00d0, 0x1000], |r| {
            r.a[0] = 0x1000;
            r.d[1] = value;
        });
        board.ram.write_u8(0x1000, 0xfb).unwrap();
        board.ram.write_u8(0x1001, 0x05).unwrap();
        board.cpu.step();
        ccr(&board.cpu.regs()) & (Z | C)
    };
    assert_eq!(run(0xfb), Z);
    assert_eq!(run(0xff), 0);
    assert_eq!(run(0x80), C);
    assert_eq!(run(0x06), C);
}

#[test]
fn cmp2_on_an_address_register_compares_32_bits_against_extended_bounds() {
    // CMP2.W (A0),A1 against -16..16, sign-extended.
    let run = |value: u32| {
        let board = m68020(&[0x02d0, 0x9000], |r| {
            r.a[0] = 0x1000;
            r.a[1] = value;
        });
        board.poke_word(0x1000, 0xfff0);
        board.poke_word(0x1002, 0x0010);
        board.cpu.step();
        ccr(&board.cpu.regs()) & (Z | C)
    };
    assert_eq!(run(0xffff_fff8), 0);
    assert_eq!(run(0x0000_fff8), C, "not -8 once all 32 bits count");
}

#[test]
fn chk2_traps_with_the_six_word_frame() {
    let board = m68020(&[0x02d0, 0x1800], |r| {
        r.a[0] = 0x1000;
        r.d[1] = 30;
    });
    board.poke_word(0x1000, 10);
    board.poke_word(0x1002, 20);
    board.handler(0, vector::CHK, 0x0900);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x900);
    let sp = u64::from(r.a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x2018, "format 2, offset $018");
    assert_eq!(board.peek_long(sp + 2), 0x404);
    assert_eq!(board.peek_long(sp + 8), 0x400);
    assert!(r.flag(C) || board.peek_word(sp) & C != 0);
}

#[test]
fn chk_l_compares_all_32_bits() {
    // CHK.L D1,D0.
    let (_, r) = one(&[0x4101], |r| {
        r.d[0] = 0x1_0000;
        r.d[1] = 0x2_0000;
    });
    assert_eq!(r.pc, 0x402, "in bounds");
    let board = m68020(&[0x4101], |r| {
        r.d[0] = 0x3_0000;
        r.d[1] = 0x2_0000;
    });
    board.handler(0, vector::CHK, 0x0900);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x900);
}

#[test]
fn pack_and_unpk_between_registers() {
    // PACK D0,D1,#0: $0304 -> $34.
    let (_, r) = one(&[0x8340, 0x0000], |r| {
        r.d[0] = 0x0304;
        r.d[1] = 0xffff_ffff;
    });
    assert_eq!(r.d[1], 0xffff_ff34);
    // UNPK D0,D1,#$3030: $34 -> $0304 + $3030.
    let (_, r) = one(&[0x8380, 0x3030], |r| {
        r.d[0] = 0x34;
        r.d[1] = 0xffff_ffff;
        r.sr |= Z;
    });
    assert_eq!(r.d[1], 0xffff_3334);
    assert!(r.flag(Z), "condition codes untouched");
}

#[test]
fn pack_and_unpk_through_memory_by_predecrement() {
    // PACK -(A0),-(A1),#0 with "34" at $0ffe.
    let board = m68020(&[0x8348, 0x0000, 0x8388, 0x3030], |r| {
        r.a[0] = 0x1000;
        r.a[1] = 0x1100;
    });
    board.ram.write_u8(0x0ffe, b'3').unwrap();
    board.ram.write_u8(0x0fff, b'4').unwrap();
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!((r.a[0], r.a[1]), (0x0ffe, 0x10ff));
    assert_eq!(board.ram.read_u8(0x10ff).unwrap(), 0x34);
    // UNPK -(A0),-(A1),#$3030 with $56 at $0ffd.
    board.ram.write_u8(0x0ffd, 0x56).unwrap();
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!((r.a[0], r.a[1]), (0x0ffd, 0x10fd));
    assert_eq!(board.ram.read_u8(0x10fd).unwrap(), b'5');
    assert_eq!(board.ram.read_u8(0x10fe).unwrap(), b'6');
}

#[test]
fn extb_extends_a_byte_to_a_long() {
    let (_, r) = one(&[0x49c0], |r| r.d[0] = 0x1234_5680);
    assert_eq!(r.d[0], 0xffff_ff80);
    assert_eq!(ccr(&r), N);
}

#[test]
fn link_l_takes_a_32_bit_displacement() {
    // LINK.L A6,#-$20.
    let (board, r) = one(&[0x480e, 0xffff, 0xffe0], |r| r.a[6] = 0xcafe);
    assert_eq!(r.a[6], 0x1ffc);
    assert_eq!(r.a[7], 0x1ffc - 0x20);
    assert_eq!(board.peek_long(0x1ffc), 0xcafe);
    assert_eq!(r.pc, 0x406);
}

#[test]
fn trapcc_traps_on_its_condition_after_skipping_its_operand() {
    // TRAPEQ.W #$1234 with Z set: vector 7, six-word frame.
    let board = m68020(&[0x57fa, 0x1234], |r| r.sr |= Z);
    board.handler(0, vector::TRAPV, 0x0900);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x900);
    let sp = u64::from(r.a[7]);
    assert_eq!(board.peek_long(sp + 2), 0x404);
    assert_eq!(board.peek_word(sp + 6), 0x201c);
    assert_eq!(board.peek_long(sp + 8), 0x400);
    // TRAPNE.L with Z set: nothing, and three words consumed.
    let (_, r) = one(&[0x56fb, 0x1234, 0x5678], |r| r.sr |= Z);
    assert_eq!(r.pc, 0x406);
    // TRAPF, the no-operand form.
    let (_, r) = one(&[0x51fc], |_| {});
    assert_eq!(r.pc, 0x402);
}

#[test]
fn a_ff_displacement_is_a_32_bit_branch() {
    // BRA.L +$100 from $400 lands at $502.
    let (_, r) = one(&[0x60ff, 0x0000, 0x0100], |_| {});
    assert_eq!(r.pc, 0x502);
    // BSR.L pushes the address after all three words.
    let (board, r) = one(&[0x61ff, 0x0000, 0x0100], |_| {});
    assert_eq!(r.pc, 0x502);
    assert_eq!(board.peek_long(u64::from(r.a[7])), 0x406);
    // BNE.L not taken falls through past them.
    let (_, r) = one(&[0x66ff, 0x0000, 0x0100], |r| r.sr |= Z);
    assert_eq!(r.pc, 0x406);
    // On a 68010 the same byte is a branch by -1, to an odd address.
    let old = super::tests_68010::Board::new(Model::M68010);
    old.boot(&[0x60ff]);
    old.handler(0, vector::ADDRESS_ERROR, 0x0800);
    old.cpu.step();
    assert_eq!(old.cpu.regs().pc, 0x800);
}

#[test]
fn callm_and_rtm_through_a_type_0_descriptor() {
    // CALLM #4,(A0) with a descriptor at $3000 whose entry word names A5.
    let board = m68020(&[0x06d0, 0x0004, 0x4e71], |r| {
        r.a[0] = 0x3000;
        r.a[5] = 0x1111;
        r.sr |= X | C;
    });
    board.poke_long(0x3000, 0x0000_0000); // opt 0, type 0
    board.poke_long(0x3004, 0x3100); // entry
    board.poke_long(0x3008, 0xda7a_0000); // module data
    board.poke_word(0x3100, 0xd000); // entry word: A5
    board.poke_word(0x3102, 0x06cd); // RTM A5
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x3102, "the word after the entry word");
    assert_eq!(r.a[5], 0xda7a_0000);
    let sp = r.a[7];
    assert_eq!(sp, 0x2000 - 24);
    let sp = u64::from(sp);
    // MC68020UM Figure 9-12.
    assert_eq!(board.peek_word(sp), 0x0000, "opt, type, access level");
    assert_eq!(
        board.peek_word(sp + 2),
        X | C,
        "the caller's condition codes"
    );
    assert_eq!(board.peek_word(sp + 4), 4, "the argument count");
    assert_eq!(board.peek_long(sp + 8), 0x3000, "the descriptor");
    assert_eq!(board.peek_long(sp + 12), 0x404, "the return address");
    assert_eq!(board.peek_long(sp + 16), 0x1111, "the saved data pointer");
    // RTM A5 undoes it and drops the four argument bytes.
    board.with_regs(|r| r.sr &= !(X | C));
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x404);
    assert_eq!(r.a[5], 0x1111);
    assert_eq!(r.a[7], 0x2000 + 4);
    assert_eq!(ccr(&r), X | C);
}

#[test]
fn callm_with_a_type_1_descriptor_is_a_format_error() {
    // Type 1 needs access-control hardware in CPU space; there is none.
    let board = m68020(&[0x06d0, 0x0000], |r| r.a[0] = 0x3000);
    board.poke_long(0x3000, 0x0100_0000);
    board.handler(0, vector::FORMAT_ERROR, 0x0900);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x900);
    assert_eq!(board.peek_long(u64::from(r.a[7]) + 2), 0x400);
}

#[test]
fn with_no_coprocessor_every_f_line_word_is_line_f_but_cpsave_is_privileged() {
    // cpSAVE (A0) for coprocessor 1, in user state: privilege violation.
    let board = m68020(&[0xf310], |r| {
        r.sr = 0;
        r.usp = 0x1800;
    });
    board.handler(0, vector::PRIVILEGE, 0x0a00);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xa00);
    // In supervisor state it is the line-F trap, as is any cpGEN.
    for words in [[0xf310u16], [0xf200]] {
        let board = m68020(&words, |_| {});
        board.handler(0, vector::LINE_F, 0x0c00);
        board.cpu.step();
        let r = board.cpu.regs();
        assert_eq!(r.pc, 0xc00);
        assert_eq!(board.peek_word(u64::from(r.a[7]) + 6), 0x002c, "format 0");
    }
}

#[test]
fn movec_reaches_the_cache_registers_and_the_stack_pointers() {
    let board = m68020(
        &[
            0x700f, // MOVEQ #15,D0
            0x4e7b, 0x0002, // MOVEC D0,CACR
            0x4e7a, 0x1002, // MOVEC CACR,D1
            0x4e7b, 0x0802, // MOVEC D0,CAAR
            0x4e7a, 0x2802, // MOVEC CAAR,D2
            0x207c, 0x0000, 0x1500, // MOVEA.L #$1500,A0
            0x4e7b, 0x8803, // MOVEC A0,MSP
            0x4e7a, 0xb804, // MOVEC ISP,A3
        ],
        |_| {},
    );
    for _ in 0..8 {
        board.cpu.step();
    }
    let r = board.cpu.regs();
    assert_eq!(r.d[1], 3, "only E and F have storage; C and CE read zero");
    assert_eq!(r.d[2], 15);
    assert_eq!(r.msp, 0x1500);
    assert_eq!(r.a[3], 0x2000, "ISP is the stack in use");
}

#[test]
fn the_disassembler_speaks_68020() {
    use super::disasm::disassemble_for;
    let text = |words: &[u16]| alloc::format!("{}", disassemble_for(Model::M68020, 0x400, words));
    assert_eq!(text(&[0x4c01, 0x2000]), "MULU.L D1,D2");
    assert_eq!(text(&[0x4c01, 0x2c03]), "MULS.L D1,D3:D2");
    assert_eq!(text(&[0x4c7c, 0x1001, 0x0000, 0x0007]), "DIVU.L #$7,D1");
    assert_eq!(text(&[0x4c7c, 0x1002, 0x0000, 0x0007]), "DIVUL.L #$7,D2:D1");
    assert_eq!(
        text(&[0x4c7c, 0x1c02, 0xffff, 0xfffd]),
        "DIVS.L #$fffffffd,D2:D1"
    );
    assert_eq!(text(&[0xe8c0, 0x0108]), "BFTST D0{4:8}");
    assert_eq!(text(&[0xebc0, 0x1884]), "BFEXTS D0{D2:4},D1");
    assert_eq!(text(&[0xefd0, 0x3780]), "BFINS D3,(A0){30:32}");
    assert_eq!(text(&[0xedc0, 0x1210]), "BFFFO D0{8:16},D1");
    assert_eq!(text(&[0x0ed0, 0x0040]), "CAS.L D0,D1,(A0)");
    assert_eq!(
        text(&[0x0efc, 0x8080, 0x90c1]),
        "CAS2.L D0:D1,D2:D3,(A0):(A1)"
    );
    assert_eq!(text(&[0x02d0, 0x1000]), "CMP2.W (A0),D1");
    assert_eq!(text(&[0x02d0, 0x9800]), "CHK2.W (A0),A1");
    assert_eq!(text(&[0x8348, 0x0000]), "PACK -(A0),-(A1),#$0");
    assert_eq!(text(&[0x8380, 0x3030]), "UNPK D0,D1,#$3030");
    assert_eq!(text(&[0x49c0]), "EXTB.L D0");
    assert_eq!(text(&[0x480e, 0xffff, 0xffe0]), "LINK.L A6,#-$20");
    assert_eq!(text(&[0x57fa, 0x1234]), "TRAPEQ.W #$1234");
    assert_eq!(text(&[0x56fb, 0x1234, 0x5678]), "TRAPNE.L #$12345678");
    assert_eq!(text(&[0x51fc]), "TRAPF");
    assert_eq!(text(&[0x60ff, 0x0000, 0x0100]), "BRA.L $000502");
    assert_eq!(text(&[0x66ff, 0xffff, 0xff00]), "BNE.L $000302");
    assert_eq!(text(&[0x06d0, 0x0004]), "CALLM #$4,(A0)");
    assert_eq!(text(&[0x06cd]), "RTM A5");
    assert_eq!(text(&[0x4101]), "CHK.L D1,D0");
    assert_eq!(text(&[0x4e7b, 0x0002]), "MOVEC D0,CACR");
    assert_eq!(text(&[0x4e7a, 0xb804]), "MOVEC ISP,A3");
    let len = |words: &[u16]| disassemble_for(Model::M68020, 0, words).len;
    assert_eq!(len(&[0x0efc, 0x8080, 0x90c1]), 6);
    assert_eq!(len(&[0x4c7c, 0x1001, 0, 7]), 8);
    assert_eq!(len(&[0x60ff, 0, 0x100]), 6);
    assert_eq!(len(&[0xe9f0, 0x1000, 0x0163, 0x0010, 0x0001, 0x2345]), 12);
}

#[test]
fn the_disassembler_and_the_68020_agree_on_the_new_instructions() {
    use super::isa::Op;
    let checked = length_sweep(
        Model::M68020,
        &[
            Op::Mull,
            Op::Divl,
            Op::Bftst,
            Op::Bfextu,
            Op::Bfexts,
            Op::Bfffo,
            Op::Bfchg,
            Op::Bfclr,
            Op::Bfset,
            Op::Bfins,
            Op::Cas,
            Op::Cas2,
            Op::Cmp2,
            Op::Pack,
            Op::Unpk,
            Op::Extb,
            Op::Chk,
            Op::Tst,
            Op::Cmpi,
            Op::Link,
            Op::Trapcc,
        ],
    );
    assert!(checked > 1_000, "only {checked} encodings were exercised");
}

#[test]
fn the_68020_disassembly_lengths_match_everywhere_the_68000_ones_did() {
    // The 68000 sweep in tests.rs, rerun on a 68020: nothing the 68000 could
    // decode may change length on the 68020 when its words do not use a
    // 68020 feature.
    use super::isa::Op;
    let checked = length_sweep(
        Model::M68020,
        &[
            Op::Move,
            Op::Movea,
            Op::Add,
            Op::Sub,
            Op::And,
            Op::Or,
            Op::Cmp,
            Op::Addi,
            Op::Lea,
            Op::Movem,
            Op::Btst,
            Op::Bset,
        ],
    );
    assert!(checked > 10_000, "only {checked} encodings were exercised");
}

// ---------------------------------------------------------------------------
// Stack frames, the master stack, and tracing
// ---------------------------------------------------------------------------

#[test]
fn a_trap_pushes_format_0_on_a_68020_too() {
    let board = m68020(&[0x4e43], |_| {}); // TRAP #3
    board.handler(0, vector::TRAP_BASE + 3, 0x0c00);
    let sr = board.cpu.regs().sr;
    board.cpu.step();
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(sp, 0x2000 - 8);
    assert_eq!(board.peek_word(sp), sr);
    assert_eq!(board.peek_long(sp + 2), 0x402);
    assert_eq!(board.peek_word(sp + 6), 0x008c);
}

#[test]
fn m_selects_the_master_stack() {
    // MOVE #$3000,SR sets S and M: A7 becomes the master stack pointer.
    let board = m68020(&[0x46fc, 0x3000, 0x4e43], |r| r.msp = 0x1800);
    board.handler(0, vector::TRAP_BASE + 3, 0x0c00);
    board.cpu.step();
    let r = board.cpu.regs();
    assert!(r.master());
    assert_eq!((r.a[7], r.ssp, r.msp), (0x1800, 0x2000, 0x1800));
    // An ordinary exception stacks on it and leaves M alone.
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.a[7], 0x1800 - 8);
    assert!(r.master());
    assert_eq!(r.ssp, 0x2000, "the interrupt stack is untouched");
}

#[test]
fn an_interrupt_in_master_state_leaves_a_throwaway_frame_on_the_interrupt_stack() {
    // MC68020UM §6.1.9: the format 0 frame on the master stack, then M
    // cleared and a format 1 copy on the interrupt stack with S set; the
    // handler runs on the interrupt stack.
    let board = m68020(&[0x4e71, 0x4e71], |r| {
        r.sr = flags::S | flags::M; // mask 0
        r.msp = 0x1800;
        r.ssp = 0x2000;
    });
    board.handler(0, vector::AUTOVECTOR_BASE + 2, 0x0e00);
    board.poke_word(0x0e00, 0x4e73); // the handler is just RTE
    let sr = board.cpu.regs().sr;
    board.cpu.set_ipl(2);
    let used = board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x0e00);
    assert!(!r.master(), "the handler runs with M clear");
    assert_eq!(r.a[7], 0x2000 - 8, "on the interrupt stack");
    assert_eq!(r.msp, 0x1800 - 8);
    // The master stack's frame, format 0.
    assert_eq!(board.peek_word(0x1800 - 8), sr);
    assert_eq!(board.peek_long(0x1800 - 6), 0x400);
    assert_eq!(board.peek_word(0x1800 - 2), 0x0068, "format 0, offset $068");
    // The throwaway frame, format 1, the status word with S set.
    assert_eq!(board.peek_word(0x2000 - 8), sr | flags::S);
    assert_eq!(board.peek_long(0x2000 - 6), 0x400);
    assert_eq!(board.peek_word(0x2000 - 2), 0x1068, "format 1, offset $068");
    assert_eq!(used, 41, "Interrupt (M-Stack), MC68020UM §8.2.17");
    // RTE pops the throwaway frame, which puts M back, then the real one.
    board.cpu.set_ipl(0);
    let used = board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x400);
    assert!(r.master());
    assert_eq!((r.a[7], r.ssp, r.msp), (0x1800, 0x2000, 0x1800));
    assert_eq!(
        used,
        16 + 21,
        "RTE (Throwaway) plus the normal frame under it"
    );
}

#[test]
fn an_odd_instruction_fetch_pushes_the_short_bus_fault_frame() {
    // JMP to an odd address. MC68020UM Table 6-5, format $A, 16 words: the
    // stacked PC is the odd target ("next instruction"), and both pipe stages
    // are marked for rerun.
    let board = m68020(&[0x4ed0], |r| r.a[0] = 0x0701);
    board.handler(0, vector::ADDRESS_ERROR, 0x0800);
    let sr = board.cpu.regs().sr;
    let used = board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x800);
    let sp = u64::from(r.a[7]);
    assert_eq!(sp, 0x2000 - 32);
    assert_eq!(board.peek_word(sp), sr, "+$00 SR");
    assert_eq!(board.peek_long(sp + 2), 0x0701, "+$02 PC");
    assert_eq!(
        board.peek_word(sp + 6),
        0xa00c,
        "+$06 format $A, offset $00C"
    );
    assert_eq!(board.peek_word(sp + 0x0a), 0x3000, "+$0A SSW: RC and RB");
    assert_eq!(used, 4 + 2 + 43, "JMP (An), then Bus Cycle Fault (Short)");
}

#[test]
fn a_data_bus_error_pushes_the_long_frame_and_rte_reruns_the_cycle() {
    // MOVE.W D0,($01002000).L on a 68020: nothing answers there.
    let board = m68020(&[0x33c0, 0x0100, 0x2000, 0x4e71], |r| r.d[0] = 0xbeef);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.poke_word(0x0800, 0x4e73); // RTE, and let it rerun
    let sr = board.cpu.regs().sr;
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x800);
    let sp = u64::from(r.a[7]);
    assert_eq!(sp, 0x2000 - 92, "46 words");
    // MC68020UM Table 6-5, format $B, and Figure 6-8 for the SSW. The status
    // word is the one at the fault: the MOVE had set its flags from $BEEF.
    assert_eq!(board.peek_word(sp), sr | flags::N, "+$00 SR");
    assert_eq!(
        board.peek_long(sp + 2),
        0x400,
        "+$02 the faulted instruction"
    );
    assert_eq!(
        board.peek_word(sp + 6),
        0xb008,
        "+$06 format $B, offset $008"
    );
    // DF, a write (RW clear), a word (SIZE 10), supervisor data (FC 5).
    assert_eq!(board.peek_word(sp + 0x0a), 0x0125, "+$0A SSW");
    assert_eq!(
        board.peek_long(sp + 0x10),
        0x0100_2000,
        "+$10 fault address"
    );
    assert_eq!(
        board.peek_long(sp + 0x18) & 0xffff,
        0xbeef,
        "+$18 data output buffer"
    );
    assert_eq!(board.peek_long(sp + 0x24), 0x404, "+$24 stage B address");
    assert_eq!(
        board.peek_word(sp + 0x36) >> 12,
        super::exec::VERSION_68020,
        "+$36 version number"
    );
    // RTE with DF still set reruns the write: the instruction starts again
    // from its first word, and faults again, on a stack the first frame was
    // popped from.
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x400, "restarted");
    assert_eq!(board.cpu.regs().a[7], 0x2000);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x800, "the rerun faulted again");
    assert_eq!(r.a[7], 0x2000 - 92);
}

#[test]
fn a_data_fault_completed_in_software_is_not_rerun() {
    // MOVE.W ($01002000).L,D1 faults; the handler puts $1234 in the data
    // input buffer, clears DF, and returns (MC68020UM §6.2.2).
    let board = m68020(&[0x3239, 0x0100, 0x2000, 0x4e71], |_| {});
    board.poke_long(u64::from(vector::BUS_ERROR) * 4, 0x0800);
    board.load(
        0x0800,
        &[
            0x2f7c, 0x0000, 0x1234, 0x002c, // MOVE.L #$1234,$2C(A7)
            0x08af, 0x0000, 0x000a, // BCLR #0,$A(A7): DF is bit 8, the low bit of +$0A
            0x4e73, // RTE
        ],
    );
    for _ in 0..5 {
        board.cpu.step();
    }
    let r = board.cpu.regs();
    assert_eq!(r.d[1] & 0xffff, 0x1234);
    assert_eq!(r.pc, 0x406, "past the MOVE, at the NOP");
    assert_eq!(r.a[7], 0x2000);
}

#[test]
fn a_prefetch_past_the_end_of_memory_is_not_a_fault_until_it_is_used() {
    // An RTS as the last word of mapped memory: the 68020 fetches ahead into
    // nothing, and must not fault for it (MC68020UM §6.1.2). The guarded
    // region ends at $11000 and nothing is mapped after it.
    let board = m68020(&[0x4ef9, 0x0001, 0x0ffe], |_| {}); // JMP $10FFE
    board.guarded.write_u8(0xffe, 0x4e).unwrap();
    board.guarded.write_u8(0xfff, 0x75).unwrap(); // RTS
    board.poke_long(0x1ffc, 0x0500);
    board.poke_word(0x500, 0x4e71);
    board.with_regs(|r| {
        r.a[7] = 0x1ffc;
        r.ssp = 0x1ffc;
    });
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.cpu.step(); // JMP
    assert_eq!(board.cpu.regs().pc, 0x10ffe);
    board.cpu.step(); // RTS
    assert_eq!(board.cpu.regs().pc, 0x500, "no bus error");
    // Falling off the end is one, when the missing word is reached, and it
    // is the short frame at the instruction boundary.
    let board = m68020(&[0x4ef9, 0x0001, 0x0ffe], |_| {});
    board.guarded.write_u8(0xffe, 0x4e).unwrap();
    board.guarded.write_u8(0xfff, 0x71).unwrap(); // NOP
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.cpu.step(); // JMP
    board.cpu.step(); // NOP
    assert_eq!(board.cpu.regs().pc, 0x11000);
    board.cpu.step(); // the missing opcode
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x800);
    let sp = u64::from(r.a[7]);
    assert_eq!(board.peek_word(sp + 6), 0xa008, "format $A");
    assert_eq!(
        board.peek_long(sp + 2),
        0x11000,
        "the instruction it could not fetch"
    );
    assert_eq!(board.peek_long(sp + 0x10), 0x11000, "and where");
}

#[test]
fn rte_rejects_a_format_the_68020_does_not_define() {
    for format in [0x3000u16, 0x9000] {
        let board = m68020(&[0x4e73], |r| {
            r.a[7] = 0x1ff0;
            r.ssp = 0x1ff0;
        });
        board.handler(0, vector::FORMAT_ERROR, 0x0900);
        board.poke_word(0x1ff0, 0x2700);
        board.poke_long(0x1ff2, 0x0500);
        board.poke_word(0x1ff6, format);
        board.cpu.step();
        let r = board.cpu.regs();
        assert_eq!(r.pc, 0x900, "format {format:04x}");
        assert_eq!(r.a[7], 0x1ff0 - 8, "the bad frame stays");
    }
}

#[test]
fn rte_returns_from_a_six_word_frame() {
    let board = m68020(&[0x4e73], |r| {
        r.a[7] = 0x1ff0;
        r.ssp = 0x1ff0;
    });
    board.poke_word(0x1ff0, 0x2700);
    board.poke_long(0x1ff2, 0x0500);
    board.poke_word(0x1ff6, 0x2018);
    board.poke_long(0x1ff8, 0x0400);
    board.poke_word(0x500, 0x4e71);
    let used = board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!((r.pc, r.a[7]), (0x500, 0x1ff0 + 12));
    assert_eq!(used, 21, "RTE (Six Word)");
}

#[test]
fn t1_traces_every_instruction_with_the_six_word_frame() {
    let board = m68020(&[0x7001], |r| r.sr |= flags::T); // MOVEQ #1,D0
    board.handler(0, vector::TRACE, 0x0f80);
    let used = board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x0f80);
    assert!(
        !r.flag(flags::T) && !r.flag(flags::T0),
        "tracing off in the handler"
    );
    let sp = u64::from(r.a[7]);
    assert_eq!(board.peek_long(sp + 2), 0x402, "the next instruction");
    assert_eq!(board.peek_word(sp + 6), 0x2024, "format 2, offset $024");
    assert_eq!(board.peek_long(sp + 8), 0x400, "the traced instruction");
    assert_eq!(used, 2 + 25, "MOVEQ, then Trace");
}

#[test]
fn t0_traces_only_a_change_of_flow() {
    // NOP then BRA.S: with T0 alone, the NOP runs untraced and the branch
    // is traced (MC68020UM §6.1.7, Table 6-2).
    let board = m68020(&[0x4e71, 0x6006], |r| r.sr |= flags::T0);
    board.handler(0, vector::TRACE, 0x0f80);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x402, "no trace after the NOP");
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x0f80);
    assert_eq!(
        board.peek_long(u64::from(r.a[7]) + 2),
        0x40a,
        "the branch target"
    );
    // A 68000 has no T0: the bit reads as zero.
    let old = super::tests_68010::Board::new(Model::M68000);
    old.boot(&[0x4e71]);
    old.with_regs(|r| r.sr |= flags::T0);
    assert_eq!(old.cpu.regs().sr & flags::T0, 0);
}

#[test]
fn a_traced_trap_is_followed_by_the_trace() {
    // TRAP #0 with T1: the trap's processing, then the trace's, so the trace
    // handler returns into the trap handler (MC68020UM §6.1.11).
    let board = m68020(&[0x4e40], |r| r.sr |= flags::T);
    board.handler(0, vector::TRAP_BASE, 0x0c00);
    board.handler(0, vector::TRACE, 0x0f80);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x0f80, "in the trace handler");
    let sp = u64::from(r.a[7]);
    assert_eq!(
        board.peek_long(sp + 2),
        0x0c00,
        "which returns to the trap handler"
    );
    assert_eq!(
        board.peek_word(sp + 12),
        board.cpu.regs().sr & !flags::T | flags::T
    );
    assert_eq!(sp, 0x2000 - 8 - 12);
}

// ---------------------------------------------------------------------------
// Timing: MC68020UM §8.2, cache case
// ---------------------------------------------------------------------------

#[test]
fn instruction_times_are_the_cache_case_column() {
    let time = |words: &[u16], edit: fn(&mut super::Regs)| {
        let board = m68020(words, edit);
        board.poke_long(0x1000, 0x0000_0003);
        board.cpu.step()
    };
    let a0 = |r: &mut super::Regs| r.a[0] = 0x1000;
    let none = |_: &mut super::Regs| {};
    // §8.2.16, §8.2.9, §8.2.6.
    assert_eq!(time(&[0x4e71], none), 2, "NOP");
    assert_eq!(time(&[0x7001], none), 2, "MOVEQ");
    assert_eq!(time(&[0x2200], none), 2, "MOVE.L D0,D1: Rn to Dn");
    assert_eq!(time(&[0x3210], a0), 6, "MOVE.W (A0),D1: (An) to Dn");
    assert_eq!(time(&[0x22bc, 0, 1], a0), 8, "MOVE.L #,(A1): #.L to (An)");
    assert_eq!(
        time(&[0x3170, 0x0000, 0x0010], a0),
        10,
        "(d8,An,Xn) to (d16,An)"
    );
    // §8.2.8: the row plus the fetch-effective-address time.
    assert_eq!(time(&[0xd050], a0), 2 + 4, "ADD.W (A0),D0");
    assert_eq!(time(&[0xd190], a0), 4 + 4, "ADD.L D0,(A0)");
    // §8.2.9: plus fetch immediate.
    assert_eq!(time(&[0x0640, 0x0001], none), 2 + 2, "ADDI.W #1,D0");
    assert_eq!(time(&[0x0690, 0, 1], a0), 4 + 4, "ADDI.L #1,(A0)");
    // The manual's own worked example (§8.2): MULU.L D7,D1:D2 is 2 + 43 and
    // DIVS.L #$10000,D3:D4 is 6 + 90.
    assert_eq!(time(&[0x4c07, 0x2401], none), 45, "MULU.L D7,D1:D2");
    assert_eq!(
        time(&[0x4c7c, 0x4c03, 0x0001, 0x0000], |r| r.d[4] = 0x0002_0000),
        96,
        "DIVS.L #$10000,D3:D4"
    );
    // §8.2.15.
    assert_eq!(time(&[0x6002], none), 6, "BRA.S, taken");
    assert_eq!(time(&[0x6702], none), 4, "BEQ.S, not taken");
    assert_eq!(time(&[0x6700, 0x0002], none), 6, "BEQ.W, not taken");
    assert_eq!(time(&[0x51c8, 0x0002], |r| r.d[0] = 0), 10, "DBF, expired");
    assert_eq!(time(&[0x51c8, 0x0002], |r| r.d[0] = 5), 6, "DBF, looping");
    // §8.2.16 and §8.2.5.
    assert_eq!(time(&[0x4e90], a0), 5 + 2, "JSR (A0)");
    assert_eq!(time(&[0x43e8, 0x0010], a0), 2 + 2, "LEA (d16,A0),A1");
    // §8.2.14: plus calculate immediate. MC68020UM's worked example charges
    // BFCLR $6000{0:8} the *fetch* immediate row, 5, where the table's own
    // footnote says calculate, 4; the footnote is what this core follows.
    assert_eq!(
        time(&[0xecf8, 0x0008, 0x6000], none),
        16 + 4,
        "BFCLR $6000.w{{0:8}}"
    );
    assert_eq!(time(&[0xecc0, 0x0008], none), 12, "BFCLR D0{{0:8}}");
    // §8.2.7: MOVEM's 4 + 3n and MOVEC's row, each plus the calculate
    // immediate address time its footnote names.
    assert_eq!(
        time(&[0x48e7, 0xc000], none),
        4 + 3 * 2 + 4,
        "MOVEM.L D0-D1,-(A7)"
    );
    assert_eq!(
        time(&[0x4cd0, 0x0003], a0),
        8 + 4 * 2 + 2,
        "MOVEM.L (A0),D0-D1"
    );
    assert_eq!(time(&[0x4e7b, 0x0801], none), 12, "MOVEC D0,VBR");
    assert_eq!(time(&[0x4e7a, 0x0801], none), 6, "MOVEC VBR,D0");
    assert_eq!(time(&[0x4e75], |r| r.a[7] = 0x1000), 10, "RTS");
    // §8.2.17.
    let trap = |words: &[u16], vector: u8| {
        let board = m68020(words, |_| {});
        board.handler(0, vector, 0x0c00);
        board.cpu.step()
    };
    assert_eq!(trap(&[0x4e40], vector::TRAP_BASE), 20, "TRAP #n");
    assert_eq!(trap(&[0x4afc], vector::ILLEGAL), 20, "illegal instruction");
    assert_eq!(
        trap(&[0x57fa, 0x0000], vector::TRAPV),
        6,
        "TRAPEQ.W, no trap"
    );
}

#[test]
fn an_interrupt_on_the_interrupt_stack_is_26_clocks() {
    let board = m68020(&[0x4e71], |r| r.sr = flags::S);
    board.handler(0, vector::AUTOVECTOR_BASE + 1, 0x0e00);
    board.cpu.set_ipl(1);
    assert_eq!(board.cpu.step(), 26);
}

#[test]
fn the_cycle_counter_advances_by_the_table_not_the_accesses() {
    let board = m68020(&[0x2210, 0x4e71], |r| r.a[0] = 0x1000); // MOVE.L (A0),D1
    let before = board.cpu.cycles();
    assert_eq!(board.cpu.step(), 6);
    assert_eq!(board.cpu.cycles() - before, 6);
}

#[test]
fn a_68020_snapshot_round_trips_every_stack_pointer_and_control_register()
-> crate::core::error::Result<()> {
    use super::tests_68010::{restore, snapshot};
    use super::{Config, M68k};
    // In master state, with the cache enabled and a poisoned prefetch owed:
    // all of it survives.
    let board = m68020(&[0x4ef9, 0x0001, 0x0ffe], |r| {
        r.sr = flags::S | flags::M;
        r.usp = 0x0111_1111;
        r.ssp = 0x0222_2222;
        r.msp = 0x1800;
        r.vbr = 0x0333_3333;
        r.sfc = 1;
        r.dfc = 5;
        r.cacr = 1;
        r.caar = 0x44;
    });
    board.guarded.write_u8(0xffe, 0x4e).unwrap();
    board.guarded.write_u8(0xfff, 0x71).unwrap();
    board.cpu.step(); // JMP to the last word: the prefetch behind it poisons
    let bytes = snapshot(&board.cpu)?;
    let other = M68k::new(Config::MC68020);
    restore(&other, &bytes)?;
    assert_eq!(other.regs(), board.cpu.regs());
    assert_eq!(snapshot(&other)?, bytes, "a round trip is a fixed point");
    // A 68EC020 is not a 68020, even with the same tail.
    assert!(restore(&M68k::new(Config::MC68EC020), &bytes).is_err());
    Ok(())
}
