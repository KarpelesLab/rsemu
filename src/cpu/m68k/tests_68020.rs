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
    let _ = length_sweep;
}
