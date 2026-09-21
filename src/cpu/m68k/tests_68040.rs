//! Hand-written tests for the MC68040, MC68LC040 and MC68EC040 models.
//!
//! There is no 68040 corpus either, so every expectation here is computed
//! from the manuals: M68040UM §3.1 for the memory management registers, §4.2
//! for `CACR`, §8.4 for the stack frames and the special status word, §10.3
//! for the cache instructions' time, and M68000PRM §4 and §6 for the
//! encodings of `MOVE16`, `CINV`, `CPUSH` and the 68040's `MOVEC` table.
//! `conformance.rs` replays the whole 68000 corpus through the 68040 on top
//! of these.

use crate::core::error::Result;

use super::tests_68010::{Board, restore, snapshot};
use super::{Config, M68k, Model, Reg, flags, isa, vector};

/// A 68040 board running `words` from `$400`.
fn m68040(words: &[u16], edit: impl FnOnce(&mut super::Regs)) -> Board {
    let board = Board::new(Model::M68040);
    board.boot(words);
    board.with_regs(edit);
    board
}

#[test]
fn the_model_property_names_all_three_packages() {
    use crate::core::props::Props;

    for (name, model) in [
        ("68040", Model::M68040),
        ("68lc040", Model::M68LC040),
        ("68ec040", Model::M68EC040),
    ] {
        let cpu = M68k::from_props(&Props::new().with("model", name)).unwrap();
        assert_eq!(cpu.model(), model);
    }
    // A 68040 has no coprocessor interface at all: its floating-point unit
    // is on the chip (M68040UM §1.1), so a 68881 cannot be attached.
    let err = M68k::from_props(&Props::new().with("model", "68040").with("fpu", "68881"))
        .expect_err("a 68881 cannot be wired to a 68040");
    assert!(
        alloc::format!("{err}").contains("no coprocessor interface"),
        "{err}"
    );
}

#[test]
fn every_68040_package_drives_all_32_address_lines() {
    // M68040UM Appendices A and B: the MC68LC040 drops the FPU and the
    // MC68EC040 drops the FPU and the MMU. Neither drops address pins, so
    // unlike the 68EC020 none of the three is a 24-bit part.
    for model in [Model::M68040, Model::M68LC040, Model::M68EC040] {
        assert_eq!(model.address_mask(), u32::MAX, "{model}");
    }
}

// ----------------------------------------------------------------------
// MOVE16
// ----------------------------------------------------------------------

/// Fill sixteen bytes at `at` with a recognisable pattern.
fn fill_line(board: &Board, at: u64) {
    for i in 0..4u64 {
        board.poke_long(at + i * 4, 0x1111_1111 * (i as u32 + 1));
    }
}

fn line(board: &Board, at: u64) -> [u32; 4] {
    [
        board.peek_long(at),
        board.peek_long(at + 4),
        board.peek_long(at + 8),
        board.peek_long(at + 12),
    ]
}

#[test]
fn move16_copies_a_line_between_two_postincrement_registers() {
    // M68000PRM §4, *MOVE16*: `1111 0110 0010 0AAA` then `1BBB 0000 0000
    // 0000`. A0 -> A1.
    let board = m68040(&[0xf620, 0x9000, 0x4e71], |r| {
        r.a[0] = 0x1000;
        r.a[1] = 0x1800;
    });
    fill_line(&board, 0x1000);
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(
        line(&board, 0x1800),
        [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444]
    );
    assert_eq!((r.a[0], r.a[1]), (0x1010, 0x1810), "both step by sixteen");
}

#[test]
fn move16_ignores_the_low_four_bits_and_still_steps_by_sixteen() {
    // The manual's own worked example, scaled into this board's memory:
    // "MOVE16 (A0)+,$FE802 with A0 = $1400F ... the line at address $14000 is
    // read ... the line is then written to the line at address $FE800 ...
    // after the instruction A0 contains $1401F".
    let board = m68040(&[0xf600, 0x0000, 0x1802, 0x4e71], |r| r.a[0] = 0x100f);
    fill_line(&board, 0x1000);
    board.cpu.step();
    assert_eq!(
        line(&board, 0x1800),
        [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444],
        "both addresses aligned down"
    );
    assert_eq!(board.cpu.regs().a[0], 0x101f, "$100F + 16");
    assert_eq!(board.peek_long(0x1810), 0, "nothing past the line");
}

#[test]
fn move16_to_one_register_named_twice_steps_it_once() {
    // M68040UM Table 1-4, note 7: "MOVE16 (ax)+,(ay)+ is functionally the
    // same as MOVE16 (ax),(ay)+ when ax = ay. The address register is only
    // incremented once, and the line is copied over itself rather than to
    // the next line."
    let board = m68040(&[0xf622, 0xa000, 0x4e71], |r| r.a[2] = 0x1000);
    fill_line(&board, 0x1000);
    board.poke_long(0x1010, 0xdead_beef);
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[2], 0x1010, "once, not twice");
    assert_eq!(board.peek_long(0x1010), 0xdead_beef, "the next line is not");
    assert_eq!(
        line(&board, 0x1000),
        [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444],
        "copied over itself"
    );
}

#[test]
fn the_four_absolute_move16_opmodes_pick_their_sides() {
    // M68000PRM §4, *MOVE16*, the opmode table: 00 (Ay)+ -> (xxx).L,
    // 01 (xxx).L -> (Ay)+, 10 (Ay) -> (xxx).L, 11 (xxx).L -> (Ay).
    for (opmode, from_register, steps) in [
        (0u16, true, true),
        (1, false, true),
        (2, true, false),
        (3, false, false),
    ] {
        let opcode = 0xf600 | (opmode << 3) | 3; // A3
        let board = m68040(&[opcode, 0x0000, 0x1800, 0x4e71], |r| r.a[3] = 0x1000);
        let (src, dst) = if from_register {
            (0x1000u64, 0x1800u64)
        } else {
            (0x1800, 0x1000)
        };
        fill_line(&board, src);
        board.cpu.step();
        assert_eq!(
            line(&board, dst),
            [0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444],
            "opmode {opmode}"
        );
        assert_eq!(
            board.cpu.regs().a[3],
            if steps { 0x1010 } else { 0x1000 },
            "opmode {opmode} postincrement"
        );
    }
}

#[test]
fn move16_is_not_privileged_and_does_not_exist_before_the_68040() {
    // M68000PRM Table A-1 gives MOVE16 to the 68040 alone, and *MOVE16* has
    // no "If Supervisor State" clause, unlike CINV and CPUSH.
    assert!(!isa::decode_for(Model::M68040, 0xf620).privileged);
    assert_eq!(isa::decode_for(Model::M68040, 0xf620).op, isa::Op::Move16);
    for model in [Model::M68020, Model::M68030, Model::M68EC030] {
        assert_eq!(
            isa::decode_for(model, 0xf620).op,
            isa::Op::LineF,
            "{model} has no MOVE16"
        );
    }
}

// ----------------------------------------------------------------------
// The caches
// ----------------------------------------------------------------------

#[test]
fn the_cache_control_register_keeps_only_the_two_enables() {
    // M68040UM Figure 4-4: DE is bit 31, IE is bit 15, and everything else
    // is undefined. The 68020's and 68030's clear-the-cache bits are gone —
    // CINV and CPUSH do that job.
    let board = m68040(
        &[0x203c, 0xffff, 0xffff, 0x4e7b, 0x0002, 0x4e7a, 0x1002],
        |_| {},
    );
    for _ in 0..3 {
        board.cpu.step();
    }
    assert_eq!(board.cpu.regs().cacr, 0x8000_8000);
    assert_eq!(board.cpu.regs().d[1], 0x8000_8000);
}

#[test]
fn cinv_and_cpush_decode_their_cache_and_scope_fields() {
    // M68000PRM §6, *CINV* and *CPUSH*: `1111 0100 CC P SS RRR`, with P
    // clear for CINV and set for CPUSH, and scope 01 line, 10 page, 11 all.
    // Scope 00 "causes illegal instruction trap" — vector 4, not line F.
    use isa::Op;
    let cases: [(u16, Op); 6] = [
        (0x08, Op::Cinvl),
        (0x10, Op::Cinvp),
        (0x18, Op::Cinva),
        (0x28, Op::Cpushl),
        (0x30, Op::Cpushp),
        (0x38, Op::Cpusha),
    ];
    for (bits, op) in cases {
        for cache in 0..4u16 {
            let opcode = 0xf400 | (cache << 6) | bits | 5;
            let insn = isa::decode_for(Model::M68040, opcode);
            assert_eq!(insn.op, op, "${opcode:04x}");
            assert!(insn.privileged, "${opcode:04x} is supervisor only");
        }
    }
    for bits in [0x00u16, 0x20] {
        assert_eq!(
            isa::decode_for(Model::M68040, 0xf400 | bits).op,
            Op::Illegal,
            "scope 00"
        );
    }
    // And nothing before the 68040 has them at all.
    assert_eq!(isa::decode_for(Model::M68030, 0xf418).op, Op::LineF);
}

#[test]
fn a_cache_instruction_in_user_state_is_a_privilege_violation() {
    let board = m68040(&[0xf418, 0x4e71], |r| {
        r.sr &= !flags::S;
        r.usp = 0x1f00;
        r.ssp = 0x2000;
        r.a[7] = 0x1f00;
    });
    board.handler(0, vector::PRIVILEGE, 0x0c00);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::PRIVILEGE));
}

#[test]
fn cinv_and_cpush_cost_the_manuals_time_and_change_nothing() {
    // M68040UM Tables 10-3 and 10-4, with Idle zero and the CPUSH best case
    // — "a cache containing no dirty entries", which is the only state this
    // core's cache is ever in, because it has none.
    for (opcode, clocks, what) in [
        (0xf4c8u16, 9u64, "CINVL"),
        (0xf4d0, 266, "CINVP"),
        (0xf4d8, 9, "CINVA"),
        (0xf4e8, 6, "CPUSHL"),
        (0xf4f0, 267, "CPUSHP"),
        (0xf4f8, 267, "CPUSHA"),
    ] {
        let board = m68040(&[opcode, 0x4e71], |r| r.a[0] = 0x1000);
        let before = board.cpu.regs();
        let used = board.cpu.step();
        let after = board.cpu.regs();
        assert_eq!(used, clocks, "{what}");
        assert_eq!(after.d, before.d, "{what} touches no register");
        assert_eq!(after.a, before.a, "{what} touches no register");
        assert_eq!(after.pc, before.pc + 2, "{what} is one word");
    }
}

// ----------------------------------------------------------------------
// MOVEC
// ----------------------------------------------------------------------

/// Run `MOVE.L #value,D0 ; MOVEC D0,Rc ; MOVEC Rc,D1` and give back `D1`.
fn movec_round_trip(model: Model, code: u16, value: u32) -> Option<u32> {
    let board = Board::new(model);
    board.boot(&[
        0x203c,
        (value >> 16) as u16,
        value as u16,
        0x4e7b,
        code,
        0x4e7a,
        0x1000 | code,
        0x4e71,
    ]);
    board.handler(0, vector::ILLEGAL, 0x0c00);
    board.cpu.step(); // MOVE.L #value,D0
    board.cpu.step(); // MOVEC D0,Rc
    // "An illegal instruction exception can also be a MOVEC instruction with
    // an undefined register specification field" (M68040UM §8.2.4), and the
    // exception has to be looked for before the next step overwrites it.
    if board.cpu.last_exception() == Some(vector::ILLEGAL) {
        return None;
    }
    board.cpu.step(); // MOVEC Rc,D1
    Some(board.cpu.regs().d[1])
}

#[test]
fn the_68040_movec_table_is_not_a_superset_of_the_68030s() {
    use isa::ctrl;
    // M68000PRM §6, *MOVEC*: the 68040 adds TC, ITT0/1, DTT0/1, MMUSR, URP
    // and SRP, and note 2 takes CAAR away — it is "for the MC68020 and
    // MC68030 only".
    assert_eq!(
        movec_round_trip(Model::M68040, ctrl::CAAR, 0x1234_5678),
        None
    );
    assert_eq!(
        movec_round_trip(Model::M68030, ctrl::CAAR, 0x1234_5678),
        Some(0x1234_5678)
    );
    // TC is sixteen bits with two of them implemented (Figure 3-4). The
    // value written here leaves **E** clear, because this board has no
    // translation tables and the instruction after the `MOVEC` still has to
    // be fetched; that **E** has storage is what every test below shows.
    assert_eq!(
        movec_round_trip(Model::M68040, ctrl::TC, 0xffff_7fff),
        Some(0x4000)
    );
    // "Bits 8–0 of an address loaded into the URP or the SRP must be zero"
    // (§3.1.1).
    for code in [ctrl::URP, ctrl::SRP] {
        assert_eq!(
            movec_round_trip(Model::M68040, code, 0xffff_ffff),
            Some(0xffff_fe00)
        );
    }
    // A transparent translation register: bits 12-10, 7, 4, 3, 1 and 0
    // always read as zero (Figure 3-5).
    for code in [ctrl::ITT0, ctrl::ITT1, ctrl::DTT0, ctrl::DTT1] {
        assert_eq!(
            movec_round_trip(Model::M68040, code, 0xffff_ffff),
            Some(0xffff_e364)
        );
    }
    // MMUSR: bits 31-12 and B, G, U1, U0, S, CM, M, W, T, R (Figure 3-6).
    assert_eq!(
        movec_round_trip(Model::M68040, ctrl::MMUSR, 0xffff_ffff),
        Some(0xffff_fff7)
    );
    // And the 68030 has none of them.
    for code in [ctrl::TC, ctrl::ITT0, ctrl::MMUSR, ctrl::URP, ctrl::SRP] {
        assert_eq!(
            movec_round_trip(Model::M68030, code, 0),
            None,
            "${code:03x} on a 68030"
        );
    }
}

#[test]
fn the_ec040_keeps_the_access_control_registers_and_nothing_else() {
    use isa::ctrl;
    // M68040UM Appendix B and M68000PRM §6, *MOVEC*: `$004`-`$007` are the
    // MC68EC040's IACR0/1 and DACR0/1, and TC, MMUSR, URP and SRP are gone.
    for code in [ctrl::ITT0, ctrl::ITT1, ctrl::DTT0, ctrl::DTT1] {
        assert_eq!(
            movec_round_trip(Model::M68EC040, code, 0xffff_ffff),
            Some(0xffff_e364),
            "${code:03x}"
        );
    }
    for code in [ctrl::TC, ctrl::MMUSR, ctrl::URP, ctrl::SRP] {
        assert_eq!(
            movec_round_trip(Model::M68EC040, code, 0),
            None,
            "${code:03x} on an MC68EC040"
        );
    }
    // The MC68LC040 has the paged unit, and so has all of them.
    assert_eq!(
        movec_round_trip(Model::M68LC040, ctrl::URP, 0x0001_2000),
        Some(0x0001_2000)
    );
}

#[test]
fn the_registers_a_debugger_lists_follow_the_package() {
    let names = |model| {
        Reg::all_for(model)
            .into_iter()
            .map(|r| alloc::format!("{r}"))
            .collect::<alloc::vec::Vec<_>>()
    };
    let full = names(Model::M68040);
    for expected in ["tc", "urp", "srp", "mmusr", "itt0", "itt1", "dtt0", "dtt1"] {
        assert!(full.iter().any(|n| n == expected), "{expected}");
    }
    assert!(!full.iter().any(|n| n == "caar"), "no CAAR on a 68040");
    let embedded = names(Model::M68EC040);
    assert!(embedded.iter().any(|n| n == "itt0"));
    assert!(!embedded.iter().any(|n| n == "urp"));
}

// ----------------------------------------------------------------------
// Exception frames
// ----------------------------------------------------------------------

#[test]
fn an_odd_instruction_fetch_pushes_the_six_word_format_2_frame() {
    // M68040UM §8.2.2: an address error is "the processor attempts to
    // prefetch an instruction from an odd address ... the stack frame is
    // generated containing the address of the instruction that caused the
    // address error and the address itself (A0 is cleared)", and §8.4.3
    // makes it format $2. A 68020 would have pushed format $A here.
    let board = m68040(&[0x4ed0], |r| r.a[0] = 0x0701);
    board.handler(0, vector::ADDRESS_ERROR, 0x0800);
    let sr = board.cpu.regs().sr;
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x800);
    let sp = u64::from(r.a[7]);
    assert_eq!(sp, 0x2000 - 12, "six words");
    assert_eq!(board.peek_word(sp), sr, "+$00 SR");
    assert_eq!(
        board.peek_long(sp + 2),
        0x400,
        "+$02 the instruction that caused it"
    );
    assert_eq!(
        board.peek_word(sp + 6),
        0x200c,
        "+$06 format $2, offset $00C"
    );
    assert_eq!(
        board.peek_long(sp + 8),
        0x0700,
        "+$08 the referenced address, bit 0 cleared"
    );
}

#[test]
fn a_data_bus_error_pushes_the_thirty_word_format_7_frame() {
    // M68040UM §8.4.6, field by field. MOVE.W D0,($01002000).L: nothing
    // answers there.
    let board = m68040(&[0x33c0, 0x0100, 0x2000, 0x4e71], |r| r.d[0] = 0xbeef);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    let sr = board.cpu.regs().sr;
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(r.pc, 0x800);
    let sp = u64::from(r.a[7]);
    assert_eq!(sp, 0x2000 - 60, "thirty words");
    assert_eq!(board.peek_word(sp), sr | flags::N, "+$00 SR");
    assert_eq!(
        board.peek_long(sp + 2),
        0x400,
        "+$02 the faulted instruction"
    );
    assert_eq!(
        board.peek_word(sp + 6),
        0x7008,
        "+$06 format $7, offset $008"
    );
    assert_eq!(
        board.peek_long(sp + 8),
        0,
        "+$08 effective address: no continuation pending"
    );
    // Figure 8-7: CP/CU/CT/CM clear, MA clear, ATC clear (a physical bus
    // error, not a translation failure), LK clear, RW clear (a write), SIZE
    // 10 for a word, TT 00 normal, TM 101 supervisor data.
    assert_eq!(board.peek_word(sp + 0x0c), 0x0045, "+$0C SSW");
    for (at, what) in [
        (0x0eu64, "write-back 3 status"),
        (0x10, "write-back 2 status"),
        (0x12, "write-back 1 status"),
    ] {
        assert_eq!(board.peek_word(sp + at), 0, "+${at:02X} {what} invalid");
    }
    assert_eq!(
        board.peek_long(sp + 0x14),
        0x0100_2000,
        "+$14 fault address"
    );
}

#[test]
fn the_special_status_word_reports_size_direction_and_transfer_modifier() {
    // Figure 8-7 and Table 5-3, one case per field that varies.
    // A user-state byte read of the supervisor-only region: RW set, SIZE 01,
    // TM 001 (user data).
    let board = m68040(&[0x1039, 0x0001, 0x0000, 0x4e71], |r| {
        r.sr &= !flags::S;
        r.usp = 0x1f00;
        r.ssp = 0x2000;
        r.a[7] = 0x1f00;
    });
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.cpu.step();
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 0x0c), 0x0121, "RW, SIZE=01, TM=001");
    assert_eq!(board.peek_long(sp + 0x14), 0x0001_0000, "fault address");

    // A long write in supervisor state: RW clear, SIZE 00, TM 101. This core
    // drives a long as two words, so the size reported is the word the bus
    // refused.
    let board = m68040(&[0x23c0, 0x0100, 0x2000, 0x4e71], |_| {});
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.cpu.step();
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 0x0c), 0x0045, "SIZE=10, TM=101");

    // A read-modify-write sets LK (bit 9): TAS on the unmapped address.
    let board = m68040(&[0x4af9, 0x0100, 0x2000, 0x4e71], |_| {});
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.cpu.step();
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(
        board.peek_word(sp + 0x0c) & 0x0200,
        0x0200,
        "LK on a locked transfer"
    );
}

#[test]
fn a_moves_to_program_space_reports_as_a_data_access() {
    // M68040UM §3.2.5: "the integer unit translates MOVES accesses to
    // instruction address spaces (SFC/DFC = $6 or $2) into data references
    // (SFC/DFC = $5 or $1) ... the resulting access error stack frame
    // contains the converted function code in the TM field".
    let board = m68040(
        &[
            0x203c, 0x0000, 0x0006, // MOVE.L #6,D0
            0x4e7b, 0x0001, // MOVEC D0,DFC
            0x0eb9, 0x1800, 0x0100, 0x2000, // MOVES.L D1,($01002000).L
            0x4e71,
        ],
        |_| {},
    );
    board.handler(0, vector::BUS_ERROR, 0x0800);
    for _ in 0..3 {
        board.cpu.step();
    }
    let sp = u64::from(board.cpu.regs().a[7]);
    let ssw = board.peek_word(sp + 0x0c);
    assert_eq!(ssw & 7, 0b101, "TM is the converted code, not $6");
    assert_eq!((ssw >> 3) & 3, 0b00, "TT is a normal access");

    // And a function code that is not one of the four ordinary ones is an
    // alternate logical access, TT = 10, carrying the code itself
    // (Table 3-2).
    let board = m68040(
        &[
            0x203c, 0x0000, 0x0003, // MOVE.L #3,D0
            0x4e7b, 0x0001, // MOVEC D0,DFC
            0x0eb9, 0x1800, 0x0100, 0x2000, // MOVES.L D1,($01002000).L
            0x4e71,
        ],
        |_| {},
    );
    board.handler(0, vector::BUS_ERROR, 0x0800);
    for _ in 0..3 {
        board.cpu.step();
    }
    let sp = u64::from(board.cpu.regs().a[7]);
    let ssw = board.peek_word(sp + 0x0c);
    assert_eq!((ssw >> 3) & 3, 0b10, "TT is an alternate access");
    assert_eq!(ssw & 7, 3, "TM is the function code");
}

#[test]
fn rte_from_the_access_error_frame_restarts_the_instruction() {
    // M68040UM §8.4.6.7: with no continuation bit set, "the processor
    // increments the active supervisor stack pointer by 30 words and resumes
    // normal instruction execution" at the stacked PC, which §8.2.1 says is
    // "the logical address of the instruction executing at the time the
    // fault was detected". This core has no write-back pipeline, so every
    // write-back status is invalid and the handler has nothing to complete.
    let board = m68040(&[0x33c0, 0x0100, 0x2000, 0x4e71], |r| r.d[0] = 0xbeef);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.poke_word(0x0800, 0x4e73); // RTE
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[7], 0x2000 - 60);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x400, "restarted from its first word");
    assert_eq!(board.cpu.regs().a[7], 0x2000, "and the frame is gone");
}

#[test]
fn rte_puts_back_an_address_register_the_faulted_instruction_had_stepped() {
    // MOVE.L (A0)+,($01002000).L: the postincrement happens, then the write
    // faults. The handler sees A0 stepped, as it would on hardware, and RTE
    // restores it so the restarted instruction steps it again exactly once.
    let board = m68040(&[0x23d8, 0x0100, 0x2000, 0x4e71], |r| r.a[0] = 0x1000);
    board.poke_long(0x1000, 0x1234_5678);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.poke_word(0x0800, 0x4e73); // RTE
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[0], 0x1004, "the handler sees it stepped");
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[0], 0x1000, "RTE puts it back");
    assert_eq!(board.cpu.regs().pc, 0x400);
}

#[test]
fn rte_refuses_a_frame_with_a_continuation_bit_set() {
    // §8.4.6.7 hands the continuation cases to microcode this core does not
    // have, and says explicitly that a handler which sets more than one
    // leaves `RTE` undefined. A format error is the honest answer.
    let board = m68040(&[0x4e73], |r| {
        r.ssp = 0x1f00;
        r.a[7] = 0x1f00;
    });
    board.handler(0, vector::FORMAT_ERROR, 0x0c00);
    // A format $7 frame with CT (bit 13) set in the SSW.
    board.poke_word(0x1f00, 0x2700);
    board.poke_long(0x1f02, 0x0500);
    board.poke_word(0x1f06, 0x7008);
    board.poke_word(0x1f0c, 0x2000);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::FORMAT_ERROR));
    assert_eq!(board.cpu.regs().a[7], 0x1f00 - 8, "the bad frame is intact");
}

#[test]
fn the_68040_does_not_recognise_the_68020s_bus_fault_frames() {
    // M68040UM §8.4 lists formats $0, $1, $2, $3, $4 and $7 and no others.
    for format in [0xau16, 0xb, 0x8, 0x9] {
        let board = m68040(&[0x4e73], |r| {
            r.ssp = 0x1f00;
            r.a[7] = 0x1f00;
        });
        board.handler(0, vector::FORMAT_ERROR, 0x0c00);
        board.poke_word(0x1f00, 0x2700);
        board.poke_long(0x1f02, 0x0500);
        board.poke_word(0x1f06, (format << 12) | 0x008);
        board.cpu.step();
        assert_eq!(
            board.cpu.last_exception(),
            Some(vector::FORMAT_ERROR),
            "format ${format:x}"
        );
    }
}

#[test]
fn a_deferred_prefetch_fault_reports_the_instruction_transfer_modifier() {
    // M68040UM §8.2.1: a bus error on a prefetch is "deferred until the
    // processor attempts to use the information", and §8.4.6 says the TM
    // field then "contains $2 and $6" for user and supervisor instruction
    // faults — the one place a program-space access is *not* folded onto
    // data. The guarded region ends at $11000 and nothing follows it.
    let board = m68040(&[0x4ef9, 0x0001, 0x0ffe], |_| {}); // JMP $10FFE
    board.guarded.write_u8(0xffe, 0x4e).unwrap();
    board.guarded.write_u8(0xfff, 0x71).unwrap(); // NOP
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.cpu.step(); // the JMP; the prefetch past the end does not fault
    assert_eq!(board.cpu.regs().pc, 0x1_0ffe);
    board.cpu.step(); // the NOP runs, and the next fetch is used
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(
        board.peek_word(sp + 6),
        0x7008,
        "format $7, the bus error vector"
    );
    let ssw = board.peek_word(sp + 0x0c);
    assert_eq!(ssw & 7, 0b110, "TM $6: a supervisor code access");
    assert_eq!((ssw >> 8) & 1, 1, "RW: a read");
    assert_eq!(board.peek_long(sp + 0x14), 0x1_1000, "the prefetch address");
}

// ----------------------------------------------------------------------
// What the 68040 does not have
// ----------------------------------------------------------------------

#[test]
fn the_68040_has_no_coprocessor_interface() {
    // MC68020UM §7.5.2.3 makes `cpSAVE` in user state a privilege violation
    // *before* the processor talks to a coprocessor. A 68040 has no
    // coprocessor interface at all (M68040UM §1.1), so the same word is
    // simply an F-line instruction and the privilege check never happens.
    let cpsave = 0xf500u16 | (4 << 9); // coprocessor id 2, cpSAVE
    assert!(isa::decode_for(Model::M68030, cpsave).privileged);
    let insn = isa::decode_for(Model::M68040, cpsave);
    assert_eq!(insn.op, isa::Op::LineF);
    assert!(!insn.privileged);
}

#[test]
fn the_68030s_mmu_instructions_are_gone() {
    // M68040UM §1.1: the 68030's PMOVE/PLOAD/PTEST/PFLUSH travel over the
    // coprocessor interface, which the 68040 does not have. Their encodings
    // fall back to the line-F exception.
    for opcode in [0xf000u16, 0xf008, 0xf010, 0xf018] {
        assert_eq!(isa::decode_for(Model::M68030, opcode).op, isa::Op::Pgen);
        assert_eq!(isa::decode_for(Model::M68040, opcode).op, isa::Op::LineF);
    }
}

#[test]
fn callm_and_rtm_are_still_unimplemented() {
    // Gone at the 68030 (MC68030UM §12.1.3) and not back.
    for opcode in [0x06d0u16, 0x06c3] {
        assert_eq!(isa::decode_for(Model::M68040, opcode).op, isa::Op::Illegal);
    }
}

// ----------------------------------------------------------------------
// The disassembler
// ----------------------------------------------------------------------

#[test]
fn the_disassembler_speaks_68040() {
    for (words, text) in [
        (&[0xf620u16, 0x9000][..], "MOVE16 (A0)+,(A1)+"),
        (&[0xf600, 0x000f, 0xe802][..], "MOVE16 (A0)+,$fe802"),
        (&[0xf61b, 0x000f, 0xe802][..], "MOVE16 $fe802,(A3)"),
        (&[0xf4c8][..], "CINVL #3,(A0)"),
        (&[0xf491][..], "CINVP #2,(A1)"),
        (&[0xf458][..], "CINVA #1"),
        (&[0xf4ea][..], "CPUSHL #3,(A2)"),
        (&[0xf4f8][..], "CPUSHA #3"),
        (&[0x4e7b, 0x0806][..], "MOVEC D0,URP"),
        (&[0x4e7a, 0x1003][..], "MOVEC TC,D1"),
    ] {
        let d = super::disasm::disassemble_for(Model::M68040, 0x400, words);
        assert_eq!(alloc::format!("{d}"), text);
        assert_eq!(
            usize::from(d.len) / 2,
            words.len(),
            "{text} is {} words",
            words.len()
        );
    }
}

#[test]
fn the_disassembler_and_the_68040_agree_on_every_new_encoding() {
    // The generator emits both, so a length the disassembler reports and a
    // length the interpreter consumes must be the same one (CLAUDE.md,
    // *CPU cores*).
    let mut checked = 0;
    for opcode in 0xf400u32..=0xf6ff {
        let opcode = opcode as u16;
        let insn = isa::decode_for(Model::M68040, opcode);
        if matches!(insn.op, isa::Op::LineF | isa::Op::Illegal) {
            continue;
        }
        let words = [opcode, 0x9000, 0x0000, 0x0000];
        let d = super::disasm::disassemble_for(Model::M68040, 0x400, &words);
        assert_eq!(
            usize::from(d.len),
            2 + 2 * usize::from(insn.ext),
            "${opcode:04x} {}",
            insn.op
        );
        checked += 1;
    }
    assert!(checked > 100, "only {checked} encodings reached");
}

// ----------------------------------------------------------------------
// Snapshots
// ----------------------------------------------------------------------

#[test]
fn a_68040_snapshot_round_trips_its_control_registers() -> Result<()> {
    let board = Board::new(Model::M68040);
    board.boot(&[0x7001, 0x4e71]);
    board.with_regs(|r| {
        r.tc = 0xc000;
        r.urp = 0x0001_2000;
        r.srp = 0x0003_4000;
        r.itt = [0x00ff_c040, 0x0100_8000];
        r.dtt = [0x4000_e000, 0x0000_0000];
        r.mmusr = 0x1234_5041;
        r.cacr = 0x8000_8000;
        r.vbr = 0x0000_8000;
    });
    let bytes = snapshot(&board.cpu)?;
    let other = M68k::new(Config::MC68040);
    restore(&other, &bytes)?;
    assert_eq!(other.regs(), board.cpu.regs());
    assert_eq!(snapshot(&other)?, bytes, "a round trip is a fixed point");
    Ok(())
}

#[test]
fn an_ec040_snapshot_carries_only_the_registers_it_has() -> Result<()> {
    let board = Board::new(Model::M68EC040);
    board.boot(&[0x7001, 0x4e71]);
    board.with_regs(|r| {
        r.itt = [0x00ff_c040, 0x0100_8000];
        r.dtt = [0x4000_e000, 0x0000_0000];
        // No paged unit, so these are dropped on the way in.
        r.urp = 0x0001_2000;
        r.mmusr = 0xffff_ffff;
    });
    let after = board.cpu.regs();
    assert_eq!(after.urp, 0, "an MC68EC040 has no URP");
    assert_eq!(after.mmusr, 0, "nor an MMUSR");
    let bytes = snapshot(&board.cpu)?;
    let other = M68k::new(Config::MC68EC040);
    restore(&other, &bytes)?;
    assert_eq!(other.regs(), after);
    Ok(())
}
// ----------------------------------------------------------------------
// The memory management unit
// ----------------------------------------------------------------------

/// Where the translation tables go in the 64 KiB board: a root table at
/// `$3000`, a pointer table at `$3200` and a page table at `$3400`.
const ROOT: u32 = 0x3000;
const POINTER: u32 = 0x3200;
const PAGE: u32 = 0x3400;

/// Build a three-level table that maps the low 256 KiB to itself.
///
/// The root index is logical address bits 31–25, the pointer index bits
/// 24–18 and the page index bits 17–12 for 4 KiB pages or 17–13 for 8 KiB
/// ones (M68040UM §3.2.1), each scaled by four — so one root entry, one
/// pointer entry and a full page table cover `$00000000`–`$0003FFFF`, which
/// is where the vectors, the code and the stack all are. Translation applies
/// to instruction fetches too, and a test that mapped only its own page
/// would fault on its first prefetch.
fn identity_tables(board: &Board, page_bits: u32) {
    board.poke_long(0, 0x2000);
    board.poke_long(4, 0x0400);
    // A resident table descriptor is UDT = 10 or 11; U is bit 3 and W bit 2.
    board.poke_long(u64::from(ROOT), POINTER | 0b10);
    board.poke_long(u64::from(POINTER), PAGE | 0b10);
    let entries = 1u32 << (18 - page_bits);
    for i in 0..entries {
        // A resident page descriptor is PDT = 01 or 11, with the physical
        // address in bits 31-12 or 31-13.
        board.poke_long(u64::from(PAGE) + u64::from(i) * 4, (i << page_bits) | 0b01);
    }
}

/// A 68040 board with that table, and one page replaced by `page_descriptor`.
fn mapped(la: u32, page_descriptor: u32) -> Board {
    let board = Board::new(Model::M68040);
    identity_tables(&board, 12);
    board.poke_long(
        u64::from(PAGE) + u64::from((la >> 12) & 0x3f) * 4,
        page_descriptor,
    );
    board.cpu.step();
    board
}

/// Load `URP`, `SRP` and `TC`, then run `words` from `$400` with paged
/// translation live.
fn translate_on(board: &Board, words: &[u16]) {
    board.load(0x400, words);
    board.with_regs(|r| {
        r.urp = ROOT;
        r.srp = u64::from(ROOT);
        // TC: E set, P clear — 4 KiB pages (Figure 3-4).
        r.tc = 0x8000;
    });
    board.at(0x400);
}

#[test]
fn the_manuals_translation_table_example_indexes_the_three_levels() {
    // M68040UM Figure 3-13: "$76543210 ... the RI field of the logical
    // address, $3B, is mapped into bits 8-2 of the SRP value ... the PI
    // field, $15 ... the PGI field, $1", with 8 KiB pages.
    let la = 0x7654_3210u32;
    assert_eq!(la >> 25, 0x3b, "root index");
    assert_eq!((la >> 18) & 0x7f, 0x15, "pointer index");
    assert_eq!((la >> 13) & 0x1f, 0x01, "page index with 8 KiB pages");
    // And with 4 KiB pages the page index is one bit wider.
    assert_eq!((la >> 12) & 0x3f, 0x03);
}

#[test]
fn a_resident_page_translates_and_the_history_bits_are_written_back() {
    // A page descriptor: physical address $5000, CM = 01 (copyback),
    // PDT = 01 resident, U and M clear so the search has to set them
    // (M68040UM Table 3-1).
    let board = mapped(0x0001_0000, 0x0000_5000 | (1 << 5) | 0b01);
    // MOVE.W #$1234,($00010000).L
    translate_on(&board, &[0x33fc, 0x1234, 0x0001, 0x0000, 0x4e71]);
    board.cpu.step();
    assert_eq!(
        board.peek_word(0x5000),
        0x1234,
        "the write landed at the physical address"
    );
    let descriptor = board.peek_long(u64::from(PAGE) + 0x10 * 4);
    assert_eq!(descriptor & 0x08, 0x08, "U set by the search");
    assert_eq!(descriptor & 0x10, 0x10, "M set by a write to a clear M");
    // The two table descriptors get their U bits too.
    assert_eq!(board.peek_long(u64::from(ROOT)) & 0x08, 0x08);
    assert_eq!(board.peek_long(u64::from(POINTER)) & 0x08, 0x08);
}

#[test]
fn a_read_does_not_set_the_modified_bit() {
    // Table 3-1, the read rows: U goes to 1, M is left alone.
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b01);
    board.poke_word(0x5000, 0xbeef);
    translate_on(&board, &[0x3039, 0x0001, 0x0000, 0x4e71]); // MOVE.W ($10000).L,D0
    board.cpu.step();
    assert_eq!(board.cpu.regs().d[0] & 0xffff, 0xbeef);
    let descriptor = board.peek_long(u64::from(PAGE) + 0x10 * 4);
    assert_eq!(descriptor & 0x18, 0x08, "U set, M still clear");
}

#[test]
fn a_write_to_a_write_protected_page_is_an_access_error_with_the_atc_bit() {
    // §3.2.6.3: "an ATC descriptor corresponding to the logical address is
    // created with the W-bit set ... the subsequent retry of the write
    // access results in an access error exception being taken", and
    // §8.4.6.2 says the SSW's ATC bit is set "for an ATC fault due to ...
    // privilege violation (write protected or supervisor only)".
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b100 | 0b01);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    translate_on(&board, &[0x33fc, 0x1234, 0x0001, 0x0000, 0x4e71]);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
    assert_eq!(board.peek_word(0x5000), 0, "nothing was written");
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x7008, "format $7");
    let ssw = board.peek_word(sp + 0x0c);
    assert_eq!(ssw & 0x0400, 0x0400, "ATC: a translation failure");
    assert_eq!(ssw & 0x0100, 0, "RW clear: a write");
    assert_eq!(
        board.peek_long(sp + 0x14),
        0x0001_0000,
        "the logical address faulted"
    );
    // And M is not set on a write-protected page (Table 3-1's WP = 1 rows).
    assert_eq!(board.peek_long(u64::from(PAGE) + 0x10 * 4) & 0x10, 0);
}

#[test]
fn a_read_of_a_write_protected_page_is_allowed() {
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b100 | 0b01);
    board.poke_word(0x5000, 0x4321);
    translate_on(&board, &[0x3039, 0x0001, 0x0000, 0x4e71]);
    board.cpu.step();
    assert_eq!(board.cpu.regs().d[0] & 0xffff, 0x4321);
    assert_eq!(board.cpu.last_exception(), None);
}

#[test]
fn the_table_search_faults_where_the_manual_says_it_faults() {
    // "00 or 01 = Invalid. These codes indicate that the table at the next
    // level is not resident or that the logical address is out of bounds"
    // (§3.2.2.3, **UDT**), and the same for **PDT** = 00 at the page level.
    // One case per level: an invalid descriptor, and a table that is not
    // there at all.
    // Each case reaches its level through a branch of the tree the identity
    // map does not use — root entry 2 rather than 0 — so the code and the
    // stack stay mapped and the fault is the only thing that goes wrong.
    // `$04000000` has root index 2 and pointer index 0; `$04040000` has root
    // index 2 and pointer index 1.
    for (la, prepare, what) in [
        (0x0400_0000u32, None, "root"),
        (0x0404_0000, Some(POINTER | 0b10), "pointer"),
        (0x0001_0000, None, "page"),
    ] {
        let board = mapped(0x0001_0000, if what == "page" { 0 } else { 0x5000 | 0b01 });
        if let Some(descriptor) = prepare {
            board.poke_long(u64::from(ROOT) + 2 * 4, descriptor);
        }
        board.handler(0, vector::BUS_ERROR, 0x0800);
        translate_on(
            &board,
            &[0x3039, (la >> 16) as u16, la as u16, 0x4e71], // MOVE.W (la).L,D0
        );
        board.cpu.step();
        assert_eq!(
            board.cpu.last_exception(),
            Some(vector::BUS_ERROR),
            "an invalid {what} descriptor"
        );
        let sp = u64::from(board.cpu.regs().a[7]);
        assert_eq!(board.peek_word(sp + 6), 0x7008, "{what}: format $7");
        assert_eq!(
            board.peek_word(sp + 0x0c) & 0x0400,
            0x0400,
            "{what}: the ATC bit"
        );
        assert_eq!(
            board.peek_long(sp + 0x14),
            la,
            "{what}: the logical address"
        );
    }
}

#[test]
fn a_bus_error_during_a_table_search_is_an_atc_fault() {
    // §8.4.6.2: the ATC bit is "set for an ATC fault due to a nonresident
    // entry (**bus error during table search** or invalid descriptor
    // encountered)". A descriptor fetch the address space refuses is that
    // first case: the entry the search creates has R clear, and the retried
    // access is what takes the exception (§3.5).
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b01);
    // Root entry 2, pointing at a pointer table nothing answers for.
    board.poke_long(u64::from(ROOT) + 2 * 4, 0x0100_0000 | 0b10);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    translate_on(&board, &[0x3039, 0x0400, 0x0000, 0x4e71]);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 0x0c) & 0x0400, 0x0400);
    // What tells the two apart afterwards is the MMUSR a PTEST reports,
    // which carries B for the transfer error and not for a merely invalid
    // descriptor — see `ptest_reports_a_transfer_error_with_b_and_nothing_else`.
}

#[test]
fn an_indirect_descriptor_is_followed_to_the_real_page() {
    // §3.2.4.1: "the address contained in the highest order 30 bits of the
    // descriptor is a pointer to the page descriptor that is to be used to
    // map the logical address".
    let board = mapped(0x0001_0000, 0x0000_3800 | 0b10); // PDT = 10, indirect
    board.poke_long(0x3800, 0x0000_5000 | 0b01);
    translate_on(&board, &[0x33fc, 0x1234, 0x0001, 0x0000, 0x4e71]);
    board.cpu.step();
    assert_eq!(board.peek_word(0x5000), 0x1234);
    // "the modified indication is maintained only in the single descriptor":
    // the history bits go to the descriptor the indirection names, not to
    // the indirect descriptor itself.
    assert_eq!(board.peek_long(0x3800) & 0x18, 0x18, "U and M there");
    assert_eq!(
        board.peek_long(u64::from(PAGE) + 0x10 * 4) & 0x18,
        0,
        "and not in the indirect descriptor"
    );
}

#[test]
fn an_indirect_descriptor_pointing_at_another_is_invalid() {
    // "This encoding is invalid for a page descriptor pointed to by an
    // indirect descriptor" (§3.2.2.3, **PDT**).
    let board = mapped(0x0001_0000, 0x0000_3800 | 0b10);
    board.poke_long(0x3800, 0x0000_3900 | 0b10);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    translate_on(&board, &[0x3039, 0x0001, 0x0000, 0x4e71]);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
}

#[test]
fn a_supervisor_only_page_refuses_a_user_access() {
    // §3.2.6.2: "when a table search for a user access encounters an S-bit
    // set in a page descriptor, the table search ends, and an ATC descriptor
    // ... is created with the S-bit set. A subsequent retry of the user
    // access results in an access error exception being taken."
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b1000_0000 | 0b01);
    board.poke_word(0x5000, 0xcafe);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    translate_on(&board, &[0x3039, 0x0001, 0x0000, 0x4e71]);
    // A supervisor read is fine.
    board.cpu.step();
    assert_eq!(board.cpu.regs().d[0] & 0xffff, 0xcafe);
    assert_eq!(board.cpu.last_exception(), None);

    // The same page from user state is not. Both root pointers name the
    // same table, which is what §3.2.6.2 says the S bit is for.
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b1000_0000 | 0b01);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    translate_on(&board, &[0x3039, 0x0001, 0x0000, 0x4e71]);
    board.with_regs(|r| {
        r.sr &= !flags::S;
        r.usp = 0x1f00;
        r.ssp = 0x2000;
        r.a[7] = 0x1f00;
    });
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 0x0c) & 0x0400, 0x0400, "an ATC fault");
}

#[test]
fn a_transparent_translation_register_works_with_translation_disabled() {
    // §3.1.3: "the TTRs operate independently of the E-bit in the TCR".
    // A data TTR covering $00xxxxxx with write protection, S = 1x.
    let board = Board::new(Model::M68040);
    board.boot(&[0x33fc, 0x1234, 0x0000, 0x5000, 0x4e71]);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.with_regs(|r| {
        // Base $00, mask $00 — sixteen megabytes from zero — enabled, S = 1x
        // so both privilege modes match, and write protected.
        r.dtt[0] = 0x8000 | (2 << 13) | 0x0004;
    });
    board.cpu.step();
    assert_eq!(
        board.cpu.last_exception(),
        Some(vector::BUS_ERROR),
        "a write to a write-protected block is aborted"
    );
    assert_eq!(board.peek_word(0x5000), 0, "and nothing landed");
}

#[test]
fn an_instruction_ttr_does_not_answer_for_a_data_access() {
    // §3.4: the instruction memory unit's registers are "only used for
    // instruction prefetches", which is the one place the 68040's merged
    // address space is not merged.
    let board = Board::new(Model::M68040);
    board.boot(&[0x33fc, 0x1234, 0x0000, 0x5000, 0x4e71]);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.with_regs(|r| {
        // Write-protect everything, but only for instruction accesses.
        r.itt[0] = 0x8000 | (2 << 13) | 0x0004;
    });
    board.cpu.step();
    assert_eq!(
        board.cpu.last_exception(),
        None,
        "a data write is untouched"
    );
    assert_eq!(board.peek_word(0x5000), 0x1234);
}

#[test]
fn the_first_transparent_register_wins_when_both_match() {
    // §3.4: "If both registers match, the TT0 status bits are used for the
    // access." TT0 permits the write, TT1 would have refused it.
    let board = Board::new(Model::M68040);
    board.boot(&[0x33fc, 0x1234, 0x0000, 0x5000, 0x4e71]);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    board.with_regs(|r| {
        r.dtt[0] = 0x8000 | (2 << 13);
        r.dtt[1] = 0x8000 | (2 << 13) | 0x0004;
    });
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), None);
    assert_eq!(board.peek_word(0x5000), 0x1234);
}

#[test]
fn the_s_field_selects_which_privilege_mode_a_block_answers_for() {
    // §3.1.3: 00 matches user only, 01 supervisor only, 1x both.
    for (s, supervisor_aborts, user_aborts) in [
        (0u32, false, true),
        (1, true, false),
        (2, true, true),
        (3, true, true),
    ] {
        for user in [false, true] {
            let board = Board::new(Model::M68040);
            board.boot(&[0x33fc, 0x1234, 0x0000, 0x5000, 0x4e71]);
            board.handler(0, vector::BUS_ERROR, 0x0800);
            board.with_regs(|r| {
                r.dtt[0] = 0x8000 | (s << 13) | 0x0004;
                if user {
                    r.sr &= !flags::S;
                    r.usp = 0x1f00;
                    r.ssp = 0x2000;
                    r.a[7] = 0x1f00;
                }
            });
            board.cpu.step();
            let aborted = board.cpu.last_exception() == Some(vector::BUS_ERROR);
            let expected = if user { user_aborts } else { supervisor_aborts };
            assert_eq!(aborted, expected, "S = {s}, user = {user}");
        }
    }
}

#[test]
fn eight_kilobyte_pages_use_a_five_bit_page_index() {
    // Figure 3-12's 8 KiB page descriptor: the physical address is bits
    // 31-13, and §3.2.1 gives the page index five bits. The pointer table
    // descriptor's page-table address then reaches down to bit 7, because
    // a 32-entry page table is 128-byte aligned (Figure 3-11).
    let la = 0x0001_0000u32;
    let board = Board::new(Model::M68040);
    identity_tables(&board, 13);
    // Two entries of the 4 KiB table become one of the 8 KiB table, so the
    // page this replaces covers $10000-$11FFF.
    board.poke_long(
        u64::from(PAGE) + u64::from((la >> 13) & 0x1f) * 4,
        0x0000_6000 | 0b01,
    );
    board.cpu.step();
    board.load(0x400, &[0x33fc, 0x1234, 0x0001, 0x0000, 0x4e71]);
    board.with_regs(|r| {
        r.urp = ROOT;
        r.srp = u64::from(ROOT);
        r.tc = 0xc000; // E and P: 8 KiB pages
    });
    board.at(0x400);
    board.cpu.step();
    assert_eq!(board.peek_word(0x6000), 0x1234);
}

#[test]
fn ptest_reports_what_the_search_found() {
    // M68000PRM §6, *PTEST* (MC68040), and M68040UM Figure 3-6: the
    // physical address in bits 31-12, then G, U1, U0, S, CM, M, W, T, R.
    // The page descriptor here has G, U1, CM = 11 and S set.
    let descriptor = 0x0000_5000 | (1 << 10) | (1 << 9) | (3 << 5) | (1 << 7) | 0b01;
    let board = mapped(0x0001_0000, descriptor);
    board.load(
        0x400,
        &[
            0x203c, 0x0000, 0x0005, // MOVE.L #5,D0   (supervisor data)
            0x4e7b, 0x0001, // MOVEC D0,DFC
            0x207c, 0x0001, 0x0000, // MOVEA.L #$10000,A0
            0xf568, // PTESTR (A0)
            0x4e7a, 0x1805, // MOVEC MMUSR,D1
            0x4e71,
        ],
    );
    board.with_regs(|r| {
        r.urp = ROOT;
        r.srp = u64::from(ROOT);
        r.tc = 0x8000;
    });
    board.at(0x400);
    for _ in 0..5 {
        board.cpu.step();
    }
    let mmusr = board.cpu.regs().d[1];
    assert_eq!(mmusr & 0xffff_f000, 0x0000_5000, "physical address");
    assert_eq!(mmusr & 1, 1, "R: resident");
    assert_eq!(mmusr & 0x0400, 0x0400, "G");
    assert_eq!(mmusr & 0x0200, 0x0200, "U1");
    assert_eq!(mmusr & 0x0100, 0, "U0 clear");
    assert_eq!(mmusr & 0x0080, 0x0080, "S");
    assert_eq!((mmusr >> 5) & 3, 3, "CM");
    assert_eq!(mmusr & 0x0010, 0, "M clear: PTESTR simulates a read");
    assert_eq!(mmusr & 0x0004, 0, "W clear");
    assert_eq!(mmusr & 0x0002, 0, "T clear: not a transparent block");
}

#[test]
fn ptestw_sets_the_modified_bit_and_ptestr_does_not() {
    // "PTESTR simulates a read access and sets the U-bit in each descriptor
    // during table searches; PTESTW simulates a write access and also sets
    // the M-bit in the descriptors, the address translation cache entry, and
    // the MMU status register" (M68000PRM §6).
    for (opcode, modified) in [(0xf568u16, false), (0xf548, true)] {
        let board = mapped(0x0001_0000, 0x0000_5000 | 0b01);
        board.load(
            0x400,
            &[
                0x203c, 0x0000, 0x0005, 0x4e7b, 0x0001, // DFC = 5
                0x207c, 0x0001, 0x0000, // MOVEA.L #$10000,A0
                opcode, 0x4e7a, 0x1805, 0x4e71,
            ],
        );
        board.with_regs(|r| {
            r.urp = ROOT;
            r.srp = u64::from(ROOT);
            r.tc = 0x8000;
        });
        board.at(0x400);
        for _ in 0..5 {
            board.cpu.step();
        }
        assert_eq!(
            board.cpu.regs().d[1] & 0x10 != 0,
            modified,
            "${opcode:04x} MMUSR M"
        );
        assert_eq!(
            board.peek_long(u64::from(PAGE) + 0x10 * 4) & 0x10 != 0,
            modified,
            "${opcode:04x} descriptor M"
        );
    }
}

#[test]
fn ptest_on_an_invalid_page_clears_the_resident_bit() {
    let board = mapped(0x0001_0000, 0);
    board.load(
        0x400,
        &[
            0x203c, 0x0000, 0x0005, 0x4e7b, 0x0001, 0x207c, 0x0001, 0x0000, 0xf568, 0x4e7a, 0x1805,
            0x4e71,
        ],
    );
    board.with_regs(|r| {
        r.urp = ROOT;
        r.srp = u64::from(ROOT);
        r.tc = 0x8000;
    });
    board.at(0x400);
    for _ in 0..5 {
        board.cpu.step();
    }
    assert_eq!(board.cpu.regs().d[1] & 1, 0, "R clear");
}

#[test]
fn ptest_on_a_transparently_translated_address_reports_t_and_r_alone() {
    // §3.1.4, **T**: "If the T-bit is set, then the PTEST address matches an
    // instruction or data TTR, the R-bit is set, and all other bits are
    // zero."
    let board = Board::new(Model::M68040);
    board.boot(&[0x4e71]);
    board.load(
        0x400,
        &[
            0x203c, 0x0000, 0x0005, 0x4e7b, 0x0001, 0x207c, 0x0001, 0x0000, 0xf568, 0x4e7a, 0x1805,
            0x4e71,
        ],
    );
    board.with_regs(|r| {
        r.dtt[0] = 0x8000 | (2 << 13);
    });
    board.at(0x400);
    for _ in 0..5 {
        board.cpu.step();
    }
    assert_eq!(board.cpu.regs().d[1], 0b11, "T and R, nothing else");
}

#[test]
fn ptest_reports_a_transfer_error_with_b_and_nothing_else() {
    // §3.1.4, **B**: "set if a transfer error is encountered during the
    // table search for the PTEST instruction. If the B-bit is set, all other
    // bits are zero."
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b01);
    // Root entry 2 — a branch the identity map does not use — pointing at a
    // pointer table nothing answers for; `$04000000` reaches it.
    board.poke_long(u64::from(ROOT) + 2 * 4, 0x0100_0000 | 0b10);
    board.load(
        0x400,
        &[
            0x203c, 0x0000, 0x0005, 0x4e7b, 0x0001, // DFC = 5
            0x207c, 0x0400, 0x0000, // MOVEA.L #$04000000,A0
            0xf568, // PTESTR (A0)
            0x4e7a, 0x1805, 0x4e71,
        ],
    );
    board.with_regs(|r| {
        r.urp = ROOT;
        r.srp = u64::from(ROOT);
        r.tc = 0x8000;
    });
    board.at(0x400);
    for _ in 0..5 {
        board.cpu.step();
    }
    assert_eq!(board.cpu.regs().d[1], 0x0800, "B alone");
}

#[test]
fn pflush_drops_the_entry_and_the_next_access_searches_again() {
    // The translation is cached, so a page descriptor changed behind the
    // unit's back is not seen until the entry is flushed — which is the
    // whole reason PFLUSH exists (§3.6.1: "a PFLUSH instruction must be
    // executed to flush all existing valid entries from the ATCs").
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b01);
    board.poke_word(0x5000, 0x1111);
    board.poke_word(0x6000, 0x2222);
    board.load(
        0x400,
        &[
            0x3039, 0x0001, 0x0000, // MOVE.W ($10000).L,D0
            0x3039, 0x0001, 0x0000, // again
            0xf518, // PFLUSHA
            0x3039, 0x0001, 0x0000, // and again
            0x4e71,
        ],
    );
    board.with_regs(|r| {
        r.urp = ROOT;
        r.srp = u64::from(ROOT);
        r.tc = 0x8000;
    });
    board.at(0x400);
    board.cpu.step();
    assert_eq!(board.cpu.regs().d[0] & 0xffff, 0x1111);
    // Repoint the page without flushing: the cached entry still answers.
    board.poke_long(u64::from(PAGE) + 0x10 * 4, 0x0000_6000 | 0b01);
    board.cpu.step();
    assert_eq!(
        board.cpu.regs().d[0] & 0xffff,
        0x1111,
        "the cache still answers"
    );
    board.cpu.step(); // PFLUSHA
    board.cpu.step();
    assert_eq!(
        board.cpu.regs().d[0] & 0xffff,
        0x2222,
        "and now the search runs again"
    );
}

#[test]
fn pflushan_spares_a_global_entry_and_pflusha_does_not() {
    // M68000PRM §6, *PFLUSH* (MC68040): "the PFLUSHN and PFLUSHAN
    // instructions have a global option specified and invalidate only
    // nonglobal entries."
    for (opcode, survives) in [(0xf510u16, true), (0xf518, false)] {
        let board = mapped(0x0001_0000, 0x0000_5000 | (1 << 10) | 0b01);
        board.poke_word(0x5000, 0x1111);
        board.poke_word(0x6000, 0x2222);
        board.load(
            0x400,
            &[
                0x3039, 0x0001, 0x0000, opcode, 0x3039, 0x0001, 0x0000, 0x4e71,
            ],
        );
        board.with_regs(|r| {
            r.urp = ROOT;
            r.srp = u64::from(ROOT);
            r.tc = 0x8000;
        });
        board.at(0x400);
        board.cpu.step();
        board.poke_long(u64::from(PAGE) + 0x10 * 4, 0x0000_6000 | 0b01);
        board.cpu.step(); // the flush
        board.cpu.step();
        let seen = board.cpu.regs().d[0] & 0xffff;
        assert_eq!(
            seen,
            if survives { 0x1111 } else { 0x2222 },
            "${opcode:04x}"
        );
    }
}

#[test]
fn pflush_by_page_only_drops_the_page_it_names() {
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b01);
    // A second page at $00011000, through the same tables.
    board.poke_long(u64::from(PAGE) + 0x11 * 4, 0x0000_7000 | 0b01);
    board.poke_word(0x5000, 0x1111);
    board.poke_word(0x7000, 0x3333);
    board.load(
        0x400,
        &[
            0x203c, 0x0000, 0x0005, 0x4e7b, 0x0001, // DFC = 5
            0x3039, 0x0001, 0x0000, // touch $10000
            0x3039, 0x0001, 0x1000, // touch $11000
            0x207c, 0x0001, 0x1000, // MOVEA.L #$11000,A0
            0xf508, // PFLUSH (A0)
            0x3039, 0x0001, 0x0000, // $10000 again
            0x4e71,
        ],
    );
    board.with_regs(|r| {
        r.urp = ROOT;
        r.srp = u64::from(ROOT);
        r.tc = 0x8000;
    });
    board.at(0x400);
    for _ in 0..4 {
        board.cpu.step();
    }
    // Repoint both pages; only the flushed one is searched again.
    board.poke_long(u64::from(PAGE) + 0x10 * 4, 0x0000_6000 | 0b01);
    board.poke_word(0x6000, 0x2222);
    for _ in 0..3 {
        board.cpu.step();
    }
    assert_eq!(
        board.cpu.regs().d[0] & 0xffff,
        0x1111,
        "$10000's entry was not the one flushed"
    );
}

#[test]
fn an_ec040_has_no_ptest_and_a_pflush_that_does_nothing() {
    // M68000PRM §6: *PTEST* is given for the MC68040 and MC68LC040 only,
    // and *PFLUSH* (MC68EC040) "should not be executed ... suspends
    // operation ... and subsequently continues with no adverse effects".
    assert_eq!(
        isa::decode_for(Model::M68EC040, 0xf568).op,
        isa::Op::LineF,
        "no PTEST on an MC68EC040"
    );
    assert_eq!(
        isa::decode_for(Model::M68EC040, 0xf518).op,
        isa::Op::Pflusha,
        "but PFLUSH decodes"
    );
    let board = Board::new(Model::M68EC040);
    board.boot(&[0xf518, 0x4e71]);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), None);
    assert_eq!(board.cpu.regs().pc, 0x402);
}

#[test]
fn the_mmu_instructions_are_privileged() {
    for opcode in [0xf500u16, 0xf508, 0xf510, 0xf518, 0xf548, 0xf568] {
        assert!(
            isa::decode_for(Model::M68040, opcode).privileged,
            "${opcode:04x}"
        );
    }
}

#[test]
fn the_disassembler_prints_the_68040s_mmu_instructions() {
    for (opcode, text) in [
        (0xf500u16, "PFLUSHN (A0)"),
        (0xf50b, "PFLUSH (A3)"),
        (0xf510, "PFLUSHAN"),
        (0xf518, "PFLUSHA"),
        (0xf54a, "PTESTW (A2)"),
        (0xf56d, "PTESTR (A5)"),
    ] {
        let d = super::disasm::disassemble_for(Model::M68040, 0x400, &[opcode]);
        assert_eq!(alloc::format!("{d}"), text);
    }
}

#[test]
fn a_snapshot_does_not_carry_the_address_translation_cache() {
    // CLAUDE.md, *Devices*: derived state is never serialized. A restored
    // core rebuilds its translations with a table search, and gets the same
    // answers.
    let board = mapped(0x0001_0000, 0x0000_5000 | 0b01);
    board.poke_word(0x5000, 0x1111);
    translate_on(&board, &[0x3039, 0x0001, 0x0000, 0x4e71]);
    board.cpu.step();
    let bytes = snapshot(&board.cpu).expect("a snapshot");
    let other = M68k::new(Config::MC68040);
    restore(&other, &bytes).expect("a restore");
    assert_eq!(other.regs(), board.cpu.regs());
    assert_eq!(snapshot(&other).expect("again"), bytes);
}

// ----------------------------------------------------------------------
// The on-chip floating-point unit
// ----------------------------------------------------------------------

use crate::float::x87::F80;

/// `1.0` and a few neighbours, in the extended format.
const FP_ONE: F80 = F80::new(0x3fff, 1 << 63);
const FP_THREE: F80 = F80::new(0x4000, 0xc000_0000_0000_0000);

/// A 68040 with its on-chip unit, running `words` at `$500`.
fn fpu_board(words: &[u16]) -> Board {
    let board = Board::new(Model::M68040);
    board.boot(&[0x4e71]);
    board.load(0x500, words);
    board.at(0x500);
    board
}

#[test]
fn the_on_chip_unit_comes_with_the_part() {
    use crate::core::props::Props;

    // An MC68040 has one and cannot be told not to; an MC68LC040 and an
    // MC68EC040 do not have one (M68040UM Appendices A and B).
    let cpu = M68k::from_props(&Props::new().with("model", "68040")).unwrap();
    assert!(cpu.config().fpu.present(), "the default follows the part");
    assert!(cpu.config().fpu.is_onchip_040());
    for model in ["68lc040", "68ec040"] {
        let cpu = M68k::from_props(&Props::new().with("model", model)).unwrap();
        assert!(!cpu.config().fpu.present(), "{model}");
    }
    let err = M68k::from_props(&Props::new().with("model", "68040").with("fpu", "none"))
        .expect_err("a 68040's unit cannot be switched off");
    assert!(alloc::format!("{err}").contains("68lc040"), "{err}");
    let err = M68k::from_props(&Props::new().with("model", "68030").with("fpu", "68040"))
        .expect_err("a 68030 has no on-chip unit");
    assert!(alloc::format!("{err}").contains("own chip"), "{err}");
}

#[test]
fn the_hardware_subset_is_computed() {
    // M68040UM Table 9-10 and M68000PRM Table A-1: FABS, FADD, FCMP, FDIV,
    // FMOVE, FMUL, FNEG, FSQRT, FSUB and FTST are hardware; everything else
    // traps. FADD FP1,FP0 with FP0 = 1.0 and FP1 = 3.0 is 4.0.
    let board = fpu_board(&[0xf200, 0x0422, 0x4e71]); // FADD.X FP1,FP0
    board.with_regs(|r| {
        r.fp[0] = FP_ONE;
        r.fp[1] = FP_THREE;
    });
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), None, "no trap");
    assert_eq!(board.cpu.regs().fp[0], F80::new(0x4001, 1 << 63), "4.0");
}

/// The opmodes of the instructions the 68040 leaves to software, with their
/// mnemonics, from M68040UM Table 9-10 and M68000PRM Table A-1.
const UNIMPLEMENTED_OPMODES: [(u16, &str); 29] = [
    (0x01, "FINT"),
    (0x02, "FSINH"),
    (0x03, "FINTRZ"),
    (0x06, "FLOGNP1"),
    (0x08, "FETOXM1"),
    (0x09, "FTANH"),
    (0x0a, "FATAN"),
    (0x0c, "FASIN"),
    (0x0d, "FATANH"),
    (0x0e, "FSIN"),
    (0x0f, "FTAN"),
    (0x10, "FETOX"),
    (0x11, "FTWOTOX"),
    (0x12, "FTENTOX"),
    (0x14, "FLOGN"),
    (0x15, "FLOG10"),
    (0x16, "FLOG2"),
    (0x19, "FCOSH"),
    (0x1c, "FACOS"),
    (0x1d, "FCOS"),
    (0x1e, "FGETEXP"),
    (0x1f, "FGETMAN"),
    (0x21, "FMOD"),
    (0x24, "FSGLDIV"),
    (0x25, "FREM"),
    (0x26, "FSCALE"),
    (0x27, "FSGLMUL"),
    (0x30, "FSINCOS"),
    (0x00, "FMOVECR"), // by its own encoding, below
];

#[test]
fn every_unimplemented_instruction_takes_vector_11_with_a_format_2_frame() {
    // M68040UM §9.6.1: "the processor creates a format $2 stack frame ...
    // The saved PC value is the logical address of the instruction that
    // follows the unimplemented floating-point instruction. The processor
    // generates exception vector number 11". §9.6.1 again: the handler
    // "checks for the format $2 stack frame to distinguish an unimplemented
    // floating-point instruction from other F-line unimplemented
    // instructions", which stack format $0.
    for (opmode, what) in UNIMPLEMENTED_OPMODES {
        if what == "FMOVECR" {
            continue;
        }
        // FxxxX FP1,FP0: opclass 000, source FP1, destination FP0.
        let command = 0x0400 | opmode;
        let board = fpu_board(&[0xf200, command, 0x4e71]);
        board.handler(0, vector::LINE_F, 0x0c00);
        let sr = board.cpu.regs().sr;
        board.cpu.step();
        assert_eq!(board.cpu.last_exception(), Some(vector::LINE_F), "{what}");
        let sp = u64::from(board.cpu.regs().a[7]);
        assert_eq!(board.peek_word(sp), sr, "{what}: +$00 SR");
        assert_eq!(
            board.peek_long(sp + 2),
            0x504,
            "{what}: +$02 the *next* instruction"
        );
        assert_eq!(
            board.peek_word(sp + 6),
            0x202c,
            "{what}: +$06 format $2, the line-F vector offset"
        );
    }
}

#[test]
fn fmovecr_is_unimplemented_on_a_68040() {
    // Table 9-10 lists FMOVECR among the monadic operations the 68040 does
    // not implement: the constant ROM is the software package's.
    let board = fpu_board(&[0xf200, 0x5c00, 0x4e71]); // FMOVECR #$00,FP0
    board.handler(0, vector::LINE_F, 0x0c00);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::LINE_F));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x202c, "format $2");
}

#[test]
fn an_f_line_word_that_is_not_an_instruction_stacks_format_0() {
    // §9.6.1: "If the processor encounters an F-line instruction and the
    // instruction patterns do not match either of the above two cases, the
    // processor takes an F-line illegal exception ... and pushes a four-word
    // stack frame format $0 on the system stack. Since the unimplemented
    // floating-point exception and the F-line illegal instruction share the
    // same vector, the exception handler uses the stack frame format ($0 or
    // $2) to distinguish between the two."
    let board = fpu_board(&[0xfa00, 0x0000, 0x4e71]); // coprocessor id 5
    board.handler(0, vector::LINE_F, 0x0c00);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::LINE_F));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x002c, "format $0");
    assert_eq!(board.peek_long(sp + 2), 0x500, "the instruction itself");
}

#[test]
fn the_effective_address_is_calculated_and_the_operand_fetched_before_the_trap() {
    // §9.6.1: "the instruction is partially decoded to allow fetching of the
    // memory source operand ... The fetched source operand is passed to the
    // FPU, which converts the operand to extended precision and saves the
    // intermediate result", and §8.4.6.2 says the format $2 frame's address
    // field is "the calculated effective address determined by the effective
    // address field of the unimplemented instruction".
    let board = fpu_board(&[0xf218, 0x418e, 0x4e71]); // FSIN.L (A0)+,FP3
    board.with_regs(|r| r.a[0] = 0x1000);
    board.poke_long(0x1000, 7);
    board.handler(0, vector::LINE_F, 0x0c00);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::LINE_F));
    assert_eq!(board.cpu.regs().a[0], 0x1004, "the postincrement happened");
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x202c, "format $2");
    assert_eq!(board.peek_long(sp + 8), 0x1000, "+$08 the calculated <ea>");
}

#[test]
fn fsave_after_an_unimplemented_instruction_writes_the_26_word_frame() {
    // M68040UM Figure 9-10(d) and Table 9-16, field by field. FSIN.X FP1,FP0
    // with FP1 = 3.0 and FP0 = 1.0: a monadic operation, so FPTEMP has no
    // destination operand in it and DTAG is zero.
    let board = fpu_board(&[0xf200, 0x040e, 0x4e71]); // FSIN.X FP1,FP0
    board.with_regs(|r| {
        r.fp[0] = FP_ONE;
        r.fp[1] = FP_THREE;
    });
    board.handler(0, vector::LINE_F, 0x0c00);
    // The handler saves the frame at $1000 and stops.
    board.load(0x0c00, &[0xf310, 0x4e71]); // FSAVE (A0)
    board.with_regs(|r| r.a[0] = 0x1000);
    board.cpu.step(); // the FSIN traps
    assert_eq!(board.cpu.last_exception(), Some(vector::LINE_F));
    board.cpu.step(); // the FSAVE
    assert_eq!(
        board.peek_long(0x1000),
        0x4130_0000,
        "+$00 version $41, a $30-byte body"
    );
    assert_eq!(board.peek_long(0x1004), 0, "+$04 CMDREG3B: an E3 field");
    assert_eq!(board.peek_long(0x1008), 0, "+$08 reserved");
    assert_eq!(
        board.peek_long(0x100c) >> 29,
        0b000,
        "+$0C STAG: a normalized source"
    );
    assert_eq!(
        board.peek_long(0x1010),
        0x040e_0000,
        "+$10 CMDREG1B: the command word"
    );
    assert_eq!(board.peek_long(0x1014) >> 29, 0, "+$14 DTAG");
    assert_eq!(
        board.peek_long(0x1018),
        1 << 26,
        "+$18 E1 set, E3 clear, T clear"
    );
    // ETEMP is the source, 3.0, in the 96-bit extended layout.
    assert_eq!(board.peek_long(0x1028), 0x4000_0000, "+$28 ETS and ETE");
    assert_eq!(board.peek_long(0x102c), 0xc000_0000, "+$2C ETM[63-32]");
    assert_eq!(board.peek_long(0x1030), 0, "+$30 ETM[31-00]");
    // A monadic operation leaves FPTEMP alone.
    assert_eq!(board.peek_long(0x101c), 0, "+$1C FPTS and FPTE");
}

#[test]
fn a_dyadic_unimplemented_instruction_saves_both_operands() {
    // Table 9-16: "FPTEMP — Destination operand, if any, is converted to
    // extended precision" and "DTAG — Destination operand tag, if any".
    // FREM is dyadic and unimplemented.
    let board = fpu_board(&[0xf200, 0x0425, 0x4e71]); // FREM.X FP1,FP0
    board.with_regs(|r| {
        r.fp[0] = FP_ONE;
        r.fp[1] = FP_THREE;
    });
    board.handler(0, vector::LINE_F, 0x0c00);
    board.load(0x0c00, &[0xf310, 0x4e71]); // FSAVE (A0)
    board.with_regs(|r| r.a[0] = 0x1000);
    board.cpu.step();
    board.cpu.step();
    assert_eq!(board.peek_long(0x101c), 0x3fff_0000, "FPTEMP is 1.0");
    assert_eq!(board.peek_long(0x1020), 0x8000_0000);
    assert_eq!(board.peek_long(0x1028), 0x4000_0000, "ETEMP is 3.0");
    assert_eq!(board.peek_long(0x1014) >> 29, 0b000, "DTAG: normalized");
}

#[test]
fn the_data_tags_name_the_operand_type() {
    // §9.7, *STAG, DTAG*: 000 normalized, 001 zero, 010 infinity, 011 NaN,
    // 100 an extended denormal or unnormal.
    for (value, tag, what) in [
        (FP_ONE, 0b000u32, "normalized"),
        (F80::ZERO, 0b001, "zero"),
        (F80::new(0x7fff, 1 << 63), 0b010, "infinity"),
        (F80::new(0x7fff, u64::MAX), 0b011, "NaN"),
        (F80::new(0x0000, 1), 0b100, "denormalized"),
        (F80::new(0x4000, 1), 0b100, "unnormalized"),
    ] {
        let board = fpu_board(&[0xf200, 0x040e, 0x4e71]); // FSIN.X FP1,FP0
        board.with_regs(|r| r.fp[1] = value);
        board.handler(0, vector::LINE_F, 0x0c00);
        board.load(0x0c00, &[0xf310, 0x4e71]);
        board.with_regs(|r| r.a[0] = 0x1000);
        board.cpu.step();
        board.cpu.step();
        assert_eq!(board.peek_long(0x100c) >> 29, tag, "STAG for {what}");
    }
}

#[test]
fn frestore_of_an_unimplemented_frame_leaves_the_unit_idle() {
    // The emulation handler pops the frame once it has produced the result,
    // and a following FSAVE reports an idle unit (M68040UM §9.7).
    let board = fpu_board(&[0xf200, 0x040e, 0x4e71]);
    board.handler(0, vector::LINE_F, 0x0c00);
    board.load(
        0x0c00,
        &[
            0xf310, // FSAVE (A0)
            0xf350, // FRESTORE (A0)
            0xf311, // FSAVE (A1)
            0x4e71,
        ],
    );
    board.with_regs(|r| {
        r.a[0] = 0x1000;
        r.a[1] = 0x1400;
    });
    board.cpu.step(); // the trap
    for _ in 0..3 {
        board.cpu.step();
    }
    assert_eq!(board.peek_long(0x1400), 0x4100_0000, "an idle frame");
}

#[test]
fn an_untouched_unit_saves_a_null_frame_and_a_used_one_saves_the_idle_frame() {
    // M68040UM Figure 9-10(b) and (c): the null frame is a zero long word,
    // and the idle frame is version $41 with a zero-length body — the whole
    // frame is the format long word.
    let board = fpu_board(&[0xf310, 0x4e71]); // FSAVE (A0)
    board.with_regs(|r| r.a[0] = 0x1000);
    board.cpu.step();
    assert_eq!(board.peek_long(0x1000), 0, "null");

    let board = fpu_board(&[0xf200, 0x0422, 0xf310, 0x4e71]); // FADD, then FSAVE
    board.with_regs(|r| r.a[0] = 0x1000);
    board.cpu.step();
    board.cpu.step();
    assert_eq!(board.peek_long(0x1000), 0x4100_0000, "idle");
}

#[test]
fn packed_decimal_is_an_unsupported_data_type_rather_than_line_f() {
    // §9.6.2: "an unsupported data type exception occurs when ... either the
    // source or destination data format is packed decimal real ...
    // Unsupported data types with operands that have opclass 010 or 000
    // ... cause a pre-instruction exception ... A format $0 ... stack frame
    // is saved, and vector number 55 is fetched."
    let board = fpu_board(&[0xf210, 0x4c00, 0x4e71]); // FMOVE.P (A0),FP0
    board.with_regs(|r| r.a[0] = 0x1000);
    board.handler(0, vector::FP_UNSUPPORTED_TYPE, 0x0c00);
    board.cpu.step();
    assert_eq!(
        board.cpu.last_exception(),
        Some(vector::FP_UNSUPPORTED_TYPE)
    );
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x00dc, "format $0, offset $0DC");
    assert_eq!(board.peek_long(sp + 2), 0x500, "the instruction itself");

    // A 68881 has no such exception: packed decimal is simply not
    // implemented there, and the encoding is line F.
    let b881 = Board::with_fpu(Model::M68030, super::Coprocessor::M68881);
    b881.boot(&[0xf210, 0x4c00, 0x4e71]);
    b881.handler(0, vector::LINE_F, 0x0c00);
    b881.cpu.step();
    assert_eq!(b881.cpu.last_exception(), Some(vector::LINE_F));
}

#[test]
fn a_packed_decimal_store_is_a_post_instruction_exception() {
    // §9.6.2: "when an unsupported data type is detected for opclass 011
    // (register-to-memory) instructions, a post-instruction exception is
    // generated immediately ... a format $3 ... stack frame is saved".
    let board = fpu_board(&[0xf210, 0x6c00, 0x4e71]); // FMOVE.P FP0,(A0){#0}
    board.with_regs(|r| {
        r.a[0] = 0x1000;
        r.fp[0] = FP_THREE;
    });
    board.handler(0, vector::FP_UNSUPPORTED_TYPE, 0x0c00);
    board.load(0x0c00, &[0xf311, 0x4e71]); // FSAVE (A1)
    board.with_regs(|r| r.a[1] = 0x1400);
    board.cpu.step();
    assert_eq!(
        board.cpu.last_exception(),
        Some(vector::FP_UNSUPPORTED_TYPE)
    );
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x30dc, "format $3, offset $0DC");
    assert_eq!(board.peek_long(sp + 2), 0x504, "the next instruction");
    assert_eq!(
        board.peek_long(sp + 8),
        0x1000,
        "+$08 the effective address"
    );
    // And the state frame says so: T is set for an opclass 3 exception
    // (§9.7, **T**; Table 9-16).
    board.cpu.step();
    assert_eq!(
        board.peek_long(0x1418),
        (1 << 26) | (1 << 20),
        "E1 and T set"
    );
}

#[test]
fn the_forced_precision_forms_round_where_they_say() {
    // M68040UM §9.4.2: "FSADD and FDADD specify single- and double-precision
    // rounding regardless of the precision specified in the FPCR PREC bits".
    // 1.0 + 2^-40 is exact in extended and in double, and rounds back to 1.0
    // in single, which has only 24 significand bits.
    let epsilon = F80::new(0x3fff - 40, 1 << 63);
    for (opmode, expected, what) in [
        (0x22u16, F80::new(0x3fff, (1u64 << 63) | (1 << 23)), "FADD"),
        (0x62, FP_ONE, "FSADD"),
        (0x66, F80::new(0x3fff, (1u64 << 63) | (1 << 23)), "FDADD"),
    ] {
        let board = fpu_board(&[0xf200, 0x0400 | opmode, 0x4e71]);
        board.with_regs(|r| {
            r.fp[0] = FP_ONE;
            r.fp[1] = epsilon;
        });
        board.cpu.step();
        assert_eq!(board.cpu.last_exception(), None, "{what}");
        assert_eq!(board.cpu.regs().fp[0], expected, "{what}");
    }
}

#[test]
fn the_sixteen_forced_precision_opmodes_are_the_only_ones_with_bit_6_set() {
    // M68000PRM §5 gives an `FSxxx`/`FDxxx` form to exactly eight
    // operations. Every other opmode with bit 6 set encodes nothing, and
    // "the processor takes an F-line illegal exception" (M68040UM §9.6.1) —
    // which here means the command word decodes to nothing at all.
    let valid: [u16; 16] = [
        0x40, 0x44, 0x41, 0x45, 0x58, 0x5c, 0x5a, 0x5e, 0x60, 0x64, 0x62, 0x66, 0x63, 0x67, 0x68,
        0x6c,
    ];
    for opmode in 0x40u16..=0x7f {
        // Opclass 000, source FP0, destination FP0: only the opmode varies.
        let decoded = super::isa::fp::decode(opmode).is_some();
        assert_eq!(decoded, valid.contains(&opmode), "opmode ${opmode:02x}");
    }
    // And the pairs map to the right operations — the mask that fits the
    // other fourteen would have put FSSQRT on FINT.
    for (opmode, mnemonic) in [
        (0x40u16, "FSMOVE"),
        (0x44, "FDMOVE"),
        (0x41, "FSSQRT"),
        (0x45, "FDSQRT"),
        (0x58, "FSABS"),
        (0x5c, "FDABS"),
        (0x5a, "FSNEG"),
        (0x5e, "FDNEG"),
        (0x60, "FSDIV"),
        (0x64, "FDDIV"),
        (0x62, "FSADD"),
        (0x66, "FDADD"),
        (0x63, "FSMUL"),
        (0x67, "FDMUL"),
        (0x68, "FSSUB"),
        (0x6c, "FDSUB"),
    ] {
        assert_eq!(super::isa::fp::mnemonic(opmode), mnemonic, "${opmode:02x}");
    }
}

#[test]
fn the_conditionals_and_the_moves_are_all_hardware() {
    // Table A-1 gives FBcc, FDBcc, FScc, FTRAPcc, FNOP, FMOVE, FMOVEM,
    // FSAVE and FRESTORE to the 68040 without a footnote. FMOVEM of two
    // registers through memory is the one that touches the most of them.
    let board = fpu_board(&[
        0xf210, 0xf0c0, // FMOVEM.X FP0-FP1,(A0)
        0xf210, 0xd030, // FMOVEM.X (A0),FP2-FP3
        0x4e71,
    ]);
    board.with_regs(|r| {
        r.a[0] = 0x1100;
        r.fp[0] = FP_ONE;
        r.fp[1] = FP_THREE;
    });
    board.cpu.step();
    board.cpu.step();
    let r = board.cpu.regs();
    assert_eq!(board.cpu.last_exception(), None);
    assert_eq!((r.fp[2], r.fp[3]), (FP_ONE, FP_THREE));
}

#[test]
fn a_68040_floating_point_snapshot_carries_what_an_fsave_still_owes() -> Result<()> {
    let board = fpu_board(&[0xf200, 0x040e, 0x4e71]); // FSIN: unimplemented
    board.with_regs(|r| r.fp[1] = FP_THREE);
    board.handler(0, vector::LINE_F, 0x0c00);
    board.cpu.step();
    let bytes = snapshot(&board.cpu)?;
    let other = M68k::new(Config::MC68040);
    restore(&other, &bytes)?;
    assert_eq!(other.regs(), board.cpu.regs());
    assert_eq!(snapshot(&other)?, bytes, "a round trip is a fixed point");
    Ok(())
}

#[test]
fn an_lc040_takes_the_exception_for_every_floating_point_instruction() {
    // M68040UM Appendix A: "the MC68LC040 does not contain an FPU, causing
    // unimplemented floating-point exceptions". With no unit at all the
    // F-line encodings are not instructions, so the frame is format $0.
    let board = Board::new(Model::M68LC040);
    board.boot(&[0xf200, 0x0422, 0x4e71]); // FADD.X FP1,FP0
    board.handler(0, vector::LINE_F, 0x0c00);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::LINE_F));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x002c, "format $0");
}

#[test]
fn a_deferred_prefetch_fault_says_whether_the_page_was_missing() {
    // M68040UM §8.2.1 defers a prefetch fault "until the processor attempts
    // to use the information", and §8.4.6.2's `ATC` bit still has to say
    // which kind of fault it was: set for "a nonresident entry ... or
    // privilege violation", clear for "a bus-errored instruction ... access".
    // The two cases below differ only in why the fetch failed.
    //
    // A page that is not resident: the search installs an entry with R
    // clear, the fetch of the word at $11000 fails, and the fault arrives
    // when the NOP at $10FFE has run and the next word is wanted.
    let board = mapped(0x0001_0000, 0x0001_0000 | 0b01);
    board.poke_long(u64::from(PAGE) + 0x11 * 4, 0); // $11000 is invalid
    board.handler(0, vector::BUS_ERROR, 0x0800);
    translate_on(&board, &[0x4ef9, 0x0001, 0x0ffe]); // JMP $10FFE
    board.guarded.write_u8(0xffe, 0x4e).unwrap();
    board.guarded.write_u8(0xfff, 0x71).unwrap(); // NOP
    board.cpu.step(); // the JMP
    board.cpu.step(); // the NOP, whose refill poisons
    board.cpu.step(); // and the fault arrives
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(board.peek_word(sp + 6), 0x7008, "format $7");
    assert_eq!(
        board.peek_word(sp + 0x0c) & 0x0400,
        0x0400,
        "ATC: the page was not resident"
    );
    assert_eq!(board.peek_long(sp + 0x14), 0x1_1000, "the fetch address");

    // A page that *is* resident, mapped to physical memory nothing answers
    // for: the same deferred fault, but a physical bus error.
    let board = mapped(0x0001_0000, 0x0001_0000 | 0b01);
    board.poke_long(u64::from(PAGE) + 0x11 * 4, 0x0100_0000 | 0b01);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    translate_on(&board, &[0x4ef9, 0x0001, 0x0ffe]);
    board.guarded.write_u8(0xffe, 0x4e).unwrap();
    board.guarded.write_u8(0xfff, 0x71).unwrap();
    board.cpu.step();
    board.cpu.step();
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(
        board.peek_word(sp + 0x0c) & 0x0400,
        0,
        "no ATC: the translation worked and the bus refused"
    );
}

#[test]
fn the_hardware_subset_agrees_with_the_68881_operation_by_operation() {
    // The 68040's ten hardware operations are the same arithmetic the 68881
    // does, computed by the same `src/float` code — so the strongest cheap
    // evidence that the 68040 path is right is that it gives the 68881's
    // answers, which `tests_fpu.rs` checks against M68881UM §4's operation
    // tables and against IEEE 754.
    let cases: [(u16, F80, F80, &str); 10] = [
        (0x22, FP_ONE, FP_THREE, "FADD"),
        (0x28, FP_ONE, FP_THREE, "FSUB"),
        (0x23, FP_THREE, FP_THREE, "FMUL"),
        (0x20, FP_THREE, FP_ONE, "FDIV"),
        (
            0x18,
            F80::new(0xc000, 0xc000_0000_0000_0000),
            FP_ONE,
            "FABS",
        ),
        (0x1a, FP_THREE, FP_ONE, "FNEG"),
        (0x04, F80::new(0x4001, 1 << 63), FP_ONE, "FSQRT"),
        (0x00, FP_THREE, FP_ONE, "FMOVE"),
        (0x38, FP_ONE, FP_THREE, "FCMP"),
        (0x3a, FP_THREE, FP_ONE, "FTST"),
    ];
    for (opmode, src, dst, what) in cases {
        let words = [0xf200u16, 0x0400 | opmode, 0x4e71];
        let forty = fpu_board(&words);
        forty.with_regs(|r| {
            r.fp[0] = dst;
            r.fp[1] = src;
        });
        forty.cpu.step();
        assert_eq!(forty.cpu.last_exception(), None, "{what} is hardware");

        let eighty_one = Board::with_fpu(Model::M68030, super::Coprocessor::M68881);
        eighty_one.boot(&[0x4e71]);
        eighty_one.load(0x500, &words);
        eighty_one.at(0x500);
        eighty_one.with_regs(|r| {
            r.fp[0] = dst;
            r.fp[1] = src;
        });
        eighty_one.cpu.step();

        let a = forty.cpu.regs();
        let b = eighty_one.cpu.regs();
        assert_eq!(a.fp[0], b.fp[0], "{what}: the result");
        assert_eq!(a.fpsr, b.fpsr, "{what}: FPSR");
    }
}
