//! The corpus, beside the harness that runs it.
//!
//! Every case here is one rule the lifter could plausibly get wrong, written
//! as the encoding that would catch it. The two sweeps at the bottom are what
//! makes the coverage a measurement rather than a list: [`opcode_sweep`] runs
//! **every sixteen-bit opcode word there is**, and [`sweep`] runs a seeded
//! pseudo-random stream through the whole encoder.

use super::*;
use alloc::vec;

/// A case that must agree, reported with the divergence if it does not.
#[track_caller]
fn agreed(case: &Case) -> Verdict {
    match compare(case) {
        Ok(v) => v,
        Err(d) => panic!("{d}"),
    }
}

/// `STOP #$2700`, which is what every program here ends with: a case that
/// falls off its own end halts rather than running into the data window, and
/// "stopped" is a column the harness compares.
const STOP: [u16; 2] = [0x4e72, 0x2700];

/// One instruction, then `STOP`.
fn one(words: &[u16]) -> Vec<u16> {
    let mut v = words.to_vec();
    v.extend_from_slice(&STOP);
    v
}

// ---------------------------------------------------------------------------
// Moves
// ---------------------------------------------------------------------------

#[test]
fn a_move_between_registers_agrees() {
    // MOVE.L D0,D1
    let v = agreed(&Case::seeded(one(&[0x2200])).with_units(2));
    assert!(matches!(v, Verdict::Agreed { .. }), "{v:?}");
}

#[test]
fn the_memory_to_memory_move_agrees_on_both_increments() {
    // MOVE.W (A2)+,(A3)+ — both operands in memory, two side-effecting
    // address registers, and the order of the increments and the accesses both
    // observable.
    agreed(&Case::seeded(one(&[0x36da])).with_units(2));
    // MOVE.B (A2)+,(A3)+ and MOVE.L (A2)+,(A3)+ — the latter falls back,
    // because a long memory destination is two stores.
    agreed(&Case::seeded(one(&[0x16da])).with_units(2));
    agreed(&Case::seeded(one(&[0x26da])).with_units(2));
}

#[test]
fn a_move_to_predecrement_agrees_on_the_order_of_its_writes() {
    // MOVE.W D0,-(A2) and MOVE.B D0,-(A2).
    agreed(&Case::seeded(one(&[0x3500])).with_units(2));
    agreed(&Case::seeded(one(&[0x1500])).with_units(2));
    // MOVE.W D0,-(A7): a byte through A7 steps by two, a word by two as well.
    agreed(&Case::seeded(one(&[0x3f00])).with_units(2));
    agreed(&Case::seeded(one(&[0x1f00])).with_units(2));
}

#[test]
fn moveq_sign_extends_its_byte_into_the_whole_register() {
    // MOVEQ #-1,D3 and MOVEQ #$7f,D3.
    agreed(&Case::seeded(one(&[0x76ff])).with_units(2));
    agreed(&Case::seeded(one(&[0x767f])).with_units(2));
}

#[test]
fn movea_word_sign_extends_and_movea_long_does_not() {
    // MOVEA.W D1,A5 with D1 = 0xffffffff, then MOVEA.L D1,A5.
    agreed(&Case::seeded(one(&[0x3a41])).with_units(2));
    agreed(&Case::seeded(one(&[0x2a41])).with_units(2));
}

#[test]
fn a_move_through_absolute_long_agrees_including_its_deferred_fetch() {
    // MOVE.W (A2),($00002400).L — the one `MOVE` whose last instruction fetch
    // happens *after* its operand write, and only when the source came out of
    // memory (`exec::resolve_ea`).
    agreed(&Case::seeded(one(&[0x33d2, 0x0000, 0x2400])).with_units(2));
    // With a register source the fetch comes first, which is the other half of
    // the same rule.
    agreed(&Case::seeded(one(&[0x33c0, 0x0000, 0x2400])).with_units(2));
}

// ---------------------------------------------------------------------------
// The condition codes, and the X flag in particular
// ---------------------------------------------------------------------------

#[test]
fn add_and_sub_write_every_flag_including_extend() {
    for ccr in 0..32u16 {
        // ADD.W D1,D0 / SUB.W D1,D0 / ADD.B / SUB.L
        agreed(&Case::seeded(one(&[0xd041])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x9041])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0xd001])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x9081])).with_ccr(ccr).with_units(2));
    }
}

#[test]
fn cmp_writes_carry_and_leaves_the_extend_of_an_addx_chain_alone() {
    // M68000PRM, *CMP*: "X — not affected". The whole reason X is a separate
    // bit, and the whole reason this harness compares `SR` rather than the
    // four condition codes.
    for ccr in 0..32u16 {
        // CMP.W D1,D0, CMP.L D1,D0, CMPI.W #$8000,D0, CMPA.W D1,A3
        agreed(&Case::seeded(one(&[0xb041])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0xb081])).with_ccr(ccr).with_units(2));
        agreed(
            &Case::seeded(one(&[0x0c40, 0x8000]))
                .with_ccr(ccr)
                .with_units(2),
        );
        agreed(&Case::seeded(one(&[0xb6c1])).with_ccr(ccr).with_units(2));
    }
}

#[test]
fn addx_and_subx_carry_a_sticky_zero() {
    // "Z is only ever *cleared* by an extended operation ... a zero result
    // leaves Z exactly as the previous step left it (M68000PRM, ADDX)" — the
    // loop-carried dependency `src/ir/mod.rs`'s decision 1 names.
    for ccr in 0..32u16 {
        // ADDX.W D1,D0, SUBX.W D1,D0, ADDX.L, SUBX.B
        agreed(&Case::seeded(one(&[0xd141])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x9141])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0xd181])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x9101])).with_ccr(ccr).with_units(2));
    }
    // A real two-step chain: ADDX.L D1,D0 twice, so the second step reads what
    // the first wrote.
    let mut program = vec![0xd181, 0xd181];
    program.extend_from_slice(&STOP);
    agreed(&Case::seeded(program).with_extend().with_units(3));
}

#[test]
fn the_memory_form_of_addx_walks_two_predecrements() {
    // ADDX.W -(A2),-(A3) and SUBX.B -(A2),-(A3): the destination is fetched as
    // part of the walk that computes its address, so re-reading it would put a
    // bus cycle on the wire that hardware does not.
    for ccr in 0..32u16 {
        agreed(&Case::seeded(one(&[0xd74a])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x970a])).with_ccr(ccr).with_units(2));
    }
    // The long form is two stores and falls back; it still has to agree.
    agreed(&Case::seeded(one(&[0xd78a])).with_extend().with_units(2));
}

#[test]
fn negx_carries_the_same_sticky_zero() {
    for ccr in 0..32u16 {
        // NEGX.W D0, NEGX.L D0, NEGX.B (A2)
        agreed(&Case::seeded(one(&[0x4040])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x4080])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x4012])).with_ccr(ccr).with_units(2));
    }
}

#[test]
fn clr_still_reads_its_destination_on_a_68000() {
    // MC68000UM Table 8-6, "and the reason CLR is not usable on a
    // read-sensitive register". The read is a real bus cycle, so it is four
    // cycles the lifted block has to spend too.
    agreed(&Case::seeded(one(&[0x4252])).with_units(2));
    agreed(&Case::seeded(one(&[0x4292])).with_units(2));
    agreed(&Case::seeded(one(&[0x4212])).with_units(2));
}

#[test]
fn the_logical_group_clears_v_and_c_and_leaves_x() {
    for ccr in 0..32u16 {
        // AND.W D1,D0, OR.L D1,D0, EOR.B D1,D0, NOT.W D0, TST.L D0
        agreed(&Case::seeded(one(&[0xc041])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x8081])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0xb301])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x4640])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x4a80])).with_ccr(ccr).with_units(2));
    }
}

#[test]
fn adda_and_suba_touch_no_flags_at_all() {
    for ccr in 0..32u16 {
        // ADDA.W D1,A5, SUBA.L D1,A5, ADDQ.W #3,A5, SUBQ.L #1,A5
        agreed(&Case::seeded(one(&[0xdac1])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x9bc1])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x564d])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0x538d])).with_ccr(ccr).with_units(2));
    }
}

// ---------------------------------------------------------------------------
// Shifts and rotates
// ---------------------------------------------------------------------------

#[test]
fn every_static_shift_count_agrees_in_every_size() {
    // ASL/ASR/LSL/LSR/ROL/ROR/ROXL/ROXR Dn, counts 1..8, all three sizes.
    for kind in 0..8u16 {
        let right = kind & 1;
        let ty = kind >> 1;
        for count in 0..8u16 {
            for size in 0..3u16 {
                let op = 0xe000 | (count << 9) | (size << 6) | ((1 - right) << 8) | (ty << 3);
                for ccr in [0u16, 0x1f, 0x10, 0x04] {
                    agreed(&Case::seeded(one(&[op])).with_ccr(ccr).with_units(2));
                }
            }
        }
    }
}

#[test]
fn asl_sets_overflow_if_the_sign_changed_at_any_point() {
    // M68000PRM, *ASL*: "V — set if the most significant bit is changed at any
    // time during the shift operation; cleared otherwise." A lifter that
    // tested only the final sign passes every other case and fails this one:
    // 0x00008000 shifted left by two as a long loses the bit and comes back
    // positive, having been negative in between.
    let case = Case::seeded(one(&[0xe580])).with_d(0, 0x0000_8000);
    agreed(&case.with_units(2));
    // and the word form of the same shape, where the sign bit is bit 15.
    let case = Case::seeded(one(&[0xe540])).with_d(0, 0x0000_4000);
    agreed(&case.with_units(2));
}

#[test]
fn the_rotates_through_extend_agree_on_every_starting_x() {
    for ccr in 0..32u16 {
        // ROXL.W #1,D0 / ROXR.L #8,D0 / ROXL.B #3,D0
        agreed(&Case::seeded(one(&[0xe350])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0xe090])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0xe710])).with_ccr(ccr).with_units(2));
    }
}

#[test]
fn a_plain_rotate_does_not_touch_the_extend_bit() {
    // "Getting this wrong quietly breaks every multi-precision routine that
    // rotates a mask between `ADDX` steps" (`exec::shift`).
    for ccr in 0..32u16 {
        agreed(&Case::seeded(one(&[0xe358])).with_ccr(ccr).with_units(2)); // ROL.W #1,D0
        agreed(&Case::seeded(one(&[0xe258])).with_ccr(ccr).with_units(2)); // ROR.W #1,D0
    }
}

#[test]
fn the_memory_shift_form_shifts_one_bit_of_one_word() {
    // ASL (A2), LSR (A2), ROXL (A2) — one bit, one word, whatever the operand.
    agreed(&Case::seeded(one(&[0xe1d2])).with_units(2));
    agreed(&Case::seeded(one(&[0xe2d2])).with_units(2));
    agreed(&Case::seeded(one(&[0xe5d2])).with_extend().with_units(2));
}

#[test]
fn a_shift_by_a_register_count_falls_back_and_still_agrees() {
    // Declined by `classify`: the count is a run-time value the flag rules
    // depend on in six ways and the cycle count depends on too.
    assert!(!lifts(&[0xe1a0]), "ASL.L D0,D0 must not be lifted");
    for ccr in 0..32u16 {
        agreed(&Case::seeded(one(&[0xe1a0])).with_ccr(ccr).with_units(2));
        agreed(&Case::seeded(one(&[0xe3b8])).with_ccr(ccr).with_units(2));
    }
}

// ---------------------------------------------------------------------------
// Bit instructions
// ---------------------------------------------------------------------------

#[test]
fn the_bit_instructions_write_only_z() {
    for ccr in 0..32u16 {
        // BTST #3,D0 / BSET #17,D0 / BCLR #1,D0 / BCHG #31,D0
        agreed(
            &Case::seeded(one(&[0x0800, 0x0003]))
                .with_ccr(ccr)
                .with_units(2),
        );
        agreed(
            &Case::seeded(one(&[0x08c0, 0x0011]))
                .with_ccr(ccr)
                .with_units(2),
        );
        agreed(
            &Case::seeded(one(&[0x0880, 0x0001]))
                .with_ccr(ccr)
                .with_units(2),
        );
        agreed(
            &Case::seeded(one(&[0x0840, 0x001f]))
                .with_ccr(ccr)
                .with_units(2),
        );
    }
    // and the memory forms, whose bit number is modulo 8.
    agreed(&Case::seeded(one(&[0x0812, 0x000b])).with_units(2));
    agreed(&Case::seeded(one(&[0x08d2, 0x000b])).with_units(2));
}

#[test]
fn a_dynamic_bit_number_into_a_register_falls_back() {
    // The write costs two cycles more when the reduced bit number is at least
    // sixteen, which is a run-time value: declined rather than guessed.
    assert!(!lifts(&[0x01c0]), "BSET D0,D0 must not be lifted");
    for value in [0u32, 15, 16, 31, 47] {
        agreed(&Case::seeded(one(&[0x01c1])).with_d(0, value).with_units(2));
        agreed(&Case::seeded(one(&[0x0181])).with_d(0, value).with_units(2));
    }
    // A dynamic bit number into *memory* is lifted, because a byte operand's
    // reduction costs the same whatever the number.
    assert!(lifts(&[0x01d2]), "BSET D0,(A2) is in the subset");
    for value in [0u32, 7, 8, 200] {
        agreed(&Case::seeded(one(&[0x01d2])).with_d(0, value).with_units(2));
    }
}

// ---------------------------------------------------------------------------
// The twelve addressing modes
// ---------------------------------------------------------------------------

#[test]
fn every_addressing_mode_agrees_as_a_source() {
    // ADD.W <ea>,D0 over every legal mode, with the indexed and PC-relative
    // extension words the mode needs.
    let cases: &[&[u16]] = &[
        &[0xd041],                 // Dn
        &[0xd049],                 // An
        &[0xd052],                 // (A2)
        &[0xd05a],                 // (A2)+
        &[0xd062],                 // -(A2)
        &[0xd06a, 0x0010],         // (16,A2)
        &[0xd072, 0x0810],         // (16,A2,D0.W)
        &[0xd072, 0x8810],         // (16,A2,A0.W) — the address file
        &[0xd072, 0x0010],         // (16,A2,D0.W) with a zero displacement
        &[0xd078, 0x2400],         // ($2400).W
        &[0xd079, 0x0000, 0x2400], // ($00002400).L
        &[0xd07a, 0x1000],         // (d16,PC)
        &[0xd07b, 0x0810],         // (d8,PC,D0.W)
        &[0xd07c, 0x1234],         // #$1234
    ];
    for words in cases {
        agreed(&Case::seeded(one(words)).with_units(2));
    }
}

#[test]
fn every_writable_mode_agrees_as_a_destination() {
    // CLR.W <ea>, which reads and writes every alterable memory mode.
    let dsts: &[&[u16]] = &[
        &[0x4240],                 // CLR.W D0
        &[0x4252],                 // CLR.W (A2)
        &[0x425a],                 // CLR.W (A2)+
        &[0x4262],                 // CLR.W -(A2)
        &[0x426a, 0x0010],         // CLR.W (16,A2)
        &[0x4272, 0x0810],         // CLR.W (16,A2,D0.W)
        &[0x4278, 0x2400],         // CLR.W ($2400).W
        &[0x4279, 0x0000, 0x2400], // CLR.W ($00002400).L
    ];
    for words in dsts {
        agreed(&Case::seeded(one(words)).with_units(2));
    }
}

#[test]
fn an_odd_word_address_is_an_address_error_in_both_engines() {
    // `A1` is odd in a seeded case on purpose. A word or long access through
    // it is an address error: the access is never made, it costs nothing, and
    // the block hands the instruction back so the interpreter builds the
    // fourteen-byte group-0 frame.
    agreed(&Case::seeded(one(&[0xd051])).with_units(3)); // ADD.W (A1),D0
    agreed(&Case::seeded(one(&[0xd091])).with_units(3)); // ADD.L (A1),D0
    agreed(&Case::seeded(one(&[0x3281])).with_units(3)); // MOVE.W D1,(A1)
    // A *byte* access through the same register has no alignment rule at all.
    agreed(&Case::seeded(one(&[0xd011])).with_units(3)); // ADD.B (A1),D0
}

#[test]
fn an_access_the_space_refuses_faults_in_both_engines() {
    // A5 is zero in a seeded case and RAM starts at zero, so point it past the
    // end: the space has nothing mapped there and refuses.
    let case = Case::seeded(one(&[0xd055])).with_a(5, 0x0010_0000);
    agreed(&case.with_units(3));
    let case = Case::seeded(one(&[0x3285]))
        .with_a(1, DATA)
        .with_a(5, 0x0010_0000);
    agreed(&case.with_units(3));
}

// ---------------------------------------------------------------------------
// Control flow
// ---------------------------------------------------------------------------

#[test]
fn every_condition_of_a_taken_and_a_not_taken_branch_agrees() {
    for cc in 2..16u16 {
        for ccr in 0..32u16 {
            // Bcc.B +2 (forward over the STOP), and Bcc.W +4.
            let mut program = vec![0x6000 | (cc << 8) | 0x04];
            program.extend_from_slice(&STOP);
            program.extend_from_slice(&STOP);
            agreed(&Case::seeded(program).with_ccr(ccr).with_units(3));

            let mut program = vec![0x6000 | (cc << 8), 0x0006];
            program.extend_from_slice(&STOP);
            program.extend_from_slice(&STOP);
            agreed(&Case::seeded(program).with_ccr(ccr).with_units(3));
        }
    }
}

#[test]
fn bra_agrees_in_both_displacement_forms() {
    let mut program = vec![0x6004];
    program.extend_from_slice(&STOP);
    program.extend_from_slice(&STOP);
    agreed(&Case::seeded(program).with_units(3));
    let mut program = vec![0x6000, 0x0006];
    program.extend_from_slice(&STOP);
    program.extend_from_slice(&STOP);
    agreed(&Case::seeded(program).with_units(3));
}

#[test]
fn a_backward_branch_runs_a_loop_to_its_end() {
    // SUBQ.W #1,D0 ; BNE.B -4 ; STOP — three instructions and a loop whose
    // back edge leaves the block every time round, which is what exercises
    // the cache.
    let program = vec![0x5340, 0x66fc, STOP[0], STOP[1]];
    let case = Case::seeded(program).with_d(0, 6).with_units(40);
    let v = agreed(&case);
    let Verdict::Agreed { lifted, steps, .. } = v;
    assert!(lifted > 0, "the loop body must have been lifted: {v:?}");
    assert!(steps > 6, "the loop must have gone round: {v:?}");
}

#[test]
fn dbcc_agrees_on_all_three_of_its_paths() {
    for cc in 0..16u16 {
        for count in [0u32, 1, 2, 0xffff, 0x1_0000] {
            // DBcc Dn,-4 with the counter at each interesting value: the
            // condition true, the counter expired, and the loop taken.
            let program = vec![0x50c8 | (cc << 8), 0xfffc, STOP[0], STOP[1]];
            let case = Case::seeded(program).with_d(0, count).with_units(6);
            agreed(&case);
        }
    }
}

#[test]
fn scc_agrees_for_every_condition_in_both_destinations() {
    for cc in 0..16u16 {
        for ccr in 0..32u16 {
            agreed(
                &Case::seeded(one(&[0x50c0 | (cc << 8)]))
                    .with_ccr(ccr)
                    .with_units(2),
            );
            agreed(
                &Case::seeded(one(&[0x50d2 | (cc << 8)]))
                    .with_ccr(ccr)
                    .with_units(2),
            );
        }
    }
}

#[test]
fn jmp_agrees_in_every_control_mode() {
    let cases: &[&[u16]] = &[
        &[0x4ed2],                 // JMP (A2)
        &[0x4eea, 0x0004],         // JMP (4,A2)
        &[0x4ef2, 0x0804],         // JMP (4,A2,D0.W)
        &[0x4ef8, 0x1004],         // JMP ($1004).W
        &[0x4ef9, 0x0000, 0x1006], // JMP ($00001006).L
        &[0x4efa, 0x0004],         // JMP (4,PC)
        &[0x4efb, 0x0804],         // JMP (4,PC,D0.W)
    ];
    for words in cases {
        let mut program = words.to_vec();
        while program.len() < 6 {
            program.push(0x4e71); // NOP
        }
        program.extend_from_slice(&STOP);
        // A2 points into the data window in a seeded case, so a JMP through it
        // lands on data — which is exactly the interesting case: both engines
        // must execute whatever is there, or fault the same way.
        agreed(&Case::seeded(program).with_a(2, CODE + 8).with_units(3));
    }
}

#[test]
fn a_jump_to_an_odd_address_is_an_address_error_in_both_engines() {
    // The fault a charge could not have raised: the refill's first fetch is a
    // real bus access with `Align::Fault`, so an odd target faults there —
    // and the target is a run-time value, so nothing at lift time could have
    // declined it.
    let case = Case::seeded(one(&[0x4ed2])).with_a(2, CODE + 1);
    agreed(&case.with_units(3));
    // RTS to an odd return address is the same fault through a different
    // instruction.
    let case = Case::seeded(one(&[0x4e75])).with_a(7, DATA + 0x200);
    agreed(&case.with_units(3));
}

#[test]
fn rts_agrees_with_a_return_address_on_the_stack() {
    // Push a return address with MOVE.L #target,-(A7) — which falls back,
    // being two stores — then RTS to it.
    let target = CODE + 12;
    let program = vec![
        0x2f3c,
        (target >> 16) as u16,
        target as u16,
        0x4e75,
        0x4e71,
        0x4e71,
        STOP[0],
        STOP[1],
    ];
    agreed(&Case::seeded(program).with_units(5));
}

#[test]
fn jsr_and_bsr_fall_back_and_still_agree() {
    // Declined because a long push is two stores on a 16-bit bus, so a fault
    // after the first could not be restarted.
    assert!(!lifts(&[0x4e92]), "JSR (A2) must not be lifted");
    assert!(!lifts(&[0x6104]), "BSR must not be lifted");
    let program = vec![0x6104, STOP[0], STOP[1], 0x4e75, STOP[0], STOP[1]];
    agreed(&Case::seeded(program).with_units(4));
    let program = vec![0x4eba, 0x0004, STOP[0], STOP[1], 0x4e75];
    agreed(&Case::seeded(program).with_units(4));
}

// ---------------------------------------------------------------------------
// MOVEM
// ---------------------------------------------------------------------------

#[test]
fn movem_into_registers_agrees_in_both_sizes_and_both_modes() {
    for mask in [0x0001u16, 0x8000, 0xffff, 0x00ff, 0xff00, 0x1234, 0x0000] {
        // MOVEM.W (A2),<list> and MOVEM.L (A2)+,<list>
        agreed(&Case::seeded(one(&[0x4c92, mask])).with_units(2));
        agreed(&Case::seeded(one(&[0x4cda, mask])).with_units(2));
        agreed(&Case::seeded(one(&[0x4c9a, mask])).with_units(2));
        agreed(&Case::seeded(one(&[0x4cd2, mask])).with_units(2));
        // and through (d16,An), which is not a walking form.
        agreed(&Case::seeded(one(&[0x4caa, mask, 0x0010])).with_units(2));
    }
}

#[test]
fn movem_reads_one_word_past_the_end_and_it_can_fault() {
    // "One word past the end, read and discarded. It is a real bus cycle, and
    // a MOVEM that ends at the top of a mapped region can fault on it." Put
    // the base so the extra read lands past the end of RAM.
    let case = Case::seeded(one(&[0x4c92, 0x0001])).with_a(2, RAM_SIZE - 2);
    agreed(&case.with_units(3));
}

#[test]
fn movem_to_memory_falls_back_and_still_agrees() {
    // One store per register: declined for restartability.
    assert!(
        !lifts(&[0x4892, 0x0001]),
        "MOVEM to memory must not be lifted"
    );
    for mask in [0x0001u16, 0xffff, 0x00ff] {
        agreed(&Case::seeded(one(&[0x4892, mask])).with_units(2));
        agreed(&Case::seeded(one(&[0x48e2, mask])).with_units(2));
    }
}

// ---------------------------------------------------------------------------
// The rest of the lifted subset
// ---------------------------------------------------------------------------

#[test]
fn the_small_register_instructions_agree() {
    let cases: &[&[u16]] = &[
        &[0x4880],         // EXT.W D0
        &[0x48c0],         // EXT.L D0
        &[0x4840],         // SWAP D0
        &[0xc141],         // EXG D0,D1
        &[0xc149],         // EXG A0,A1
        &[0xc189],         // EXG D0,A1
        &[0x4e71],         // NOP
        &[0x41d2],         // LEA (A2),A0
        &[0x41ea, 0x0010], // LEA (16,A2),A0
        &[0x41f2, 0x0810], // LEA (16,A2,D0.W),A0
        &[0x41f8, 0x2400], // LEA ($2400).W,A0
        &[0x41fa, 0x0010], // LEA (16,PC),A0
        &[0x40c0],         // MOVE SR,D0
        &[0x40d2],         // MOVE SR,(A2)
        &[0x44c0],         // MOVE D0,CCR
        &[0x44fc, 0x001f], // MOVE #$1f,CCR
        &[0x003c, 0x0005], // ORI #5,CCR
        &[0x023c, 0x0010], // ANDI #$10,CCR
        &[0x0a3c, 0x000f], // EORI #$f,CCR
    ];
    for words in cases {
        for ccr in [0u16, 0x1f, 0x10] {
            agreed(&Case::seeded(one(words)).with_ccr(ccr).with_units(2));
        }
    }
}

#[test]
fn link_and_unlk_round_trip_with_link_on_the_fallback() {
    // LINK pushes a long and is declined; UNLK only reads one and is lifted.
    assert!(!lifts(&[0x4e52, 0xfff0]), "LINK must not be lifted");
    assert!(lifts(&[0x4e5a]), "UNLK is in the subset");
    let program = vec![0x4e52, 0xfff0, 0x4e5a, STOP[0], STOP[1]];
    agreed(&Case::seeded(program).with_units(4));
}

// ---------------------------------------------------------------------------
// The fallback, and the shape of a run
// ---------------------------------------------------------------------------

#[test]
fn the_fallback_is_really_exercised() {
    // MULU.W D1,D0 — data-dependent cycles, outside the subset — and the
    // statistics say the interpreter took it rather than a block.
    assert!(!lifts(&[0xc0c1]), "MULU must not be lifted");
    let stats = stats_for(&Case::seeded(one(&[0xc0c1])).with_units(1))
        .expect("the subject runs the translated engine");
    // Two: the reset sequence a case boots through, and the MULU. Nothing
    // executed in a block, which is the claim.
    assert_eq!(stats.interpreted, 2, "{stats:?}");
    assert_eq!(stats.executed, 0, "{stats:?}");
    assert_eq!(stats.retired, 0, "{stats:?}");
    agreed(&Case::seeded(one(&[0xc0c1])).with_units(2));
}

#[test]
fn a_block_is_really_lifted_and_really_reused() {
    // Three lifted instructions in a row, run twice round a loop, so the
    // second pass is a cache hit rather than a lift.
    let program = vec![0x5340, 0xd041, 0x66fa, STOP[0], STOP[1]];
    let stats = stats_for(&Case::seeded(program.clone()).with_d(0, 3).with_units(12))
        .expect("the subject runs the translated engine");
    assert!(
        stats.retired >= 6,
        "instructions must retire in blocks: {stats:?}"
    );
    assert!(
        stats.executed > stats.lifted,
        "a block must be reused rather than lifted every time: {stats:?}"
    );
    agreed(&Case::seeded(program).with_d(0, 3).with_units(12));
}

#[test]
fn an_illegal_instruction_agrees_through_its_exception() {
    // The ILLEGAL encoding, an unassigned word, an A-line word and an F-line
    // word: four different vectors, none of them lifted, all of them
    // exercising the fallback *and* the exception the interpreter takes.
    for word in [0x4afc_u16, 0x4ac0, 0xa000, 0xf000] {
        agreed(&Case::seeded(one(&[word])).with_units(3));
    }
}

#[test]
fn a_privileged_instruction_in_user_state_agrees() {
    // MOVE to SR in user state is a privilege violation. Not lifted, because
    // lifting it would need the block keyed on **S**.
    let case = Case::seeded(one(&[0x46c0]));
    let mut user = case.clone();
    user.sr &= !super::super::flags::S;
    agreed(&user.with_units(3));
}

#[test]
fn a_trap_agrees_through_its_vector() {
    for n in 0..16u16 {
        agreed(&Case::seeded(one(&[0x4e40 | n])).with_units(3));
    }
    // TRAPV with V set and with V clear.
    agreed(&Case::seeded(one(&[0x4e76])).with_ccr(0).with_units(3));
    agreed(&Case::seeded(one(&[0x4e76])).with_ccr(0x02).with_units(3));
}

#[test]
fn a_divide_by_zero_agrees_through_its_vector() {
    let case = Case::seeded(one(&[0x80c1])).with_d(1, 0); // DIVU.W D1,D0
    agreed(&case.with_units(3));
}

#[test]
fn a_store_into_the_running_blocks_own_window_ends_the_block() {
    // MOVE.W D0,(A2) with A2 pointing at the instruction after it, then two
    // NOPs the store overwrites. The block must leave at the boundary after
    // the store, or it executes bytes the guest has already replaced.
    let program = vec![0x3480, 0x4e71, 0x4e71, STOP[0], STOP[1]];
    let case = Case::seeded(program)
        .with_a(2, CODE + 2)
        .with_d(0, 0x4e71)
        .with_units(6);
    agreed(&case);
    // and with a store that really changes the next instruction: write a
    // `STOP` opcode over it.
    let program = vec![0x3480, 0x4e71, 0x2700, 0x4e71, STOP[0], STOP[1]];
    let case = Case::seeded(program)
        .with_a(2, CODE + 2)
        .with_d(0, 0x4e72)
        .with_units(6);
    agreed(&case);
}

#[test]
fn an_engine_switch_is_refused_for_a_model_the_frontend_does_not_lift() {
    use crate::core::props::Props;
    let props = Props::new()
        .with("engine", crate::core::props::Value::Str("ir".into()))
        .with("model", crate::core::props::Value::Str("68020".into()));
    let err = M68k::from_props(&props).expect_err("a 68020 has no IR frontend");
    let text = alloc::format!("{err}");
    assert!(text.contains("MC68000 only"), "{text}");
}

// ---------------------------------------------------------------------------
// The sweeps: a rate, not a verdict
// ---------------------------------------------------------------------------

#[test]
fn every_opcode_word_agrees() {
    // The whole sixteen-bit decode space, one case each, with extension words
    // that make an encoding that wants them well formed. A few hundred lift
    // and the rest exercise the fallback; nothing is skipped, because the
    // fallback has to leave the guest where a block would have.
    //
    // Strided here and run in full by `tests/m68k_lift_differential.rs`, which
    // is where a long run belongs.
    let (cases, found) = opcode_sweep(7, &[0x0010, 0x0000, 0x2400]);
    assert!(found.is_none(), "{cases} cases: {}", found.unwrap());
    assert!(cases > 9000, "the sweep must really have run: {cases}");
}

#[test]
fn a_seeded_random_stream_agrees() {
    let (cases, found) = sweep(0x6800_0000_0000_0007, 200, 4);
    assert!(found.is_none(), "{cases} cases: {}", found.unwrap());
}
