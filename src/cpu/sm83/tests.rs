//! Hand-written tests for the SM83 core.
//!
//! These carry the behaviours no downloadable corpus covers — the interrupt
//! dispatch, the `HALT` bug, `EI`'s delay, `ie_push` — plus a timing table
//! checked against Pan Docs, opcode by opcode, so that a regression in the
//! derived cycle counts shows up here rather than three suites later.
//!
//! The conformance runners next door (`conformance.rs`) are the *measurement*;
//! this file is the part that has to keep passing offline.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::props::Props;
use crate::core::space::{AddressSpace, MemAttrs, RamStore, Region};
use crate::core::state::{ChunkWriter, MachineShape, StateWriter};
use crate::core::value::Width;

use super::isa::{Class, Op, decode, decode_cb};
use super::{Config, Regs, Sm83, flags, interrupt};

/// A core with a flat 64 KiB of RAM, at `$0100` with the post-boot register
/// file, and `code` assembled there.
fn cpu_with(code: &[u8]) -> (Arc<Sm83>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(0x1_0000));
    for (i, b) in code.iter().enumerate() {
        ram.write_u8(0x0100 + i as u64, *b).expect("in range");
    }
    let space = AddressSpace::new("cpu", 16);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), 0)
        .expect("maps");
    let cpu = Arc::new(Sm83::new(Config::default()));
    cpu.attach_space(Arc::new(space));
    Device::reset(cpu.as_ref(), ResetKind::Cold);
    (cpu, ram)
}

/// Run one instruction and report the machine cycles it cost.
fn step(cpu: &Sm83) -> u64 {
    cpu.step()
}

// ---------------------------------------------------------------------------
// Timing — the table Pan Docs states, checked against what the bus sequence
// actually produces
// ---------------------------------------------------------------------------

/// The documented M-cycle count of every first-page opcode, or `None` for the
/// eleven holes.
///
/// This is the *only* place in the crate where cycle counts are written down,
/// and it is a test rather than the implementation on purpose: the interpreter
/// derives its timing from the bus accesses it makes, and this asserts that the
/// derivation agrees with the published table. A conditional instruction is
/// listed with its **not-taken** count; the taken cases are checked separately.
#[rustfmt::skip]
const CYCLES: [Option<u8>; 256] = {
    let n = None;
    [
    //     0        1        2        3        4        5        6        7
    //     8        9        a        b        c        d        e        f
    /*0*/ Some(1), Some(3), Some(2), Some(2), Some(1), Some(1), Some(2), Some(1),
          Some(5), Some(2), Some(2), Some(2), Some(1), Some(1), Some(2), Some(1),
    /*1*/ Some(2), Some(3), Some(2), Some(2), Some(1), Some(1), Some(2), Some(1),
          Some(3), Some(2), Some(2), Some(2), Some(1), Some(1), Some(2), Some(1),
    /*2*/ Some(2), Some(3), Some(2), Some(2), Some(1), Some(1), Some(2), Some(1),
          Some(2), Some(2), Some(2), Some(2), Some(1), Some(1), Some(2), Some(1),
    /*3*/ Some(2), Some(3), Some(2), Some(2), Some(3), Some(3), Some(3), Some(1),
          Some(2), Some(2), Some(2), Some(2), Some(1), Some(1), Some(2), Some(1),
    /*4*/ Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*5*/ Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*6*/ Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*7*/ Some(2), Some(2), Some(2), Some(2), Some(2), Some(2), Some(1), Some(2),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*8*/ Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*9*/ Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*a*/ Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*b*/ Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
          Some(1), Some(1), Some(1), Some(1), Some(1), Some(1), Some(2), Some(1),
    /*c*/ Some(2), Some(3), Some(3), Some(4), Some(3), Some(4), Some(2), Some(4),
          Some(2), Some(4), Some(3), n      , Some(3), Some(6), Some(2), Some(4),
    /*d*/ Some(2), Some(3), Some(3), n      , Some(3), Some(4), Some(2), Some(4),
          Some(2), Some(4), Some(3), n      , Some(3), n      , Some(2), Some(4),
    /*e*/ Some(3), Some(3), Some(2), n      , n      , Some(4), Some(2), Some(4),
          Some(4), Some(1), Some(4), n      , n      , n      , Some(2), Some(4),
    /*f*/ Some(3), Some(3), Some(2), Some(1), n      , Some(4), Some(2), Some(4),
          Some(3), Some(2), Some(4), Some(1), n      , n      , Some(2), Some(4),
    ]
};

#[test]
fn the_derived_timing_matches_the_published_table() {
    for opcode in 0..=255u8 {
        // `$CB` is measured on the page it selects, by the test below: a fetch
        // of the prefix always runs a second opcode, so the table cannot
        // describe it with one number.
        if decode(opcode).op == Op::PREFIX {
            continue;
        }
        let Some(want) = CYCLES[opcode as usize] else {
            assert!(
                !decode(opcode).class.is_documented(),
                "{opcode:#04x} has no published timing but is a real instruction"
            );
            continue;
        };
        // Every conditional is listed not-taken, so start with all flags clear
        // and pick the condition that fails. `$20`/`$30`/`$c0`… test NZ/NC and
        // are not taken with Z and C clear; `$28`/`$38`/`$c8`… test Z/C and are
        // taken, so those need the flags set the other way.
        let (cpu, _ram) = cpu_with(&[opcode, 0x00, 0x00]);
        let mut regs = Regs::post_boot_dmg();
        regs.f = 0;
        // A `JR`/`JP`/`CALL`/`RET` on Z or C must *fail*, so leave Z and C
        // clear and flip the sense for the NZ/NC forms.
        let taken_when_clear = matches!(
            opcode,
            0x20 | 0x30 | 0xc0 | 0xc2 | 0xc4 | 0xd0 | 0xd2 | 0xd4
        );
        if taken_when_clear {
            regs.f = flags::Z | flags::C;
        }
        cpu.set_regs(regs);
        let got = step(&cpu);
        assert_eq!(
            got,
            u64::from(want),
            "{opcode:#04x} ({}) took {got} M-cycles, table says {want}",
            decode(opcode).op.mnemonic()
        );
    }
}

#[test]
fn the_taken_branches_cost_one_more_cycle() {
    // JR cc taken: 3, not taken 2. JP cc taken 4. CALL cc taken 6. RET cc taken 5.
    for (opcode, taken, not_taken) in [
        (0x28u8, 3u64, 2u64), // JR Z
        (0xca, 4, 3),         // JP Z
        (0xcc, 6, 3),         // CALL Z
        (0xc8, 5, 2),         // RET Z
    ] {
        for (flag, want) in [(flags::Z, taken), (0, not_taken)] {
            let (cpu, _ram) = cpu_with(&[opcode, 0x00, 0x02]);
            let mut regs = Regs::post_boot_dmg();
            regs.f = flag;
            cpu.set_regs(regs);
            assert_eq!(step(&cpu), want, "{opcode:#04x} with F={flag:#04x}");
        }
    }
}

#[test]
fn the_cb_page_costs_two_cycles_or_four_through_hl() {
    for opcode in 0..=255u8 {
        let (cpu, _ram) = cpu_with(&[0xcb, opcode]);
        let insn = decode_cb(opcode);
        let touches_hl =
            insn.dst == super::isa::Operand::MHL || insn.src == super::isa::Operand::MHL;
        let want = match (touches_hl, insn.op) {
            (false, _) => 2,
            // `BIT n,(HL)` reads and does not write back, so it is one cycle
            // shorter than `RES`/`SET`.
            (true, Op::BIT) => 3,
            (true, _) => 4,
        };
        assert_eq!(
            step(&cpu),
            want,
            "cb {opcode:#04x} ({})",
            insn.op.mnemonic()
        );
    }
}

// ---------------------------------------------------------------------------
// Flags
// ---------------------------------------------------------------------------

/// Run one instruction with a prepared accumulator and flags, and report
/// `(A, F)`.
fn alu(code: &[u8], a: u8, f: u8) -> (u8, u8) {
    let (cpu, _ram) = cpu_with(code);
    let mut regs = Regs::post_boot_dmg();
    regs.a = a;
    regs.f = f;
    cpu.set_regs(regs);
    step(&cpu);
    let out = cpu.regs();
    (out.a, out.f)
}

#[test]
fn addition_sets_the_half_carry_out_of_bit_three() {
    // $0f + $01 carries out of the low nibble but not out of the byte.
    assert_eq!(alu(&[0xc6, 0x01], 0x0f, 0), (0x10, flags::H));
    // $ff + $01 wraps: zero, half-carry and carry.
    assert_eq!(
        alu(&[0xc6, 0x01], 0xff, 0),
        (0x00, flags::Z | flags::H | flags::C)
    );
    // ADC brings the carry into both the nibble and the byte sums.
    assert_eq!(alu(&[0xce, 0x0f], 0x00, flags::C), (0x10, flags::H));
}

#[test]
fn subtraction_sets_n_and_borrows_the_other_way() {
    assert_eq!(alu(&[0xd6, 0x01], 0x10, 0), (0x0f, flags::N | flags::H));
    assert_eq!(
        alu(&[0xd6, 0x01], 0x00, 0),
        (0xff, flags::N | flags::H | flags::C)
    );
    // CP is SUB that keeps the flags and throws the result away.
    assert_eq!(alu(&[0xfe, 0x10], 0x10, 0), (0x10, flags::Z | flags::N));
    // SBC borrows the carry in.
    assert_eq!(
        alu(&[0xde, 0x00], 0x00, flags::C),
        (0xff, flags::N | flags::H | flags::C)
    );
}

#[test]
fn and_is_the_only_logical_operation_that_sets_the_half_carry() {
    assert_eq!(alu(&[0xe6, 0x0f], 0xf0, 0), (0x00, flags::Z | flags::H));
    assert_eq!(alu(&[0xe6, 0xff], 0x0f, 0), (0x0f, flags::H));
    assert_eq!(alu(&[0xf6, 0x0f], 0xf0, 0), (0xff, 0));
    assert_eq!(alu(&[0xee, 0xff], 0xff, 0), (0x00, flags::Z));
}

#[test]
fn the_accumulator_rotates_always_clear_zero() {
    // This is the flag rule emulators get wrong most often: `RLCA` on zero
    // leaves Z *clear*, while `CB 07` (`RLC A`) on zero sets it.
    assert_eq!(alu(&[0x07], 0x00, flags::Z), (0x00, 0));
    assert_eq!(alu(&[0x0f], 0x00, flags::Z), (0x00, 0));
    assert_eq!(alu(&[0x17], 0x00, flags::Z), (0x00, 0));
    assert_eq!(alu(&[0x1f], 0x00, flags::Z), (0x00, 0));
    assert_eq!(alu(&[0xcb, 0x07], 0x00, 0), (0x00, flags::Z));
    // And the carry comes out of the bit that left.
    assert_eq!(alu(&[0x07], 0x80, 0), (0x01, flags::C));
    assert_eq!(alu(&[0x0f], 0x01, 0), (0x80, flags::C));
    assert_eq!(alu(&[0x17], 0x80, flags::C), (0x01, flags::C));
    assert_eq!(alu(&[0x1f], 0x01, flags::C), (0x80, flags::C));
}

#[test]
fn swap_is_a_nibble_exchange_that_clears_every_other_flag() {
    assert_eq!(alu(&[0xcb, 0x37], 0xab, flags::C), (0xba, 0));
    assert_eq!(alu(&[0xcb, 0x37], 0x00, flags::C), (0x00, flags::Z));
}

#[test]
fn sra_duplicates_bit_seven_and_srl_does_not() {
    assert_eq!(alu(&[0xcb, 0x2f], 0x81, 0), (0xc0, flags::C));
    assert_eq!(alu(&[0xcb, 0x3f], 0x81, 0), (0x40, flags::C));
    assert_eq!(alu(&[0xcb, 0x27], 0x81, 0), (0x02, flags::C));
}

#[test]
fn bit_leaves_the_carry_alone_and_sets_the_half_carry() {
    // `BIT 7,A` with A = $00: Z set, H set, C preserved.
    assert_eq!(
        alu(&[0xcb, 0x7f], 0x00, flags::C),
        (0x00, flags::Z | flags::H | flags::C)
    );
    assert_eq!(alu(&[0xcb, 0x7f], 0x80, 0), (0x80, flags::H));
}

#[test]
fn scf_and_ccf_touch_only_the_carry_side() {
    assert_eq!(
        alu(&[0x37], 0x12, flags::Z | flags::N | flags::H),
        (0x12, flags::Z | flags::C)
    );
    assert_eq!(alu(&[0x3f], 0x12, flags::Z | flags::C), (0x12, flags::Z));
    assert_eq!(alu(&[0x3f], 0x12, flags::Z), (0x12, flags::Z | flags::C));
    assert_eq!(alu(&[0x2f], 0x0f, 0), (0xf0, flags::N | flags::H));
}

#[test]
fn daa_corrects_in_the_direction_the_n_flag_records() {
    // $09 + $01 = $0a, which DAA turns into $10.
    let (cpu, _ram) = cpu_with(&[0xc6, 0x01, 0x27]);
    let mut regs = Regs::post_boot_dmg();
    regs.a = 0x09;
    regs.f = 0;
    cpu.set_regs(regs);
    step(&cpu);
    step(&cpu);
    assert_eq!(cpu.regs().a, 0x10);
    assert!(!cpu.regs().flag(flags::H), "DAA always clears H");

    // $10 - $01 = $0f, which DAA turns back into $09 because N is set.
    let (cpu, _ram) = cpu_with(&[0xd6, 0x01, 0x27]);
    let mut regs = Regs::post_boot_dmg();
    regs.a = 0x10;
    regs.f = 0;
    cpu.set_regs(regs);
    step(&cpu);
    step(&cpu);
    assert_eq!(cpu.regs().a, 0x09);
    assert!(cpu.regs().flag(flags::N), "DAA leaves N alone");

    // $99 + $01 = $9a, which decimal-adjusts to $00 with a carry.
    let (cpu, _ram) = cpu_with(&[0xc6, 0x01, 0x27]);
    let mut regs = Regs::post_boot_dmg();
    regs.a = 0x99;
    regs.f = 0;
    cpu.set_regs(regs);
    step(&cpu);
    step(&cpu);
    assert_eq!(cpu.regs().a, 0x00);
    assert!(cpu.regs().flag(flags::Z));
    assert!(cpu.regs().flag(flags::C));
}

#[test]
fn daa_never_clears_a_carry_it_found_set() {
    // A decimal carry that already happened cannot un-happen, so DAA only ever
    // sets C. This is the rule that makes multi-byte BCD addition work.
    let (a, f) = alu(&[0x27], 0x00, flags::C);
    assert_eq!(a, 0x60);
    assert!(f & flags::C != 0);
}

// ---------------------------------------------------------------------------
// 16-bit arithmetic
// ---------------------------------------------------------------------------

#[test]
fn add_hl_carries_out_of_bit_eleven_and_leaves_zero_alone() {
    let (cpu, _ram) = cpu_with(&[0x09]); // ADD HL,BC
    let mut regs = Regs::post_boot_dmg();
    regs.set_hl(0x0fff);
    regs.b = 0x00;
    regs.c = 0x01;
    regs.f = flags::Z;
    cpu.set_regs(regs);
    step(&cpu);
    assert_eq!(cpu.regs().hl(), 0x1000);
    assert!(cpu.regs().flag(flags::H));
    assert!(cpu.regs().flag(flags::Z), "ADD HL,rr does not touch Z");
    assert!(!cpu.regs().flag(flags::C));
}

#[test]
fn the_stack_relative_forms_take_their_flags_from_the_low_byte() {
    // `LD HL,SP+e8` and `ADD SP,e8` both clear Z and N and take H and C from
    // the *unsigned* low-byte addition, however negative the displacement is.
    let (cpu, _ram) = cpu_with(&[0xf8, 0x01]); // LD HL,SP+1
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0x000f;
    regs.f = flags::Z | flags::N;
    cpu.set_regs(regs);
    assert_eq!(step(&cpu), 3);
    assert_eq!(cpu.regs().hl(), 0x0010);
    assert_eq!(cpu.regs().f, flags::H);

    let (cpu, _ram) = cpu_with(&[0xe8, 0xff]); // ADD SP,-1
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0x0000;
    cpu.set_regs(regs);
    assert_eq!(step(&cpu), 4);
    assert_eq!(cpu.regs().sp, 0xffff);
    // $00 + $ff carries out of neither nibble nor byte.
    assert_eq!(cpu.regs().f, 0);
}

#[test]
fn sixteen_bit_increments_touch_no_flags() {
    let (cpu, _ram) = cpu_with(&[0x03]); // INC BC
    let mut regs = Regs::post_boot_dmg();
    regs.b = 0x00;
    regs.c = 0xff;
    regs.f = flags::Z | flags::N | flags::H | flags::C;
    cpu.set_regs(regs);
    step(&cpu);
    assert_eq!(cpu.regs().bc(), 0x0100);
    assert_eq!(cpu.regs().f, flags::Z | flags::N | flags::H | flags::C);
}

// ---------------------------------------------------------------------------
// Loads and the stack
// ---------------------------------------------------------------------------

#[test]
fn the_hl_post_adjust_forms_move_hl_exactly_once() {
    let (cpu, ram) = cpu_with(&[0x22, 0x2a]); // LD (HL+),A ; LD A,(HL+)
    let mut regs = Regs::post_boot_dmg();
    regs.set_hl(0xc000);
    regs.a = 0x42;
    cpu.set_regs(regs);
    assert_eq!(step(&cpu), 2);
    assert_eq!(cpu.regs().hl(), 0xc001);
    assert_eq!(ram.read_u8(0xc000).unwrap(), 0x42);
    ram.write_u8(0xc001, 0x99).unwrap();
    step(&cpu);
    assert_eq!(cpu.regs().a, 0x99);
    assert_eq!(cpu.regs().hl(), 0xc002);
}

#[test]
fn pop_af_cannot_put_bits_in_the_flag_registers_bottom_nibble() {
    let (cpu, ram) = cpu_with(&[0xf1]); // POP AF
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc000;
    cpu.set_regs(regs);
    ram.write_u8(0xc000, 0xff).unwrap();
    ram.write_u8(0xc001, 0x12).unwrap();
    assert_eq!(step(&cpu), 3);
    assert_eq!(cpu.regs().a, 0x12);
    assert_eq!(cpu.regs().f, 0xf0, "the low nibble has no storage");
}

#[test]
fn push_and_pop_round_trip_through_memory_in_the_documented_order() {
    let (cpu, ram) = cpu_with(&[0xc5]); // PUSH BC
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc002;
    regs.b = 0x12;
    regs.c = 0x34;
    cpu.set_regs(regs);
    assert_eq!(step(&cpu), 4);
    assert_eq!(cpu.regs().sp, 0xc000);
    // High byte first means it lands at the higher address.
    assert_eq!(ram.read_u8(0xc000).unwrap(), 0x34);
    assert_eq!(ram.read_u8(0xc001).unwrap(), 0x12);
}

#[test]
fn ld_from_the_address_immediate_writes_both_halves_of_sp() {
    let (cpu, ram) = cpu_with(&[0x08, 0x00, 0xc0]); // LD ($c000),SP
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xbeef;
    cpu.set_regs(regs);
    assert_eq!(step(&cpu), 5);
    assert_eq!(ram.read_u8(0xc000).unwrap(), 0xef);
    assert_eq!(ram.read_u8(0xc001).unwrap(), 0xbe);
}

#[test]
fn the_high_page_forms_address_ff00_plus_the_operand() {
    let (cpu, ram) = cpu_with(&[0xe0, 0x80, 0xf0, 0x80]);
    let mut regs = Regs::post_boot_dmg();
    regs.a = 0x5a;
    cpu.set_regs(regs);
    assert_eq!(step(&cpu), 3);
    assert_eq!(ram.read_u8(0xff80).unwrap(), 0x5a);
    cpu.set_reg(super::Reg::A, 0);
    assert_eq!(step(&cpu), 3);
    assert_eq!(cpu.regs().a, 0x5a);
}

// ---------------------------------------------------------------------------
// Control flow
// ---------------------------------------------------------------------------

#[test]
fn a_relative_branch_counts_from_the_next_instruction() {
    let (cpu, _ram) = cpu_with(&[0x18, 0xfe]); // JR -2: the classic self-loop
    step(&cpu);
    assert_eq!(cpu.regs().pc, 0x0100);
}

#[test]
fn call_pushes_the_address_after_the_operand() {
    let (cpu, ram) = cpu_with(&[0xcd, 0x00, 0x20]);
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc002;
    cpu.set_regs(regs);
    assert_eq!(step(&cpu), 6);
    assert_eq!(cpu.regs().pc, 0x2000);
    assert_eq!(ram.read_u8(0xc000).unwrap(), 0x03);
    assert_eq!(ram.read_u8(0xc001).unwrap(), 0x01);
}

#[test]
fn rst_jumps_to_the_vector_in_its_own_opcode() {
    for (opcode, vector) in [(0xc7u8, 0x00u16), (0xcf, 0x08), (0xef, 0x28), (0xff, 0x38)] {
        let (cpu, _ram) = cpu_with(&[opcode]);
        let mut regs = Regs::post_boot_dmg();
        regs.sp = 0xc002;
        cpu.set_regs(regs);
        assert_eq!(step(&cpu), 4);
        assert_eq!(cpu.regs().pc, vector);
    }
}

#[test]
fn an_unimplemented_opcode_hangs_the_processor_until_reset() {
    let (cpu, _ram) = cpu_with(&[0xd3, 0x00]);
    step(&cpu);
    assert!(cpu.is_locked());
    let before = cpu.regs().pc;
    // A locked core still consumes time — its clock is running, which is how a
    // machine around it keeps working — but fetches nothing.
    assert_eq!(step(&cpu), 1);
    assert_eq!(cpu.regs().pc, before);
    Device::reset(cpu.as_ref(), ResetKind::Cold);
    assert!(!cpu.is_locked());
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

#[test]
fn a_dispatch_takes_five_cycles_and_pushes_the_return_address() {
    let (cpu, ram) = cpu_with(&[0x00]);
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc002;
    cpu.set_regs(regs);
    cpu.set_ime(true);
    cpu.set_interrupt_enable(1 << interrupt::TIMER);
    cpu.request_interrupt(interrupt::TIMER);
    assert_eq!(step(&cpu), 5);
    assert_eq!(cpu.regs().pc, 0x0050);
    assert_eq!(cpu.regs().sp, 0xc000);
    assert_eq!(ram.read_u8(0xc000).unwrap(), 0x00);
    assert_eq!(ram.read_u8(0xc001).unwrap(), 0x01);
    assert!(!cpu.ime(), "dispatch clears IME");
    assert_eq!(cpu.interrupt_flags() & (1 << interrupt::TIMER), 0);
}

#[test]
fn the_lowest_pending_bit_wins() {
    let (cpu, _ram) = cpu_with(&[0x00]);
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc010;
    cpu.set_regs(regs);
    cpu.set_ime(true);
    cpu.set_interrupt_enable(0x1f);
    cpu.request_interrupt(interrupt::JOYPAD);
    cpu.request_interrupt(interrupt::VBLANK);
    step(&cpu);
    assert_eq!(cpu.regs().pc, 0x0040);
    // The joypad request is still pending: only the taken one is cleared.
    assert_eq!(cpu.interrupt_flags() & 0x1f, 1 << interrupt::JOYPAD);
}

#[test]
fn ei_is_one_instruction_late_and_di_cancels_it() {
    // EI ; NOP ; <dispatch>
    let (cpu, _ram) = cpu_with(&[0xfb, 0x00, 0x00]);
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc010;
    cpu.set_regs(regs);
    cpu.set_interrupt_enable(1 << interrupt::VBLANK);
    cpu.request_interrupt(interrupt::VBLANK);
    step(&cpu); // EI: still masked at the boundary that follows it
    assert_eq!(cpu.regs().pc, 0x0101);
    step(&cpu); // NOP, with IME now set
    assert_eq!(cpu.regs().pc, 0x0102);
    step(&cpu); // the dispatch
    assert_eq!(cpu.regs().pc, 0x0040);

    // EI ; DI : nothing gets through, because DI clears what EI armed.
    let (cpu, _ram) = cpu_with(&[0xfb, 0xf3, 0x00]);
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc010;
    cpu.set_regs(regs);
    cpu.set_interrupt_enable(1 << interrupt::VBLANK);
    cpu.request_interrupt(interrupt::VBLANK);
    step(&cpu);
    step(&cpu);
    step(&cpu);
    assert_eq!(cpu.regs().pc, 0x0103, "no dispatch happened");
    assert!(!cpu.ime());
}

#[test]
fn reti_enables_interrupts_immediately() {
    let (cpu, ram) = cpu_with(&[0xd9]);
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc000;
    cpu.set_regs(regs);
    ram.write_u8(0xc000, 0x00).unwrap();
    ram.write_u8(0xc001, 0x02).unwrap();
    assert_eq!(step(&cpu), 4);
    assert_eq!(cpu.regs().pc, 0x0200);
    assert!(cpu.ime());
}

#[test]
fn the_vector_is_chosen_after_the_pushes_so_a_stack_on_ffff_changes_it() {
    // Gekkio's `ie_push`. `IE` lives at `$FFFF`, so a stack pointer of `$0000`
    // pushes the high byte of `PC` straight over it. What is enabled *after*
    // that write is what decides the vector.
    let ram = Arc::new(RamStore::new(0x1_0000));
    ram.write_u8(0x0100, 0x00).unwrap(); // NOP, never reached
    let space = AddressSpace::new("cpu", 16);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), 0)
        .expect("maps");
    let cpu = Arc::new(Sm83::new(Config::default()));
    cpu.attach_space(Arc::new(space));
    Device::reset(cpu.as_ref(), ResetKind::Cold);
    // Map `IE` over the top of that RAM so the push really lands on it.
    {
        let space = space_of(&cpu);
        let mut topo = space.topology();
        topo.map(
            Device::region(cpu.as_ref(), super::IE_REGION).expect("the IE aperture"),
            super::IE_ADDRESS,
        )
        .expect("maps over the RAM");
    }
    let mut regs = Regs::post_boot_dmg();
    // `PC` = $0100, so the high byte pushed to $FFFF is $01 — which enables
    // exactly VBlank, whatever was enabled before.
    regs.pc = 0x0100;
    regs.sp = 0x0000;
    cpu.set_regs(regs);
    cpu.set_ime(true);
    cpu.set_interrupt_enable(0x1f);
    cpu.request_interrupt(interrupt::JOYPAD);
    cpu.request_interrupt(interrupt::VBLANK);
    step(&cpu);
    assert_eq!(
        cpu.regs().pc,
        0x0040,
        "the re-read of IE after the push chose the vector"
    );
    assert_eq!(cpu.interrupt_enable(), 0x01, "the push overwrote IE");
}

fn space_of(cpu: &Sm83) -> Arc<AddressSpace> {
    cpu.space().expect("a space was attached")
}

#[test]
fn a_push_that_leaves_nothing_enabled_dispatches_to_zero() {
    let ram = Arc::new(RamStore::new(0x1_0000));
    let space = Arc::new(AddressSpace::new("cpu", 16));
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), 0)
        .expect("maps");
    let cpu = Arc::new(Sm83::new(Config::default()));
    cpu.attach_space(Arc::clone(&space));
    Device::reset(cpu.as_ref(), ResetKind::Cold);
    space
        .topology()
        .map(
            Device::region(cpu.as_ref(), super::IE_REGION).expect("IE"),
            super::IE_ADDRESS,
        )
        .expect("maps");
    let mut regs = Regs::post_boot_dmg();
    // `PC` high byte $00 lands on `$FFFF`, disabling everything.
    regs.pc = 0x0020;
    regs.sp = 0x0000;
    cpu.set_regs(regs);
    cpu.set_ime(true);
    cpu.set_interrupt_enable(0x1f);
    cpu.request_interrupt(interrupt::VBLANK);
    step(&cpu);
    assert_eq!(cpu.regs().pc, 0x0000);
}

#[test]
fn halt_stops_the_core_but_not_its_clock() {
    let (cpu, _ram) = cpu_with(&[0x76, 0x00]);
    cpu.set_interrupt_enable(1 << interrupt::TIMER);
    step(&cpu);
    assert!(cpu.is_halted());
    // Time still passes — which is the only reason anything will ever wake it.
    assert_eq!(step(&cpu), 1);
    assert!(cpu.is_halted());
    cpu.request_interrupt(interrupt::TIMER);
    step(&cpu);
    assert!(!cpu.is_halted());
    // IME was clear, so the handler does not run: execution simply resumes.
    assert_eq!(cpu.regs().pc, 0x0102);
}

#[test]
fn the_halt_bug_reads_the_next_byte_twice() {
    // HALT with IME clear and an interrupt already pending does not halt. The
    // byte after it is fetched, and PC fails to advance past it — so `INC A`
    // executes twice (Pan Docs, *Halt*).
    let (cpu, _ram) = cpu_with(&[0x76, 0x3c, 0x00]);
    let mut regs = Regs::post_boot_dmg();
    regs.a = 0;
    cpu.set_regs(regs);
    cpu.set_interrupt_enable(1 << interrupt::TIMER);
    cpu.request_interrupt(interrupt::TIMER);
    step(&cpu); // HALT: does not halt
    assert!(!cpu.is_halted());
    step(&cpu); // INC A, with PC left where it was
    assert_eq!(cpu.regs().a, 1);
    assert_eq!(cpu.regs().pc, 0x0101, "PC did not advance past the byte");
    step(&cpu); // the same INC A again
    assert_eq!(cpu.regs().a, 2);
    assert_eq!(cpu.regs().pc, 0x0102);
}

#[test]
fn halt_with_ime_set_runs_the_handler_on_wake() {
    let (cpu, _ram) = cpu_with(&[0x76, 0x00]);
    let mut regs = Regs::post_boot_dmg();
    regs.sp = 0xc010;
    cpu.set_regs(regs);
    cpu.set_ime(true);
    cpu.set_interrupt_enable(1 << interrupt::SERIAL);
    step(&cpu);
    assert!(cpu.is_halted());
    cpu.request_interrupt(interrupt::SERIAL);
    step(&cpu);
    assert_eq!(cpu.regs().pc, 0x0058);
}

// ---------------------------------------------------------------------------
// The device surface
// ---------------------------------------------------------------------------

#[test]
fn the_interrupt_registers_read_back_the_way_hardware_does() {
    let (cpu, _ram) = cpu_with(&[]);
    let space = space_of(&cpu);
    space
        .topology()
        .map(
            Device::region(cpu.as_ref(), super::IF_REGION).expect("IF"),
            super::IF_ADDRESS,
        )
        .expect("maps");
    // `IF`'s top three bits are not implemented and read as ones.
    assert_eq!(
        space
            .read(super::IF_ADDRESS, Width::U8, MemAttrs::DEFAULT)
            .unwrap(),
        0xe0
    );
    space
        .write(super::IF_ADDRESS, Width::U8, 0xff, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(cpu.interrupt_flags(), 0xff);
    assert_eq!(cpu.pending_interrupts(), 0, "nothing is enabled yet");
}

#[test]
fn the_pins_are_edge_triggered_which_is_what_stat_blocking_needs() {
    use crate::core::wire::{Level, Wire, WireIdAllocator};

    let (cpu, _ram) = cpu_with(&[]);
    let ids = WireIdAllocator::new();
    let src = ids.alloc();
    let pin = Arc::new(super::InterruptPin::for_cpu(&cpu, interrupt::STAT, &[src]));
    let wire = Wire::builder()
        .source(src)
        .sink(Arc::clone(&pin) as Arc<dyn crate::core::wire::WireSink>, 0)
        .build_shared();

    wire.set(src, Level::High);
    assert_eq!(cpu.interrupt_flags() & (1 << interrupt::STAT), 0x02);
    cpu.clear_interrupt(interrupt::STAT);
    // Still high: no second edge, so no second request. That is exactly the
    // behaviour a hand-written implementation has to add on purpose.
    wire.set(src, Level::High);
    assert_eq!(cpu.interrupt_flags() & (1 << interrupt::STAT), 0);
    wire.set(src, Level::Low);
    wire.set(src, Level::High);
    assert_eq!(cpu.interrupt_flags() & (1 << interrupt::STAT), 0x02);
}

#[test]
fn a_snapshot_round_trip_reproduces_the_state_exactly() {
    let (cpu, _ram) = cpu_with(&[0x3e, 0x42, 0xfb, 0x76]);
    cpu.set_interrupt_enable(0x0f);
    cpu.request_interrupt(interrupt::TIMER);
    step(&cpu);
    step(&cpu);
    let before = cpu.regs();
    let cycles = cpu.cycles();

    let mut writer = StateWriter::new(MachineShape::new());
    {
        let mut chunk: ChunkWriter<'_> = writer
            .chunk("cpu", super::CLASS.name, super::CLASS.version)
            .expect("a chunk");
        Device::save(cpu.as_ref(), &mut chunk).expect("saves");
    }
    let bytes = writer.to_vec().expect("serialises");

    let (restored, _ram2) = cpu_with(&[]);
    let reader = crate::core::state::StateReader::new(&bytes).expect("well formed");
    let chunk = reader
        .load(
            "cpu",
            super::CLASS.name,
            super::CLASS.version,
            &crate::core::state::Migrations::new(),
        )
        .expect("finds the chunk");
    Device::load(restored.as_ref(), &mut chunk.reader()).expect("loads");
    assert_eq!(restored.regs(), before);
    assert_eq!(restored.cycles(), cycles);
    assert_eq!(restored.interrupt_enable(), 0x0f);
    assert!(restored.ime() || restored.pending_interrupts() != 0);
}

#[test]
fn the_class_constructs_from_properties() {
    let cpu = Sm83::from_props(&Props::new().with("post-boot", false)).expect("valid");
    assert!(!cpu.config().post_boot);
    Device::reset(&cpu, ResetKind::Cold);
    assert_eq!(
        cpu.regs().pc,
        0x0000,
        "no boot ROM substitute was asked for"
    );

    let err = Sm83::from_props(&Props::new().with("postboot", true)).expect_err("typo");
    assert!(alloc::format!("{err}").contains("postboot"), "{err}");
}

#[test]
fn the_disassembler_and_the_interpreter_agree_on_length() {
    // Both derive it from the same row, but the interpreter derives it by
    // *executing* — this is the check that the two derivations are the same
    // derivation.
    for opcode in 0..=255u8 {
        let insn = decode(opcode);
        if !matches!(insn.class, Class::Documented) || insn.op == Op::PREFIX {
            continue;
        }
        // Control flow moves PC somewhere else entirely, so only the
        // straight-line encodings can be measured this way.
        if matches!(
            insn.op,
            Op::JR
                | Op::JP
                | Op::CALL
                | Op::RET
                | Op::RETI
                | Op::RST
                | Op::HALT
                | Op::STOP
                | Op::LOCK
        ) {
            continue;
        }
        let (cpu, _ram) = cpu_with(&[opcode, 0x00, 0xc0]);
        let mut regs = Regs::post_boot_dmg();
        regs.set_hl(0xc000);
        regs.sp = 0xc100;
        cpu.set_regs(regs);
        step(&cpu);
        assert_eq!(
            cpu.regs().pc,
            0x0100 + insn.bytes(),
            "{opcode:#04x} ({}) advanced PC by the wrong amount",
            insn.op.mnemonic()
        );
    }
}

#[test]
fn disassembly_of_live_memory_uses_debug_attributes() {
    let (cpu, _ram) = cpu_with(&[0x21, 0x34, 0x12, 0x00]);
    let out: Vec<_> = cpu.disassemble(0x0100, 2);
    assert_eq!(alloc::format!("{}", out[0]), "LD HL,$1234");
    assert_eq!(alloc::format!("{}", out[1]), "NOP");
}

// ---------------------------------------------------------------------------
// The tick cursor
// ---------------------------------------------------------------------------

/// A device sampled from inside an instruction has to see the cycle the access
/// really happened on, which means the core must publish its counter as it runs
/// rather than only when it returns (`ROADMAP.md` §4.2). The value is read *by*
/// the access, so what matters is that it has already moved when the bus
/// operation is issued — hence the read below happens through a region that
/// records the cursor at the moment it answers.
#[test]
fn every_machine_cycle_is_published_before_its_bus_access() {
    use crate::core::sched::TickCursor;
    use crate::core::space::{AccessConstraints, MemOps, MemResult, Region as MmioRegion};
    use crate::core::sync::Mutex;
    use core::fmt;

    struct Watcher {
        cursor: TickCursor,
        seen: Mutex<Vec<u64>>,
    }
    impl fmt::Debug for Watcher {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_struct("Watcher").finish_non_exhaustive()
        }
    }
    impl MemOps for Watcher {
        fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
            self.seen.lock().push(self.cursor.get());
            dst[0] = 0x00;
            Ok(())
        }
        fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
            self.seen.lock().push(self.cursor.get());
            Ok(())
        }
        fn constraints(&self) -> AccessConstraints {
            AccessConstraints::IO.with_widths(Width::U8, Width::U8)
        }
    }

    let cursor = TickCursor::new();
    let watcher = Arc::new(Watcher {
        cursor: cursor.clone(),
        seen: Mutex::new(Vec::new()),
    });

    let ram = Arc::new(RamStore::new(0x1_0000));
    // `LD A,($C000)`: fetch, two immediate bytes, then the read.
    for (i, b) in [0xfa_u8, 0x00, 0xc0].iter().enumerate() {
        ram.write_u8(0x0100 + i as u64, *b).expect("in range");
    }
    let space = AddressSpace::new("cpu", 16);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), 0)
        .expect("maps");
    space
        .topology()
        .map(
            Arc::new(MmioRegion::io(
                "watch",
                1,
                Arc::clone(&watcher) as Arc<dyn MemOps>,
            )),
            0xc000,
        )
        .expect("maps");

    let cpu = Sm83::new(Config::default());
    cpu.attach_space(Arc::new(space));
    Device::reset(&cpu, ResetKind::Cold);
    cpu.attach_cursor(cursor.clone());

    assert_eq!(cpu.step(), 4, "LD A,(nn) is four machine cycles");
    // The watched region answers the fourth of them, and the counter has
    // already reached 4 by the time it does.
    assert_eq!(&*watcher.seen.lock(), &[4]);
    assert_eq!(cursor.get(), 4, "and it is left where the step ended");
}

/// The counter is ticks-since-power-on, not an offset into a budget: a core that
/// overran one budget carries the overshoot as debt, and the scheduler relies on
/// the cursor still reporting the truth.
#[test]
fn the_cursor_is_free_running_across_budgets() {
    use crate::core::sched::TickCursor;

    let cursor = TickCursor::new();
    // Four `NOP`s and a `JR -2`, so the core never runs out of work.
    let (cpu, _ram) = cpu_with(&[0x00, 0x00, 0x00, 0x00, 0x18, 0xfe]);
    cpu.attach_cursor(cursor.clone());
    assert_eq!(cpu.run_budget(3), 3);
    assert_eq!(cursor.get(), cpu.cycles());
    let before = cursor.get();
    cpu.run_budget(3);
    assert!(cursor.get() > before, "the counter never restarts");
    assert_eq!(cursor.get(), cpu.cycles());
}
