//! Unit tests for the ARMv7E-M core.
//!
//! These are the tests that say something about *one* mechanism: a decode
//! rule, a flag, an exception sequence. The two suites that say something
//! about the core as a whole are elsewhere — the differential against
//! `cpu::arm::aprofile` in `differential.rs` and the built corpus in
//! `conformance.rs` — and neither replaces the other.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::props::{Props, Value};
use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;

use super::isa::{
    Cond, DpOp, Insn, MemOffset, Operand, ShiftType, Size, decode, decode_imm_shift, is_32bit,
    thumb_expand_imm,
};
use super::sys::{Access, Exception, Sys, ccr, exc_return, fsr, shcsr};
use super::*;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Where the test harness puts the vector table and the code.
const VECTORS: u32 = 0;
/// Where the harness starts executing.
const ENTRY: u32 = 0x200;
/// The initial stack pointer.
const STACK: u32 = 0x1000;
/// How much RAM a harness gets.
const RAM: u64 = 0x4000;

/// A core with RAM, a vector table, and the given halfwords at [`ENTRY`].
struct Harness {
    cpu: Arc<ArmV7m>,
    ram: Arc<RamStore>,
}

impl Harness {
    fn new(cfg: Config, code: &[u16]) -> Harness {
        let ram = Arc::new(RamStore::new(RAM));
        ram.write_at(u64::from(VECTORS), &STACK.to_le_bytes())
            .unwrap();
        ram.write_at(u64::from(VECTORS) + 4, &(ENTRY | 1).to_le_bytes())
            .unwrap();
        // Every other vector points at a `B .` at 0x100, so an unexpected
        // exception parks rather than running off into whatever was there.
        ram.write_at(0x100, &0xe7feu16.to_le_bytes()).unwrap();
        for n in 2..48u64 {
            ram.write_at(n * 4, &0x101u32.to_le_bytes()).unwrap();
        }
        for (i, half) in code.iter().enumerate() {
            ram.write_at(u64::from(ENTRY) + (i as u64) * 2, &half.to_le_bytes())
                .unwrap();
        }
        let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), 0)
            .unwrap();
        let cpu = Arc::new(ArmV7m::new(cfg));
        cpu.attach_space(Arc::new(space));
        // Consume the reset sequence.
        cpu.step();
        Harness { cpu, ram }
    }

    /// A Cortex-M4 running `code`.
    fn m4(code: &[u16]) -> Harness {
        Harness::new(Config::CORTEX_M4, code)
    }

    fn word(&self, addr: u32) -> u32 {
        let mut v = 0u32;
        for k in 0..4 {
            v |= u32::from(self.ram.read_u8(u64::from(addr) + k).unwrap()) << (8 * k);
        }
        v
    }

    fn set_word(&self, addr: u32, value: u32) {
        self.ram
            .write_at(u64::from(addr), &value.to_le_bytes())
            .unwrap();
    }
}

/// Split a thirty-two-bit encoding into the two halfwords a fetch sees.
const fn wide(encoding: u32) -> [u16; 2] {
    [(encoding >> 16) as u16, encoding as u16]
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

#[test]
fn the_three_escape_prefixes_start_a_wide_instruction() {
    // `0b11100` is the sixteen-bit unconditional branch, which is exactly why
    // the test is not "the top three bits are ones".
    assert!(!is_32bit(0xe000));
    assert!(!is_32bit(0xe7ff));
    assert!(is_32bit(0xe800));
    assert!(is_32bit(0xf000));
    assert!(is_32bit(0xffff));
}

#[test]
fn thumb_expand_imm_covers_both_halves_of_its_encoding() {
    // The four replication patterns leave the carry alone.
    assert_eq!(thumb_expand_imm(0x0ab), (0x0000_00ab, None));
    assert_eq!(thumb_expand_imm(0x1ab), (0x00ab_00ab, None));
    assert_eq!(thumb_expand_imm(0x2ab), (0xab00_ab00, None));
    assert_eq!(thumb_expand_imm(0x3ab), (0xabab_abab, None));
    // The rotated half forces bit 7 set and reports bit 31 of the result.
    let (value, carry) = thumb_expand_imm(0x87f);
    assert_eq!(value, 0x00ff_0000);
    assert_eq!(carry, Some(false));
    let (value, carry) = thumb_expand_imm(0x400);
    assert_eq!(value, 0x8000_0000);
    assert_eq!(carry, Some(true));
}

#[test]
fn decode_imm_shift_rewrites_the_three_zero_amounts() {
    assert_eq!(decode_imm_shift(0, 0).ty, ShiftType::Lsl);
    assert_eq!(decode_imm_shift(0, 0).amount, 0);
    assert_eq!(decode_imm_shift(1, 0).amount, 32, "LSR #0 means LSR #32");
    assert_eq!(decode_imm_shift(2, 0).amount, 32, "ASR #0 means ASR #32");
    assert_eq!(decode_imm_shift(3, 0).ty, ShiftType::Rrx, "ROR #0 is RRX");
    assert_eq!(decode_imm_shift(3, 5).ty, ShiftType::Ror);
}

#[test]
fn the_sixteen_bit_encodings_decode_to_what_they_say() {
    assert_eq!(
        decode(0x2042, 0),
        Insn::DataProc {
            op: DpOp::Mov,
            s: true,
            rd: 0,
            rn: 0,
            operand: Operand::Imm {
                value: 0x42,
                carry: None
            },
        }
    );
    assert_eq!(decode(0x4770, 0), Insn::Bx { rm: 14 });
    assert_eq!(
        decode(0xb10a, 0),
        Insn::Cbz {
            nonzero: false,
            rn: 2,
            offset: 2
        }
    );
    assert_eq!(
        decode(0xbf18, 0),
        Insn::It {
            cond: Cond(1),
            mask: 8
        }
    );
    assert_eq!(
        decode(0xbf00, 0),
        Insn::Hint {
            op: isa::HintOp::Nop
        }
    );
    assert_eq!(decode(0xdf07, 0), Insn::Svc { imm: 7 });
    assert_eq!(decode(0xde00, 0), Insn::Udf { imm: 0 });
}

#[test]
fn the_wide_encodings_decode_to_what_they_say() {
    // `MOV.W r1, #0xab`
    let [a, b] = wide(0xf04f_01ab);
    assert_eq!(
        decode(a, b),
        Insn::DataProc {
            op: DpOp::Mov,
            s: false,
            rd: 1,
            rn: 0,
            operand: Operand::Imm {
                value: 0xab,
                carry: None
            },
        }
    );
    // `CMP.W r12, #0` aliases `SUB` with `Rd == PC`.
    let [a, b] = wide(0xf1bc_0f00);
    assert!(matches!(
        decode(a, b),
        Insn::DataProc {
            op: DpOp::Cmp,
            rn: 12,
            ..
        }
    ));
    // `CLZ r3, r1`
    let [a, b] = wide(0xfab1_f381);
    assert_eq!(
        decode(a, b),
        Insn::Misc {
            op: isa::MiscOp::Clz,
            rd: 3,
            rm: 1
        }
    );
    // `PKHTB r3, r1, r2, ASR #16` — `tb` is hw2 bit 5, not bit 4.
    let [a, b] = wide(0xeac1_4322);
    assert!(matches!(
        decode(a, b),
        Insn::Pkh {
            tb: true,
            rd: 3,
            rn: 1,
            rm: 2,
            ..
        }
    ));
    // `SADD16 r3, r1, r2`
    let [a, b] = wide(0xfa91_f302);
    assert_eq!(
        decode(a, b),
        Insn::Simd {
            mode: isa::SimdMode::Signed,
            shape: isa::SimdShape::Add16,
            rd: 3,
            rn: 1,
            rm: 2
        }
    );
    // `LDR.W r1, [r0, #4]`
    let [a, b] = wide(0xf8d0_1004);
    assert_eq!(
        decode(a, b),
        Insn::LoadStore {
            load: true,
            size: Size::Word,
            signed: false,
            rt: 1,
            rn: 0,
            offset: MemOffset::Imm(4),
            index: true,
            add: true,
            wback: false,
            unpriv: false,
        }
    );
    // `TBB [pc, r2]`
    let [a, b] = wide(0xe8df_f002);
    assert_eq!(
        decode(a, b),
        Insn::TableBranch {
            rn: 15,
            rm: 2,
            half: false
        }
    );
    // A coprocessor encoding is distinct from an undefined one: the fault
    // differs, and firmware reads exactly that bit to find out there is no
    // FPU.
    let [a, b] = wide(0xeeb0_0a40);
    assert!(matches!(decode(a, b), Insn::Coproc { cp: 10 }));
}

#[test]
fn the_disassembler_and_the_decoder_are_the_same_description() {
    let show = |encoding: u32| {
        let [a, b] = wide(encoding);
        format!("{}", decode(a, b))
    };
    let show16 = |half: u16| format!("{}", decode(half, 0));
    assert_eq!(show16(0x2042), "MOVS r0, #66");
    assert_eq!(show16(0x4770), "BX lr");
    assert_eq!(show16(0xb510), "PUSH {r4, lr}");
    assert_eq!(show16(0xbd10), "POP {r4, pc}");
    assert_eq!(show16(0xbf18), "IT NE");
    assert_eq!(show(0xfab1_f381), "CLZ r3, r1");
    assert_eq!(show(0xfa91_f302), "SADD16 r3, r1, r2");
    assert_eq!(show(0xf8d0_1004), "LDR r1, [r0, #4]");
    assert_eq!(show(0xfb02_f303), "MUL r3, r2, r3");
    assert_eq!(show(0xf364_0207), "BFI r2, r4, #0, #8");
    // `QADD Rd, Rm, Rn` — the doubled operand of the QD forms is last.
    assert_eq!(show(0xfa81_f382), "QADD r3, r2, r1");
}

#[test]
fn an_it_block_disassembles_with_its_then_and_else_letters() {
    // `ITTEE EQ` is condition 0000 with mask 0111.
    assert_eq!(format!("{}", decode(0xbf07, 0)), "ITTEE EQ");
    // `ITE NE` is condition 0001 with mask 0100: the terminating one is bit
    // 2, and the bit above it differs from the condition's low bit, so the
    // second slot is an `E`.
    assert_eq!(format!("{}", decode(0xbf14, 0)), "ITE NE");
    assert_eq!(format!("{}", decode(0xbf1c, 0)), "ITT NE");
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

#[test]
fn reset_takes_sp_and_pc_from_the_vector_table() {
    let h = Harness::m4(&[0xbf00]);
    assert_eq!(h.cpu.pc(), ENTRY);
    assert_eq!(h.cpu.msp(), STACK);
    assert!(!h.cpu.regs().in_handler());
}

#[test]
fn a_reset_vector_with_bit_zero_clear_is_an_invalid_state_fault() {
    let ram = Arc::new(RamStore::new(RAM));
    ram.write_at(0, &STACK.to_le_bytes()).unwrap();
    // No Thumb bit: the architecture loads `EPSR.T` from it, and executing
    // with `T` clear is a UsageFault (DDI 0403 B1.5.5).
    ram.write_at(4, &ENTRY.to_le_bytes()).unwrap();
    ram.write_at(0x100, &0xe7feu16.to_le_bytes()).unwrap();
    for n in 2..48u64 {
        ram.write_at(n * 4, &0x101u32.to_le_bytes()).unwrap();
    }
    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), 0)
        .unwrap();
    let cpu = ArmV7m::new(Config::CORTEX_M4);
    cpu.attach_space(Arc::new(space));
    cpu.step();
    assert_eq!(cpu.xpsr() & xpsr::T, 0);
    cpu.step();
    assert!(cpu.with_sys(|s| s.cfsr & fsr::UF_INVSTATE != 0));
}

#[test]
fn the_pc_reads_as_the_instruction_plus_four_in_both_widths() {
    // `ADD r0, pc, #0` then the wide `ADD.W r1, pc, #0` — both must see the
    // same rule, which is what makes a literal pool work.
    let h = Harness::m4(&[0xa000, 0xf20f_0100u32 as u16, 0]);
    h.step_and(|_| {});
    assert_eq!(h.cpu.reg(0), (ENTRY + 4) & !3);
}

impl Harness {
    fn step_and(&self, f: impl FnOnce(&ArmV7m)) {
        self.cpu.step();
        f(&self.cpu);
    }
}

#[test]
fn an_it_block_suppresses_the_flags_of_its_sixteen_bit_slots() {
    // `MOVS r1, #0` ; `CMP r1, #0` ; `ITT EQ` ; `MOVEQ r2, #2` ;
    // `MOVEQ r3, #3`
    //
    // Both slots must run. If the first one set the flags — as the same
    // encoding does outside an IT block — `Z` would clear and the second
    // slot would be skipped, which is the bug this asserts against.
    let h = Harness::m4(&[0x2100, 0x2900, 0xbf04, 0x2202, 0x2303]);
    for _ in 0..5 {
        h.cpu.step();
    }
    assert_eq!(h.cpu.reg(2), 2);
    assert_eq!(h.cpu.reg(3), 3);
}

#[test]
fn an_it_block_runs_its_else_slots_when_the_condition_fails() {
    // `MOVS r1, #1` ; `CMP r1, #0` ; `ITE EQ` ; `MOVEQ r2, #2` ;
    // `MOVNE r3, #3`
    let h = Harness::m4(&[0x2101, 0x2900, 0xbf0c, 0x2202, 0x2303]);
    for _ in 0..5 {
        h.cpu.step();
    }
    assert_eq!(h.cpu.reg(2), 0);
    assert_eq!(h.cpu.reg(3), 3);
}

#[test]
fn unaligned_access_works_unless_ccr_says_otherwise() {
    // `LDR.W r1, [r0]`
    let h = Harness::m4(&wide(0xf8d0_1000));
    h.set_word(0x800, 0x1122_3344);
    h.set_word(0x804, 0x5566_7788);
    h.cpu.set_reg(0, 0x801);
    h.cpu.step();
    assert_eq!(h.cpu.reg(1), 0x8811_2233);

    // The same access with `CCR.UNALIGN_TRP` set is a UsageFault.
    let h = Harness::m4(&wide(0xf8d0_1000));
    h.set_word(0x800, 0x1122_3344);
    h.cpu.set_reg(0, 0x801);
    h.cpu.with_sys(|s| s.ccr |= ccr::UNALIGN_TRP);
    h.cpu.with_sys(|s| s.shcsr |= shcsr::USGFAULTENA);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::USAGE_FAULT);
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_UNALIGNED != 0));
}

#[test]
fn a_block_transfer_is_always_word_aligned() {
    // `LDMIA r0, {r1, r2}` with an unaligned base faults whatever
    // `UNALIGN_TRP` says: the architecture never splits these.
    let h = Harness::m4(&wide(0xe890_0006));
    h.cpu.set_reg(0, 0x802);
    h.cpu.with_sys(|s| s.shcsr |= shcsr::USGFAULTENA);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::USAGE_FAULT);
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_UNALIGNED != 0));
}

#[test]
fn a_branch_to_an_even_address_is_an_invalid_state_fault() {
    // `BX r0` with bit 0 clear asks for ARM state, which this architecture
    // does not have.
    let h = Harness::m4(&[0x4700]);
    h.cpu.set_reg(0, 0x300);
    h.cpu.with_sys(|s| s.shcsr |= shcsr::USGFAULTENA);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::USAGE_FAULT);
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_INVSTATE != 0));
}

#[test]
fn the_exclusive_monitor_lets_exactly_one_store_through() {
    // `LDREX r1, [r0]` ; `STREX r2, r3, [r0]` ; `STREX r4, r3, [r0]`
    let mut code = Vec::new();
    code.extend_from_slice(&wide(0xe850_1f00));
    code.extend_from_slice(&wide(0xe840_3200));
    code.extend_from_slice(&wide(0xe840_3400));
    let h = Harness::m4(&code);
    h.cpu.set_reg(0, 0x900);
    h.cpu.set_reg(3, 0xabcd);
    h.cpu.step();
    h.cpu.step();
    assert_eq!(h.cpu.reg(2), 0, "the tagged store succeeds");
    assert_eq!(h.word(0x900), 0xabcd);
    h.cpu.step();
    assert_eq!(h.cpu.reg(4), 1, "the tag is consumed");
}

#[test]
fn wfi_sleeps_and_an_interrupt_wakes_it() {
    let h = Harness::m4(&[0xbf30, 0xbf00]);
    h.cpu.step();
    assert!(h.cpu.is_asleep());
    let before = h.cpu.cycles();
    h.cpu.step();
    assert!(h.cpu.is_asleep(), "still asleep with nothing pending");
    assert!(h.cpu.cycles() > before, "a sleeping core still spends time");
    h.cpu.with_sys(|s| s.set_enable(Exception::IRQ0, true));
    h.cpu.pend_irq(0);
    h.cpu.step();
    assert!(!h.cpu.is_asleep());
}

#[test]
fn a_hardfault_inside_a_hardfault_locks_the_core_up() {
    // `UDF #0`, with every configurable fault disabled so it escalates.
    let h = Harness::m4(&[0xde00]);
    // Point the HardFault vector back at the UDF so the handler faults too.
    h.set_word(3 * 4, ENTRY | 1);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    h.cpu.step();
    assert!(h.cpu.is_locked_up());
    assert_eq!(h.cpu.pc(), 0xffff_fffe);
    // A locked-up core still consumes budget, so a scheduler is not starved.
    let before = h.cpu.cycles();
    h.cpu.step();
    assert!(h.cpu.cycles() > before);
}

#[test]
fn a_cortex_m3_has_no_dsp_extension() {
    // `SADD16 r3, r1, r2` runs on an M4 and traps on an M3, which is how a
    // guest discovers which it is on.
    let code = wide(0xfa91_f302);
    let m4 = Harness::new(Config::CORTEX_M4, &code);
    m4.cpu.set_reg(1, 0x0001_0002);
    m4.cpu.set_reg(2, 0x0003_0004);
    m4.cpu.step();
    assert_eq!(m4.cpu.reg(3), 0x0004_0006);

    let m3 = Harness::new(Config::CORTEX_M3, &code);
    m3.cpu.with_sys(|s| s.shcsr |= shcsr::USGFAULTENA);
    m3.cpu.step();
    assert_eq!(m3.cpu.current_exception(), Exception::USAGE_FAULT);
    assert!(m3.cpu.with_sys(|s| s.cfsr & fsr::UF_UNDEFINSTR != 0));
}

#[test]
fn a_coprocessor_access_is_nocp_rather_than_undefined() {
    // `VMOV.F32 s0, s0` — there is no FPU, and firmware distinguishes
    // "absent" from "not an instruction" by exactly this bit.
    let h = Harness::m4(&wide(0xeeb0_0a40));
    h.cpu.with_sys(|s| s.shcsr |= shcsr::USGFAULTENA);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::USAGE_FAULT);
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_NOCP != 0));
}

#[test]
fn big_endian_is_byte_invariant() {
    // BE-8: each byte keeps its address and a word load returns them
    // reversed (DDI 0403 A3.3). Instructions stay little-endian whatever the
    // data endianness, so only the vector table and the data are laid out
    // big-endian here.
    let ram = Arc::new(RamStore::new(RAM));
    ram.write_at(0, &STACK.to_be_bytes()).unwrap();
    ram.write_at(4, &(ENTRY | 1).to_be_bytes()).unwrap();
    for (i, half) in wide(0xf8d0_1000).iter().enumerate() {
        ram.write_at(u64::from(ENTRY) + (i as u64) * 2, &half.to_le_bytes())
            .unwrap();
    }
    ram.write_at(0x900, &[0x11, 0x22, 0x33, 0x44]).unwrap();
    let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), 0)
        .unwrap();
    let cpu = ArmV7m::new(Config::CORTEX_M4.with_endian(crate::core::value::Endian::Big));
    cpu.attach_space(Arc::new(space));
    cpu.step();
    assert_eq!(cpu.pc(), ENTRY);
    cpu.set_reg(0, 0x900);
    cpu.step();
    assert_eq!(cpu.reg(1), 0x1122_3344);
}

// ---------------------------------------------------------------------------
// The exception model
// ---------------------------------------------------------------------------

#[test]
fn an_svc_stacks_a_frame_and_returns_through_exc_return() {
    // `SVC #7` ; `MOVS r0, #9`, with the SVCall vector pointing at a
    // `BX lr` we plant ourselves.
    let h = Harness::m4(&[0xdf07, 0x2009]);
    h.set_word(11 * 4, 0x301);
    h.set_word(0x300, 0x4770); // `BX lr`
    h.cpu.set_reg(0, 0xa0);
    h.cpu.set_reg(1, 0xa1);
    h.cpu.set_reg(2, 0xa2);
    h.cpu.set_reg(3, 0xa3);
    h.cpu.set_reg(12, 0xac);

    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
    assert_eq!(h.cpu.reg(14), exc_return::THREAD_MSP);
    assert_eq!(h.cpu.last_svc(), 7);
    let sp = h.cpu.msp();
    assert_eq!(h.word(sp), 0xa0);
    assert_eq!(h.word(sp + 4), 0xa1);
    assert_eq!(h.word(sp + 8), 0xa2);
    assert_eq!(h.word(sp + 12), 0xa3);
    assert_eq!(h.word(sp + 16), 0xac);
    assert_eq!(
        h.word(sp + 24),
        ENTRY + 2,
        "SVC stacks the *next* instruction"
    );
    assert_eq!(h.word(sp + 28) & xpsr::T, xpsr::T);

    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.msp(), STACK);
    assert_eq!(h.cpu.pc(), ENTRY + 2);
    h.cpu.step();
    assert_eq!(h.cpu.reg(0), 9);
}

#[test]
fn a_synchronous_fault_stacks_the_faulting_instruction() {
    // `UDF #0` — a fault's handler must see the address that faulted, not
    // the one after it, or it cannot examine the instruction.
    let h = Harness::m4(&[0xde00]);
    h.set_word(6 * 4, 0x301);
    h.set_word(0x300, 0x4770);
    h.cpu.with_sys(|s| s.shcsr |= shcsr::USGFAULTENA);
    h.cpu.step();
    let sp = h.cpu.msp();
    assert_eq!(h.word(sp + 24), ENTRY);
}

#[test]
fn thread_mode_can_run_on_the_process_stack() {
    // `MSR psp, r0` ; `MSR control, r1` ; `ISB` ; `MOV r2, sp`
    let mut code = Vec::new();
    code.extend_from_slice(&wide(0xf380_8809)); // MSR psp, r0
    code.extend_from_slice(&wide(0xf381_8814)); // MSR control, r1
    code.extend_from_slice(&wide(0xf3bf_8f6f)); // ISB
    code.push(0x466a); // MOV r2, sp
    let h = Harness::m4(&code);
    h.cpu.set_reg(0, 0x0c00);
    h.cpu.set_reg(1, 2);
    for _ in 0..4 {
        h.cpu.step();
    }
    assert_eq!(h.cpu.reg(2), 0x0c00);
    assert_eq!(h.cpu.psp(), 0x0c00);
    assert_eq!(h.cpu.msp(), STACK, "the main stack is untouched");
}

#[test]
fn an_exception_from_the_process_stack_returns_with_fffffffd() {
    let mut code = Vec::new();
    code.extend_from_slice(&wide(0xf380_8809)); // MSR psp, r0
    code.extend_from_slice(&wide(0xf381_8814)); // MSR control, r1
    code.extend_from_slice(&wide(0xf3bf_8f6f)); // ISB
    code.push(0xdf00); // SVC #0
    let h = Harness::m4(&code);
    h.set_word(11 * 4, 0x301);
    h.set_word(0x300, 0x4770);
    h.cpu.set_reg(0, 0x0c00);
    h.cpu.set_reg(1, 2);
    for _ in 0..4 {
        h.cpu.step();
    }
    assert_eq!(h.cpu.reg(14), exc_return::THREAD_PSP);
    // The frame went on the process stack; the main stack never moved.
    assert_eq!(h.cpu.msp(), STACK);
    assert!(h.cpu.psp() < 0x0c00);
}

#[test]
fn stack_alignment_pads_an_odd_frame_and_records_it() {
    // With `CCR.STKALIGN` set — and it is RAO on this part — an entry from a
    // stack pointer that is only word-aligned pads by four and sets bit 9 of
    // the stacked xPSR, so the return can undo it.
    let h = Harness::m4(&[0xdf00]);
    h.set_word(11 * 4, 0x301);
    h.set_word(0x300, 0x4770);
    h.cpu.set_reg(13, STACK - 4);
    h.cpu.set_regs(Regs {
        msp: STACK - 4,
        ..h.cpu.regs()
    });
    h.cpu.step();
    let sp = h.cpu.msp();
    assert_eq!(sp % 8, 0, "the frame is eight-byte aligned");
    assert_ne!(h.word(sp + 28) & (1 << 9), 0, "and it says it padded");
    h.cpu.step();
    assert_eq!(h.cpu.msp(), STACK - 4, "the padding is undone on return");
}

#[test]
fn a_higher_priority_interrupt_preempts_a_lower_one() {
    // IRQ0's handler is a `B .`; IRQ1 is more urgent and must interrupt it.
    let h = Harness::m4(&[0xbf00, 0xbf00, 0xbf00, 0xbf00]);
    h.set_word(16 * 4, 0x301);
    h.set_word(0x300, 0xe7fe); // `B .`
    h.set_word(17 * 4, 0x401);
    h.set_word(0x400, 0xe7fe);
    h.cpu.with_sys(|s| {
        s.set_enable(Exception::IRQ0, true);
        s.set_enable(Exception(17), true);
        s.priority[16] = 0x80;
        s.priority[17] = 0x00;
    });
    h.cpu.pend_irq(0);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::IRQ0);
    h.cpu.pend_irq(1);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception(17));
    assert_eq!(h.cpu.reg(14), exc_return::HANDLER_MSP);
    assert!(h.cpu.with_sys(|s| s.is_active(Exception::IRQ0)));
}

#[test]
fn an_equal_priority_interrupt_waits_for_the_handler_to_finish() {
    let h = Harness::m4(&[0xbf00]);
    h.set_word(16 * 4, 0x301);
    h.set_word(0x300, 0xe7fe);
    h.set_word(17 * 4, 0x401);
    h.set_word(0x400, 0xe7fe);
    h.cpu.with_sys(|s| {
        s.set_enable(Exception::IRQ0, true);
        s.set_enable(Exception(17), true);
    });
    h.cpu.pend_irq(0);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::IRQ0);
    h.cpu.pend_irq(1);
    h.cpu.step();
    assert_eq!(
        h.cpu.current_exception(),
        Exception::IRQ0,
        "same priority does not preempt"
    );
    assert!(h.cpu.with_sys(|s| s.is_pending(Exception(17))));
}

#[test]
fn an_exception_return_tail_chains_rather_than_unstacking_twice() {
    // IRQ0's handler is `BX lr`; IRQ1 pends while it runs. The return must
    // go straight into IRQ1's handler with the stack where it is.
    let h = Harness::m4(&[0xbf00]);
    h.set_word(16 * 4, 0x301);
    h.set_word(0x300, 0x4770); // `BX lr`
    h.set_word(17 * 4, 0x401);
    h.set_word(0x400, 0xe7fe);
    h.cpu.with_sys(|s| {
        s.set_enable(Exception::IRQ0, true);
        s.set_enable(Exception(17), true);
    });
    h.cpu.pend_irq(0);
    h.cpu.step();
    let stacked_sp = h.cpu.msp();
    assert_eq!(h.cpu.current_exception(), Exception::IRQ0);
    h.cpu.pend_irq(1);
    h.cpu.step(); // `BX lr` -> exception return -> tail-chain
    assert_eq!(h.cpu.current_exception(), Exception(17));
    assert_eq!(
        h.cpu.msp(),
        stacked_sp,
        "the frame stayed where it was: no pop, no second push"
    );
    assert_eq!(h.cpu.reg(14), exc_return::THREAD_MSP);
    assert!(!h.cpu.with_sys(|s| s.is_active(Exception::IRQ0)));
}

#[test]
fn primask_and_basepri_hold_an_interrupt_off() {
    let h = Harness::m4(&[0xb672, 0xbf00, 0xb662, 0xbf00]); // CPSID i ; NOP ; CPSIE i ; NOP
    h.set_word(16 * 4, 0x301);
    h.set_word(0x300, 0xe7fe);
    h.cpu.with_sys(|s| s.set_enable(Exception::IRQ0, true));
    h.cpu.step(); // CPSID i
    h.cpu.pend_irq(0);
    h.cpu.step(); // NOP, still masked
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    h.cpu.step(); // CPSIE i
    h.cpu.step(); // now it lands
    assert_eq!(h.cpu.current_exception(), Exception::IRQ0);
}

#[test]
fn faultmask_masks_everything_but_nmi() {
    let h = Harness::m4(&[0xb673, 0xbf00, 0xbf00]); // CPSID f ; NOP ; NOP
    h.set_word(16 * 4, 0x301);
    h.set_word(0x300, 0xe7fe);
    h.set_word(2 * 4, 0x401);
    h.set_word(0x400, 0xe7fe);
    h.cpu.with_sys(|s| s.set_enable(Exception::IRQ0, true));
    h.cpu.step();
    assert_eq!(h.cpu.execution_priority(), -1);
    h.cpu.pend_irq(0);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    h.cpu.with_sys(|s| s.set_pending(Exception::NMI, true));
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::NMI);
}

// ---------------------------------------------------------------------------
// The system block
// ---------------------------------------------------------------------------

#[test]
fn only_the_implemented_priority_bits_stick() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 3, 8);
    sys.write_word(0xe000_e400, 0xffff_ffff);
    assert_eq!(sys.priority[16], 0xe0);
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    sys.write_word(0xe000_e400, 0xffff_ffff);
    assert_eq!(sys.priority[16], 0xff);
}

#[test]
fn prigroup_masks_the_sub_priority_out_of_a_comparison() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    sys.priority[16] = 0x11;
    sys.priority[17] = 0x10;
    assert_eq!(
        sys.priority_of(Exception(16)),
        0x10,
        "prigroup 0 drops bit 0"
    );
    assert_eq!(sys.priority_of(Exception(17)), 0x10);
    sys.prigroup = 0;
    // With the sub-priority masked away the two are equal, so the lower
    // exception number wins.
    sys.set_enable(Exception(16), true);
    sys.set_enable(Exception(17), true);
    sys.set_pending(Exception(16), true);
    sys.set_pending(Exception(17), true);
    assert_eq!(sys.highest_pending().unwrap().0, Exception(16));
}

#[test]
fn the_architectural_priorities_are_negative_and_fixed() {
    let sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    assert_eq!(sys.priority_of(Exception::RESET), -3);
    assert_eq!(sys.priority_of(Exception::NMI), -2);
    assert_eq!(sys.priority_of(Exception::HARD_FAULT), -1);
}

#[test]
fn systick_counts_down_and_reloads() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    sys.syst_rvr = 4;
    sys.syst_cvr = 4;
    sys.syst_csr = 1;
    assert!(!sys.tick_systick(3));
    assert_eq!(sys.syst_cvr, 1);
    assert!(sys.tick_systick(1), "reaching zero is the wrap");
    assert_ne!(sys.syst_csr & (1 << 16), 0, "COUNTFLAG");
    // Reading CSR clears COUNTFLAG; a debug read must not.
    assert_ne!(sys.read_word(0xe000_e010, true).unwrap() & (1 << 16), 0);
    assert_ne!(sys.syst_csr & (1 << 16), 0, "a debug read is not a read");
    assert_ne!(sys.read_word(0xe000_e010, false).unwrap() & (1 << 16), 0);
    assert_eq!(sys.syst_csr & (1 << 16), 0);
}

#[test]
fn aircr_ignores_a_write_without_its_key() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    sys.write_word(0xe000_ed0c, 0x0000_0700);
    assert_eq!(sys.prigroup, 0);
    sys.write_word(0xe000_ed0c, 0x05fa_0500);
    assert_eq!(sys.prigroup, 5);
    sys.write_word(0xe000_ed0c, 0x05fa_0004);
    assert!(sys.reset_requested);
}

#[test]
fn the_nvic_set_and_clear_registers_address_one_bitmap() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    sys.write_word(0xe000_e100, 0b0101);
    assert_eq!(sys.read_word(0xe000_e100, false).unwrap(), 0b0101);
    sys.write_word(0xe000_e180, 0b0001);
    assert_eq!(sys.read_word(0xe000_e100, false).unwrap(), 0b0100);
    assert!(sys.is_enabled(Exception(18)));
    assert!(!sys.is_enabled(Exception(16)));
}

#[test]
fn the_configurable_faults_are_enabled_by_shcsr_and_nothing_else() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    assert!(!sys.is_enabled(Exception::USAGE_FAULT));
    sys.write_word(0xe000_ed24, shcsr::USGFAULTENA);
    assert!(sys.is_enabled(Exception::USAGE_FAULT));
    assert!(!sys.is_enabled(Exception::BUS_FAULT));
    // NMI and HardFault have no enable bit at all.
    assert!(sys.is_enabled(Exception::NMI));
    assert!(sys.is_enabled(Exception::HARD_FAULT));
}

#[test]
fn an_unimplemented_ppb_register_reads_as_zero_rather_than_faulting() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    // The DWT's cycle counter: firmware probes for it, and a fault would be
    // a worse answer than "not present".
    assert_eq!(sys.read_word(0xe000_1004, false), Some(0));
    // Outside the PPB there is nothing to answer with.
    assert_eq!(sys.read_word(0x2000_0000, false), None);
}

#[test]
fn the_mpu_permission_matrix_matches_the_manual() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    // Region 0: 256 bytes at 0x1000, privileged read/write, no user access.
    sys.mpu_rbar[0] = 0x1000;
    sys.mpu_rasr[0] = (0b001 << 24) | (7 << 1) | 1;
    sys.mpu_ctrl = 0b101; // ENABLE | PRIVDEFENA
    assert!(sys.mpu_permits(0x1000, Access::Write, true, 0));
    assert!(!sys.mpu_permits(0x1000, Access::Write, false, 0));
    assert!(!sys.mpu_permits(0x1000, Access::Read, false, 0));
    // Outside every region, privileged code falls back on the default map.
    assert!(sys.mpu_permits(0x2000, Access::Write, true, 0));
    assert!(!sys.mpu_permits(0x2000, Access::Write, false, 0));

    // AP 0b110 is read-only for everybody.
    sys.mpu_rasr[0] = (0b110 << 24) | (7 << 1) | 1;
    assert!(sys.mpu_permits(0x1000, Access::Read, false, 0));
    assert!(!sys.mpu_permits(0x1000, Access::Write, true, 0));

    // XN forbids a fetch and nothing else.
    sys.mpu_rasr[0] = (0b011 << 24) | (1 << 28) | (7 << 1) | 1;
    assert!(sys.mpu_permits(0x1000, Access::Read, false, 0));
    assert!(!sys.mpu_permits(0x1000, Access::Fetch, false, 0));

    // With HFNMIENA clear the MPU is off while the priority is negative,
    // which is what keeps a fault handler runnable when the MPU is what
    // broke.
    sys.mpu_rasr[0] = (7 << 1) | 1; // AP 000: no access to anybody
    assert!(!sys.mpu_permits(0x1000, Access::Read, true, 0));
    assert!(sys.mpu_permits(0x1000, Access::Read, true, -1));
}

#[test]
fn a_disabled_mpu_sub_region_falls_through_to_the_next_match() {
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    // 2 KiB at 0x1000 with the second eighth switched out.
    sys.mpu_rbar[0] = 0x1000;
    sys.mpu_rasr[0] = (1 << 9) | (10 << 1) | 1; // AP 000, sub-region 1 off
    sys.mpu_ctrl = 0b101;
    assert!(!sys.mpu_permits(0x1000, Access::Read, true, 0));
    assert!(
        sys.mpu_permits(0x1100, Access::Read, true, 0),
        "the disabled eighth is background, and privileged code has the \
         default map"
    );
}

#[test]
fn the_private_peripheral_bus_is_reachable_from_guest_memory() {
    // `LDR.W r1, [r0]` against CPUID: the block is part of the processor, so
    // it answers whether or not the machine mapped anything there.
    let h = Harness::m4(&wide(0xf8d0_1000));
    h.cpu.set_reg(0, 0xe000_ed00);
    h.cpu.step();
    assert_eq!(h.cpu.reg(1), CPUID_CORTEX_M4);
}

// ---------------------------------------------------------------------------
// The device surface
// ---------------------------------------------------------------------------

#[test]
fn the_state_round_trips_through_a_snapshot() {
    let h = Harness::m4(&[0x2042]);
    h.cpu.step();
    h.cpu.set_irq(3, true);
    h.cpu.pend_irq(5);
    h.cpu.with_sys(|s| {
        s.vtor = 0x2000_0000;
        s.priority[20] = 0x40;
        s.cfsr = fsr::UF_UNDEFINSTR;
        s.mpu_rbar[2] = 0x1234_5600;
    });
    h.cpu.set_reg(9, 0x9999);

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("cpu", CLASS.name, CLASS.version).unwrap();
        Device::save(h.cpu.as_ref(), &mut chunk).unwrap();
    }
    let bytes = writer.to_vec().unwrap();

    let restored = ArmV7m::new(Config::CORTEX_M4);
    let reader = StateReader::new(&bytes).unwrap();
    let migrations = Migrations::new();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &migrations)
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    assert_eq!(restored.regs(), h.cpu.regs());
    assert_eq!(restored.cycles(), h.cpu.cycles());
    assert!(restored.irq_asserted(3));
    assert!(restored.with_sys(|s| s.is_pending(Exception(21))));
    assert_eq!(restored.vtor(), 0x2000_0000);
    assert_eq!(restored.with_sys(|s| s.priority[20]), 0x40);
    assert_eq!(restored.with_sys(|s| s.cfsr), fsr::UF_UNDEFINSTR);
    assert_eq!(restored.with_sys(|s| s.mpu_rbar[2]), 0x1234_5600);
}

#[test]
fn the_device_surface_is_wired_up() {
    let h = Harness::m4(&[0x2042, 0x2043]);
    assert!(Device::is_runnable(h.cpu.as_ref()));
    assert_eq!(h.cpu.class().name, "cpu.arm.v7m");
    let used = Device::run(
        h.cpu.as_ref(),
        crate::core::sched::Budget {
            until: crate::core::clock::GlobalTime::ZERO,
            ticks: 1,
        },
    );
    assert!(used.ticks > 0);
    Device::reset(h.cpu.as_ref(), crate::core::device::ResetKind::Cold);
    assert!(h.cpu.reset_pending());
    h.cpu.step();
    assert_eq!(h.cpu.pc(), ENTRY);
}

#[test]
fn from_props_names_a_part_and_rejects_a_typo() {
    let props = Props::new().with("part", Value::from("cortex-m7"));
    let cpu = ArmV7m::from_props(&props).unwrap();
    assert_eq!(cpu.config().cpuid, CPUID_CORTEX_M7);
    assert_eq!(cpu.config().priority_bits, 4);

    let props = Props::new().with("part", Value::from("cortex-m0"));
    assert!(ArmV7m::from_props(&props).is_err());

    // A property nobody accepts is nearly always a typo, and silently
    // ignoring it is how an afternoon disappears.
    let props = Props::new().with("prioritybits", Value::from(4u64));
    assert!(ArmV7m::from_props(&props).is_err());

    let props = Props::new()
        .with("part", Value::from("cortex-m4"))
        .with("dsp", Value::from(false));
    let cpu = ArmV7m::from_props(&props).unwrap();
    assert!(!cpu.config().ext.dsp);
}

#[test]
fn the_disassembler_walks_both_widths() {
    let mut code = Vec::new();
    code.push(0x2042);
    code.extend_from_slice(&wide(0xf8d0_1004));
    code.push(0x4770);
    let h = Harness::m4(&code);
    let listed = h.cpu.disassemble(ENTRY, 3);
    assert_eq!(listed.len(), 3);
    assert_eq!(listed[0].width, 2);
    assert_eq!(listed[1].width, 4);
    assert_eq!(listed[2].addr, ENTRY + 6);
    let text: Vec<String> = listed.iter().map(|l| format!("{}", l.insn)).collect();
    assert_eq!(text[0], "MOVS r0, #66");
    assert_eq!(text[1], "LDR r1, [r0, #4]");
    assert_eq!(text[2], "BX lr");
}

#[test]
fn a_debug_read_does_not_disturb_the_system_block() {
    let h = Harness::m4(&[0xbf00]);
    h.cpu.with_sys(|s| {
        s.syst_csr = 1 | (1 << 16);
    });
    // The disassembler reads with debug attributes; so does anything else
    // that must not have side effects (`ROADMAP.md` §15, invariant 5).
    let _ = h.cpu.disassemble(ENTRY, 1);
    assert_ne!(h.cpu.with_sys(|s| s.syst_csr) & (1 << 16), 0);
}

#[test]
fn an_interrupt_pin_drives_the_nvic() {
    use crate::core::wire::{Level, WireId, WireSink};
    let h = Harness::m4(&[0xbf00]);
    let src = WireId::new(1);
    let pin = InterruptPin::new(Arc::clone(&h.cpu), 3, &[src]);
    assert_eq!(pin.irq(), 3);
    pin.set_level(src, 0, Level::High);
    assert!(h.cpu.irq_asserted(3));
    pin.set_level(src, 0, Level::Low);
    assert!(!h.cpu.irq_asserted(3));
}

#[test]
fn a_level_input_re_pends_until_it_is_released() {
    let h = Harness::m4(&[0xbf00, 0xbf00, 0xbf00, 0xbf00]);
    h.set_word(16 * 4, 0x301);
    h.set_word(0x300, 0x4770); // the handler returns immediately
    h.cpu.with_sys(|s| s.set_enable(Exception::IRQ0, true));
    h.cpu.set_irq(0, true);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::IRQ0);
    h.cpu.step(); // `BX lr`: return, and the level re-pends
    h.cpu.step();
    assert_eq!(
        h.cpu.current_exception(),
        Exception::IRQ0,
        "a level that nobody cleared comes straight back"
    );
    h.cpu.set_irq(0, false);
    h.cpu.step();
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
}

#[test]
fn a_pended_interrupt_does_not_re_pend_on_its_own() {
    let h = Harness::m4(&[0xbf00, 0xbf00, 0xbf00]);
    h.set_word(16 * 4, 0x301);
    h.set_word(0x300, 0x4770);
    h.cpu.with_sys(|s| s.set_enable(Exception::IRQ0, true));
    h.cpu.pend_irq(0);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::IRQ0);
    h.cpu.step();
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
}

#[test]
fn a_bus_fault_names_the_address_it_could_not_reach() {
    // Nothing is mapped above RAM, and an unmapped access is an external
    // abort rather than a silent zero (`ROADMAP.md` §4.1).
    let h = Harness::m4(&wide(0xf8d0_1000));
    h.cpu.set_reg(0, 0x4000_0000);
    h.cpu.with_sys(|s| s.shcsr |= shcsr::BUSFAULTENA);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::BUS_FAULT);
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::BF_PRECISERR != 0));
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::BF_BFARVALID != 0));
    assert_eq!(h.cpu.with_sys(|s| s.bfar), 0x4000_0000);
    let (count, last) = h.cpu.bus_faults();
    assert_eq!(count, 1);
    assert_eq!(last, 0x4000_0000);
}

#[test]
fn a_fault_with_its_handler_disabled_escalates() {
    let h = Harness::m4(&[0xde00]);
    h.set_word(3 * 4, 0x301);
    h.set_word(0x300, 0xe7fe);
    // `USGFAULTENA` is clear out of reset, so this must land in HardFault.
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    assert!(h.cpu.with_sys(|s| s.hfsr & fsr::HF_FORCED != 0));
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_UNDEFINSTR != 0));
}

#[test]
fn access_width_reads_the_right_slice_of_a_ppb_word() {
    // `SHPR2` byte 3 is SVCall's priority: byte access to the priority
    // registers is the one sub-word access the architecture guarantees.
    let h = Harness::m4(&[0xbf00]);
    h.cpu.with_sys(|s| s.priority[11] = 0xc0);
    let space = h.cpu.space().unwrap();
    let _ = space;
    let mut sys = Sys::new(CPUID_CORTEX_M4, 8, 8);
    sys.priority[11] = 0xc0;
    assert_eq!(sys.read_word(0xe000_ed1c, false).unwrap() >> 24, 0xc0);
}

#[test]
fn a_part_without_an_mpu_says_so_and_permits_everything() {
    let cfg = Config {
        ext: Extensions {
            mpu: false,
            ..Config::CORTEX_M4.ext
        },
        ..Config::CORTEX_M4
    };
    let h = Harness::new(cfg, &wide(0xf8d0_1000));
    // `MPU_TYPE.DREGION` is how firmware discovers there is no MPU.
    h.cpu.set_reg(0, 0xe000_ed90);
    h.cpu.step();
    assert_eq!(h.cpu.reg(1) & 0xff00, 0);
    // Enabling it is write-ignored, and no access can be refused.
    h.cpu.with_sys(|s| {
        s.write_word(0xe000_ed94, 0b001);
        assert_eq!(s.mpu_ctrl, 0);
        assert!(s.mpu_permits(0x1000, Access::Write, false, 0));
    });
}

#[test]
fn the_configuration_never_claims_a_floating_point_unit() {
    // There is none, so `config()` must not say there is: firmware that
    // trusted it would take a `NOCP` UsageFault on its first `VMOV`.
    let cfg = Config {
        ext: Extensions {
            fp: true,
            ..Config::CORTEX_M4.ext
        },
        ..Config::CORTEX_M4
    };
    assert!(!ArmV7m::new(cfg).config().ext.fp);
}

#[test]
fn regs_display_names_the_mode_it_is_in() {
    let regs = Regs {
        xpsr: xpsr::T | xpsr::Z | 11,
        ..Regs::new()
    };
    let text = format!("{regs}");
    assert!(text.contains("nZcvq"), "{text}");
    assert!(text.contains("svcall"), "{text}");
}

#[test]
fn width_matches_the_fetch_the_decoder_would_ask_for() {
    assert_eq!(Insn::width_of(0x2042), 2);
    assert_eq!(Insn::width_of(0xf8d0), 4);
    assert_eq!(Width::U16.bytes(), 2);
}
