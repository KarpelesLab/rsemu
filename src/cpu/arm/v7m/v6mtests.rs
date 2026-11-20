//! Unit tests for the ARMv6-M half of the core (`super::v6m`).
//!
//! Every encoding below was checked against a real assembler (`llvm-mc
//! -triple=thumbv6m-none-eabi -mcpu=cortex-m0`, and `thumbv7m` for the
//! Cortex-M3 side, run as a black box) before it was written down — the
//! v6-M assembler refusing an encoding is the independent confirmation that
//! it is outside the v6-M set. Every expected behaviour comes from DDI 0419E
//! as `super::v6m` states it.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::Device;
use crate::core::props::{Props, Value};
use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

use super::isa::{Insn, decode, decode_for};
use super::sys::{Exception, control, exc_return, shcsr};
use super::v6m::{CPUID_CORTEX_M0, CPUID_CORTEX_M0PLUS, CPUID_SC000, permits_32};
use super::{Arch, ArmV7m, Config, xpsr};

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

const ENTRY: u32 = 0x200;
const HANDLER: u32 = 0x300;
const STACK: u32 = 0x1000;
/// An aligned scratch word in RAM, for the loads and stores below.
const DATA: u32 = 0x800;
const RAM: u64 = 0x4000;

/// `B .`
const PARK: u16 = 0xe7fe;
/// `BX lr`
const BX_LR: u16 = 0x4770;
/// `SVC #0`
const SVC: u16 = 0xdf00;
/// `NOP`
const NOP: u16 = 0xbf00;

struct Harness {
    cpu: Arc<ArmV7m>,
    ram: Arc<RamStore>,
}

impl Harness {
    fn new(cfg: Config, code: &[u16]) -> Harness {
        let ram = Arc::new(RamStore::new(RAM));
        ram.write_at(0, &STACK.to_le_bytes()).unwrap();
        ram.write_at(4, &(ENTRY | 1).to_le_bytes()).unwrap();
        ram.write_at(u64::from(HANDLER), &PARK.to_le_bytes())
            .unwrap();
        for n in 2..48u64 {
            ram.write_at(n * 4, &(HANDLER | 1).to_le_bytes()).unwrap();
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
        cpu.step();
        // Somewhere harmless for every load/store/divide below to point at.
        cpu.set_reg(1, DATA);
        cpu.set_reg(2, 3);
        Harness { cpu, ram }
    }

    fn m0(code: &[u16]) -> Harness {
        Harness::new(Config::CORTEX_M0, code)
    }

    fn m0plus(code: &[u16]) -> Harness {
        Harness::new(Config::CORTEX_M0PLUS, code)
    }

    fn sc000(code: &[u16]) -> Harness {
        Harness::new(Config::SC000, code)
    }

    fn handler(&self, code: &[u16]) {
        for (i, half) in code.iter().enumerate() {
            self.ram
                .write_at(u64::from(HANDLER) + (i as u64) * 2, &half.to_le_bytes())
                .unwrap();
        }
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

    /// A system-block register, read the way a debugger would.
    fn ppb_read(&self, addr: u32) -> u32 {
        self.cpu.with_sys(|s| s.read_word(addr, true).unwrap())
    }

    fn ppb_write(&self, addr: u32, value: u32) {
        assert!(self.cpu.with_sys(|s| s.write_word(addr, value)));
    }

    fn steps(&self, n: usize) {
        for _ in 0..n {
            self.cpu.step();
        }
    }
}

/// Split a thirty-two-bit encoding into the two halfwords a fetch sees.
const fn wide(encoding: u32) -> [u16; 2] {
    [(encoding >> 16) as u16, encoding as u16]
}

fn code(encodings: &[u32]) -> Vec<u16> {
    let mut out = Vec::new();
    for &e in encodings {
        if e > 0xffff {
            out.extend_from_slice(&wide(e));
        } else {
            out.push(e as u16);
        }
    }
    out
}

/// Encodings ARMv7-M (without DSP or an FPU) executes and ARMv6-M does not
/// have, each with what it is.
const V7M_ONLY: &[(u32, &str)] = &[
    (0xb100, "CBZ r0"),
    (0xb900, "CBNZ r0"),
    (0xbf08, "IT EQ"),
    (0xf241_2034, "MOVW r0, #0x1234"),
    (0xf2c5_6078, "MOVT r0, #0x5678"),
    (0xe851_0f00, "LDREX r0, [r1]"),
    (0xe841_0200, "STREX r2, r0, [r1]"),
    (0xfb91_f0f2, "SDIV r0, r1, r2"),
    (0xfbb1_f0f2, "UDIV r0, r1, r2"),
    (0xf000_b800, "B.W"),
    (0xf3bf_8f2f, "CLREX"),
    (0xf3af_8000, "NOP.W"),
    (0xf8d1_0004, "LDR.W r0, [r1, #4]"),
    (0xeb01_0002, "ADD.W r0, r1, r2"),
    (0xf301_0007, "SSAT r0, #8, r1"),
    (0xf3c1_1007, "UBFX r0, r1, #4, #8"),
    (0xfb01_f002, "MUL.W r0, r1, r2"),
    (0xe9d1_2300, "LDRD r2, r3, [r1]"),
];

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#[test]
fn the_presets_carry_the_trm_values() {
    let m0 = Config::CORTEX_M0;
    assert_eq!(m0.cpuid, 0x410c_c200);
    assert_eq!(CPUID_CORTEX_M0, 0x410c_c200);
    assert_eq!(m0.arch, Arch::V6m);
    assert!(m0.is_v6m());
    assert_eq!(m0.priority_bits, 2);
    assert_eq!(m0.vtor_mask, 0);
    assert!(!m0.bit_band);
    assert!(!m0.ext.dsp && !m0.ext.mpu && !m0.ext.unpriv);

    let plus = Config::CORTEX_M0PLUS;
    assert_eq!(plus.cpuid, 0x410c_c601);
    assert_eq!(CPUID_CORTEX_M0PLUS, 0x410c_c601);
    assert_eq!(plus.vtor_mask, 0xffff_ff00);
    assert!(plus.ext.mpu && plus.ext.unpriv);
    assert!(plus.is_v6m());

    let sc = Config::SC000;
    assert!(sc.is_v6m());
    assert_eq!(sc.cpuid, CPUID_SC000);
    // Implementer ARM, architecture ARMv6-M: the two fields DDI 0419E fixes.
    assert_eq!(sc.cpuid >> 24, 0x41);
    assert_eq!((sc.cpuid >> 16) & 0xf, 0xc);
    assert_eq!(sc.vtor_mask, 0xffff_ff80);
    // A PMSAv6 core: the MPU, and the extension PMSAv6 cannot be without.
    assert!(sc.ext.unpriv && sc.ext.mpu);

    assert!(!Config::CORTEX_M4.is_v6m());
    assert!(!Config::CORTEX_M3.is_v6m());
}

#[test]
fn new_holds_a_v6m_configuration_to_the_architecture() {
    let cfg = Config {
        priority_bits: 4,
        bit_band: true,
        ext: super::Extensions {
            dsp: true,
            ..Config::CORTEX_M0.ext
        },
        reset_vtor: 0x1000,
        ..Config::CORTEX_M0
    };
    let got = ArmV7m::new(cfg).config();
    assert_eq!(got.priority_bits, 2);
    assert!(!got.bit_band);
    assert!(!got.ext.dsp);
    // No VTOR, so nowhere to boot from but zero.
    assert_eq!(got.reset_vtor, 0);
    // PMSAv6 needs the unprivileged extension; without it there is no MPU.
    let cfg = Config {
        ext: super::Extensions {
            mpu: true,
            unpriv: false,
            ..Config::CORTEX_M0.ext
        },
        ..Config::CORTEX_M0
    };
    assert!(!ArmV7m::new(cfg).config().ext.mpu);
}

#[test]
fn from_props_names_every_v6m_part() {
    for (name, cpuid) in [
        ("cortex-m0", CPUID_CORTEX_M0),
        ("cortex-m0plus", CPUID_CORTEX_M0PLUS),
        ("cortex-m0+", CPUID_CORTEX_M0PLUS),
        ("sc000", CPUID_SC000),
    ] {
        let props = Props::new().with("part", Value::from(name));
        let cpu = ArmV7m::from_props(&props).unwrap();
        assert!(cpu.config().is_v6m(), "{name}");
        assert_eq!(cpu.config().cpuid, cpuid, "{name}");
        assert_eq!(cpu.config().priority_bits, 2, "{name}");
    }
    // A board that knows its SC000's real CPUID says so.
    let props = Props::new()
        .with("part", Value::from("sc000"))
        .with("cpuid", Value::from(0x4100_0000u64 | 0x000c_1234));
    let cpu = ArmV7m::from_props(&props).unwrap();
    assert_eq!(cpu.config().cpuid, 0x410c_1234);
}

#[test]
fn from_props_refuses_what_the_part_cannot_do() {
    // A Cortex-M0 has no VTOR.
    let props = Props::new()
        .with("part", Value::from("cortex-m0"))
        .with("vtor", Value::from(0x1000u64));
    assert!(ArmV7m::from_props(&props).is_err());
    // A Cortex-M0+'s VTOR holds bits [31:8], so 0x80 is not a table base.
    let props = Props::new()
        .with("part", Value::from("cortex-m0plus"))
        .with("vtor", Value::from(0x80u64));
    assert!(ArmV7m::from_props(&props).is_err());
    let props = Props::new()
        .with("part", Value::from("cortex-m0plus"))
        .with("vtor", Value::from(0x100u64));
    assert_eq!(ArmV7m::from_props(&props).unwrap().vtor(), 0x100);
    // Two priority bits, and no other number.
    let props = Props::new()
        .with("part", Value::from("cortex-m0"))
        .with("priority-bits", Value::from(3u64));
    assert!(ArmV7m::from_props(&props).is_err());
    let props = Props::new()
        .with("part", Value::from("cortex-m0"))
        .with("priority-bits", Value::from(2u64));
    assert!(ArmV7m::from_props(&props).is_ok());
}

// ---------------------------------------------------------------------------
// Decode gating
// ---------------------------------------------------------------------------

#[test]
fn every_v7m_only_encoding_is_a_hardfault_on_m0_and_runs_on_m3() {
    for &(encoding, name) in V7M_ONLY {
        let m0 = Harness::m0(&code(&[encoding, u32::from(PARK)]));
        m0.cpu.step();
        assert_eq!(
            m0.cpu.current_exception(),
            Exception::HARD_FAULT,
            "{name} must be UNDEFINED on a Cortex-M0"
        );
        assert_eq!(m0.cpu.pc(), HANDLER, "{name}");
        // The stacked return address is the faulting instruction.
        assert_eq!(m0.word(STACK - 0x20 + 0x18), ENTRY, "{name}");

        let m3 = Harness::new(Config::CORTEX_M3, &code(&[encoding, u32::from(PARK)]));
        m3.cpu.step();
        assert_eq!(
            m3.cpu.current_exception(),
            Exception::THREAD,
            "{name} must execute on a Cortex-M3"
        );
    }
}

#[test]
fn dsp_and_coprocessor_encodings_are_hardfaults_too() {
    for (encoding, name) in [
        (0xfa82_f081u32, "QADD r0, r1, r2"),
        (0xee00_0a10, "VMOV s0, r0"),
        (0xf7f0_a000, "UDF.W #0"),
        (0xde00, "UDF #0"),
    ] {
        let h = Harness::m0(&code(&[encoding]));
        h.cpu.step();
        assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT, "{name}");
    }
}

#[test]
fn the_permitted_32_bit_set_runs() {
    // BL to the next instruction, MSR/MRS PRIMASK, the three barriers.
    let h = Harness::m0(&code(&[
        0xf000_f800, // BL +0
        0x2001,      // MOVS r0, #1
        0xf380_8810, // MSR PRIMASK, r0
        0xf3ef_8310, // MRS r3, PRIMASK
        0xf3bf_8f5f, // DMB SY
        0xf3bf_8f4f, // DSB SY
        0xf3bf_8f6f, // ISB SY
        u32::from(PARK),
    ]));
    h.steps(7);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.reg(14), (ENTRY + 4) | 1, "BL set LR");
    assert!(h.cpu.regs().primask);
    assert_eq!(h.cpu.reg(3), 1);
    assert_eq!(h.cpu.pc(), ENTRY + 4 + 2 + 4 * 5);
    for encoding in [
        0xf000_f800u32,
        0xf380_8810,
        0xf3ef_8010,
        0xf3bf_8f5f,
        0xf3bf_8f4f,
        0xf3bf_8f6f,
        0xf7f0_a000,
    ] {
        let [a, b] = wide(encoding);
        assert!(permits_32(a, b), "{encoding:#010x}");
    }
}

#[test]
fn sixteen_bit_hints_still_decode_and_unallocated_ones_are_nops() {
    // YIELD, SEV, and a hint number nothing assigns (0xbf70): NOPs all.
    let h = Harness::m0(&[0xbf10, 0xbf40, 0xbf70, NOP, PARK]);
    h.steps(4);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.pc(), ENTRY + 8);
}

#[test]
fn the_disassembler_is_honest_about_the_architecture() {
    assert_eq!(decode_for(Arch::V6m, 0xb100, 0), Insn::Undefined);
    assert_eq!(decode_for(Arch::V6m, 0xbf08, 0), Insn::Undefined);
    assert!(matches!(decode_for(Arch::V7m, 0xb100, 0), Insn::Cbz { .. }));
    let [a, b] = wide(0xf241_2034);
    assert_eq!(decode_for(Arch::V6m, a, b), Insn::Undefined);
    assert_eq!(decode_for(Arch::V7m, a, b), decode(a, b));
    // Permitted encodings decode exactly as the shared table has them.
    let [a, b] = wide(0xf3ef_8010);
    assert_eq!(decode_for(Arch::V6m, a, b), decode(a, b));

    let h = Harness::m0(&code(&[0xb100, 0x2042]));
    let listing = h.cpu.disassemble(ENTRY, 2);
    assert_eq!(listing[0].insn, Insn::Undefined);
    assert!(!matches!(listing[1].insn, Insn::Undefined));
}

/// Every sixteen-bit halfword decodes on ARMv6-M to exactly what it decodes
/// to on ARMv7-M, except `CBZ`, `CBNZ` and `IT`, which are UNDEFINED there
/// (DDI 0419E A5.2: "Thumb instructions present in other architecture
/// variants are UNDEFINED in ARMv6-M"). Exhaustive: all 65,536 values.
#[test]
fn every_sixteen_bit_encoding_decodes_as_on_v7m_but_cbz_and_it() {
    let mut cut = 0usize;
    for raw in 0..=0xffffu16 {
        if super::isa::is_32bit(raw) {
            continue;
        }
        let v6 = decode_for(Arch::V6m, raw, 0);
        let v7 = decode_for(Arch::V7m, raw, 0);
        match v7 {
            Insn::Cbz { .. } | Insn::It { .. } => {
                assert_eq!(v6, Insn::Undefined, "{raw:04x} exists on ARMv6-M");
                cut += 1;
            }
            // Armv8-M's BXNS/BLXNS are a BX/BLX with should-be-zero bits
            // set on any other architecture.
            #[cfg(feature = "cpu-arm-v8m")]
            Insn::Bxns { link, rm } => {
                let plain = if link {
                    Insn::Blx { rm }
                } else {
                    Insn::Bx { rm }
                };
                assert_eq!(v6, plain, "{raw:04x}");
            }
            _ => assert_eq!(v6, v7, "{raw:04x} decodes differently on ARMv6-M"),
        }
    }
    // CBZ/CBNZ: op x i x imm5 x Rn = 2 * 2 * 32 * 8 = 1024 encodings; IT:
    // 16 conditions x 15 non-zero masks = 240. The count is the check that
    // the cut is exactly those and nothing besides.
    assert_eq!(cut, 1024 + 240);
}

/// Of the thirty-two-bit space, ARMv6-M has `BL`, `MSR`, `MRS`, `DSB`,
/// `DMB`, `ISB` and `UDF.W` (DDI 0419E Tables A5-9 to A5-11); every other
/// encoding is UNDEFINED, and the permitted ones decode exactly as on
/// ARMv7-M. Every first halfword against a fixed walk of second halfwords
/// plus a pseudo-random sample, and the whole second-halfword space behind
/// the system-instruction first halfwords.
///
/// The reference set here is written out again from the tables rather than
/// by calling `permits_32`, so a transcription error in one shows up as a
/// disagreement with the other; `conformance::v6m_encoding_set_matches_llvm`
/// is the third, independent, reading.
#[test]
fn every_thirty_two_bit_encoding_outside_the_v6m_set_is_undefined() {
    // DDI 0419E Table A5-10, read directly: the "branch and miscellaneous
    // control" class is hw1[15:11] = 11110 with hw2[15] = 1.
    fn in_v6m(first: u16, second: u16) -> bool {
        if first >> 11 != 0b11110 || second >> 15 != 1 {
            return false;
        }
        let op1 = (first >> 4) & 0x7f;
        let op2 = (second >> 12) & 0x7;
        match op2 {
            // BL: op2 = 1x1.
            0b101 | 0b111 => true,
            // op2 = 0x0: MSR (011100x), MRS (011111x), and Table A5-11
            // under 0111011 (DSB 0100, DMB 0101, ISB 0110 in hw2[7:4]).
            0b000 | 0b010 => match op1 {
                0b011_1000 | 0b011_1001 | 0b011_1110 | 0b011_1111 => true,
                0b011_1011 => matches!((second >> 4) & 0xf, 0b0100..=0b0110),
                // UDF.W: op1 = 1111111, op2 = 010.
                0b111_1111 => op2 == 0b010,
                _ => false,
            },
            _ => false,
        }
    }

    let check = |first: u16, second: u16| {
        let v6 = decode_for(Arch::V6m, first, second);
        if in_v6m(first, second) {
            assert!(permits_32(first, second), "{first:04x} {second:04x}");
            assert_eq!(v6, decode(first, second), "{first:04x} {second:04x}");
            assert_ne!(v6, Insn::Undefined, "{first:04x} {second:04x}");
        } else {
            assert!(!permits_32(first, second), "{first:04x} {second:04x}");
            assert_eq!(v6, Insn::Undefined, "{first:04x} {second:04x} exists");
        }
    };
    let mut seed = 0x9e37_79b9_7f4a_7c15u64;
    for first in 0xe800..=0xffffu16 {
        for op2 in 0..8u16 {
            for low in [
                0x0000u16, 0x0f4f, 0x0f5f, 0x0f6f, 0x0f2f, 0x0800, 0x0014, 0x0fff,
            ] {
                check(first, (op2 << 12) | low | 0x8000);
                check(first, (op2 << 12) | low);
            }
        }
        for _ in 0..16 {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            check(first, seed as u16);
        }
    }
    for first in [0xf3bfu16, 0xf3ef, 0xf380, 0xf381, 0xf7f0, 0xf7ff, 0xf000] {
        for second in 0..=0xffffu16 {
            check(first, second);
        }
    }
}

// ---------------------------------------------------------------------------
// The exception model
// ---------------------------------------------------------------------------

#[test]
fn every_fault_is_a_hardfault_and_the_status_registers_read_zero() {
    let h = Harness::m0(&code(&[0xb100]));
    // Software cannot enable the configurable faults: SHCSR's enables do not
    // exist on ARMv6-M.
    h.ppb_write(
        0xe000_ed24,
        shcsr::USGFAULTENA | shcsr::BUSFAULTENA | shcsr::MEMFAULTENA,
    );
    assert_eq!(h.ppb_read(0xe000_ed24), 0);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    assert_eq!(h.cpu.regs().r[14], exc_return::THREAD_MSP);
    // CFSR, HFSR, MMFAR, BFAR: reserved, so zero to the guest.
    for addr in [0xe000_ed28u32, 0xe000_ed2c, 0xe000_ed34, 0xe000_ed38] {
        assert_eq!(h.ppb_read(addr), 0, "{addr:#x}");
    }
}

#[test]
fn unaligned_accesses_always_fault() {
    // CCR reads 0x208 — STKALIGN and UNALIGN_TRP — and ignores writes.
    let h = Harness::m0(&[0x6808]); // LDR r0, [r1]
    assert_eq!(h.ppb_read(0xe000_ed14), 0x208);
    h.ppb_write(0xe000_ed14, 0);
    assert_eq!(h.ppb_read(0xe000_ed14), 0x208);
    h.cpu.set_reg(1, DATA + 2);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);

    let h = Harness::m0(&[0x8808]); // LDRH r0, [r1]
    h.cpu.set_reg(1, DATA + 1);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);

    // A byte access cannot be unaligned.
    let h = Harness::m0(&[0x7808, PARK]); // LDRB r0, [r1]
    h.cpu.set_reg(1, DATA + 1);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
}

#[test]
fn an_svc_that_cannot_preempt_escalates_and_one_in_hardfault_locks_up() {
    // CPSID i, then SVC: PRIMASK holds the priority at 0 and SVCall is at 0.
    let h = Harness::m0(&[0xb672, SVC]);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);

    // Inside HardFault an SVC has nowhere to go: lockup (Table B1-7).
    let h = Harness::m0(&[0xb100]);
    h.handler(&[SVC]);
    h.steps(2);
    assert!(h.cpu.is_locked_up());
    assert_eq!(h.cpu.pc(), 0xffff_fffe);

    // So does a BKPT.
    let h = Harness::m0(&[0xb100]);
    h.handler(&[0xbe00]);
    h.steps(2);
    assert!(h.cpu.is_locked_up());

    // And an ordinary SVC at Thread priority is taken.
    let h = Harness::m0(&[SVC]);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
}

#[test]
fn an_nmi_ends_a_lockup_in_hardfault_but_not_one_in_nmi() {
    // Lock up in HardFault (UDF, then an SVC in the handler).
    let h = Harness::m0(&[0xde00]);
    h.handler(&[SVC]);
    h.steps(2);
    assert!(h.cpu.is_locked_up());

    // "If locked up at priority -1 and an NMI exception occurs, the NMI is
    // activated as normal. The NMI return link is the address used for the
    // Lockup state" (DDI 0419E B1.5.15).
    h.cpu.set_nmi(true);
    h.cpu.step();
    h.cpu.set_nmi(false);
    assert!(!h.cpu.is_locked_up());
    assert_eq!(h.cpu.current_exception(), Exception::NMI);
    assert_eq!(h.cpu.pc(), HANDLER);
    let sp = h.cpu.regs().msp;
    assert_eq!(h.word(sp + 0x18), 0xffff_fffe);

    // The same SVC inside NMI locks up at -2, and nothing but a reset (or a
    // debugger) ends that.
    h.cpu.step();
    assert!(h.cpu.is_locked_up());
    h.cpu.set_nmi(true);
    h.steps(2);
    assert!(h.cpu.is_locked_up());
    assert_eq!(h.cpu.current_exception(), Exception::NMI);
}

/// Nothing is mapped here: the harness's space faults outside its RAM.
const NOWHERE: u32 = 0x0010_0000;

fn hardfault_pending(h: &Harness) -> bool {
    h.cpu.with_sys(|s| s.is_pending(Exception::HARD_FAULT))
}

#[test]
fn a_fault_on_the_stacking_writes_is_a_derived_hardfault_or_a_lockup() {
    // MSR MSP, r0 (nowhere); SVC. The frame is lost: from Thread priority
    // that is a pending HardFault, taken as soon as it can preempt
    // (DDI 0419E B1.5.11).
    let h = Harness::m0(&code(&[0xf380_8808, u32::from(SVC), u32::from(PARK)]));
    h.cpu.set_reg(0, NOWHERE);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
    assert!(hardfault_pending(&h));
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    assert!(!h.cpu.is_locked_up());

    // From HardFault priority — an NMI entered from a HardFault handler
    // that lost its stack — it is a lockup (B1.5.11, Table B1-7).
    let h = Harness::m0(&[0xde00]);
    h.handler(&code(&[0xf380_8808, u32::from(PARK)]));
    h.cpu.set_reg(0, NOWHERE);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    h.cpu.set_nmi(true);
    h.cpu.step();
    assert!(h.cpu.is_locked_up());
}

#[test]
fn a_fault_on_the_unstacking_reads_locks_up_from_msp_and_pends_from_psp() {
    // The handler points MSP at nothing and returns: "must use the Main
    // stack, SP_main, a lockup at HardFault priority occurs" (DDI 0419E
    // B1.5.8).
    let h = Harness::m0(&[SVC, PARK]);
    h.handler(&code(&[0xf380_8808, u32::from(BX_LR)]));
    h.cpu.set_reg(0, NOWHERE);
    h.steps(3);
    assert!(h.cpu.is_locked_up());
    assert_eq!(h.cpu.pc(), 0xffff_fffe);

    // Thread on the process stack (MSR PSP, r0; MSR CONTROL, r1; SVC), and
    // the handler points PSP at nothing (MSR PSP, r2): "a Pending HardFault
    // exception is generated".
    let h = Harness::m0(&code(&[
        0xf380_8809,
        0xf381_8814,
        u32::from(SVC),
        u32::from(PARK),
    ]));
    h.handler(&code(&[0xf382_8809, u32::from(BX_LR)]));
    h.cpu.set_reg(0, 0x0c00);
    h.cpu.set_reg(1, control::SPSEL);
    h.cpu.set_reg(2, NOWHERE);
    h.steps(5);
    assert!(!h.cpu.is_locked_up());
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert!(hardfault_pending(&h));
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
}

#[test]
fn an_exception_sets_the_event_register_so_wfe_does_not_sleep_through_it() {
    // SVC (standing in for an interrupt that arrives just before the
    // `WFE` of a `while (!flag) __WFE();` loop), then WFE. Exception entry
    // and return both SetEventRegister() (DDI 0419E B1.5.6, B1.5.8), so the
    // WFE consumes the event and falls through instead of sleeping.
    let h = Harness::m0(&[SVC, 0xbf20, PARK]);
    h.handler(&[BX_LR]);
    h.steps(2);
    assert_eq!(h.cpu.pc(), ENTRY + 2);
    h.cpu.step();
    assert!(!h.cpu.is_asleep());
    assert_eq!(h.cpu.pc(), ENTRY + 4);

    // With the event consumed, a second WFE sleeps.
    let h = Harness::m0(&[SVC, 0xbf20, 0xbf20, PARK]);
    h.handler(&[BX_LR]);
    h.steps(4);
    assert!(h.cpu.is_asleep());
}

#[test]
fn cmsis_priority_discovery_finds_two_bits() {
    // STR r0, [r1]; LDR r3, [r1] — through the guest's own word accesses,
    // as CMSIS's `NVIC_SetPriority` probe does.
    for (addr, expect) in [
        (0xe000_e400u32, 0xc0c0_c0c0u32), // NVIC_IPR0
        (0xe000_e41c, 0xc0c0_c0c0),       // NVIC_IPR7
        (0xe000_e420, 0),                 // IPR8: past the 32 interrupts
        (0xe000_ed1c, 0xc000_0000),       // SHPR2: SVCall only
        (0xe000_ed20, 0xc0c0_0000),       // SHPR3: PendSV, SysTick
        (0xe000_ed18, 0),                 // SHPR1: reserved
    ] {
        let h = Harness::m0(&[0x6008, 0x680b, PARK]);
        h.cpu.set_reg(0, 0xffff_ffff);
        h.cpu.set_reg(1, addr);
        h.steps(2);
        assert_eq!(h.cpu.current_exception(), Exception::THREAD, "{addr:#x}");
        assert_eq!(h.cpu.reg(3), expect, "{addr:#x}");
    }
}

#[test]
fn the_v6m_register_subset() {
    let h = Harness::m0(&[PARK]);
    // 32 interrupts: one word of each NVIC bitmap.
    h.ppb_write(0xe000_e100, 0xffff_ffff);
    assert_eq!(h.ppb_read(0xe000_e100), 0xffff_ffff);
    h.ppb_write(0xe000_e104, 0xffff_ffff);
    assert_eq!(h.ppb_read(0xe000_e104), 0);
    // No IABR, no STIR, no ICTR, no CPACR, no DEMCR, no DWT cycle counter.
    h.ppb_write(0xe000_ef00, 3);
    assert!(!h.cpu.with_sys(|s| s.is_pending(Exception(19))));
    for addr in [
        0xe000_e300u32,
        0xe000_e004,
        0xe000_ed88,
        0xe000_edfc,
        0xe000_1000,
    ] {
        h.ppb_write(addr, 0xffff_ffff);
        assert_eq!(h.ppb_read(addr), 0, "{addr:#x}");
    }
    // AIRCR has no PRIGROUP.
    h.ppb_write(0xe000_ed0c, 0x05fa_0700);
    assert_eq!(h.ppb_read(0xe000_ed0c) & 0x700, 0);
    // No VTOR on a Cortex-M0.
    h.ppb_write(0xe000_ed08, 0xffff_ffff);
    assert_eq!(h.ppb_read(0xe000_ed08), 0);
    // No MPU on a Cortex-M0.
    assert_eq!(h.ppb_read(0xe000_ed90), 0);
    // ICSR: no RETTOBASE.
    assert_eq!(h.ppb_read(0xe000_ed04) & (1 << 11), 0);
    // SHCSR.SVCALLPENDED is the one bit there.
    h.ppb_write(0xe000_ed24, 1 << 15);
    assert_eq!(h.ppb_read(0xe000_ed24), 1 << 15);
    assert!(h.cpu.with_sys(|s| s.is_pending(Exception::SVCALL)));

    let plus = Harness::m0plus(&[PARK]);
    // A Cortex-M0+ VTOR implements bits [31:8].
    plus.ppb_write(0xe000_ed08, 0xffff_ffff);
    assert_eq!(plus.ppb_read(0xe000_ed08), 0xffff_ff00);
    // Eight PMSAv6 regions, and no RBAR/RASR aliases.
    assert_eq!(plus.ppb_read(0xe000_ed90), 8 << 8);
    plus.ppb_write(0xe000_eda4, 0xffff_ffff);
    assert_eq!(plus.ppb_read(0xe000_eda4), 0);
    // RBAR.ADDR is bits [31:8].
    plus.ppb_write(0xe000_ed9c, 0x2000_00f0 | (1 << 4) | 3);
    assert_eq!(plus.ppb_read(0xe000_ed9c), 0x2000_0003);
    assert_eq!(plus.ppb_read(0xe000_ed98), 3);
}

#[test]
fn aircr_reports_a_big_endian_build() {
    // LDR r3, [r1] at AIRCR. ENDIANNESS is bit 15 (DDI 0419E B3.2.6); the
    // Cortex-M0 TRM's Table 4-1 gives 0x00008000 for a big-endian build.
    let cfg = Config {
        endian: crate::core::value::Endian::Big,
        ..Config::CORTEX_M0
    };
    let h = Harness::new(cfg, &[0x680b, PARK]);
    // Data, and so the vector table, is big-endian on this build: put the
    // reset vectors back the way it reads them and reset.
    h.ram.write_at(0, &STACK.to_be_bytes()).unwrap();
    h.ram.write_at(4, &(ENTRY | 1).to_be_bytes()).unwrap();
    h.cpu.reset(crate::core::device::ResetKind::Warm);
    h.cpu.step();
    h.cpu.set_reg(1, 0xe000_ed0c);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.reg(3) & 0xffff, 0x8000);

    let h = Harness::m0(&[0x680b, PARK]);
    h.cpu.set_reg(1, 0xe000_ed0c);
    h.cpu.step();
    assert_eq!(h.cpu.reg(3) & 0xffff, 0);
}

#[test]
fn a_cortex_m0_boots_from_zero_whatever_it_is_told() {
    let h = Harness::m0(&[PARK]);
    h.cpu.set_vtor(0x2000);
    assert_eq!(h.cpu.vtor(), 0);
    h.cpu.request_reset();
    h.cpu.step();
    assert_eq!(h.cpu.pc(), ENTRY);
}

#[test]
fn special_registers_are_the_v6m_subset() {
    // MSR BASEPRI, r0; MRS r3, FAULTMASK; CPSID f; MSR APSR, r0;
    // MRS r4, CONTROL — then MSR CONTROL, r0 with nPRIV and SPSEL set.
    let h = Harness::m0(&code(&[
        0xf380_8811, // MSR BASEPRI, r0
        0xf3ef_8313, // MRS r3, FAULTMASK
        0xb671,      // CPSID f
        0xf380_8800, // MSR APSR, r0
        0xf380_8814, // MSR CONTROL, r0
        u32::from(PARK),
    ]));
    h.cpu.set_reg(0, 0xffff_ffff);
    h.cpu.set_reg(3, 0x1234);
    h.steps(5);
    let regs = h.cpu.regs();
    assert_eq!(regs.basepri, 0);
    assert!(!regs.faultmask);
    assert_eq!(h.cpu.reg(3), 0);
    // N, Z, C, V set; Q and GE are not ARMv6-M's.
    assert_eq!(
        regs.xpsr & (xpsr::N | xpsr::Z | xpsr::C | xpsr::V),
        0xf000_0000
    );
    assert_eq!(regs.xpsr & (xpsr::Q | xpsr::GE), 0);
    // A Cortex-M0 has no nPRIV; SPSEL took.
    assert_eq!(regs.control, control::SPSEL);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);

    // A Cortex-M0+ with the unprivileged extension can drop privilege.
    let h = Harness::m0plus(&code(&[0xf380_8814, u32::from(PARK)]));
    h.cpu.set_reg(0, control::NPRIV);
    h.cpu.step();
    assert_eq!(h.cpu.regs().control, control::NPRIV);

    // And then reads zero for both stack pointers: "An unprivileged read of
    // any register other than APSR or CONTROL returns 0" (DDI 0419E
    // B4.2.2). MRS r3, MSP; MRS r4, PSP; MRS r5, CONTROL.
    let h = Harness::m0plus(&code(&[
        0xf380_8814,
        0xf3ef_8308,
        0xf3ef_8409,
        0xf3ef_8514,
        u32::from(PARK),
    ]));
    h.cpu.set_reg(0, control::NPRIV);
    h.cpu.set_reg(3, 0x5555);
    h.cpu.set_reg(4, 0x5555);
    h.steps(4);
    assert_eq!(h.cpu.reg(3), 0);
    assert_eq!(h.cpu.reg(4), 0);
    assert_eq!(h.cpu.reg(5), control::NPRIV);
}

#[test]
fn unprivileged_ppb_access_is_a_hardfault() {
    // MSR CONTROL, r0 (nPRIV); LDR r3, [r1] with r1 at CPUID.
    let h = Harness::m0plus(&code(&[0xf380_8814, 0x680b, u32::from(PARK)]));
    h.cpu.set_reg(0, control::NPRIV);
    h.cpu.set_reg(1, 0xe000_ed00);
    h.cpu.set_reg(3, 0x5555);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    assert_eq!(h.cpu.reg(3), 0x5555, "the access is not made");

    // Privileged, it reads the CPUID.
    let h = Harness::m0plus(&[0x680b, PARK]);
    h.cpu.set_reg(1, 0xe000_ed00);
    h.cpu.step();
    assert_eq!(h.cpu.reg(3), CPUID_CORTEX_M0PLUS);
}

#[test]
fn unprivileged_access_to_the_boards_ppb_window_is_a_hardfault_too() {
    // "The general rules for the PPB, address range 0xE0000000 to
    // 0xE0100000 ... Unprivileged accesses to the PPB generate a HardFault
    // error, without causing a PPB access" (DDI 0419E B3.1.1) — including
    // the part of it a board decodes.
    const VENDOR: u32 = 0xe004_2000;
    let run = |privileged: bool| {
        let h = Harness::m0plus(&code(&[0xf380_8814, 0x680b, u32::from(PARK)]));
        let vendor = Arc::new(RamStore::new(0x1000));
        vendor.write_at(0, &0xcafe_f00du32.to_le_bytes()).unwrap();
        let space = AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&h.ram)), 0)
            .unwrap();
        space
            .topology()
            .map(Region::ram("vendor", vendor), u64::from(VENDOR))
            .unwrap();
        h.cpu.attach_space(Arc::new(space));
        h.cpu
            .set_reg(0, if privileged { 0 } else { control::NPRIV });
        h.cpu.set_reg(1, VENDOR);
        h.cpu.set_reg(3, 0x5555);
        h.steps(2);
        (h.cpu.current_exception(), h.cpu.reg(3))
    };
    assert_eq!(run(false), (Exception::HARD_FAULT, 0x5555));
    assert_eq!(run(true), (Exception::THREAD, 0xcafe_f00d));
}

#[test]
fn an_mpu_violation_is_a_hardfault() {
    // Region 0: the low 4 KiB — code, vectors and stack — with full access.
    // Region 1, which wins where they overlap: 256 bytes at DATA,
    // privileged-only (AP = 001). The MPU on, with the background region for
    // privileged code.
    let h = Harness::m0plus(&code(&[0xf380_8814, 0x680b, u32::from(PARK)]));
    h.ppb_write(0xe000_ed9c, 1 << 4);
    h.ppb_write(0xe000_eda0, (0b011 << 24) | (11 << 1) | 1);
    h.ppb_write(0xe000_ed9c, DATA | (1 << 4) | 1);
    h.ppb_write(0xe000_eda0, (0b001 << 24) | (7 << 1) | 1);
    h.ppb_write(0xe000_ed94, 0b101);
    h.cpu.set_reg(0, control::NPRIV);
    h.cpu.set_reg(1, DATA);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);

    // The same load from privileged code is permitted: it was the region's
    // AP that refused it, not anything else.
    let h = Harness::m0plus(&code(&[0x680b, u32::from(PARK)]));
    h.ppb_write(0xe000_ed9c, 1 << 4);
    h.ppb_write(0xe000_eda0, (0b011 << 24) | (11 << 1) | 1);
    h.ppb_write(0xe000_ed9c, DATA | (1 << 4) | 1);
    h.ppb_write(0xe000_eda0, (0b001 << 24) | (7 << 1) | 1);
    h.ppb_write(0xe000_ed94, 0b101);
    h.set_word(DATA, 0xfeed_f00d);
    h.cpu.set_reg(1, DATA);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.reg(3), 0xfeed_f00d);
}

#[test]
fn the_sc000_is_a_pmsav6_core_with_a_vtor() {
    let h = Harness::sc000(&[PARK]);
    // VTOR at 0xE000ED08, bits [31:7].
    h.ppb_write(0xe000_ed08, 0xffff_ffff);
    assert_eq!(h.ppb_read(0xe000_ed08), 0xffff_ff80);
    // MPU_TYPE: eight unified regions (DDI 0419E B3.5.5).
    assert_eq!(h.ppb_read(0xe000_ed90), 8 << 8);

    // An unprivileged load from a privileged-only region is refused by the
    // MPU and is a HardFault — the same set-up as the Cortex-M0+ test above.
    let h = Harness::sc000(&code(&[0xf380_8814, 0x680b, u32::from(PARK)]));
    h.ppb_write(0xe000_ed9c, 1 << 4);
    h.ppb_write(0xe000_eda0, (0b011 << 24) | (11 << 1) | 1);
    h.ppb_write(0xe000_ed9c, DATA | (1 << 4) | 1);
    h.ppb_write(0xe000_eda0, (0b001 << 24) | (7 << 1) | 1);
    h.ppb_write(0xe000_ed94, 0b101);
    h.cpu.set_reg(0, control::NPRIV);
    h.cpu.set_reg(1, DATA);
    h.cpu.set_reg(3, 0x5555);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    assert_eq!(h.cpu.reg(3), 0x5555, "the access is not made");

    // And unprivileged code cannot reach the PPB to switch the MPU off.
    let h = Harness::sc000(&code(&[0xf380_8814, 0x680b, u32::from(PARK)]));
    h.cpu.set_reg(0, control::NPRIV);
    h.cpu.set_reg(1, 0xe000_ed94);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
}

#[test]
fn exception_return_takes_only_the_v6m_values_and_the_v6m_xpsr() {
    // An SVC whose handler returns: the round trip works.
    let h = Harness::m0(&[SVC, PARK]);
    h.handler(&[BX_LR]);
    h.steps(3);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.pc(), ENTRY + 2);

    // An extended-frame EXC_RETURN is not a v6-M value: HardFault.
    let h = Harness::m0(&[SVC, PARK]);
    h.handler(&[0x4700 | (2 << 3)]); // BX r2
    h.cpu.step();
    h.cpu.set_reg(2, exc_return::THREAD_MSP_FP);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);

    // A frame whose stacked xPSR carries Q, GE and ITSTATE gets none of
    // them back.
    let h = Harness::m0(&[SVC, PARK]);
    h.handler(&[BX_LR]);
    h.cpu.step();
    let sp = h.cpu.regs().msp;
    let forged = xpsr::T | xpsr::Q | xpsr::GE | (0x3f << 10) | xpsr::N;
    h.set_word(sp + 0x1c, forged);
    h.cpu.step();
    let x = h.cpu.xpsr();
    assert_eq!(x, xpsr::T | xpsr::N);
}

#[test]
fn a_return_to_thread_takes_no_ipsr_from_the_frame() {
    // MSR CONTROL, r0 (nPRIV), then SVC. The handler returns to
    // unprivileged Thread mode through a frame whose stacked IPSR claims
    // SVCall: `PopStack` forces IPSR to zero there (DDI 0419E B1.5.8), so
    // the forged number must not put the core in Handler mode — which
    // would also make it privileged.
    let h = Harness::m0plus(&code(&[0xf380_8814, u32::from(SVC), u32::from(PARK)]));
    h.handler(&[BX_LR]);
    h.cpu.set_reg(0, control::NPRIV);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
    let sp = h.cpu.regs().msp;
    h.set_word(sp + 0x1c, xpsr::T | u32::from(Exception::SVCALL.0));
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.xpsr() & xpsr::EXCEPTION, 0);
    assert_eq!(h.cpu.regs().control & control::NPRIV, control::NPRIV);
    assert_eq!(h.cpu.pc(), ENTRY + 6);
}

#[test]
fn a_return_to_handler_restores_six_bits_of_ipsr() {
    // SVC, and inside SVCall a second, PendSV, pended through ICSR: PendSV
    // returns to SVCall through a frame whose stacked xPSR has bits [8:6]
    // set. ARMv6-M's IPSR is bits [5:0] (DDI 0419E B1.5.8 `PopStack`).
    let h = Harness::m0(&[SVC, PARK]);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
    // Lower SVCall's priority so PendSV (priority 0) can preempt it.
    h.ppb_write(0xe000_ed1c, 0xc000_0000);
    h.ppb_write(0xe000_ed04, 1 << 28);
    h.handler(&[BX_LR]);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::PEND_SV);
    let sp = h.cpu.regs().msp;
    h.set_word(sp + 0x1c, xpsr::T | 0x1c0 | u32::from(Exception::SVCALL.0));
    h.cpu.step();
    assert_eq!(
        h.cpu.xpsr() & xpsr::EXCEPTION,
        u32::from(Exception::SVCALL.0)
    );
}

#[test]
fn icsr_vectactive_reports_the_running_exception() {
    // A guest read of `ICSR` (0xE000ED04) must report `VECTACTIVE` — the
    // exception number currently executing (DDI 0419E B3.2.4). Inside the
    // SVCall handler that is 11. The value is the core's `IPSR`, not held in
    // the system block, so it is injected on the PPB read path.
    let h = Harness::m0(&[SVC, PARK]);
    h.handler(&[0x6808, BX_LR]); // LDR r0,[r1] ; BX lr
    h.cpu.set_reg(1, 0xe000_ed04); // r1 -> ICSR
    h.cpu.step(); // SVC -> SVCall entry
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
    h.cpu.step(); // r0 = *ICSR
    assert_eq!(
        h.cpu.reg(0) & 0x1ff,
        u32::from(Exception::SVCALL.0),
        "ICSR.VECTACTIVE should be the running exception number"
    );
}

#[test]
fn bl_and_the_system_instructions_take_cortex_m0_cycles() {
    // BL is 4 on a Cortex-M0 (TRM Table 3-1) and 3 in the shared model.
    let m0 = Harness::m0(&code(&[0xf000_f800, u32::from(PARK)]));
    let before = m0.cpu.cycles();
    m0.cpu.step();
    assert_eq!(m0.cpu.cycles() - before, 4);

    let m3 = Harness::new(Config::CORTEX_M3, &code(&[0xf000_f800, u32::from(PARK)]));
    let before = m3.cpu.cycles();
    m3.cpu.step();
    assert_eq!(m3.cpu.cycles() - before, 3);

    // DMB is 4; MOVS is 1.
    let m0 = Harness::m0(&code(&[0xf3bf_8f5f, 0x2001, u32::from(PARK)]));
    let before = m0.cpu.cycles();
    m0.cpu.step();
    assert_eq!(m0.cpu.cycles() - before, 4);
    let before = m0.cpu.cycles();
    m0.cpu.step();
    assert_eq!(m0.cpu.cycles() - before, 1);

    // WFI and WFE are 2, not counting the sleep (Table 3-1, note f); SEV
    // is 1. WFE with the event register set does not sleep at all.
    for (insn, expect) in [(0xbf30u16, 2u64), (0xbf20, 2), (0xbf40, 1)] {
        let m0 = Harness::m0(&[0xbf40, insn, PARK]);
        m0.cpu.step();
        let before = m0.cpu.cycles();
        m0.cpu.step();
        assert_eq!(m0.cpu.cycles() - before, expect, "{insn:#06x}");
    }
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

fn snapshot(cpu: &ArmV7m) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("cpu", super::CLASS.name).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer
            .chunk("cpu", super::CLASS.name, super::CLASS.version)
            .unwrap();
        Device::save(cpu, &mut chunk).unwrap();
    }
    writer.to_vec().unwrap()
}

fn restore(cpu: &ArmV7m, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let migrations = Migrations::new();
    let chunk = reader
        .load("cpu", super::CLASS.name, super::CLASS.version, &migrations)
        .unwrap();
    Device::load(cpu, &mut chunk.reader()).unwrap();
}

#[test]
fn the_v6m_state_round_trips_through_a_snapshot() {
    let h = Harness::m0plus(&[SVC, PARK]);
    h.cpu.step();
    h.ppb_write(0xe000_ed08, 0x0000_0400);
    h.ppb_write(0xe000_e400, 0x4080_c000);
    h.ppb_write(0xe000_ed20, 0x8040_0000);
    h.ppb_write(0xe000_ed9c, 0x2000_0000 | (1 << 4) | 2);
    h.ppb_write(0xe000_eda0, (0b011 << 24) | (9 << 1) | 1);
    h.ppb_write(0xe000_e010, 0b11);
    h.ppb_write(0xe000_e014, 0x1234);
    h.cpu.set_irq(3, true);
    let bytes = snapshot(&h.cpu);

    let fresh = Harness::m0plus(&[PARK]);
    restore(&fresh.cpu, &bytes);
    assert_eq!(fresh.cpu.current_exception(), Exception::SVCALL);
    assert_eq!(fresh.cpu.vtor(), 0x400);
    assert_eq!(fresh.ppb_read(0xe000_e400), 0x4080_c000);
    assert_eq!(fresh.ppb_read(0xe000_ed20), 0x8040_0000);
    assert_eq!(fresh.ppb_read(0xe000_ed9c), 0x2000_0002);
    assert!(fresh.cpu.irq_asserted(3));
    assert_eq!(snapshot(&fresh.cpu), bytes, "an identical state hash");
}
