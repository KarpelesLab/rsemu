//! Unit tests for the FPv4-SP / FPv5-SP extension.
//!
//! Three kinds, deliberately separated:
//!
//! 1. **Mechanism tests** — one decode rule, one exception sequence, one
//!    `FPSCR` bit — in the [`Harness`] style `super::tests` uses.
//! 2. **A ledger of expected results** ([`LEDGER`]): a table of
//!    `(operation, operands, FPSCR mode bits) -> (result bits, cumulative
//!    flags)`, each row taken from DDI 0403E's pseudocode and IEEE 754-2019
//!    and checked in by hand. `ROADMAP.md` §0: accuracy is measured, never
//!    asserted — and for floating point the measurement is a table, because
//!    the ARMv5TE core the differential harness usually runs against has no
//!    VFP to be an oracle.
//! 3. **A differential against the A64 core's floating-point wrapper**, where
//!    that feature is compiled in. Both wrappers sit on the same
//!    [`crate::float`], but they were written separately from two different
//!    manuals, and the Arm-specific rules — `FPProcessNaNs3`'s order,
//!    `FPMulAdd`'s `∞ × 0` override, `FPMaxNum`'s substitution, the four-way
//!    compare — are exactly where two transcriptions of the same pseudocode
//!    drift apart. That makes it a real oracle for the paperwork even though
//!    it is not one for the arithmetic.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::Device;
use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::float::{Env, Flags, Round};

use super::fp::{self, fpscr};
use super::fpisa;
use super::isa::{Insn, decode};
use super::sys::{Exception, exc_return, fpccr, fsr};
use super::{ArmV7m, Config, FpUnit};

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Where the vector table goes.
const VECTORS: u32 = 0;
/// Where the harness starts executing.
const ENTRY: u32 = 0x200;
/// Where an exception handler the test installs goes.
const HANDLER: u32 = 0x300;
/// The initial stack pointer. Eight-byte aligned, so no frame ever needs the
/// alignment pad and a size assertion can be exact.
const STACK: u32 = 0x1000;
/// How much RAM a harness gets.
const RAM: u64 = 0x4000;

/// A core with RAM, a vector table, and code at [`ENTRY`].
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
        // Every other vector parks at a `B .` unless the test overwrites it.
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
        cpu.step();
        Harness { cpu, ram }
    }

    /// A Cortex-M4F with the coprocessor already enabled, which is the state
    /// every test that is not *about* `CPACR` wants to start in.
    fn m4f(code: &[u16]) -> Harness {
        let h = Harness::new(Config::CORTEX_M4F, code);
        h.enable_fp();
        h
    }

    /// A Cortex-M7 with the FPv5 single-precision unit, coprocessor enabled.
    fn m7f(code: &[u16]) -> Harness {
        let h = Harness::new(Config::CORTEX_M7F, code);
        h.enable_fp();
        h
    }

    /// `CPACR.CP10 = CP11 = 0b11`, which is what a C runtime's `SystemInit`
    /// writes before the first `VMOV`.
    fn enable_fp(&self) {
        self.cpu.with_sys(|s| s.cpacr = 0xf << 20);
    }

    /// Enable the UsageFault handler, so a fault vectors to *it* rather than
    /// escalating to a HardFault and hiding which fault it was.
    fn enable_usage_fault(&self) {
        self.cpu
            .with_sys(|s| s.shcsr |= super::sys::shcsr::USGFAULTENA);
    }

    /// Install a handler for every exception the harness vectors.
    fn handler(&self, code: &[u16]) {
        for (i, half) in code.iter().enumerate() {
            self.ram
                .write_at(u64::from(HANDLER) + (i as u64) * 2, &half.to_le_bytes())
                .unwrap();
        }
        for n in 2..48u64 {
            self.ram
                .write_at(n * 4, &(HANDLER | 1).to_le_bytes())
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

/// Concatenate wide encodings into a halfword stream.
fn program(words: &[u32]) -> Vec<u16> {
    let mut out = Vec::new();
    for w in words {
        out.extend_from_slice(&wide(*w));
    }
    out
}

// The encodings the tests below use, spelled once. Every one is from DDI
// 0403E A7.7's field layout for that instruction; the comment is what an
// assembler would have been given.
/// `VMOV S0, R0`.
const VMOV_S0_R0: u32 = 0xee00_0a10;
/// `VMOV S1, R1`.
const VMOV_S1_R1: u32 = 0xee00_1a90;
/// `VMOV R0, S2`.
const VMOV_R0_S2: u32 = 0xee11_0a10;
/// `VMOV R0, S0`.
const VMOV_R0_S0: u32 = 0xee10_0a10;
/// `VADD.F32 S2, S0, S1`.
const VADD_S2_S0_S1: u32 = 0xee30_1a20;
/// `VSUB.F32 S2, S0, S1`.
const VSUB_S2_S0_S1: u32 = 0xee30_1a60;
/// `VMUL.F32 S2, S0, S1`.
const VMUL_S2_S0_S1: u32 = 0xee20_1a20;
/// `VDIV.F32 S2, S0, S1`.
const VDIV_S2_S0_S1: u32 = 0xee80_1a20;
/// `VMLA.F32 S2, S0, S1`.
const VMLA_S2_S0_S1: u32 = 0xee00_1a20;
/// `VFMA.F32 S2, S0, S1`.
const VFMA_S2_S0_S1: u32 = 0xeea0_1a20;
/// `VCVT.S32.F32 S2, S0` — round toward zero.
const VCVT_S32_S2_S0: u32 = 0xeebd_1ac0;
/// `VCMP.F32 S0, S1`.
const VCMP_S0_S1: u32 = 0xeeb4_0a60;
/// `VMRS APSR_nzcv, FPSCR`.
const VMRS_APSR: u32 = 0xeef1_fa10;
/// `VMRS R0, FPSCR`.
const VMRS_R0: u32 = 0xeef1_0a10;
/// `VMSR FPSCR, R0`.
const VMSR_R0: u32 = 0xeee1_0a10;
/// `VMOV.F32 S0, #1.0`.
const VMOV_S0_ONE: u32 = 0xeeb7_0a00;
/// `VLDR S0, [R0]`.
const VLDR_S0_R0: u32 = 0xed90_0a00;
/// `VSTR S0, [R0]`.
const VSTR_S0_R0: u32 = 0xed80_0a00;
/// `VPUSH {S0-S15}`.
const VPUSH_S0_S15: u32 = 0xed2d_0a10;
/// `VPOP {S0-S15}`.
const VPOP_S0_S15: u32 = 0xecbd_0a10;
/// `VMAXNM.F32 S2, S0, S1` — FPv5.
const VMAXNM_S2_S0_S1: u32 = 0xfe80_1a20;

/// `SVC #0`.
const SVC0: u16 = 0xdf00;
/// `B .`.
const PARK: u16 = 0xe7fe;
/// `BX LR`.
const BX_LR: u16 = 0x4770;

// ---------------------------------------------------------------------------
// CPACR gating
// ---------------------------------------------------------------------------

#[test]
fn a_vfp_instruction_is_nocp_until_cpacr_enables_the_coprocessor() {
    // `CPACR` is zero out of reset even on a part that has the unit, so the
    // very first `VMOV` faults — which is how a C runtime that forgot its
    // `SystemInit` fails, and it must fail exactly this way.
    let h = Harness::new(Config::CORTEX_M4F, &wide(VMOV_S0_R0));
    h.enable_usage_fault();
    h.cpu.step();
    assert_eq!(h.cpu.xpsr() & 0x1ff, u32::from(Exception::USAGE_FAULT.0));
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_NOCP != 0));
}

#[test]
fn enabling_cpacr_from_guest_code_lets_the_next_instruction_run() {
    // The sequence a C runtime's `SystemInit` runs, executed rather than
    // poked: build `0xE000ED88` with `MOVW`/`MOVT`, `0x00F00000` with a
    // modified immediate, store, then use the FPU.
    let mut halves = Vec::new();
    halves.extend_from_slice(&wide(0xf64e_5088)); // MOVW r0, #0xED88
    halves.extend_from_slice(&wide(0xf2ce_0000)); // MOVT r0, #0xE000
    halves.extend_from_slice(&wide(0xf44f_0170)); // MOV.W r1, #0x00F00000
    halves.extend_from_slice(&wide(0xf8c0_1000)); // STR.W r1, [r0]
    halves.extend_from_slice(&wide(VMOV_S0_R0));
    halves.push(PARK);
    let h = Harness::new(Config::CORTEX_M4F, &halves);
    h.steps(4);
    assert_eq!(
        h.cpu.with_sys(|s| s.cpacr),
        0x00f0_0000,
        "CPACR.CP10 and CP11 both full access"
    );
    h.cpu.set_reg(0, 0x4213_3700);
    h.steps(1);
    assert_eq!(h.cpu.xpsr() & 0x1ff, 0, "still in Thread mode: no fault");
    assert_eq!(h.cpu.s(0), 0x4213_3700);
}

#[test]
fn cpacr_is_raz_wi_on_a_part_with_no_floating_point_unit() {
    // A plain Cortex-M4: writing `CPACR` must not make the coprocessor
    // appear, because firmware probes for the FPU exactly this way.
    let h = Harness::new(Config::CORTEX_M4, &wide(VMOV_S0_R0));
    h.cpu.with_sys(|s| {
        s.write_word(0xe000_ed88, 0x00f0_0000);
    });
    assert_eq!(h.cpu.with_sys(|s| s.cpacr), 0);
    assert_eq!(h.cpu.with_sys(|s| s.read_word(0xe000_ef40, false)), Some(0));
    h.cpu.step();
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_NOCP != 0));
}

#[test]
fn the_feature_registers_name_the_part() {
    let m4f = Harness::new(Config::CORTEX_M4F, &[PARK]);
    assert_eq!(
        m4f.cpu.with_sys(|s| s.read_word(0xe000_ef40, false)),
        Some(0x1011_0021)
    );
    assert_eq!(
        m4f.cpu.with_sys(|s| s.read_word(0xe000_ef48, false)),
        Some(0),
        "MVFR2.FPMisc is zero on FPv4: no VSEL, no VRINT"
    );
    let m7f = Harness::new(Config::CORTEX_M7F, &[PARK]);
    assert_eq!(
        m7f.cpu.with_sys(|s| s.read_word(0xe000_ef48, false)),
        Some(0x0000_0040),
        "MVFR2.FPMisc == 4 on FPv5"
    );
    // `FPCCR` comes up with both preservation bits set.
    assert_eq!(
        m4f.cpu.with_sys(|s| s.read_word(0xe000_ef34, false)),
        Some(fpccr::ASPEN | fpccr::LSPEN)
    );
}

// ---------------------------------------------------------------------------
// Arithmetic
// ---------------------------------------------------------------------------

/// Run `code` with `r0` and `r1` preloaded into `S0` and `S1`, and answer
/// what the program left in `r0`.
fn compute(cfg: Config, a: u32, b: u32, fpscr_mode: u32, body: &[u32]) -> (u32, u32) {
    let mut words = alloc::vec![VMSR_R0, VMOV_S0_R0, VMOV_S1_R1];
    words.extend_from_slice(body);
    words.push(VMOV_R0_S2);
    let mut halves = program(&words);
    halves.push(PARK);
    let h = Harness::new(cfg, &halves);
    h.enable_fp();
    // The three set-up registers are written directly rather than with a
    // literal pool, which would need one to be in range.
    h.cpu.set_reg(0, fpscr_mode);
    h.steps(1); // VMSR
    h.cpu.set_reg(0, a);
    h.cpu.set_reg(1, b);
    // `VMOV S0` and `VMOV S1`, then the body, then `VMOV R0, S2`.
    h.steps(3 + body.len());
    (h.cpu.reg(0), h.cpu.fpscr())
}

#[test]
fn single_precision_add_produces_the_ieee_sum() {
    let (r, _) = compute(
        Config::CORTEX_M4F,
        0x3fc0_0000, // 1.5
        0x4010_0000, // 2.25
        0,
        &[VADD_S2_S0_S1],
    );
    assert_eq!(r, 0x4070_0000, "3.75");
}

#[test]
fn vdiv_and_vcvt_round_as_fpscr_says() {
    // 7.0 / 2.0 is exact whatever the mode, and `VCVT` with round-toward-zero
    // truncates it.
    let (quotient, _) = compute(
        Config::CORTEX_M4F,
        0x40e0_0000, // 7.0
        0x4000_0000, // 2.0
        3 << fpscr::RMODE_SHIFT,
        &[VDIV_S2_S0_S1],
    );
    assert_eq!(quotient, 0x4060_0000, "3.5");

    // 1.0 / 3.0 is where the mode shows: toward zero and toward negative
    // truncate, toward positive rounds up, ties-even gives the nearest.
    let third = |mode: u32| {
        compute(
            Config::CORTEX_M4F,
            0x3f80_0000,
            0x4040_0000,
            mode << fpscr::RMODE_SHIFT,
            &[VDIV_S2_S0_S1],
        )
        .0
    };
    assert_eq!(third(0b00), 0x3eaa_aaab, "ties to even rounds up here");
    assert_eq!(third(0b01), 0x3eaa_aaab, "toward +inf");
    assert_eq!(third(0b10), 0x3eaa_aaaa, "toward -inf");
    assert_eq!(third(0b11), 0x3eaa_aaaa, "toward zero");

    // `VCVT.S32.F32` always truncates, whatever `FPSCR.RMode` says.
    let (int, flags) = compute(
        Config::CORTEX_M4F,
        0x4060_0000, // 3.5
        0,
        0,
        &[0xeebd_0ac0, 0xeeb0_1a40], // VCVT.S32.F32 S0, S0 ; VMOV.F32 S2, S0
    );
    assert_eq!(int, 3);
    assert!(flags & fpscr::IXC != 0, "3.5 -> 3 is inexact");
}

#[test]
fn a_chained_multiply_accumulate_rounds_twice_and_a_fused_one_does_not() {
    // `VMLA` is `FPAdd(Sd, FPMul(Sn, Sm))`: the product is rounded, and here
    // the rounding is exactly what cancels. `VFMA` keeps the whole product,
    // so the difference survives. Getting this wrong is invisible to every
    // test that uses round numbers.
    let one_plus_ulp = 0x3f80_0001u32;
    let product_rounded = 0x3f80_0002u32;

    let setup = |op: u32| {
        let mut halves = program(&[
            VMOV_S0_R0, // S0 = 1 + 2^-23
            VMOV_S1_R1, // S1 = 1 + 2^-23
        ]);
        halves.extend_from_slice(&wide(0xee01_2a10)); // VMOV S2, R2
        halves.extend_from_slice(&wide(op));
        halves.extend_from_slice(&wide(VMOV_R0_S2));
        halves.push(PARK);
        let h = Harness::new(Config::CORTEX_M4F, &halves);
        h.enable_fp();
        h.cpu.set_reg(0, one_plus_ulp);
        h.cpu.set_reg(1, one_plus_ulp);
        h.cpu.set_reg(2, product_rounded | 0x8000_0000); // -(1 + 2^-22)
        h.steps(5);
        h.cpu.reg(0)
    };

    assert_eq!(setup(VMLA_S2_S0_S1), 0, "chained: the product was rounded");
    assert_eq!(
        setup(VFMA_S2_S0_S1),
        0x2880_0000,
        "fused: 2^-46 survives, rounded once"
    );
}

#[test]
fn vcmp_writes_the_four_way_result_and_vmrs_moves_it_to_apsr() {
    let run = |a: u32, b: u32| {
        let mut halves = program(&[VMOV_S0_R0, VMOV_S1_R1, VCMP_S0_S1, VMRS_APSR]);
        halves.push(PARK);
        let h = Harness::new(Config::CORTEX_M4F, &halves);
        h.enable_fp();
        h.cpu.set_reg(0, a);
        h.cpu.set_reg(1, b);
        h.steps(4);
        (h.cpu.fpscr() >> 28, h.cpu.xpsr() >> 28)
    };
    // Less: N only. Equal: Z and C. Greater: C only. Unordered: C and V —
    // which is what makes `BVS` the "was a NaN involved" test.
    assert_eq!(run(0x3f80_0000, 0x4000_0000), (0b1000, 0b1000));
    assert_eq!(run(0x3f80_0000, 0x3f80_0000), (0b0110, 0b0110));
    assert_eq!(run(0x4000_0000, 0x3f80_0000), (0b0010, 0b0010));
    assert_eq!(run(0x7fc0_0000, 0x3f80_0000), (0b0011, 0b0011));
}

#[test]
fn flush_to_zero_and_default_nan_are_fpscr_bits_the_arithmetic_honours() {
    let tiny = 0x0000_0001u32; // the smallest positive subnormal
    // With `FZ` clear the subnormal is used exactly.
    let (exact, flags) = compute(Config::CORTEX_M4F, tiny, 0x3f80_0000, 0, &[VMUL_S2_S0_S1]);
    assert_eq!(exact, tiny);
    assert_eq!(flags & fpscr::IDC, 0);

    // With `FZ` set it is replaced by a zero of the same sign, and `IDC`
    // records that it was.
    let (flushed, flags) = compute(
        Config::CORTEX_M4F,
        tiny,
        0x3f80_0000,
        fpscr::FZ,
        &[VMUL_S2_S0_S1],
    );
    assert_eq!(flushed, 0);
    assert!(flags & fpscr::IDC != 0);

    // `DN` clear propagates the operand NaN's payload; `DN` set replaces it
    // with the default NaN.
    let payload = 0x7fc0_1234u32;
    let (kept, _) = compute(
        Config::CORTEX_M4F,
        payload,
        0x3f80_0000,
        0,
        &[VADD_S2_S0_S1],
    );
    assert_eq!(kept, payload);
    let (default, _) = compute(
        Config::CORTEX_M4F,
        payload,
        0x3f80_0000,
        fpscr::DN,
        &[VADD_S2_S0_S1],
    );
    assert_eq!(default, 0x7fc0_0000);
}

#[test]
fn the_fpv5_group_is_undefined_on_an_fpv4_part_and_runs_on_an_fpv5_one() {
    // An FPv4-SP part has no `VMAXNM`, and firmware probes for FPv5 by
    // executing one — so it must take an UNDEFINSTR UsageFault, not `NOCP`
    // and not a silent success.
    let h = Harness::m4f(&wide(VMAXNM_S2_S0_S1));
    h.enable_usage_fault();
    h.cpu.step();
    assert_eq!(h.cpu.xpsr() & 0x1ff, u32::from(Exception::USAGE_FAULT.0));
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_UNDEFINSTR != 0));
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_NOCP == 0));

    // The same encoding on a Cortex-M7F runs, and a quiet NaN loses to a
    // number — which is what separates `VMAXNM` from `VMAX`.
    let mut halves = program(&[VMOV_S0_R0, VMOV_S1_R1, VMAXNM_S2_S0_S1, VMOV_R0_S2]);
    halves.push(PARK);
    let h = Harness::m7f(&halves);
    h.cpu.set_reg(0, 0x7fc0_0000);
    h.cpu.set_reg(1, 0x3f80_0000);
    h.steps(4);
    assert_eq!(h.cpu.reg(0), 0x3f80_0000);
}

#[test]
fn a_disabled_coprocessor_is_nocp_even_for_an_instruction_the_part_lacks() {
    // The coprocessor check comes before the instruction check, so a `VMAXNM`
    // on a *disabled* FPv4 part reports `NOCP` rather than UNDEFINSTR.
    let h = Harness::new(Config::CORTEX_M4F, &wide(VMAXNM_S2_S0_S1));
    h.cpu.step();
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_NOCP != 0));
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_UNDEFINSTR == 0));
}

#[test]
fn a_double_precision_encoding_is_undefined_on_a_single_precision_part() {
    // `VADD.F64 D2, D0, D1` — coprocessor eleven. The FPU is present and
    // enabled, so this is a defined instruction the part does not implement:
    // UNDEFINED, not `NOCP`.
    let h = Harness::m4f(&wide(0xee30_1b01));
    h.cpu.step();
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_UNDEFINSTR != 0));
}

// ---------------------------------------------------------------------------
// Memory and the transfers
// ---------------------------------------------------------------------------

#[test]
fn vldr_and_vstr_move_a_word_and_reject_an_unaligned_address() {
    let mut halves = program(&[VLDR_S0_R0, VSTR_S0_R0]);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    h.set_word(0x800, 0x1234_5678);
    h.cpu.set_reg(0, 0x800);
    h.steps(1);
    assert_eq!(h.cpu.s(0), 0x1234_5678);
    h.cpu.set_reg(0, 0x804);
    h.steps(1);
    assert_eq!(h.word(0x804), 0x1234_5678);

    // An extension-register access is never allowed to be unaligned, whatever
    // `CCR.UNALIGN_TRP` says.
    let h = Harness::m4f(&wide(VLDR_S0_R0));
    h.cpu.set_reg(0, 0x802);
    h.cpu.step();
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_UNALIGNED != 0));
}

#[test]
fn vpush_and_vpop_round_trip_sixteen_registers() {
    let mut halves = program(&[VPUSH_S0_S15, VPOP_S0_S15]);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    for k in 0..16u8 {
        h.cpu.set_s(k, 0x1000_0000 + u32::from(k));
    }
    let sp = h.cpu.reg(13);
    h.steps(1);
    assert_eq!(h.cpu.reg(13), sp - 0x40, "sixteen words below the old SP");
    assert_eq!(h.word(sp - 0x40), 0x1000_0000);
    assert_eq!(h.word(sp - 4), 0x1000_000f);
    for k in 0..16u8 {
        h.cpu.set_s(k, 0);
    }
    h.steps(1);
    assert_eq!(h.cpu.reg(13), sp);
    for k in 0..16u8 {
        assert_eq!(h.cpu.s(k), 0x1000_0000 + u32::from(k));
    }
}

#[test]
fn vmrs_and_vmsr_reach_fpscr_and_the_mode_bits_stick() {
    let mut halves = program(&[VMSR_R0, VMRS_R0]);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    // Every writable bit, plus `AHP` and a trap enable, neither of which this
    // core implements: they must read back as zero so a guest can tell.
    h.cpu.set_reg(0, 0xffff_ffff);
    h.steps(2);
    assert_eq!(h.cpu.reg(0), fpscr::WRITABLE);
    assert_eq!(h.cpu.reg(0) & fpscr::AHP, 0, "AHP is RES0 on this core");
}

#[test]
fn vmov_immediate_expands_the_way_the_manual_says() {
    // `VFPExpandImm`, checked against the four values an assembler prints.
    assert_eq!(fp::expand_imm(0x70), 0x3f80_0000, "1.0");
    assert_eq!(fp::expand_imm(0xf0), 0xbf80_0000, "-1.0");
    assert_eq!(fp::expand_imm(0x00), 0x4000_0000, "2.0");
    assert_eq!(fp::expand_imm(0x1f), 0x40f8_0000, "7.75");

    let mut halves = program(&[VMOV_S0_ONE, VMOV_R0_S0]);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    h.steps(2);
    assert_eq!(h.cpu.reg(0), 0x3f80_0000);
}

// ---------------------------------------------------------------------------
// The exception model
// ---------------------------------------------------------------------------

#[test]
fn an_exception_taken_with_fpca_set_pushes_the_extended_frame() {
    let mut halves = program(&[VMOV_S0_R0]);
    halves.push(SVC0);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    h.handler(&[PARK]);
    let sp_before = h.cpu.reg(13);
    h.steps(1);
    assert!(
        h.cpu.regs().control & super::sys::control::FPCA != 0,
        "a VFP instruction claims a floating-point context"
    );
    h.steps(1); // the SVC
    assert_eq!(
        sp_before - h.cpu.reg(13),
        exc_return::EXTENDED_FRAME,
        "26 words"
    );
    assert_eq!(
        h.cpu.reg(14) & exc_return::FP_FRAME,
        0,
        "EXC_RETURN[4] clear means the frame carries S0-S15"
    );
    assert_eq!(
        h.cpu.regs().control & super::sys::control::FPCA,
        0,
        "the handler starts with no floating-point context of its own"
    );
}

#[test]
fn without_fpca_the_frame_is_the_plain_eight_words() {
    let h = Harness::m4f(&[SVC0, PARK]);
    h.handler(&[PARK]);
    let sp_before = h.cpu.reg(13);
    h.steps(1);
    assert_eq!(sp_before - h.cpu.reg(13), exc_return::BASIC_FRAME);
    assert_ne!(h.cpu.reg(14) & exc_return::FP_FRAME, 0);
}

#[test]
fn lazy_stacking_reserves_the_frame_but_defers_the_store() {
    let mut halves = program(&[VMOV_S0_R0]);
    halves.push(SVC0);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    h.handler(&[PARK]);
    h.cpu.set_reg(0, 0xdead_beef);
    let sp_before = h.cpu.reg(13);
    // Pre-fill the whole frame area with a sentinel, so "the core did not
    // write here" is something the test can actually see.
    for k in 0..26u32 {
        h.set_word(sp_before - exc_return::EXTENDED_FRAME + k * 4, 0x5a5a_5a5a);
    }
    h.steps(2);

    let frame = h.cpu.reg(13);
    assert_eq!(sp_before - frame, exc_return::EXTENDED_FRAME);
    assert!(
        h.cpu.with_sys(|s| s.fpccr & fpccr::LSPACT != 0),
        "a push is owed"
    );
    assert_eq!(
        h.cpu.with_sys(|s| s.fpcar),
        frame + exc_return::FP_OFFSET,
        "FPCAR points at the S0 slot"
    );
    assert!(
        h.cpu.with_sys(|s| s.fpccr & fpccr::THREAD != 0),
        "the reservation was made from Thread mode"
    );
    // The eight core words *were* written; the sixteen S slots were not.
    assert_eq!(h.word(frame), 0xdead_beef, "R0");
    for k in 0..16u32 {
        assert_eq!(
            h.word(frame + exc_return::FP_OFFSET + k * 4),
            0x5a5a_5a5a,
            "S{k} was reserved, not written"
        );
    }
}

#[test]
fn the_first_vfp_instruction_in_the_handler_performs_the_deferred_push() {
    let mut halves = program(&[VMOV_S0_R0]);
    halves.push(SVC0);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    // The handler's first instruction is itself a VFP one, which is the
    // event that has to resolve the reservation.
    h.handler(&program(&[VMOV_S0_R0]));
    h.cpu.set_reg(0, 0x1111_2222);
    let sp_before = h.cpu.reg(13);
    for k in 0..26u32 {
        h.set_word(sp_before - exc_return::EXTENDED_FRAME + k * 4, 0x5a5a_5a5a);
    }
    h.steps(2);
    let frame = h.cpu.reg(13);
    h.cpu.set_reg(0, 0x3333_4444);
    h.steps(1);

    assert!(
        h.cpu.with_sys(|s| s.fpccr & fpccr::LSPACT == 0),
        "the debt is paid"
    );
    assert_eq!(
        h.word(frame + exc_return::FP_OFFSET),
        0x1111_2222,
        "S0 as the interrupted code left it, not as the handler set it"
    );
    assert_eq!(h.cpu.s(0), 0x3333_4444, "the handler's own write stands");
    // `FPSCR` lands in the last populated word of the frame.
    assert_ne!(h.word(frame + exc_return::FP_OFFSET + 0x40), 0x5a5a_5a5a);
}

#[test]
fn an_exception_return_with_lspact_still_set_discards_the_reservation() {
    // The handler never touches the FPU, so `S0`-`S15` were never written to
    // the frame and never need reading back — which is the whole point.
    let mut halves = program(&[VMOV_S0_R0]);
    halves.push(SVC0);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    h.handler(&[BX_LR]);
    h.cpu.set_reg(0, 0xabcd_0123);
    let sp_before = h.cpu.reg(13);
    h.steps(2);
    let frame = h.cpu.reg(13);
    // Corrupt the (never written) S0 slot: a core that restored it anyway
    // would pick this up, which is exactly the bug to catch.
    h.set_word(frame + exc_return::FP_OFFSET, 0xffff_ffff);
    h.steps(1); // BX LR, the exception return

    assert_eq!(h.cpu.reg(13), sp_before, "the whole 26 words came back");
    assert_eq!(h.cpu.s(0), 0xabcd_0123, "S0 was never disturbed");
    assert!(h.cpu.with_sys(|s| s.fpccr & fpccr::LSPACT == 0));
    assert!(
        h.cpu.regs().control & super::sys::control::FPCA != 0,
        "the interrupted context owns a floating-point context again"
    );
}

#[test]
fn eager_stacking_writes_the_registers_at_entry() {
    let mut halves = program(&[VMOV_S0_R0]);
    halves.push(SVC0);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    h.handler(&[PARK]);
    // `FPCCR.LSPEN = 0`: the architecture's other configuration, and the one
    // a latency-insensitive system picks for simplicity.
    h.cpu.with_sys(|s| s.fpccr &= !fpccr::LSPEN);
    h.cpu.set_reg(0, 0x0f0f_0f0f);
    h.steps(2);
    let frame = h.cpu.reg(13);
    assert!(h.cpu.with_sys(|s| s.fpccr & fpccr::LSPACT == 0));
    assert_eq!(h.word(frame + exc_return::FP_OFFSET), 0x0f0f_0f0f);
}

#[test]
fn fpdscr_is_loaded_into_fpscr_on_entry_and_the_frame_carries_the_old_one() {
    let mut halves = program(&[VMSR_R0, VMOV_S0_R0]);
    halves.push(SVC0);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    h.handler(&[PARK]);
    // The handler should start in round-toward-zero with flush-to-zero on,
    // whatever the interrupted code was using.
    h.cpu
        .with_sys(|s| s.fpdscr = fpscr::FZ | (3 << fpscr::RMODE_SHIFT));
    h.cpu.with_sys(|s| s.fpccr &= !fpccr::LSPEN);
    h.cpu.set_reg(0, fpscr::DN | (1 << fpscr::RMODE_SHIFT));
    h.steps(3);
    assert_eq!(h.cpu.fpscr(), fpscr::FZ | (3 << fpscr::RMODE_SHIFT));
    let frame = h.cpu.reg(13);
    assert_eq!(
        h.word(frame + exc_return::FP_OFFSET + 0x40),
        fpscr::DN | (1 << fpscr::RMODE_SHIFT),
        "the interrupted context's FPSCR is in the frame"
    );
}

#[test]
fn the_whole_frame_round_trips_through_a_handler_that_uses_the_fpu() {
    let mut halves = program(&[VMOV_S0_R0]);
    halves.push(SVC0);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    // The handler clobbers S0 and returns.
    let mut handler = program(&[VMOV_S0_R0]);
    handler.push(BX_LR);
    h.handler(&handler);
    h.cpu.set_reg(0, 0x1357_9bdf);
    h.steps(2);
    h.cpu.set_reg(0, 0);
    h.steps(2); // the handler's VMOV, then the return
    assert_eq!(
        h.cpu.s(0),
        0x1357_9bdf,
        "the interrupted context's S0 came back"
    );
}

#[test]
fn an_exception_return_naming_an_fp_frame_faults_on_a_part_with_no_fpu() {
    // A stray `EXC_RETURN` with bit 4 clear is not something a Cortex-M4
    // without the option can ever have produced.
    let h = Harness::new(Config::CORTEX_M4, &[SVC0, BX_LR, PARK]);
    h.handler(&[BX_LR]);
    h.steps(1);
    h.cpu.set_reg(14, exc_return::THREAD_MSP_FP);
    h.steps(1);
    assert!(h.cpu.with_sys(|s| s.cfsr & fsr::UF_INVPC != 0));
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// Save one core's whole device chunk.
fn snapshot(h: &Harness) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("cpu", super::CLASS.name).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer
            .chunk("cpu", super::CLASS.name, super::CLASS.version)
            .unwrap();
        Device::save(h.cpu.as_ref(), &mut chunk).unwrap();
    }
    writer.to_vec().unwrap()
}

#[test]
fn fp_state_is_part_of_the_snapshot() {
    let mut halves = program(&[VMOV_S0_R0]);
    halves.push(PARK);
    let h = Harness::m4f(&halves);
    for k in 0..32u8 {
        h.cpu.set_s(k, 0xc0de_0000 + u32::from(k));
    }
    h.cpu
        .set_fpscr(fpscr::DN | fpscr::FZ | (2 << fpscr::RMODE_SHIFT));
    h.cpu.with_sys(|s| {
        s.fpccr |= fpccr::LSPACT;
        s.fpcar = 0x2000_0120;
        s.fpdscr = 3 << fpscr::RMODE_SHIFT;
    });

    let bytes = snapshot(&h);

    let fresh = Harness::m4f(&[PARK]);
    let reader = StateReader::new(&bytes).unwrap();
    let migrations = Migrations::new();
    let chunk = reader
        .load("cpu", super::CLASS.name, super::CLASS.version, &migrations)
        .unwrap();
    Device::load(fresh.cpu.as_ref(), &mut chunk.reader()).unwrap();

    for k in 0..32u8 {
        assert_eq!(fresh.cpu.s(k), 0xc0de_0000 + u32::from(k));
    }
    assert_eq!(fresh.cpu.fpscr(), h.cpu.fpscr());
    assert_eq!(fresh.cpu.with_sys(|s| s.fpccr), h.cpu.with_sys(|s| s.fpccr));
    assert_eq!(fresh.cpu.with_sys(|s| s.fpcar), 0x2000_0120);
    assert_eq!(fresh.cpu.with_sys(|s| s.fpdscr), 3 << fpscr::RMODE_SHIFT);
}

#[test]
fn a_part_without_the_unit_writes_the_same_snapshot_it_always_did() {
    // The floating-point block is written only where there is one, so a
    // Cortex-M3 or an FPU-less M4 keeps the chunk layout version 1 of this
    // class has always produced.
    let plain = Harness::new(Config::CORTEX_M4, &[PARK]);
    let withfp = Harness::new(Config::CORTEX_M4F, &[PARK]);
    // Three system words plus thirty-three register words.
    assert_eq!(
        snapshot(&withfp).len() - snapshot(&plain).len(),
        (3 + 33) * 4
    );
}

// ---------------------------------------------------------------------------
// Decode and disassembly
// ---------------------------------------------------------------------------

/// Disassemble one wide encoding.
fn disasm(encoding: u32) -> String {
    let [hw1, hw2] = wide(encoding);
    format!("{}", decode(hw1, hw2))
}

#[test]
fn the_decoder_and_the_disassembler_agree_because_they_are_one_description() {
    assert_eq!(disasm(VADD_S2_S0_S1), "VADD.F32 s2, s0, s1");
    assert_eq!(disasm(VSUB_S2_S0_S1), "VSUB.F32 s2, s0, s1");
    assert_eq!(disasm(VMUL_S2_S0_S1), "VMUL.F32 s2, s0, s1");
    assert_eq!(disasm(VDIV_S2_S0_S1), "VDIV.F32 s2, s0, s1");
    assert_eq!(disasm(VMLA_S2_S0_S1), "VMLA.F32 s2, s0, s1");
    assert_eq!(disasm(VFMA_S2_S0_S1), "VFMA.F32 s2, s0, s1");
    assert_eq!(disasm(0xeeb1_0ac0), "VSQRT.F32 s0, s0");
    assert_eq!(disasm(0xeeb0_0ac0), "VABS.F32 s0, s0");
    assert_eq!(disasm(0xeeb1_0a40), "VNEG.F32 s0, s0");
    assert_eq!(disasm(VCMP_S0_S1), "VCMP.F32 s0, s1");
    assert_eq!(disasm(0xeeb5_0a40), "VCMP.F32 s0, #0.0");
    assert_eq!(disasm(0xeeb5_0ac0), "VCMPE.F32 s0, #0.0");
    assert_eq!(disasm(VCVT_S32_S2_S0), "VCVT.S32.F32 s2, s0");
    assert_eq!(disasm(0xeeb8_0ac0), "VCVT.F32.S32 s0, s0");
    assert_eq!(disasm(VMOV_S0_R0), "VMOV s0, r0");
    assert_eq!(disasm(VMOV_R0_S2), "VMOV r0, s2");
    assert_eq!(disasm(VMRS_APSR), "VMRS APSR_nzcv, FPSCR");
    assert_eq!(disasm(VMSR_R0), "VMSR FPSCR, r0");
    assert_eq!(disasm(VLDR_S0_R0), "VLDR s0, [r0]");
    assert_eq!(disasm(VSTR_S0_R0), "VSTR s0, [r0]");
    assert_eq!(disasm(VPUSH_S0_S15), "VPUSH {s0-s15}");
    assert_eq!(disasm(VPOP_S0_S15), "VPOP {s0-s15}");
    assert_eq!(disasm(VMAXNM_S2_S0_S1), "VMAXNM.F32 s2, s0, s1");
    assert_eq!(disasm(0xfebd_0ac0), "VCVTN.S32.F32 s0, s0");
    assert_eq!(disasm(0xfeb8_0a40), "VRINTA.F32 s0, s0");
    assert_eq!(disasm(0xfeb9_0a40), "VRINTN.F32 s0, s0");
    assert_eq!(disasm(0xfe00_0a00), "VSELEQ.F32 s0, s0, s0");
    assert_eq!(disasm(0xfe10_0a00), "VSELVS.F32 s0, s0, s0");
    assert_eq!(disasm(0xfe20_0a00), "VSELGE.F32 s0, s0, s0");
    assert_eq!(disasm(0xfe30_0a00), "VSELGT.F32 s0, s0, s0");
}

#[test]
fn the_odd_register_halves_land_in_the_right_place() {
    // `Vd:D`, `Vn:N` and `Vm:M` each put the *low* bit of a single-precision
    // number in a separate field, and putting it back the wrong way round is
    // the classic transcription slip. `VADD.F32 s3, s5, s7`.
    assert_eq!(disasm(0xee72_1aa3), "VADD.F32 s3, s5, s7");
    // `VLDR s1, [r2, #8]` — the `D` bit is bit 22 here too, and the immediate
    // is scaled by four.
    assert_eq!(disasm(0xedd2_0a02), "VLDR s1, [r2, #8]");
    assert_eq!(disasm(0xed12_0a02), "VLDR s0, [r2, #-8]");
}

#[test]
fn a_coprocessor_encoding_the_fpu_does_not_claim_stays_a_coprocessor_encoding() {
    // Coprocessor fourteen, which nothing here implements.
    assert!(matches!(decode(0xee00, 0x0e10), Insn::Coproc { cp: 14 }));
    // Coprocessor eleven: the double-precision half of the FPU, not decoded.
    assert!(matches!(decode(0xee30, 0x1b01), Insn::Coproc { cp: 11 }));
}

#[test]
fn every_decoded_form_prints_something_and_nothing_panics() {
    // A sweep of the three encoding windows: no panic, and anything that
    // decodes must also disassemble. This is the cheap version of a fuzz
    // target, and `fuzz/` has the other one.
    let mut decoded = 0u32;
    for hw1 in [0xee00u16, 0xeeb0, 0xec00, 0xed00, 0xfe00] {
        for low in 0..0x1000u16 {
            for hw1 in [hw1, hw1 | 0x00f0] {
                let hw2 = (low << 4) | 0x000a;
                if let Some(fp) = fpisa::decode(hw1, hw2) {
                    decoded += 1;
                    let text = format!("{fp}");
                    assert!(!text.is_empty());
                }
            }
        }
    }
    assert!(decoded > 1000, "the sweep decoded {decoded} encodings");
}

// ---------------------------------------------------------------------------
// The ledger
// ---------------------------------------------------------------------------

/// Which operation a ledger row names.
#[derive(Debug, Clone, Copy)]
enum Op {
    Add,
    Sub,
    Mul,
    Div,
    Sqrt,
    /// `FPMulAdd(a, b, c)` — the addend first, as Arm spells it.
    MulAdd(u32),
    MaxNum,
    MinNum,
    ToSigned,
    ToUnsigned,
    FromSigned,
    RoundInt,
    HalfToSingle,
    SingleToHalf,
}

/// One row: an operation, its operands, the `FPSCR` mode bits it runs under,
/// and the result and cumulative flags it must produce.
struct Row {
    op: Op,
    a: u32,
    b: u32,
    mode: u32,
    result: u32,
    flags: u32,
}

const fn row(op: Op, a: u32, b: u32, mode: u32, result: u32, flags: u32) -> Row {
    Row {
        op,
        a,
        b,
        mode,
        result,
        flags,
    }
}

/// Rounding-mode shorthand for the table.
const RN: u32 = 0;
/// Toward `+∞`.
const RP: u32 = 1 << fpscr::RMODE_SHIFT;
/// Toward `−∞`.
const RM: u32 = 2 << fpscr::RMODE_SHIFT;
/// Toward zero.
const RZ: u32 = 3 << fpscr::RMODE_SHIFT;

/// The ledger.
///
/// Every expected value is IEEE 754-2019 §5's result for the operation at
/// binary32, with DDI 0403E A7.7's Arm-specific choices where IEEE leaves one
/// open — which NaN survives, what an out-of-range conversion gives, when
/// underflow is signalled. Nothing here was produced by running this code:
/// the point of a ledger is that it is written down independently and then
/// checked.
static LEDGER: &[Row] = &[
    // --- exact arithmetic --------------------------------------------------
    row(Op::Add, 0x3fc0_0000, 0x4010_0000, RN, 0x4070_0000, 0),
    row(Op::Sub, 0x4070_0000, 0x4010_0000, RN, 0x3fc0_0000, 0),
    row(Op::Mul, 0x4000_0000, 0x4040_0000, RN, 0x40c0_0000, 0),
    row(Op::Div, 0x40e0_0000, 0x4000_0000, RN, 0x4060_0000, 0),
    row(Op::Sqrt, 0x4110_0000, 0, RN, 0x4040_0000, 0),
    // --- the four rounding directions on 1/3 -------------------------------
    row(
        Op::Div,
        0x3f80_0000,
        0x4040_0000,
        RN,
        0x3eaa_aaab,
        fpscr::IXC,
    ),
    row(
        Op::Div,
        0x3f80_0000,
        0x4040_0000,
        RP,
        0x3eaa_aaab,
        fpscr::IXC,
    ),
    row(
        Op::Div,
        0x3f80_0000,
        0x4040_0000,
        RM,
        0x3eaa_aaaa,
        fpscr::IXC,
    ),
    row(
        Op::Div,
        0x3f80_0000,
        0x4040_0000,
        RZ,
        0x3eaa_aaaa,
        fpscr::IXC,
    ),
    // --- the exceptional cases ---------------------------------------------
    // 1/0 is division by zero, not invalid, and gives a correctly signed
    // infinity (IEEE 754-2019 §7.3).
    row(
        Op::Div,
        0x3f80_0000,
        0x0000_0000,
        RN,
        0x7f80_0000,
        fpscr::DZC,
    ),
    row(
        Op::Div,
        0xbf80_0000,
        0x0000_0000,
        RN,
        0xff80_0000,
        fpscr::DZC,
    ),
    // 0/0 and inf-inf are invalid and give the default NaN.
    row(
        Op::Div,
        0x0000_0000,
        0x0000_0000,
        RN,
        0x7fc0_0000,
        fpscr::IOC,
    ),
    row(
        Op::Sub,
        0x7f80_0000,
        0x7f80_0000,
        RN,
        0x7fc0_0000,
        fpscr::IOC,
    ),
    // sqrt of a negative is invalid; sqrt(-0) is -0 and raises nothing.
    row(Op::Sqrt, 0xbf80_0000, 0, RN, 0x7fc0_0000, fpscr::IOC),
    row(Op::Sqrt, 0x8000_0000, 0, RN, 0x8000_0000, 0),
    // Overflow to infinity under round-to-nearest, and to the largest finite
    // magnitude under round-toward-zero — both with overflow and inexact.
    row(
        Op::Mul,
        0x7f7f_ffff,
        0x4000_0000,
        RN,
        0x7f80_0000,
        fpscr::OFC | fpscr::IXC,
    ),
    row(
        Op::Mul,
        0x7f7f_ffff,
        0x4000_0000,
        RZ,
        0x7f7f_ffff,
        fpscr::OFC | fpscr::IXC,
    ),
    // Gradual underflow: the smallest normal halved is the largest subnormal
    // *exactly*, so nothing is raised at all. `UFC` is set for a result that
    // is tiny **and** inexact, not for one that is merely tiny (DDI 0403E
    // A2.5.4), and a core that raised it here would be over-reporting.
    row(Op::Mul, 0x0080_0000, 0x3f00_0000, RN, 0x0040_0000, 0),
    // Tiny *and* inexact: `2^-126 × (0.5 + 2^-24)` is `2^-127 + 2^-150`,
    // which is below the smallest normal and half an ulp past a subnormal,
    // so it rounds to even and both flags fire.
    row(
        Op::Mul,
        0x0080_0000,
        0x3f00_0001,
        RN,
        0x0040_0000,
        fpscr::UFC | fpscr::IXC,
    ),
    // --- NaN propagation ---------------------------------------------------
    // A signaling NaN is quietened and propagated, with invalid raised.
    row(
        Op::Add,
        0x7fa0_0000,
        0x3f80_0000,
        RN,
        0x7fe0_0000,
        fpscr::IOC,
    ),
    // A quiet NaN propagates untouched and raises nothing.
    row(Op::Add, 0x7fc0_1234, 0x3f80_0000, RN, 0x7fc0_1234, 0),
    // With two NaNs the signaling one wins, whichever operand it is.
    row(
        Op::Add,
        0x7fc0_1234,
        0x7fa0_5678,
        RN,
        0x7fe0_5678,
        fpscr::IOC,
    ),
    // --- FPMulAdd's two Arm-specific rules ---------------------------------
    // The addend is searched for a NaN first: with a quiet NaN in the addend
    // and a signaling one in op1, the *signaling* one still wins, because the
    // search is by kind and then by position.
    row(
        Op::MulAdd(0x3f80_0000),
        0x7fa0_00ff,
        0x3f80_0000,
        RN,
        0x7fe0_00ff,
        fpscr::IOC,
    ),
    // A quiet-NaN addend with an `inf * 0` product gives the *default* NaN,
    // not the addend: the propagation is overridden.
    row(
        Op::MulAdd(0x7fc0_abcd),
        0x7f80_0000,
        0x0000_0000,
        RN,
        0x7fc0_0000,
        fpscr::IOC,
    ),
    // The ordinary fused case: one rounding over the whole product.
    row(
        Op::MulAdd(0xbf80_0002),
        0x3f80_0001,
        0x3f80_0001,
        RN,
        0x2880_0000,
        0,
    ),
    // --- min/max -----------------------------------------------------------
    // A quiet NaN loses to a number; a signaling one does not, and raises.
    row(Op::MaxNum, 0x7fc0_0000, 0x3f80_0000, RN, 0x3f80_0000, 0),
    row(Op::MinNum, 0x7fc0_0000, 0x3f80_0000, RN, 0x3f80_0000, 0),
    row(
        Op::MaxNum,
        0x7fa0_0000,
        0x3f80_0000,
        RN,
        0x7fe0_0000,
        fpscr::IOC,
    ),
    // -0 is less than +0 for both.
    row(Op::MinNum, 0x8000_0000, 0x0000_0000, RN, 0x8000_0000, 0),
    row(Op::MaxNum, 0x8000_0000, 0x0000_0000, RN, 0x0000_0000, 0),
    // --- integer conversion ------------------------------------------------
    row(Op::ToSigned, 0x4060_0000, 0, RZ, 3, fpscr::IXC),
    row(Op::ToSigned, 0xc060_0000, 0, RZ, (-3i32) as u32, fpscr::IXC),
    row(Op::ToSigned, 0x4060_0000, 0, RN, 4, fpscr::IXC),
    row(Op::ToSigned, 0x40a0_0000, 0, RN, 5, 0),
    // Out of range saturates; a NaN gives zero. Both are invalid.
    row(Op::ToSigned, 0x7f80_0000, 0, RZ, 0x7fff_ffff, fpscr::IOC),
    row(Op::ToSigned, 0xff80_0000, 0, RZ, 0x8000_0000, fpscr::IOC),
    row(Op::ToSigned, 0x7fc0_0000, 0, RZ, 0, fpscr::IOC),
    row(Op::ToUnsigned, 0xbf80_0000, 0, RZ, 0, fpscr::IOC),
    row(Op::ToUnsigned, 0x4f80_0000, 0, RZ, 0xffff_ffff, fpscr::IOC),
    // 2^31 exactly: representable unsigned, out of range signed.
    row(Op::ToUnsigned, 0x4f00_0000, 0, RZ, 0x8000_0000, 0),
    row(Op::ToSigned, 0x4f00_0000, 0, RZ, 0x7fff_ffff, fpscr::IOC),
    // Integer to float, with the rounding a 24-bit significand forces.
    row(Op::FromSigned, 1, 0, RN, 0x3f80_0000, 0),
    // `2^24 - 1` needs exactly twenty-four significand bits, which binary32
    // has, so it converts exactly; `2^24 + 1` needs twenty-five and does not.
    row(Op::FromSigned, 0x00ff_ffff, 0, RN, 0x4b7f_ffff, 0),
    row(Op::FromSigned, 0x0100_0001, 0, RN, 0x4b80_0000, fpscr::IXC),
    row(Op::FromSigned, 0x8000_0000, 0, RN, 0xcf00_0000, 0),
    // --- round to integral -------------------------------------------------
    row(Op::RoundInt, 0x3fc0_0000, 0, RN, 0x4000_0000, 0),
    row(Op::RoundInt, 0x4020_0000, 0, RN, 0x4000_0000, 0),
    row(Op::RoundInt, 0x3fc0_0000, 0, RZ, 0x3f80_0000, 0),
    row(Op::RoundInt, 0x3fc0_0000, 0, RM, 0x3f80_0000, 0),
    row(Op::RoundInt, 0x3fc0_0000, 0, RP, 0x4000_0000, 0),
    // --- half precision ----------------------------------------------------
    row(Op::HalfToSingle, 0x3c00, 0, RN, 0x3f80_0000, 0),
    row(Op::HalfToSingle, 0x0001, 0, RN, 0x3380_0000, 0),
    row(Op::HalfToSingle, 0x7c00, 0, RN, 0x7f80_0000, 0),
    row(Op::SingleToHalf, 0x3f80_0000, 0, RN, 0x3c00, 0),
    // 3.125 has four significant fraction bits, so it survives the narrowing
    // exactly; π does not, and rounds to nearest.
    row(Op::SingleToHalf, 0x4048_0000, 0, RN, 0x4240, 0),
    row(Op::SingleToHalf, 0x4049_0fdb, 0, RN, 0x4248, fpscr::IXC),
    // Too large for a half: overflow to infinity, with inexact.
    row(
        Op::SingleToHalf,
        0x7f00_0000,
        0,
        RN,
        0x7c00,
        fpscr::OFC | fpscr::IXC,
    ),
];

#[test]
fn the_ledger_holds() {
    for (i, r) in LEDGER.iter().enumerate() {
        let env = fp::env(r.mode);
        let (value, flags) = match r.op {
            Op::Add => fp::add(r.a, r.b, env),
            Op::Sub => fp::sub(r.a, r.b, env),
            Op::Mul => fp::mul(r.a, r.b, env),
            Op::Div => fp::div(r.a, r.b, env),
            Op::Sqrt => fp::sqrt(r.a, env),
            Op::MulAdd(addend) => fp::mul_add(addend, r.a, r.b, env),
            Op::MaxNum => fp::max_min_num(r.a, r.b, false, env),
            Op::MinNum => fp::max_min_num(r.a, r.b, true, env),
            Op::ToSigned => fp::to_fixed(r.a, 32, false, 0, env),
            Op::ToUnsigned => fp::to_fixed(r.a, 32, true, 0, env),
            Op::FromSigned => fp::from_fixed(r.a, 32, false, 0, env),
            Op::RoundInt => fp::round_int(r.a, env, false),
            Op::HalfToSingle => fp::half_to_single(r.a as u16, env),
            Op::SingleToHalf => {
                let (v, f) = fp::single_to_half(r.a, env);
                (u32::from(v), f)
            }
        };
        let mut cumulative = 0u32;
        fp::accumulate(&mut cumulative, flags);
        assert_eq!(
            (value, cumulative),
            (r.result, r.flags),
            "ledger row {i}: {:?} of {:08x}, {:08x} at mode {:08x}",
            r.op,
            r.a,
            r.b,
            r.mode
        );
    }
}

#[test]
fn the_ledger_runs_through_the_interpreter_too() {
    // The table above exercises the wrapper; this runs the two-operand half
    // of it through real instructions, so an encoding that reached the wrong
    // helper cannot hide behind a correct helper.
    for (i, r) in LEDGER.iter().enumerate() {
        let op = match r.op {
            Op::Add => VADD_S2_S0_S1,
            Op::Sub => VSUB_S2_S0_S1,
            Op::Mul => VMUL_S2_S0_S1,
            Op::Div => VDIV_S2_S0_S1,
            _ => continue,
        };
        let (value, fpscr_after) = compute(Config::CORTEX_M4F, r.a, r.b, r.mode, &[op]);
        assert_eq!(value, r.result, "ledger row {i} through the interpreter");
        assert_eq!(
            fpscr_after & fpscr::CUMULATIVE,
            r.flags,
            "ledger row {i}: cumulative flags"
        );
    }
}

// ---------------------------------------------------------------------------
// Differential against the A64 core's wrapper
// ---------------------------------------------------------------------------

/// A corpus of interesting single-precision bit patterns.
///
/// Not random: the values that separate two transcriptions of the same
/// pseudocode are the boundaries — zeros of both signs, the smallest and
/// largest subnormal, the smallest and largest normal, both infinities, and
/// both kinds of NaN with a payload.
#[cfg(feature = "cpu-arm-a64")]
const CORPUS: &[u32] = &[
    0x0000_0000,
    0x8000_0000,
    0x0000_0001,
    0x807f_ffff,
    0x0080_0000,
    0x7f7f_ffff,
    0x3f80_0000,
    0xbf80_0000,
    0x4000_0000,
    0x3f80_0001,
    0x4049_0fdb,
    0x7f80_0000,
    0xff80_0000,
    0x7fc0_1234,
    0x7fa0_5678,
    0xffc0_0001,
];

#[test]
#[cfg(feature = "cpu-arm-a64")]
fn the_arm_rules_agree_with_the_a64_core_which_transcribed_them_separately() {
    use crate::cpu::arm::a64::fp as a64;

    // `FPSCR` and `FPCR` put `RMode`, `FZ` and `DN` in the same places, which
    // is not a coincidence: `FPSCR` is the AArch32 view of the same register.
    for mode in [0, RP, RM, RZ, fpscr::FZ, fpscr::DN, fpscr::FZ | fpscr::DN] {
        let ours = fp::env(mode);
        let theirs = a64::env(u64::from(mode), a64::Prec::Single);
        assert_eq!(ours, theirs, "the environments differ at mode {mode:08x}");

        for &a in CORPUS {
            let (v, f) = fp::sqrt(a, ours);
            let (v2, f2) = a64::sqrt(a64::Prec::Single, u64::from(a), theirs);
            assert_eq!((u64::from(v), f), (v2, f2), "VSQRT {a:08x} at {mode:08x}");

            let (v, f) = fp::round_int(a, ours, false);
            let (v2, f2) = a64::round_int(a64::Prec::Single, u64::from(a), theirs, false);
            assert_eq!((u64::from(v), f), (v2, f2), "VRINT {a:08x} at {mode:08x}");

            for &b in CORPUS {
                for (name, ours_r, theirs_r) in [
                    (
                        "VADD",
                        fp::add(a, b, ours),
                        a64::add(a64::Prec::Single, u64::from(a), u64::from(b), theirs),
                    ),
                    (
                        "VMUL",
                        fp::mul(a, b, ours),
                        a64::mul(a64::Prec::Single, u64::from(a), u64::from(b), theirs),
                    ),
                    (
                        "VDIV",
                        fp::div(a, b, ours),
                        a64::div(a64::Prec::Single, u64::from(a), u64::from(b), theirs),
                    ),
                    (
                        "VMAXNM",
                        fp::max_min_num(a, b, false, ours),
                        a64::max_min_num(
                            a64::Prec::Single,
                            u64::from(a),
                            u64::from(b),
                            false,
                            theirs,
                        ),
                    ),
                    (
                        "VMINNM",
                        fp::max_min_num(a, b, true, ours),
                        a64::max_min_num(
                            a64::Prec::Single,
                            u64::from(a),
                            u64::from(b),
                            true,
                            theirs,
                        ),
                    ),
                ] {
                    assert_eq!(
                        (u64::from(ours_r.0), ours_r.1),
                        theirs_r,
                        "{name} {a:08x}, {b:08x} at mode {mode:08x}"
                    );
                }

                // `FPMulAdd`, where the operand order and the `inf * 0`
                // override live. A third operand from the corpus would make
                // this cubic, so the addend rotates through it instead.
                for &c in &[0x3f80_0000u32, 0x7fc0_abcd, 0x7fa0_0001, 0x0000_0000] {
                    let (v, f) = fp::mul_add(c, a, b, ours);
                    let (v2, f2) = a64::mul_add(
                        a64::Prec::Single,
                        u64::from(c),
                        u64::from(a),
                        u64::from(b),
                        theirs,
                    );
                    assert_eq!(
                        (u64::from(v), f),
                        (v2, f2),
                        "VFMA {c:08x} + {a:08x} * {b:08x} at mode {mode:08x}"
                    );
                }
            }
        }
    }
}

#[test]
#[cfg(feature = "cpu-arm-a64")]
fn the_four_way_compare_agrees_with_the_a64_cores() {
    use crate::cpu::arm::a64::fp as a64;
    for &a in CORPUS {
        for &b in CORPUS {
            for signal_all in [false, true] {
                let env = fp::env(0);
                let (nzcv, flags) = fp::compare(a, b, signal_all, env);
                let (their_nzcv, their_flags) = a64::compare(
                    a64::Prec::Single,
                    u64::from(a),
                    u64::from(b),
                    signal_all,
                    a64::env(0, a64::Prec::Single),
                );
                // `FPSCR`'s flags are in bits 31:28, the same order `Nzcv`
                // uses; the A64 type spells them as four booleans.
                let mut expected = 0u32;
                if their_nzcv.n() {
                    expected |= fpscr::N;
                }
                if their_nzcv.z() {
                    expected |= fpscr::Z;
                }
                if their_nzcv.c() {
                    expected |= fpscr::C;
                }
                if their_nzcv.v() {
                    expected |= fpscr::V;
                }
                assert_eq!(
                    (nzcv, flags),
                    (expected, their_flags),
                    "VCMP{} {a:08x}, {b:08x}",
                    if signal_all { "E" } else { "" }
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Housekeeping
// ---------------------------------------------------------------------------

#[test]
fn the_environment_maps_every_fpscr_mode_bit() {
    assert_eq!(fp::rounding(0), Round::TiesEven);
    assert_eq!(fp::rounding(RP), Round::TowardPositive);
    assert_eq!(fp::rounding(RM), Round::TowardNegative);
    assert_eq!(fp::rounding(RZ), Round::TowardZero);
    assert_eq!(fp::env(0), Env::ARM);
    assert_eq!(fp::env(fpscr::DN).nan, Env::ARM_DEFAULT_NAN.nan);
    assert!(fp::env(fpscr::FZ).flush_outputs);
    assert!(fp::env(fpscr::FZ).subnormal_inputs.reports());
}

#[test]
fn the_flags_reach_fpscr_in_the_architectures_order() {
    let mut v = 0u32;
    fp::accumulate(&mut v, Flags::INVALID);
    assert_eq!(v, fpscr::IOC);
    fp::accumulate(&mut v, Flags::DIV_BY_ZERO);
    assert_eq!(v, fpscr::IOC | fpscr::DZC);
    fp::accumulate(&mut v, Flags::OVERFLOW | Flags::UNDERFLOW | Flags::INEXACT);
    assert_eq!(v, fpscr::CUMULATIVE & !fpscr::IDC);
    fp::accumulate(&mut v, Flags::DENORMAL);
    assert_eq!(v, fpscr::CUMULATIVE);
}

#[test]
fn a_config_without_the_unit_reports_none() {
    assert_eq!(Config::CORTEX_M4.ext.fp, FpUnit::None);
    assert_eq!(Config::CORTEX_M4F.ext.fp, FpUnit::V4Sp);
    assert_eq!(Config::CORTEX_M7F.ext.fp, FpUnit::V5Sp);
    assert!(!FpUnit::V4Sp.has_v5());
    assert!(FpUnit::V5Sp.has_v5());
    assert_eq!(FpUnit::None.mvfr(), [0; 3]);
}

#[test]
fn the_part_property_selects_the_unit() {
    use crate::core::props::{Props, Value};
    let props = Props::new().with("part", Value::from("cortex-m4f"));
    let cpu = ArmV7m::from_props(&props).unwrap();
    assert_eq!(cpu.config().ext.fp, FpUnit::V4Sp);

    // `fp = false` models the same silicon with the option left out.
    let props = Props::new()
        .with("part", Value::from("cortex-m7f"))
        .with("fp", Value::from(false));
    let cpu = ArmV7m::from_props(&props).unwrap();
    assert_eq!(cpu.config().ext.fp, FpUnit::None);
}
