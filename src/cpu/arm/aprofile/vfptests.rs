//! Tests for the VFP unit, driven through the core.
//!
//! [`super::vfp`] tests the arithmetic and [`super::vfpisa`] the decoder
//! against binutils; this file runs instructions and checks what the guest
//! would see — the register file, `FPSCR`, the `APSR` flags, memory, and
//! which instructions take an Undefined Instruction exception.
//!
//! Encodings are raw words with the assembler syntax in a comment, produced by
//! `arm-none-eabi-as -mfpu=vfpv3` (or `neon-vfpv4` for the encodings that must
//! *not* execute) and read back with `objdump`.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

use super::cp::{AccessKind, Fault, Mmu, Pa, PhysMem, Regime, Va};
use super::vfp::{self, VfpRegs, fpexc, fpscr};
use super::*;

/// Where each test's program starts: clear of the vector table, so an
/// Undefined Instruction exception is visible as a PC of `0x04`.
const CODE: u32 = 0x100;
/// RAM size; everything above it faults.
const RAM: u64 = 0x1_0000;

struct Rig {
    cpu: Arc<Arm>,
    ram: Arc<RamStore>,
}

fn a9() -> Config {
    Config {
        arch: Arch::CORTEX_A9,
        ..Config::ARM926EJS
    }
}

impl Rig {
    /// A Cortex-A9-architecture core with no CP15 (so `CPACR` reads all
    /// ones), reset, in Supervisor mode at [`CODE`], with the FPU **off**.
    fn off(cfg: Config, program: &[u32]) -> Rig {
        let ram = Arc::new(RamStore::new(RAM));
        let space = AddressSpace::new("cpu", 32).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::ram("ram", ram.clone()), 0)
            .expect("ram maps");
        let cpu = Arc::new(Arm::try_new(cfg).expect("the feature is on"));
        cpu.attach_space(Arc::new(space));
        let rig = Rig { cpu, ram };
        for (i, word) in program.iter().enumerate() {
            rig.write32(CODE + 4 * i as u32, *word);
        }
        rig.cpu.step();
        rig.cpu.set_pc(CODE);
        rig
    }

    /// The same with `FPEXC.EN` set, which is where most tests start.
    fn on(program: &[u32]) -> Rig {
        let rig = Rig::off(a9(), program);
        rig.edit(|v| v.fpexc = fpexc::EN);
        rig
    }

    fn write32(&self, at: u32, value: u32) {
        for (i, b) in value.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(u64::from(at) + i as u64, *b).unwrap();
        }
    }

    fn read32(&self, at: u32) -> u32 {
        let mut v = 0;
        for i in 0..4 {
            v |= u32::from(self.ram.read_u8(u64::from(at) + i).unwrap()) << (8 * i);
        }
        v
    }

    fn vfp(&self) -> VfpRegs {
        self.cpu.vfp().expect("a Cortex-A9 has VFP")
    }

    fn edit(&self, f: impl FnOnce(&mut VfpRegs)) {
        let mut v = self.vfp();
        f(&mut v);
        self.cpu.set_vfp(v);
    }

    fn step(&self, n: usize) {
        for _ in 0..n {
            self.cpu.step();
        }
    }

    /// Whether the last instruction took an Undefined Instruction exception.
    fn took_undef(&self) -> bool {
        self.cpu.mode() == Mode::UNDEFINED && self.cpu.pc() == 0x04
    }
}

fn d(v: f64) -> u64 {
    v.to_bits()
}

fn s(v: f32) -> u32 {
    v.to_bits()
}

// ---------------------------------------------------------------------------
// Arithmetic through the core
// ---------------------------------------------------------------------------

#[test]
fn arithmetic_runs_at_both_precisions() {
    let rig = Rig::on(&[
        0xee31_0b02, // vadd.f64 d0, d1, d2
        0xee81_3b02, // vdiv.f64 d3, d1, d2
        0xee30_0a81, // vadd.f32 s0, s1, s2   (overwrites the low half of d0)
        0xee01_0b02, // vmla.f64 d0, d1, d2
        0xeeb1_1bc1, // vsqrt.f64 d1, d1
    ]);
    rig.edit(|v| {
        v.set_d(1, d(9.0));
        v.set_d(2, d(4.0));
    });
    rig.step(2);
    assert_eq!(rig.vfp().d(0), d(13.0));
    assert_eq!(rig.vfp().d(3), d(2.25));
    // s1 is the high half of d0 (13.0's top word); s2 is the low half of d1.
    rig.step(1);
    let v = rig.vfp();
    let expect = vfp::add(false, u64::from(v.s(1)), u64::from(v.s(2)), vfp::env(0)).0;
    assert_eq!(u64::from(v.s(0)), expect);
    rig.edit(|v| v.set_d(0, d(1.0)));
    rig.step(2);
    assert_eq!(rig.vfp().d(0), d(37.0));
    assert_eq!(rig.vfp().d(1), d(3.0));
    // 9/4 and sqrt(9) are exact; nothing cumulative was raised.
    assert_eq!(rig.vfp().fpscr & fpscr::CUMULATIVE, 0);
}

#[test]
fn cumulative_flags_accumulate_and_stay() {
    let rig = Rig::on(&[
        0xee81_0b02, // vdiv.f64 d0, d1, d2   (1/0: DZC)
        0xee81_0b02, // vdiv.f64 d0, d1, d2   (again)
        0xee81_3b04, // vdiv.f64 d3, d1, d4   (1/3: IXC)
    ]);
    rig.edit(|v| {
        v.set_d(1, d(1.0));
        v.set_d(4, d(3.0));
    });
    rig.step(3);
    let v = rig.vfp();
    assert_eq!(v.d(0), d(f64::INFINITY));
    assert_eq!(v.fpscr & fpscr::CUMULATIVE, fpscr::DZC | fpscr::IXC);
}

#[test]
fn default_nan_and_rounding_mode_come_from_fpscr() {
    let rig = Rig::on(&[
        0xee31_0b02, // vadd.f64 d0, d1, d2
        0xee31_0b02, // vadd.f64 d0, d1, d2
        0xeefd_0b41, // vcvtr.s32.f64 s1, d1
        0xeebd_0bc1, // vcvt.s32.f64 s0, d1
    ]);
    let nan = 0x7ff0_0000_0000_0001; // signaling, payload 1
    rig.edit(|v| {
        v.set_d(1, nan);
        v.set_d(2, d(1.0));
    });
    rig.step(1);
    assert_eq!(
        rig.vfp().d(0),
        0x7ff8_0000_0000_0001,
        "quietened, payload kept"
    );
    assert_eq!(rig.vfp().fpscr & fpscr::IOC, fpscr::IOC);
    rig.edit(|v| v.fpscr = fpscr::DN);
    rig.step(1);
    assert_eq!(rig.vfp().d(0), 0x7ff8_0000_0000_0000, "the default NaN");
    // Toward −∞ for VCVTR; VCVT truncates whatever the mode.
    rig.edit(|v| {
        v.fpscr = 2 << fpscr::RMODE_SHIFT;
        v.set_d(1, d(-2.5));
    });
    rig.step(2);
    assert_eq!(rig.vfp().s(1), (-3i32) as u32);
    assert_eq!(rig.vfp().s(0), (-2i32) as u32);
}

#[test]
fn conversions_saturate_and_fixed_point_scales() {
    let rig = Rig::on(&[
        0xeebd_0bc1, // vcvt.s32.f64 s0, d1
        0xeeb8_2b41, // vcvt.f64.u32 d2, s2
        0xeebe_3b44, // vcvt.s16.f64 d3, d3, #8
        0xeebb_4bc8, // vcvt.f64.u32 d4, d4, #16
        0xeeb7_5bc9, // vcvt.f32.f64 s10, d9
        0xeeb3_6a66, // vcvtb.f16.f32 s12, s13
        0xeeb2_7ac6, // vcvtt.f32.f16 s14, s12
    ]);
    rig.edit(|v| {
        v.set_d(1, d(3e9));
        v.set_s(2, 0xffff_ffff);
        v.set_d(3, d(-1.5));
        v.set_d(4, 0xdead_beef_0003_8000); // only the low 32 bits are read
        v.set_d(9, d(0.1));
        v.set_s(13, s(-2.0));
    });
    rig.step(7);
    let v = rig.vfp();
    assert_eq!(v.s(0), 0x7fff_ffff, "saturated");
    assert_eq!(v.fpscr & fpscr::IOC, fpscr::IOC);
    assert_eq!(v.d(2), d(4_294_967_295.0));
    // -1.5 × 256 = -384, sign-extended to 64 bits.
    assert_eq!(v.d(3), (-384i64) as u64);
    // 0x38000 / 65536 = 3.5.
    assert_eq!(v.d(4), d(3.5));
    assert_eq!(v.s(10), s(0.1));
    assert_eq!(v.s(12) & 0xffff, 0xc000, "-2.0 as a half");
    // VCVTT reads the *top* half of s12, which the VCVTB left alone (zero).
    assert_eq!(v.s(14), 0);
}

#[test]
fn signed_zero_survives() {
    let rig = Rig::on(&[
        0xee31_0b42, // vsub.f64 d0, d1, d2
        0xeeb1_4b44, // vneg.f64 d4, d4
    ]);
    rig.edit(|v| {
        v.set_d(1, d(1.0));
        v.set_d(2, d(1.0));
        v.fpscr = 2 << fpscr::RMODE_SHIFT;
    });
    rig.step(2);
    assert_eq!(rig.vfp().d(0), d(-0.0), "x - x toward −∞ is −0");
    assert_eq!(rig.vfp().d(4), d(-0.0), "VNEG of +0");
}

// ---------------------------------------------------------------------------
// Register file and transfers
// ---------------------------------------------------------------------------

#[test]
fn singles_and_doubles_alias_through_the_transfers() {
    let rig = Rig::on(&[
        0xee00_1a10, // vmov s0, r1
        0xee00_2a90, // vmov s1, r2
        0xec54_3b10, // vmov r3, r4, d0
        0xee20_1b90, // vmov.32 d16[1], r1
        0xee11_5a90, // vmov r5, s3
        0xeeb7_3b08, // vmov.f64 d3, #1.5
    ]);
    rig.cpu.set_reg(1, 0x1111_1111);
    rig.cpu.set_reg(2, 0x2222_2222);
    rig.edit(|v| v.set_d(1, 0xaaaa_aaaa_bbbb_bbbb));
    rig.step(6);
    let v = rig.vfp();
    assert_eq!(v.d(0), 0x2222_2222_1111_1111);
    assert_eq!(rig.cpu.reg(3), 0x1111_1111);
    assert_eq!(rig.cpu.reg(4), 0x2222_2222);
    assert_eq!(v.d(16), 0x1111_1111_0000_0000);
    assert_eq!(rig.cpu.reg(5), 0xaaaa_aaaa, "s3 is the high half of d1");
    assert_eq!(v.d(3), d(1.5));
}

#[test]
fn vmrs_apsr_nzcv_carries_the_comparison() {
    let rig = Rig::on(&[
        0xeeb4_0b41, // vcmp.f64 d0, d1
        0xeef1_fa10, // vmrs APSR_nzcv, fpscr
    ]);
    // 1.0 < 2.0: N set, the others clear.
    rig.edit(|v| {
        v.set_d(0, d(1.0));
        v.set_d(1, d(2.0));
    });
    rig.cpu.set_cpsr(rig.cpu.cpsr() | psr::Z | psr::C | psr::V);
    rig.step(2);
    let flags = rig.cpu.cpsr() & 0xf000_0000;
    assert_eq!(flags, psr::N);
    // Unordered: C and V.
    rig.cpu.set_pc(CODE);
    rig.edit(|v| v.set_d(1, 0x7ff8_0000_0000_0000));
    rig.step(2);
    assert_eq!(rig.cpu.cpsr() & 0xf000_0000, psr::C | psr::V);
    // Equal, and the mode bits are not disturbed.
    rig.cpu.set_pc(CODE);
    rig.edit(|v| v.set_d(1, d(1.0)));
    rig.step(2);
    assert_eq!(rig.cpu.cpsr() & 0xf000_0000, psr::Z | psr::C);
    assert_eq!(rig.cpu.mode(), Mode::SUPERVISOR);
}

#[test]
fn the_id_registers_read_from_pl1_with_the_unit_off() {
    let rig = Rig::off(
        a9(),
        &[
            0xe3a0_0101, // mov r0, #0x40000000
            0xeee8_0a10, // vmsr fpexc, r0
            0xeef8_1a10, // vmrs r1, fpexc
            0xeef0_2a10, // vmrs r2, fpsid
            0xeef7_3a10, // vmrs r3, mvfr0
            0xeef6_4a10, // vmrs r4, mvfr1
        ],
    );
    rig.step(6);
    assert!(!rig.took_undef());
    assert_eq!(rig.cpu.reg(1), fpexc::EN);
    assert_eq!(rig.cpu.reg(2), 0x4103_3094);
    assert_eq!(rig.cpu.reg(3), 0x1011_0222);
    assert_eq!(rig.cpu.reg(4), 0x0100_0011, "no Advanced SIMD reported");
    assert!(rig.vfp().enabled());
}

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------

#[test]
fn load_and_store_multiple_with_writeback() {
    let rig = Rig::on(&[
        0xed20_0b06, // vstmdb r0!, {d0-d2}
        0xecf0_0b06, // vldmia r0!, {d16-d18}
        0xed2d_0a03, // vpush {s0-s2}
        0xecbd_2a03, // vpop {s4-s6}
        0xed10_5b02, // vldr d5, [r0, #-8]
        0xedc0_3a01, // vstr s7, [r0, #4]
    ]);
    rig.cpu.set_reg(0, 0x8000);
    rig.cpu.set_reg(13, 0x9000);
    rig.edit(|v| {
        v.set_d(0, 0x0000_0001_0000_0000);
        v.set_d(1, 0x0000_0003_0000_0002);
        v.set_d(2, 0x0000_0005_0000_0004);
    });
    rig.step(1);
    assert_eq!(rig.cpu.reg(0), 0x8000 - 24);
    // Little-endian: each double's low word first.
    let words: Vec<u32> = (0..6).map(|i| rig.read32(0x8000 - 24 + 4 * i)).collect();
    assert_eq!(words, [0, 1, 2, 3, 4, 5]);
    rig.step(1);
    assert_eq!(rig.cpu.reg(0), 0x8000);
    let v = rig.vfp();
    assert_eq!([v.d(16), v.d(17), v.d(18)], [v.d(0), v.d(1), v.d(2)]);
    rig.step(2);
    assert_eq!(rig.cpu.reg(13), 0x9000);
    let v = rig.vfp();
    assert_eq!([v.s(4), v.s(5), v.s(6)], [0, 1, 2]);
    rig.step(2);
    // d5 from [0x7ff8]: words 4 and 5 of what the VSTMDB stored.
    assert_eq!(rig.vfp().d(5), 0x0000_0005_0000_0004);
    // s7 is the high half of d3, still zero; stored over word 1 at 0x8004.
    assert_eq!(rig.read32(0x8004), 0);
}

#[test]
fn fldmx_and_fstmx_move_the_base_one_word_further() {
    let rig = Rig::on(&[
        0xed20_0b05, // fstmdbx r0!, {d0-d1}
        0xecb0_4b05, // fldmiax r0!, {d4-d5}
    ]);
    rig.cpu.set_reg(0, 0x8000);
    rig.edit(|v| {
        v.set_d(0, 0x1111_1111_2222_2222);
        v.set_d(1, 0x3333_3333_4444_4444);
    });
    rig.step(1);
    assert_eq!(rig.cpu.reg(0), 0x8000 - 20, "imm8 = 5 words");
    rig.step(1);
    assert_eq!(rig.cpu.reg(0), 0x8000);
    assert_eq!(rig.vfp().d(4), 0x1111_1111_2222_2222);
    assert_eq!(rig.vfp().d(5), 0x3333_3333_4444_4444);
}

#[test]
fn an_abort_part_way_through_vldm_changes_no_register() {
    let rig = Rig::on(&[
        0xecb0_0b08, // vldmia r0!, {d0-d3}
    ]);
    // Two doubles in RAM, then the end of it.
    let base = (RAM as u32) - 16;
    rig.cpu.set_reg(0, base);
    rig.edit(|v| {
        for i in 0..4 {
            v.set_d(i, 0x5555_0000 + u64::from(i));
        }
    });
    rig.write32(base, 0xffff_ffff);
    rig.step(1);
    assert_eq!(rig.cpu.mode(), Mode::ABORT);
    assert_eq!(rig.cpu.pc(), 0x10);
    assert_eq!(rig.cpu.reg(0), base, "the base is restored");
    let v = rig.vfp();
    for i in 0..4 {
        assert_eq!(v.d(i), 0x5555_0000 + u64::from(i), "d{i} untouched");
    }
}

#[test]
fn an_unaligned_vldr_is_an_alignment_fault_whatever_sctlr_a_says() {
    let rig = Rig::on(&[
        0xed90_0b00, // vldr d0, [r0]
    ]);
    rig.cpu.set_reg(0, 0x8002);
    rig.step(1);
    assert_eq!(rig.cpu.mode(), Mode::ABORT);
    assert_eq!(rig.cpu.pc(), 0x10);
}

// ---------------------------------------------------------------------------
// Who may execute what
// ---------------------------------------------------------------------------

#[test]
fn fpexc_en_clear_makes_vfp_undefined() {
    let rig = Rig::off(a9(), &[0xee31_0b02]); // vadd.f64 d0, d1, d2
    rig.step(1);
    assert!(rig.took_undef());
    assert_eq!(rig.cpu.reg(14), CODE + 4);
    // VMRS FPSCR needs EN too.
    let rig = Rig::off(a9(), &[0xeef1_0a10]); // vmrs r0, fpscr
    rig.step(1);
    assert!(rig.took_undef());
}

#[test]
fn fpexc_is_privileged() {
    let rig = Rig::on(&[
        0xeef1_0a10, // vmrs r0, fpscr   (fine in User mode)
        0xeef8_1a10, // vmrs r1, fpexc   (not)
    ]);
    rig.cpu.set_cpsr(u32::from(Mode::USER.0));
    rig.step(1);
    assert!(!rig.took_undef());
    rig.step(1);
    assert!(rig.took_undef());
}

/// An MMU that reports a chosen `CPACR`.
#[derive(Debug)]
struct Cpacr(u32);

impl Mmu for Cpacr {
    fn regime(&self) -> Regime {
        Regime {
            cp_access: self.0,
            ..Regime::FLAT
        }
    }

    fn translate(
        &self,
        _mem: &dyn PhysMem,
        va: Va,
        _kind: AccessKind,
        _privileged: bool,
    ) -> core::result::Result<Pa, Fault> {
        Ok(Pa(va.0))
    }
}

#[test]
fn cpacr_gates_each_coprocessor_by_privilege() {
    // vadd.f32 is cp10, vadd.f64 is cp11.
    let program = [0xee30_0a81, 0xee31_0b02];
    let run = |cpacr: u32, user: bool, at: u32| {
        let rig = Rig::on(&program);
        rig.cpu.attach_mmu(Arc::new(Cpacr(cpacr)));
        if user {
            rig.cpu.set_cpsr(u32::from(Mode::USER.0));
        }
        rig.cpu.set_pc(at);
        rig.step(1);
        rig.took_undef()
    };
    let (sp, dp) = (CODE, CODE + 4);
    // Nothing granted.
    assert!(run(0, false, sp));
    // cp10 full, cp11 nothing: single works, double traps.
    assert!(!run(0b11 << 20, true, sp));
    assert!(run(0b11 << 20, false, dp));
    // PL1-only: Supervisor may, User may not.
    let pl1 = 0b0101 << 20;
    assert!(!run(pl1, false, dp));
    assert!(run(pl1, true, dp));
    // The reserved 0b10 grants nothing.
    assert!(run(0b1010 << 20, false, sp));
}

#[test]
fn short_vectors_make_data_processing_undefined() {
    let rig = Rig::on(&[
        0xee31_0b02, // vadd.f64 d0, d1, d2
        0xeef1_0a10, // vmrs r0, fpscr
    ]);
    rig.edit(|v| v.fpscr = 1 << 16); // Len = 1
    rig.step(1);
    assert!(rig.took_undef());
    // A transfer is not data processing and still runs.
    let rig = Rig::on(&[0xeef1_0a10]);
    rig.edit(|v| v.fpscr = 1 << 20); // Stride = 1
    rig.step(1);
    assert!(!rig.took_undef());
    assert_eq!(rig.cpu.reg(0), 1 << 20);
}

#[test]
fn neon_and_vfpv4_encodings_are_undefined() {
    for word in [
        0xf221_0802, // vadd.i32 d0, d1, d2
        0xf420_078f, // vld1.32 {d0}, [r0]
        0xee40_1b30, // vmov.8 d0[1], r1
        0xeea1_0b02, // vfma.f64 d0, d1, d2
    ] {
        let rig = Rig::on(&[word]);
        rig.cpu.set_reg(0, 0x8000);
        rig.step(1);
        assert!(rig.took_undef(), "{word:08x} executed");
    }
}

#[test]
fn a_d16_part_has_no_d16_to_d31() {
    let cfg = Config {
        arch: Arch {
            ext: Extensions {
                vfp: Some(Vfp::V3_D16),
                ..Arch::CORTEX_A9.ext
            },
            ..Arch::CORTEX_A9
        },
        ..Config::ARM926EJS
    };
    let rig = Rig::off(cfg, &[0xee71_0bef, 0xee31_0b02]); // vsub.f64 d16, d17, d31; vadd.f64 d0-d2
    rig.edit(|v| v.fpexc = fpexc::EN);
    rig.step(1);
    assert!(rig.took_undef());
    rig.cpu.set_pc(CODE + 4);
    rig.cpu
        .set_cpsr(u32::from(Mode::SUPERVISOR.0) | psr::I | psr::F);
    rig.step(1);
    assert!(!rig.took_undef());
}

#[test]
fn a_part_without_vfp_leaves_cp10_to_the_coprocessor_seam() {
    let rig = Rig::off(Config::ARM926EJS, &[0xee31_0b02]);
    assert!(rig.cpu.vfp().is_none());
    rig.step(1);
    assert!(rig.took_undef());
}

#[test]
fn reset_turns_the_unit_off_and_keeps_the_registers() {
    let rig = Rig::on(&[]);
    rig.edit(|v| v.set_d(7, 77));
    rig.cpu.request_reset();
    rig.step(1);
    let v = rig.vfp();
    assert!(!v.enabled());
    assert_eq!(v.d(7), 77);
}

// ---------------------------------------------------------------------------
// Construction, snapshot, disassembly
// ---------------------------------------------------------------------------

#[test]
fn cortex_a9_constructs_from_props() {
    let props = crate::core::props::Props::new().with("cpu", "cortex-a9");
    let cpu = Arm::from_props(&props).expect("the feature is on");
    assert_eq!(cpu.config().arch.ext.vfp, Some(Vfp::V3_D32));
    let neon = Config {
        arch: Arch {
            ext: Extensions {
                neon: true,
                ..Arch::CORTEX_A9.ext
            },
            ..Arch::CORTEX_A9
        },
        ..Config::ARM926EJS
    };
    assert!(Arm::try_new(neon).is_err(), "NEON is not implemented");
}

fn snapshot(cpu: &Arm) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("cpu", CLASS.name, CLASS.version).unwrap();
        cpu.save(&mut chunk).unwrap();
    }
    writer.to_vec().unwrap()
}

#[test]
fn the_vfp_state_round_trips_through_a_snapshot() {
    let rig = Rig::on(&[0xee31_0b02]);
    rig.edit(|v| {
        for i in 0..32 {
            v.set_d(i, 0x0101_0101_0101_0101 * u64::from(i));
        }
        v.fpscr = fpscr::N | fpscr::DN | fpscr::IXC | (1 << fpscr::RMODE_SHIFT);
    });
    rig.step(1);
    let bytes = snapshot(&rig.cpu);

    let restored = Arm::try_new(a9()).unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    restored.load(&mut chunk.reader()).unwrap();

    assert_eq!(restored.vfp(), rig.cpu.vfp());
    assert_eq!(restored.regs(), rig.cpu.regs());
    // The whole chunk, re-saved, is byte-identical: the state hash matches.
    assert_eq!(snapshot(&restored), bytes);
}

#[test]
fn the_listing_prints_vfp_in_ual() {
    let listed = disasm::disassemble_arm(0x100, 0x0e31_0b02);
    assert_eq!(
        alloc::format!("{listed}"),
        "00000100: 0e310b02  VADDEQ.F64 d0, d1, d2"
    );
    let listed = disasm::disassemble_arm(0x104, 0xed2d_8b10);
    assert_eq!(
        alloc::format!("{listed}"),
        "00000104: ed2d8b10  VPUSH {d8-d15}"
    );
}
