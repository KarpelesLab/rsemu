//! Unit tests for the Armv8-M Mainline half of the core (`super::v8m`).
//!
//! Every encoding below was checked against a real assembler
//! (`arm-none-eabi-as -mcpu=cortex-m33 -mcmse`, run as a black box) before it
//! was written down; every expected behaviour comes from DDI 0553 as
//! `super::v8m` states it.

use alloc::format;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::props::{Props, Value};
use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

use super::isa::{decode, is_32bit};
use super::sys::{Access, Exception, ccr, control, exc_return, fsr, shcsr};
use super::v8m::{CPUID_CORTEX_M33, CPUID_CORTEX_M35P, decode_exc_return};
use super::{ArmV7m, Config};

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

const ENTRY: u32 = 0x200;
const HANDLER: u32 = 0x300;
const STACK: u32 = 0x1000;
const RAM: u64 = 0x4000;

/// `B .`
const PARK: u16 = 0xe7fe;
/// `SVC #0`
const SVC: u16 = 0xdf00;
/// `BX lr`
const BX_LR: u16 = 0x4770;

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
        Harness { cpu, ram }
    }

    fn m33(code: &[u16]) -> Harness {
        Harness::new(Config::CORTEX_M33, code)
    }

    fn handler(&self, code: &[u16]) {
        for (i, half) in code.iter().enumerate() {
            self.ram
                .write_at(u64::from(HANDLER) + (i as u64) * 2, &half.to_le_bytes())
                .unwrap();
        }
    }

    fn enable_faults(&self) {
        self.cpu.with_sys(|s| {
            s.shcsr |= shcsr::USGFAULTENA | shcsr::MEMFAULTENA | shcsr::BUSFAULTENA;
        });
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

    /// Write a system-block register the way a debugger would.
    fn ppb_write(&self, addr: u32, value: u32) {
        assert!(self.cpu.with_sys(|s| s.write_word(addr, value)));
    }

    fn ppb_read(&self, addr: u32) -> u32 {
        self.cpu.with_sys(|s| s.read_word(addr, true).unwrap())
    }

    fn cfsr(&self) -> u32 {
        self.cpu.with_sys(|s| s.cfsr)
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

fn show(encoding: u32) -> alloc::string::String {
    let [a, b] = wide(encoding);
    if encoding <= 0xffff {
        assert!(!is_32bit(encoding as u16));
        format!("{}", decode(encoding as u16, 0))
    } else {
        format!("{}", decode(a, b))
    }
}

// ---------------------------------------------------------------------------
// Decode and disassembly
// ---------------------------------------------------------------------------

/// Every new encoding, as `arm-none-eabi-objdump` lists it.
const LISTING: &[(u32, &str)] = &[
    (0xe8d1_0faf, "LDA r0, [r1]"),
    (0xe8d1_0f8f, "LDAB r0, [r1]"),
    (0xe8d1_0f9f, "LDAH r0, [r1]"),
    (0xe8c1_0faf, "STL r0, [r1]"),
    (0xe8c1_0f8f, "STLB r0, [r1]"),
    (0xe8c1_0f9f, "STLH r0, [r1]"),
    (0xe8d1_0fef, "LDAEX r0, [r1]"),
    (0xe8d1_0fcf, "LDAEXB r0, [r1]"),
    (0xe8d1_0fdf, "LDAEXH r0, [r1]"),
    (0xe8c1_0fe2, "STLEX r2, r0, [r1]"),
    (0xe8c1_0fc2, "STLEXB r2, r0, [r1]"),
    (0xe8c1_0fd2, "STLEXH r2, r0, [r1]"),
    (0xe841_f000, "TT r0, r1"),
    (0xe841_f040, "TTT r0, r1"),
    (0xe841_f080, "TTA r0, r1"),
    (0xe841_f0c0, "TTAT r0, r1"),
    (0xe97f_e97f, "SG"),
    (0x470c, "BXNS r1"),
    (0x478c, "BLXNS r1"),
    (0xf3af_8014, "CSDB"),
    (0xf3bf_8f40, "SSBB"),
    (0xf3bf_8f44, "PSSBB"),
    (0xf380_880a, "MSR MSPLIM, r0"),
    (0xf3ef_800b, "MRS r0, PSPLIM"),
];

#[test]
fn every_new_encoding_disassembles_as_the_assembler_lists_it() {
    for &(encoding, text) in LISTING {
        assert_eq!(show(encoding), text, "{encoding:#010x}");
    }
}

#[test]
fn the_v7m_neighbours_of_the_new_encodings_are_unchanged() {
    // `STREX` and `STREXB` share the first halfword with `TT` and `STLB`;
    // `BX` and `BLX` share theirs with `BXNS` and `BLXNS`.
    assert_eq!(show(0xe841_3200), "STREX r2, r3, [r1, #0]");
    assert_eq!(show(0xe8c1_0f42), "STREXB r2, r0, [r1, #0]");
    assert_eq!(show(0x4708), "BX r1");
    assert_eq!(show(0x4788), "BLX r1");
    assert_eq!(show(0xf3bf_8f4f), "DSB #15");
}

/// The new encodings that are UNDEFINED on ARMv7-M.
const V8M_ONLY: &[u32] = &[
    0xe8d1_0faf,
    0xe8d1_0f8f,
    0xe8d1_0f9f,
    0xe8c1_0faf,
    0xe8c1_0f8f,
    0xe8c1_0f9f,
    0xe8d1_0fef,
    0xe8d1_0fcf,
    0xe8d1_0fdf,
    0xe8c1_0fe2,
    0xe8c1_0fc2,
    0xe8c1_0fd2,
    0xe841_f000,
    0xe841_f040,
    0xe841_f080,
    0xe841_f0c0,
    0xe97f_e97f,
];

#[test]
fn each_new_encoding_is_undefined_on_a_cortex_m4() {
    for &encoding in V8M_ONLY {
        let h = Harness::new(Config::CORTEX_M4, &code(&[encoding]));
        h.enable_faults();
        h.cpu.set_reg(1, 0x800);
        h.cpu.step();
        assert_eq!(
            h.cpu.current_exception(),
            Exception::USAGE_FAULT,
            "{encoding:#010x}"
        );
        assert_eq!(h.cfsr(), fsr::UF_UNDEFINSTR, "{encoding:#010x}");
        assert_eq!(h.word(STACK - 0x20 + 24), ENTRY, "stacked at the encoding");
    }
}

#[test]
fn bxns_is_still_a_bx_on_a_cortex_m4_and_undefined_on_an_m33() {
    let h = Harness::new(Config::CORTEX_M4, &[0x470c]);
    h.cpu.set_reg(1, 0x401);
    h.cpu.step();
    assert_eq!(h.cpu.pc(), 0x400);

    for raw in [0x470c, 0x478c] {
        let h = Harness::m33(&[raw]);
        h.enable_faults();
        h.cpu.set_reg(1, 0x401);
        h.cpu.step();
        assert_eq!(h.cpu.current_exception(), Exception::USAGE_FAULT);
        assert_eq!(h.cfsr(), fsr::UF_UNDEFINSTR);
    }
}

#[test]
fn sg_csdb_ssbb_and_pssbb_are_nops_on_an_m33() {
    let h = Harness::m33(&code(&[0xe97f_e97f, 0xf3af_8014, 0xf3bf_8f40, 0xf3bf_8f44]));
    let before = h.cpu.regs();
    h.steps(4);
    assert_eq!(h.cpu.pc(), ENTRY + 16);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.regs().r[..15], before.r[..15]);
}

// ---------------------------------------------------------------------------
// Load-acquire / store-release
// ---------------------------------------------------------------------------

#[test]
fn load_acquire_and_store_release_move_data_at_every_size() {
    // STL r0,[r1] ; LDA r2,[r1] ; STLB r0,[r3] ; LDAB r4,[r3] ; STLH r0,[r5] ; LDAH r6,[r5]
    let h = Harness::m33(&code(&[
        0xe8c1_0faf,
        0xe8d1_2faf,
        0xe8c3_0f8f,
        0xe8d3_4f8f,
        0xe8c5_0f9f,
        0xe8d5_6f9f,
    ]));
    h.cpu.set_reg(0, 0x8765_4321);
    h.cpu.set_reg(1, 0x800);
    h.cpu.set_reg(3, 0x810);
    h.cpu.set_reg(5, 0x820);
    h.set_word(0x810, 0xffff_ffff);
    h.set_word(0x820, 0xffff_ffff);
    h.steps(6);
    assert_eq!(h.word(0x800), 0x8765_4321);
    assert_eq!(h.cpu.reg(2), 0x8765_4321);
    assert_eq!(h.word(0x810), 0xffff_ff21);
    assert_eq!(h.cpu.reg(4), 0x21, "LDAB zero-extends");
    assert_eq!(h.word(0x820), 0xffff_4321);
    assert_eq!(h.cpu.reg(6), 0x4321);
}

#[test]
fn the_exclusive_forms_share_the_local_monitor() {
    // LDAEX r0,[r1] ; STLEX r2,r3,[r1] ; STLEX r4,r3,[r1]
    let h = Harness::m33(&code(&[0xe8d1_0fef, 0xe8c1_3fe2, 0xe8c1_3fe4]));
    h.set_word(0x800, 7);
    h.cpu.set_reg(1, 0x800);
    h.cpu.set_reg(3, 9);
    h.steps(3);
    assert_eq!(h.cpu.reg(0), 7);
    assert_eq!(h.cpu.reg(2), 0, "the first store succeeds");
    assert_eq!(h.cpu.reg(4), 1, "the monitor is clear for the second");
    assert_eq!(h.word(0x800), 9);
}

#[test]
fn an_unaligned_load_acquire_faults_even_without_unalign_trp() {
    let h = Harness::m33(&code(&[0xe8d1_0faf]));
    h.enable_faults();
    h.cpu.set_reg(1, 0x802);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::USAGE_FAULT);
    assert_eq!(h.cfsr(), fsr::UF_UNALIGNED);
}

// ---------------------------------------------------------------------------
// Stack limits
// ---------------------------------------------------------------------------

#[test]
fn the_stack_limit_registers_round_trip_through_mrs_and_msr() {
    // MSR MSPLIM,r0 ; MSR PSPLIM,r1 ; MRS r2,MSPLIM ; MRS r3,PSPLIM
    let h = Harness::m33(&code(&[0xf380_880a, 0xf381_880b, 0xf3ef_820a, 0xf3ef_830b]));
    h.cpu.set_reg(0, 0x0000_0807);
    h.cpu.set_reg(1, 0x0000_0c0f);
    h.steps(4);
    assert_eq!(h.cpu.reg(2), 0x800, "bits [2:0] are RES0");
    assert_eq!(h.cpu.reg(3), 0xc08);
    assert_eq!(h.cpu.stack_limits(), (0x800, 0xc08));
}

#[test]
fn unprivileged_code_cannot_touch_the_limits_and_ns_aliases_read_zero() {
    // MSR MSPLIM,r0 ; MRS r2,MSPLIM ; MRS r3,MSP_NS ; MSR MSP_NS,r0
    let h = Harness::m33(&code(&[0xf380_880a, 0xf3ef_820a, 0xf3ef_8388, 0xf380_8888]));
    h.cpu.set_stack_limits(0x400, 0);
    let mut regs = h.cpu.regs();
    regs.control = control::NPRIV;
    regs.r[0] = 0x800;
    regs.r[3] = 0xdead;
    h.cpu.set_regs(regs);
    h.steps(4);
    assert_eq!(h.cpu.stack_limits(), (0x400, 0));
    assert_eq!(h.cpu.reg(2), 0);
    assert_eq!(h.cpu.reg(3), 0, "no Security Extension: MSP_NS is RAZ");
    assert_eq!(h.cpu.msp(), STACK, "and WI");
}

#[test]
fn an_sp_write_below_the_limit_faults_and_leaves_sp_alone() {
    // SUB sp, #0x80
    let h = Harness::m33(&[0xb0a0]);
    h.enable_faults();
    h.cpu.set_stack_limits(STACK - 0x40, 0);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::USAGE_FAULT);
    assert_eq!(h.cfsr(), fsr::UF_STKOF);
    assert_eq!(h.cfsr(), 1 << 20, "CFSR bit 20");
    // The frame went where the *old* SP said, and records the faulting
    // instruction.
    assert_eq!(h.cpu.msp(), STACK - 0x20);
    assert_eq!(h.word(STACK - 0x20 + 24), ENTRY);
}

#[test]
fn the_same_write_is_fine_on_a_cortex_m4_or_above_the_limit() {
    let h = Harness::new(Config::CORTEX_M4, &[0xb0a0]);
    h.cpu.step();
    assert_eq!(h.cpu.msp(), STACK - 0x80);

    let h = Harness::m33(&[0xb0a0]);
    h.cpu.set_stack_limits(STACK - 0x80, 0);
    h.cpu.step();
    assert_eq!(h.cpu.msp(), STACK - 0x80, "the limit itself is allowed");
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
}

#[test]
fn a_push_that_would_overflow_writes_nothing() {
    // PUSH {r0-r7, lr}: 36 bytes, with only 32 above the limit — exactly
    // room for the fault's own frame.
    let h = Harness::m33(&[0xb5ff]);
    h.enable_faults();
    h.cpu.set_stack_limits(STACK - 0x20, 0);
    for k in 0..9u32 {
        h.set_word(STACK - 4 - k * 4, 0x5e5e_0000 + k);
    }
    for r in 0..8u8 {
        h.cpu.set_reg(r, 0xa0 + u32::from(r));
    }
    h.cpu.step();
    assert_eq!(h.cfsr(), fsr::UF_STKOF);
    assert_eq!(h.cpu.msp(), STACK - 0x20, "the fault frame, nothing else");
    // The word below the limit the PUSH would have written is untouched.
    assert_eq!(h.word(STACK - 36), 0x5e5e_0008);
    assert_eq!(h.word(STACK - 0x20 + 24), ENTRY);
}

#[test]
fn msr_msp_is_not_checked_against_the_limit() {
    // MSR MSP, r0
    let h = Harness::m33(&wide(0xf380_8808));
    h.cpu.set_stack_limits(0x800, 0);
    h.cpu.set_reg(0, 0x400);
    h.cpu.step();
    assert_eq!(h.cpu.msp(), 0x400);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
}

#[test]
fn exception_entry_across_the_limit_stops_sp_at_it_and_pends_stkof() {
    let h = Harness::m33(&[SVC]);
    h.enable_faults();
    h.cpu.set_stack_limits(STACK - 0x10, 0);
    for k in 0..8u32 {
        h.set_word(STACK - 0x20 + k * 4, 0x5e5e_0000 + k);
    }
    h.cpu.set_reg(12, 0xc12);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
    assert_eq!(h.cpu.msp(), STACK - 0x10, "SP is left at the limit");
    assert_eq!(h.cfsr(), fsr::UF_STKOF);
    assert!(h.cpu.with_sys(|s| s.is_pending(Exception::USAGE_FAULT)));
    // Below the limit: untouched. At and above it: the top half of the
    // frame, `R12`, `LR`, the return address and `xPSR`.
    for k in 0..4u32 {
        assert_eq!(h.word(STACK - 0x20 + k * 4), 0x5e5e_0000 + k);
    }
    assert_eq!(h.word(STACK - 0x10), 0xc12);
    assert_eq!(h.word(STACK - 0x08), ENTRY + 2);
}

#[test]
fn stkofhfnmign_ignores_the_limit_at_negative_priority() {
    let h = Harness::m33(&[0xb0a0, 0xb0a0]);
    h.cpu.set_stack_limits(STACK - 0x40, 0);
    h.ppb_write(0xe000_ed14, ccr::STKOFHFNMIGN);
    let mut regs = h.cpu.regs();
    regs.faultmask = true;
    h.cpu.set_regs(regs);
    h.cpu.step();
    assert_eq!(h.cpu.msp(), STACK - 0x80);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
}

// ---------------------------------------------------------------------------
// EXC_RETURN
// ---------------------------------------------------------------------------

#[test]
fn exception_entry_puts_the_v8m_exc_return_in_lr() {
    // From Thread mode on the main stack.
    let h = Harness::m33(&[SVC]);
    h.cpu.step();
    assert_eq!(h.cpu.reg(14), 0xffff_ffb8);

    // From Thread mode on the process stack — FreeRTOS `ARM_CM33_NTZ`'s
    // initial value.
    let h = Harness::m33(&[SVC]);
    let mut regs = h.cpu.regs();
    regs.control = control::SPSEL;
    regs.psp = 0xc00;
    h.cpu.set_regs(regs);
    h.cpu.step();
    assert_eq!(h.cpu.reg(14), 0xffff_ffbc);
    assert_eq!(h.cpu.reg(14), exc_return::V8_THREAD_PSP);

    // From Handler mode: an `SVC` inside the SVCall handler escalates to
    // HardFault, taken from Handler mode.
    let h = Harness::m33(&[SVC]);
    h.handler(&[SVC]);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::HARD_FAULT);
    assert_eq!(h.cpu.reg(14), 0xffff_ffb0);
}

#[test]
fn a_v8m_exc_return_brings_the_process_stack_back() {
    let h = Harness::m33(&[SVC, 0x2009]);
    h.handler(&[BX_LR]);
    let mut regs = h.cpu.regs();
    regs.control = control::SPSEL;
    regs.psp = 0xc00;
    h.cpu.set_regs(regs);
    h.steps(3);
    assert_eq!(h.cpu.current_exception(), Exception::THREAD);
    assert_eq!(h.cpu.psp(), 0xc00);
    assert_eq!(h.cpu.msp(), STACK);
    assert_eq!(h.cpu.reg(0), 9);
}

#[test]
fn a_v7m_exc_return_is_invpc_on_an_m33() {
    // `BX r0` with r0 = 0xFFFFFFFD: `S` and `ES` set, which a core with one
    // security state never produces.
    let h = Harness::m33(&[SVC]);
    h.handler(&[0x4700]);
    h.enable_faults();
    h.cpu.set_reg(0, 0xffff_fffd);
    h.steps(2);
    assert_ne!(h.cfsr() & fsr::UF_INVPC, 0);
}

#[test]
fn the_exc_return_decoder_takes_exactly_the_legal_values() {
    for (value, handler, psp, fp) in [
        (0xffff_ffb0, true, false, false),
        (0xffff_ffb8, false, false, false),
        (0xffff_ffbc, false, true, false),
        (0xffff_ffa0, true, false, true),
        (0xffff_ffa8, false, false, true),
        (0xffff_ffac, false, true, true),
    ] {
        let r = decode_exc_return(value).unwrap();
        assert_eq!((r.to_handler, r.use_psp, r.fp_frame), (handler, psp, fp));
    }
    for bad in [
        0xffff_fffd, // S and ES
        0xffff_ff9c, // DCRS clear
        0xffff_ffbd, // ES
        0xffff_ffbe, // bit 1
        0xffff_ffb4, // Handler mode on the process stack
        0xfeff_ffbc, // not the 0xFF prefix
        0xffff_7fbc, // a RES1 bit clear
    ] {
        assert!(decode_exc_return(bad).is_none(), "{bad:#x}");
    }
}

#[cfg(feature = "cpu-arm-v7m-fp")]
#[test]
fn a_floating_point_context_clears_ftype() {
    // VMOV s0, r0 ; SVC #0
    let h = Harness::new(Config::CORTEX_M33F, &code(&[0xee00_0a10, SVC as u32]));
    h.cpu.with_sys(|s| s.cpacr = 0xf << 20);
    h.steps(2);
    assert_eq!(h.cpu.current_exception(), Exception::SVCALL);
    assert_eq!(h.cpu.reg(14), 0xffff_ffa8);
}

// ---------------------------------------------------------------------------
// The system block
// ---------------------------------------------------------------------------

#[test]
fn the_identity_and_reset_values_are_a_cortex_m33s() {
    let h = Harness::m33(&[PARK]);
    assert_eq!(h.ppb_read(0xe000_ed00), CPUID_CORTEX_M33);
    const { assert!(CPUID_CORTEX_M33 == 0x410f_d214) };
    assert_eq!(h.ppb_read(0xe000_ed14), 0x0000_0201, "CCR");
    assert_eq!(h.ppb_read(0xe000_ed90), 8 << 8, "MPU_TYPE.DREGION");
    // CCR: NONBASETHRDENA and STKALIGN are RES1, STKOFHFNMIGN is writable,
    // the cache enables are RAZ on a part with no caches.
    h.ppb_write(0xe000_ed14, 0);
    assert_eq!(h.ppb_read(0xe000_ed14), 0x201);
    h.ppb_write(0xe000_ed14, 0x0007_0410);
    assert_eq!(h.ppb_read(0xe000_ed14), 0x611);
    // The SAU and NVIC_ITNS: no Security Extension, so zero.
    for addr in [0xe000_edd0, 0xe000_edd4, 0xe000_e380] {
        h.ppb_write(addr, 0xffff_ffff);
        assert_eq!(h.ppb_read(addr), 0, "{addr:#x}");
    }
    const { assert!(!Config::CORTEX_M33.bit_band) };
}

#[test]
fn the_pmsav8_registers_and_their_aliases() {
    let h = Harness::m33(&[PARK]);
    h.ppb_write(0xe000_ed98, 4); // RNR
    h.ppb_write(0xe000_ed9c, 0x1000_0003); // RBAR -> region 4
    h.ppb_write(0xe000_eda0, 0x1000_0ff1); // RLAR, PXN bit RES0
    h.ppb_write(0xe000_eda4, 0x2000_0000); // RBAR_A1 -> region 5
    h.ppb_write(0xe000_edb0, 0x3000_0fe1); // RLAR_A2 -> region 6
    h.ppb_write(0xe000_edb4, 0x4000_0000); // RBAR_A3 -> region 7
    h.ppb_write(0xe000_edc0, 0x0044_ff04); // MAIR0
    h.ppb_write(0xe000_edc4, 0x1122_3344); // MAIR1
    h.cpu.with_sys(|s| {
        assert_eq!(s.mpu_rbar[4], 0x1000_0003);
        assert_eq!(s.mpu_rasr[4], 0x1000_0fe1);
        assert_eq!(s.mpu_rbar[5], 0x2000_0000);
        assert_eq!(s.mpu_rasr[6], 0x3000_0fe1);
        assert_eq!(s.mpu_rbar[7], 0x4000_0000);
        assert_eq!(s.mpu_mair, [0x0044_ff04, 0x1122_3344]);
    });
    assert_eq!(h.ppb_read(0xe000_eda4), 0x2000_0000);
    h.ppb_write(0xe000_ed98, 6);
    assert_eq!(
        h.ppb_read(0xe000_eda0),
        0x3000_0fe1,
        "RLAR reads region RNR"
    );
    assert_eq!(h.ppb_read(0xe000_edac), 0x0000_0000, "RBAR_A2 -> region 6");
    assert_eq!(h.ppb_read(0xe000_edc4), 0x1122_3344);
}

#[test]
fn the_pmsav8_permission_matrix() {
    let h = Harness::m33(&[PARK]);
    h.cpu.with_sys(|s| {
        s.mpu_rbar[0] = 0x1000; // AP 00: privileged read/write
        s.mpu_rasr[0] = 0x1fe0 | 1;
        s.mpu_ctrl = 0b001;
        assert!(s.mpu_permits(0x1000, Access::Write, true, 0));
        assert!(s.mpu_permits(0x1fff, Access::Read, true, 0));
        assert!(!s.mpu_permits(0x1000, Access::Read, false, 0));
        // Outside every region, without PRIVDEFENA: nobody.
        assert!(!s.mpu_permits(0x2000, Access::Read, true, 0));
        s.mpu_ctrl = 0b101;
        assert!(s.mpu_permits(0x2000, Access::Read, true, 0));
        assert!(!s.mpu_permits(0x2000, Access::Read, false, 0));
        // AP 01: anyone, read/write.
        s.mpu_rbar[0] = 0x1000 | (0b01 << 1);
        assert!(s.mpu_permits(0x1000, Access::Write, false, 0));
        // AP 10: privileged read-only.
        s.mpu_rbar[0] = 0x1000 | (0b10 << 1);
        assert!(s.mpu_permits(0x1000, Access::Read, true, 0));
        assert!(!s.mpu_permits(0x1000, Access::Write, true, 0));
        assert!(!s.mpu_permits(0x1000, Access::Read, false, 0));
        // AP 11: anyone, read-only; XN refuses a fetch.
        s.mpu_rbar[0] = 0x1000 | (0b11 << 1) | 1;
        assert!(s.mpu_permits(0x1000, Access::Read, false, 0));
        assert!(!s.mpu_permits(0x1000, Access::Write, true, 0));
        assert!(!s.mpu_permits(0x1000, Access::Fetch, true, 0));
        // A second region overlapping the first: a fault whatever either
        // says, even for privileged code — no priority among regions.
        s.mpu_rbar[3] = 0x1800 | (0b01 << 1);
        s.mpu_rasr[3] = 0x27e0 | 1;
        assert!(!s.mpu_permits(0x1800, Access::Read, true, 0));
        assert!(
            s.mpu_permits(0x2000, Access::Read, false, 0),
            "region 3 alone"
        );
        // HFNMIENA clear: off at negative priority.
        assert!(s.mpu_permits(0x1800, Access::Read, true, -1));
    });
}

#[test]
fn an_overlap_is_a_memmanage_fault_with_mmfar() {
    // LDR r0, [r1]
    let h = Harness::m33(&[0x6808]);
    h.enable_faults();
    h.cpu.with_sys(|s| {
        s.mpu_rbar[0] = 0x800 | (0b01 << 1);
        s.mpu_rasr[0] = 0x8e0 | 1;
        s.mpu_rbar[1] = 0x880 | (0b01 << 1);
        s.mpu_rasr[1] = 0x9e0 | 1;
        s.mpu_ctrl = 0b101;
    });
    h.cpu.set_reg(1, 0x8a0);
    h.cpu.step();
    assert_eq!(h.cpu.current_exception(), Exception::MEM_MANAGE);
    assert_eq!(h.cfsr(), fsr::MM_DACCVIOL | fsr::MM_MMARVALID);
    assert_eq!(h.cpu.with_sys(|s| s.mmfar), 0x8a0);
}

#[test]
fn tt_reports_the_mpu_region_and_the_permissions() {
    // TT r0, r1 ; TTT r2, r1 ; TT r3, r4
    let h = Harness::m33(&code(&[0xe841_f000, 0xe841_f240, 0xe844_f300]));
    h.cpu.with_sys(|s| {
        s.mpu_rbar[2] = 0x800 | (0b10 << 1); // privileged read-only
        s.mpu_rasr[2] = 0x8e0 | 1;
        s.mpu_ctrl = 0b101;
    });
    h.cpu.set_reg(1, 0x840);
    h.cpu.set_reg(4, 0x2000);
    h.steps(3);
    let mrvalid = 1 << 16;
    let r = 1 << 18;
    let rw = 1 << 19;
    assert_eq!(h.cpu.reg(0), 2 | mrvalid | r);
    assert_eq!(h.cpu.reg(2), 2 | mrvalid, "TTT asks as unprivileged code");
    assert_eq!(h.cpu.reg(3), r | rw, "background region: no MREGION");

    // With the MPU off nothing is a region, and everything is accessible.
    let h = Harness::m33(&code(&[0xe841_f000]));
    h.cpu.set_reg(1, 0x840);
    h.cpu.step();
    assert_eq!(h.cpu.reg(0), r | rw);
}

#[test]
fn tta_is_undefined_without_the_security_extension() {
    let h = Harness::m33(&code(&[0xe841_f080]));
    h.enable_faults();
    h.cpu.step();
    assert_eq!(h.cfsr(), fsr::UF_UNDEFINSTR);
}

// ---------------------------------------------------------------------------
// Construction and snapshots
// ---------------------------------------------------------------------------

#[test]
fn from_props_names_the_m33_parts() {
    let props = Props::new().with("part", Value::from("cortex-m33"));
    let cpu = ArmV7m::from_props(&props).unwrap();
    assert_eq!(cpu.config(), Config::CORTEX_M33);
    assert!(cpu.config().is_v8m());
    assert!(!Config::CORTEX_M4.is_v8m());

    let props = Props::new().with("part", Value::from("cortex-m33f"));
    #[cfg(feature = "cpu-arm-v7m-fp")]
    assert_eq!(
        ArmV7m::from_props(&props).unwrap().config(),
        Config::CORTEX_M33F
    );
    #[cfg(not(feature = "cpu-arm-v7m-fp"))]
    assert!(ArmV7m::from_props(&props).is_err());
}

#[test]
fn the_m35p_is_an_m33_with_its_own_cpuid_and_no_instruction_cache() {
    // Arm Cortex-M35P r1p1: implementer 0x41, r1 in Variant, architecture
    // 0xF, p1 in Revision; the unpublished part number reads zero.
    const { assert!(CPUID_CORTEX_M35P == 0x411f_0001) };
    assert_eq!(
        Config {
            cpuid: CPUID_CORTEX_M33,
            ..Config::CORTEX_M35P
        },
        Config::CORTEX_M33,
        "everything bar CPUID is the Cortex-M33's"
    );
    assert_eq!(Config::CORTEX_M35PF.ext.fp, Config::CORTEX_M33F.ext.fp);
    assert_eq!(Config::CORTEX_M35PF.cpuid, CPUID_CORTEX_M35P);

    let h = Harness::new(Config::CORTEX_M35P, &[PARK]);
    assert_eq!(h.ppb_read(0xe000_ed00), CPUID_CORTEX_M35P);
    // Built without the optional instruction cache: CLIDR and CTR describe
    // no cache, and CCR.IC (bit 17) does not stick.
    assert_eq!(h.ppb_read(0xe000_ed78), 0, "CLIDR");
    assert_eq!(h.ppb_read(0xe000_ed7c), 0, "CTR");
    h.ppb_write(0xe000_ed14, 1 << 17);
    assert_eq!(h.ppb_read(0xe000_ed14) & (1 << 17), 0, "CCR.IC");
    assert_eq!(h.ppb_read(0xe000_ed14), 0x0000_0201, "the M33's CCR");
}

#[test]
fn from_props_names_the_m35p_parts() {
    let props = Props::new().with("part", Value::from("cortex-m35p"));
    let cpu = ArmV7m::from_props(&props).unwrap();
    assert_eq!(cpu.config(), Config::CORTEX_M35P);
    assert!(cpu.config().is_v8m());

    let props = Props::new().with("part", Value::from("cortex-m35pf"));
    #[cfg(feature = "cpu-arm-v7m-fp")]
    assert_eq!(
        ArmV7m::from_props(&props).unwrap().config(),
        Config::CORTEX_M35PF
    );
    #[cfg(not(feature = "cpu-arm-v7m-fp"))]
    assert!(ArmV7m::from_props(&props).is_err());

    // A board that knows its silicon's CPUID says so.
    let props = Props::new()
        .with("part", Value::from("cortex-m35p"))
        .with("cpuid", Value::from(0x411f_1231u64));
    assert_eq!(
        ArmV7m::from_props(&props).unwrap().config().cpuid,
        0x411f_1231
    );
}

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
fn the_v8m_state_round_trips_through_a_snapshot() {
    let h = Harness::m33(&[PARK]);
    h.cpu.set_stack_limits(0x400, 0x808);
    h.ppb_write(0xe000_edc0, 0xdead_beef);
    h.ppb_write(0xe000_edc4, 0x0bad_f00d);
    h.ppb_write(0xe000_ed14, ccr::STKOFHFNMIGN);
    let bytes = snapshot(&h.cpu);

    let fresh = Harness::m33(&[PARK]);
    restore(&fresh.cpu, &bytes);
    assert_eq!(fresh.cpu.stack_limits(), (0x400, 0x808));
    assert_eq!(
        fresh.cpu.with_sys(|s| s.mpu_mair),
        [0xdead_beef, 0x0bad_f00d]
    );
    assert_eq!(snapshot(&fresh.cpu), bytes, "an identical state hash");
}

#[test]
fn a_v7m_snapshot_is_the_shape_it_always_was() {
    let m4 = ArmV7m::new(Config::CORTEX_M4);
    let m33 = ArmV7m::new(Config {
        bit_band: true,
        ..Config::CORTEX_M33
    });
    // The limits and MAIR0/1: four words, and nothing else.
    assert_eq!(snapshot(&m33).len() - snapshot(&m4).len(), 4 * 4);
}

#[test]
fn a_warm_reset_clears_the_limits() {
    let h = Harness::m33(&[PARK]);
    h.cpu.set_stack_limits(0x400, 0x800);
    Device::reset(h.cpu.as_ref(), ResetKind::Warm);
    h.cpu.step();
    assert_eq!(h.cpu.stack_limits(), (0, 0));
    assert_eq!(h.cpu.msp(), STACK);
}
