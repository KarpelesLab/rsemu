//! AVX, executed as a guest sees it: the gate, the upper halves, and each
//! instruction on hand-checked values.
//!
//! Every expected value is either from the *Intel SDM* — the section or the
//! instruction page is named on each test — or arithmetic that can be checked
//! by hand: `1.0 + 2.0` is `3.0` in any format. The encodings are built by
//! [`vex`] from the fields the SDM's encoding lines give (`VEX.256.66.0F38.W0
//! 18 /r` and so on), so each test names the instruction it means.

use super::*;
use crate::cpu::x86::fpu::mxcsr;
use crate::cpu::x86::prot::{cr4, xcr0};

/// Scratch memory, thirty-two-byte aligned.
const DATA: u64 = 0xb000;
/// A second scratch area.
const OUT: u64 = 0xc000;
/// Where a fault handler sits, and `RIP` once its `hlt` has retired.
const HANDLER: u64 = 0x3800;

/// The `VEX.mmmmm` maps.
const M0F: u8 = 1;
const M38: u8 = 2;
const M3A: u8 = 3;
/// The `VEX.pp` columns.
const NP: u8 = 0;
const P66: u8 = 1;
const PF3: u8 = 2;
const PF2: u8 = 3;

type Ymm = [[u64; 2]; 2];

/// A three-byte VEX instruction for registers 0-7: `R̄X̄B̄` all set, then
/// `map`, `W`, `vvvv` (inverted here), `L` and `pp`, the opcode, and `rest`
/// (ModRM, displacement, immediate).
fn vex(map: u8, pp: u8, l: bool, w: bool, vvvv: u8, opcode: u8, rest: &[u8]) -> Vec<u8> {
    let mut out = alloc::vec![
        0xc4,
        0xe0 | map,
        (u8::from(w) << 7) | ((!vvvv & 15) << 3) | (u8::from(l) << 2) | pp,
        opcode,
    ];
    out.extend_from_slice(rest);
    out
}

/// A register-form ModRM byte.
const fn rr(reg: u8, rm: u8) -> u8 {
    0xc0 | (reg << 3) | rm
}

/// A `disp32` ModRM naming `reg` and the absolute address `addr`.
fn m32(reg: u8, addr: u64) -> Vec<u8> {
    let mut out = alloc::vec![(reg << 3) | 0b101];
    out.extend_from_slice(&(addr as u32).to_le_bytes());
    out
}

/// `rest` with an immediate appended.
fn with(mut rest: Vec<u8>, imm: u8) -> Vec<u8> {
    rest.push(imm);
    rest
}

fn features() -> Features {
    Features {
        xsave: true,
        avx: true,
        ..Features::X86_64_V2
    }
}

/// A part with `features` in flat 32-bit protected mode, with the SSE bits
/// of `CR4`, `CR4.OSXSAVE` and `XCR0 = x87 | SSE | AVX` — what an operating
/// system that uses AVX sets — and a `hlt` handler for every vector a test
/// here can raise.
fn pc_with(features: Features) -> Pc {
    let pc = Pc::with_features(Variant::X86_64, features);
    pc.start_protected();
    let mut sys = pc.cpu.sys();
    sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
    if features.xsave {
        sys.cr4 |= cr4::OSXSAVE;
        sys.xcr0 = xcr0::X87 | xcr0::SSE | if features.avx { xcr0::AVX } else { 0 };
    }
    pc.cpu.set_sys(sys);
    for v in [6u64, 7, 13, 19] {
        let at = HANDLER + v * 16;
        pc.idt(v, gate(0x08, at as u32, sys_type::INT_GATE32, 0));
        pc.write(at, &[0xf4]);
    }
    pc
}

fn avx() -> Pc {
    pc_with(features())
}

/// Run `code` and a `hlt`; the vector it faulted with, or `None`.
fn exec(pc: &Pc, code: &[u8]) -> Option<u64> {
    let mut all = code.to_vec();
    all.push(0xf4);
    pc.write(at::CODE0, &all);
    let mut regs = pc.regs();
    regs.rip = at::CODE0;
    pc.cpu.set_regs(regs);
    pc.cpu.session.lock().state.halted = false;
    assert!(pc.run(400) < 400, "the program halted");
    let rip = pc.regs().rip;
    if rip == at::CODE0 + all.len() as u64 {
        return None;
    }
    let vector = [6u64, 7, 13, 19]
        .into_iter()
        .find(|v| rip == HANDLER + v * 16 + 1);
    assert!(vector.is_some(), "stopped at {rip:#x}");
    vector
}

/// Run `code` and require that it does not fault.
fn run(pc: &Pc, code: &[u8]) {
    assert_eq!(exec(pc, code), None, "{code:02x?}");
}

fn ymm(pc: &Pc, n: u8) -> Ymm {
    pc.cpu.sse().ymm(n)
}

fn set_ymm(pc: &Pc, values: &[(u8, Ymm)]) {
    let mut sse = pc.cpu.sse();
    for &(n, v) in values {
        sse.set_ymm(n, v);
    }
    pc.cpu.set_sse(sse);
}

fn write_ymm(pc: &Pc, addr: u64, v: Ymm) {
    for (i, q) in [v[0][0], v[0][1], v[1][0], v[1][1]].into_iter().enumerate() {
        write64(pc, addr + 8 * i as u64, q);
    }
}

fn write64(pc: &Pc, addr: u64, value: u64) {
    pc.write(addr, &value.to_le_bytes());
}

fn read64(pc: &Pc, addr: u64) -> u64 {
    (0..8u64).fold(0, |v, i| {
        v | (u64::from(pc.ram.read_u8(addr + i).unwrap()) << (8 * i))
    })
}

/// Eight singles, low lane first, as a `YMM` value.
fn ps(v: [u32; 8]) -> Ymm {
    let q = |i: usize| u64::from(v[i]) | (u64::from(v[i + 1]) << 32);
    [[q(0), q(2)], [q(4), q(6)]]
}

/// Four doubles (any four quadwords), low first.
const fn pd(v: [u64; 4]) -> Ymm {
    [[v[0], v[1]], [v[2], v[3]]]
}

// Singles.
const S1: u32 = 0x3f80_0000;
const S2: u32 = 0x4000_0000;
const S3: u32 = 0x4040_0000;
const S4: u32 = 0x4080_0000;
const S5: u32 = 0x40a0_0000;
const S6: u32 = 0x40c0_0000;
const S7: u32 = 0x40e0_0000;
const S8: u32 = 0x4100_0000;
const S9: u32 = 0x4110_0000;
const S16: u32 = 0x4180_0000;
const SQNAN: u32 = 0x7fc0_0000;
// Doubles.
const D1: u64 = 0x3ff0_0000_0000_0000;
const D2: u64 = 0x4000_0000_0000_0000;
const D3: u64 = 0x4008_0000_0000_0000;
const D4: u64 = 0x4010_0000_0000_0000;

/// `vaddps ymm1, ymm2, ymm3`.
fn vaddps256() -> Vec<u8> {
    vex(M0F, NP, true, false, 2, 0x58, &[rr(1, 3)])
}

// -- The gate -----------------------------------------------------------------

#[test]
fn cpuid_reports_avx_and_validate_wants_xsave_and_sse42() {
    let leaf1 = |features: Features| {
        let pc = pc_with(features);
        run(&pc, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2]);
        pc.regs().rcx & 0xffff_ffff
    };
    assert_ne!(leaf1(features()) & (1 << 28), 0, "CPUID.01H:ECX.AVX");
    assert_eq!(
        leaf1(Features {
            avx: false,
            ..features()
        }) & (1 << 28),
        0
    );
    assert!(features().validate().is_ok());
    for impossible in [
        Features {
            xsave: false,
            ..features()
        },
        Features {
            sse42: false,
            ..features()
        },
    ] {
        assert!(impossible.validate().is_err());
    }
}

#[test]
fn every_gate_is_undefined_opcode_except_the_task_switched_flag() {
    // *Intel SDM* volume 1 §14.3: `#UD` without the CPUID bit, without
    // `CR4.OSXSAVE`, or with `XCR0[2:1]` not `11b`; then `#NM` for `CR0.TS`.
    let code = vaddps256();
    let without = Features {
        avx: false,
        ..features()
    };
    assert_eq!(exec(&pc_with(without), &code), Some(6), "no AVX");
    let pc = avx();
    let mut sys = pc.cpu.sys();
    sys.cr4 &= !cr4::OSXSAVE;
    pc.cpu.set_sys(sys);
    assert_eq!(exec(&pc, &code), Some(6), "no OSXSAVE");
    for enabled in [xcr0::X87 | xcr0::SSE, xcr0::X87] {
        let pc = avx();
        let mut sys = pc.cpu.sys();
        sys.xcr0 = enabled;
        pc.cpu.set_sys(sys);
        assert_eq!(exec(&pc, &code), Some(6), "XCR0 = {enabled:#x}");
    }
    let pc = avx();
    let mut sys = pc.cpu.sys();
    sys.cr0 |= cr0::TS;
    pc.cpu.set_sys(sys);
    assert_eq!(exec(&pc, &code), Some(7), "CR0.TS");
    // `VZEROUPPER` is gated like the rest.
    assert_eq!(exec(&pc, &[0xc5, 0xf8, 0x77]), Some(7));
}

#[test]
fn cr0_em_and_cr4_osfxsr_do_not_gate_a_vex_instruction() {
    // Legacy SSE is `#UD` with `CR0.EM` set or `CR4.OSFXSR` clear; a VEX
    // encoding consults neither (*Intel SDM* volume 2 §2.4, the exception
    // class tables, and volume 1 §14.3).
    let pc = avx();
    let mut sys = pc.cpu.sys();
    sys.cr0 |= cr0::EM;
    sys.cr4 &= !cr4::OSFXSR;
    pc.cpu.set_sys(sys);
    set_ymm(&pc, &[(2, ps([S1; 8])), (3, ps([S2; 8]))]);
    run(&pc, &vaddps256());
    assert_eq!(ymm(&pc, 1), ps([S3; 8]));
    // While the legacy encoding of the same operation is refused.
    assert_eq!(exec(&pc, &[0x0f, 0x58, 0xca]), Some(6));
}

#[test]
fn invalid_vex_encodings_are_undefined_opcode() {
    let pc = avx();
    // `VEX.L = 1` on a 128-bit-only row: `vpaddb ymm1, ymm2, ymm3`.
    assert_eq!(
        exec(&pc, &vex(M0F, P66, true, false, 2, 0xfc, &[rr(1, 3)])),
        Some(6)
    );
    // `vvvv` not `1111b` on a row that does not read it: `vsqrtps`.
    assert_eq!(
        exec(&pc, &vex(M0F, NP, false, false, 2, 0x51, &[rr(1, 3)])),
        Some(6)
    );
    // `VEX.W1` on a `W0` row: `vpermilps`.
    assert_eq!(
        exec(&pc, &vex(M38, P66, false, true, 2, 0x0c, &[rr(1, 3)])),
        Some(6)
    );
    // A legacy prefix in front: `66 vaddps`.
    let mut code = alloc::vec![0x66];
    code.extend(vaddps256());
    assert_eq!(exec(&pc, &code), Some(6));
    // `LOCK` in front.
    let mut code = alloc::vec![0xf0];
    code.extend(vaddps256());
    assert_eq!(exec(&pc, &code), Some(6));
    // And the encodings that do read `vvvv` accept it.
    run(&pc, &vex(M0F, P66, false, false, 2, 0xfc, &[rr(1, 3)]));
}

#[test]
fn in_real_mode_vex_is_undefined_and_c4_with_memory_is_les() {
    // *Intel SDM* volume 2 §2.3.5 and the exception classes: VEX is `#UD`
    // in real-address mode, where `C4` and `C5` with a memory operand are
    // `LES` and `LDS` as they always were.
    let pc = Pc::with_features(Variant::X86_64, features());
    let mut sys = pc.cpu.sys();
    sys.cr4 |= cr4::OSXSAVE;
    sys.xcr0 = 7;
    pc.cpu.set_sys(sys);
    // Interrupt vector 6 at 0000:0500, a `hlt`.
    pc.write(6 * 4, &[0x00, 0x05, 0x00, 0x00]);
    pc.write(0x500, &[0xf4]);
    // les ax, [0x0600], then vaddps xmm1, xmm2, xmm3.
    pc.write(0x600, &[0x34, 0x12, 0x78, 0x56]);
    pc.write(
        0x7c00,
        &[0xc4, 0x06, 0x00, 0x06, 0xc5, 0xe8, 0x58, 0xcb, 0xf4],
    );
    pc.start_real(0, 0x7c00);
    assert!(pc.run(20) < 20);
    let regs = pc.regs();
    assert_eq!(regs.rax & 0xffff, 0x1234);
    assert_eq!(regs.es, 0x5678);
    assert_eq!(
        (regs.cs, regs.rip),
        (0, 0x501),
        "#UD at the VEX instruction"
    );
}

// -- The upper halves ---------------------------------------------------------

#[test]
fn legacy_sse_keeps_the_upper_half_and_vex_128_zeroes_it() {
    // *Intel SDM* volume 1 §14.1.3: a legacy SSE instruction leaves bits
    // 255:128 of its destination as they were; a `VEX.128` one zeroes them;
    // a `VEX.256` one writes them.
    let pc = avx();
    let upper = [0x1111, 0x2222];
    set_ymm(
        &pc,
        &[
            (1, [[0, 0], upper]),
            (2, ps([S1, S1, S1, S1, S2, S2, S2, S2])),
            (3, ps([S2, S2, S2, S2, S4, S4, S4, S4])),
        ],
    );
    run(&pc, &[0x0f, 0x28, 0xca, 0x0f, 0x58, 0xcb]); // movaps xmm1, xmm2 ; addps xmm1, xmm3
    assert_eq!(ymm(&pc, 1), [ps([S3; 8])[0], upper]);
    run(&pc, &vex(M0F, NP, false, false, 2, 0x58, &[rr(1, 3)])); // vaddps xmm1, xmm2, xmm3
    assert_eq!(ymm(&pc, 1), [ps([S3; 8])[0], [0, 0]]);
    run(&pc, &vaddps256());
    assert_eq!(ymm(&pc, 1), ps([S3, S3, S3, S3, S6, S6, S6, S6]));
    // A `VEX.128` integer instruction and a `VEX.128` move zero it too.
    set_ymm(&pc, &[(1, [[0, 0], upper])]);
    run(&pc, &vex(M0F, P66, false, false, 2, 0xfc, &[rr(1, 3)])); // vpaddb
    assert_eq!(ymm(&pc, 1)[1], [0, 0]);
    set_ymm(&pc, &[(1, [[0, 0], upper])]);
    run(&pc, &vex(M0F, NP, false, false, 0, 0x28, &[rr(1, 3)])); // vmovaps xmm1, xmm3
    assert_eq!(ymm(&pc, 1), [ymm(&pc, 3)[0], [0, 0]]);
    // So does a `VEX.128` transfer from a general register.
    set_ymm(&pc, &[(1, [[0, 0], upper])]);
    run(&pc, &vex(M0F, P66, false, false, 0, 0x6e, &[rr(1, 0)])); // vmovd xmm1, eax
    assert_eq!(ymm(&pc, 1)[1], [0, 0]);
}

#[test]
fn vzeroupper_and_vzeroall_touch_only_the_architectural_registers() {
    // *Intel SDM* volume 2: outside 64-bit mode they modify `YMM0`-`YMM7`
    // only.
    let pc = avx();
    let mut sse = pc.cpu.sse();
    for n in 0..16u8 {
        sse.set_ymm(n, [[u64::from(n) + 1, 2], [3, 4]]);
    }
    pc.cpu.set_sse(sse);
    run(&pc, &[0xc5, 0xf8, 0x77]); // vzeroupper
    for n in 0..8u8 {
        assert_eq!(ymm(&pc, n), [[u64::from(n) + 1, 2], [0, 0]]);
    }
    assert_eq!(ymm(&pc, 8), [[9, 2], [3, 4]], "YMM8 is not state here");
    run(&pc, &[0xc5, 0xfc, 0x77]); // vzeroall
    for n in 0..8u8 {
        assert_eq!(ymm(&pc, n), [[0, 0], [0, 0]]);
    }
    assert_eq!(ymm(&pc, 8), [[9, 2], [3, 4]]);
}

// -- Arithmetic, at both lengths ----------------------------------------------

#[test]
fn the_256_bit_arithmetic_works_lane_by_lane() {
    let pc = avx();
    set_ymm(
        &pc,
        &[
            (2, pd([D4, D3, D2, D4])),
            (3, pd([D1, D1, D1, D2])),
            (4, ps([S4, S9, S16, S1, S4, S9, S16, S1])),
        ],
    );
    run(&pc, &vex(M0F, P66, true, false, 2, 0x5c, &[rr(1, 3)])); // vsubpd ymm1, ymm2, ymm3
    assert_eq!(ymm(&pc, 1), pd([D3, D2, D1, D2]));
    run(&pc, &vex(M0F, NP, true, false, 0, 0x51, &[rr(5, 4)])); // vsqrtps ymm5, ymm4
    assert_eq!(ymm(&pc, 5), ps([S2, S3, S4, S1, S2, S3, S4, S1]));
    // A memory source needs no alignment: `vmulpd ymm1, ymm2, [DATA+8]`.
    write_ymm(&pc, DATA + 8, pd([D2, D2, D2, D2]));
    run(&pc, &vex(M0F, P66, true, false, 2, 0x59, &m32(1, DATA + 8)));
    assert_eq!(
        ymm(&pc, 1),
        pd([
            0x4020_0000_0000_0000,
            0x4018_0000_0000_0000,
            D4,
            0x4020_0000_0000_0000
        ])
    );
}

#[test]
fn a_scalar_takes_the_rest_of_its_register_from_vvvv() {
    // `vmulss xmm1, xmm2, xmm3`: bits 127:32 from `xmm2`, 255:128 zeroed
    // (*Intel SDM* volume 2, `MULSS`, "VMULSS (VEX.128 encoded version)").
    let pc = avx();
    set_ymm(
        &pc,
        &[
            (1, [[!0, !0], [!0, !0]]),
            (2, ps([S2, S5, S6, S7, S8, S8, S8, S8])),
            (3, ps([S3, S9, S9, S9, S9, S9, S9, S9])),
        ],
    );
    run(&pc, &vex(M0F, PF3, false, false, 2, 0x59, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), [ps([S6, S5, S6, S7, 0, 0, 0, 0])[0], [0, 0]]);
    // `VEX.L` is ignored: the same with `L = 1` is the same instruction.
    set_ymm(&pc, &[(1, [[!0, !0], [!0, !0]])]);
    run(&pc, &vex(M0F, PF3, true, false, 2, 0x59, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), [ps([S6, S5, S6, S7, 0, 0, 0, 0])[0], [0, 0]]);
}

#[test]
fn every_one_of_the_thirty_two_compare_predicates() {
    // *Intel SDM* volume 2, `CMPPS`, table 3-1. Each lane pair is one of the
    // four relations: equal, less, greater, unordered (a quiet NaN). The
    // expected mask is the table's, written out per predicate as
    // `EQ | LT << 1 | GT << 2 | UNORD << 3`, with whether a quiet NaN
    // raises `#I` (the `S` predicates) beside it.
    const TABLE: [(u8, bool); 16] = [
        (0b0001, false), // EQ_OQ
        (0b0010, true),  // LT_OS
        (0b0011, true),  // LE_OS
        (0b1000, false), // UNORD_Q
        (0b1110, false), // NEQ_UQ
        (0b1101, true),  // NLT_US
        (0b1100, true),  // NLE_US
        (0b0111, false), // ORD_Q
        (0b1001, false), // EQ_UQ
        (0b1010, true),  // NGE_US
        (0b1011, true),  // NGT_US
        (0b0000, false), // FALSE_OQ
        (0b0110, false), // NEQ_OQ
        (0b0101, true),  // GE_OS
        (0b0100, true),  // GT_OS
        (0b1111, false), // TRUE_UQ
    ];
    let pc = avx();
    let a = ps([S1, S1, S2, SQNAN, S1, S1, S2, SQNAN]);
    let b = ps([S1, S2, S1, S1, S1, S2, S1, S1]);
    for p in 0..32u8 {
        let (bits, signals) = TABLE[usize::from(p & 15)];
        // Predicates 16-31 are 0-15 with the signalling behaviour reversed.
        let signals = signals != (p >= 16);
        set_ymm(&pc, &[(1, a), (2, b)]);
        let mut sse = pc.cpu.sse();
        sse.mxcsr = mxcsr::RESET;
        pc.cpu.set_sse(sse);
        // vcmpps ymm0, ymm1, ymm2, p
        run(&pc, &vex(M0F, NP, true, false, 1, 0xc2, &[rr(0, 2), p]));
        let lane = |i: u32| if bits & (1 << i) != 0 { 0xffff_ffff } else { 0 };
        let want = ps([0, 1, 2, 3, 0, 1, 2, 3].map(lane));
        assert_eq!(ymm(&pc, 0), want, "predicate {p}");
        let ie = pc.cpu.sse().mxcsr & 1 != 0;
        assert_eq!(ie, signals, "predicate {p}: #I on a quiet NaN");
    }
    // The double-precision form, and the legacy encoding reading only three
    // bits of the immediate: `cmppd xmm0, xmm1, 0x0d` is predicate 5, not 13.
    set_ymm(&pc, &[(0, pd([D1, D2, 0, 0])), (1, pd([D1, D1, 0, 0]))]);
    run(&pc, &[0x66, 0x0f, 0xc2, 0xc1, 0x0d]);
    assert_eq!(ymm(&pc, 0)[0], [u64::MAX, u64::MAX], "NLT: 1 !< 1, 2 !< 1");
    set_ymm(&pc, &[(0, pd([D1, D2, 0, 0])), (1, pd([D1, D1, 0, 0]))]);
    // vcmppd xmm0, xmm0, xmm1, 0x0d — GE_OS.
    run(
        &pc,
        &vex(M0F, P66, false, false, 0, 0xc2, &[rr(0, 1), 0x0d]),
    );
    assert_eq!(ymm(&pc, 0)[0], [u64::MAX, u64::MAX]);
    // vcmppd xmm0, xmm0, xmm1, 0x0e — GT_OS: 1 > 1 is false.
    set_ymm(&pc, &[(0, pd([D1, D2, 0, 0]))]);
    run(
        &pc,
        &vex(M0F, P66, false, false, 0, 0xc2, &[rr(0, 1), 0x0e]),
    );
    assert_eq!(ymm(&pc, 0)[0], [0, u64::MAX]);
}

#[test]
fn an_unmasked_exception_leaves_the_whole_destination_alone() {
    // *Intel SDM* volume 1 §11.5.1: no lane is written when any lane raises
    // an unmasked exception — here a divide by zero in the upper half.
    let pc = avx();
    let before = ps([S9; 8]);
    set_ymm(
        &pc,
        &[
            (1, before),
            (2, ps([S1; 8])),
            (3, ps([S1, S1, S1, S1, S1, 0, S1, S1])),
        ],
    );
    let mut sse = pc.cpu.sse();
    sse.mxcsr = mxcsr::RESET & !(1 << 9); // unmask ZE
    pc.cpu.set_sse(sse);
    let vdivps = vex(M0F, NP, true, false, 2, 0x5e, &[rr(1, 3)]);
    assert_eq!(exec(&pc, &vdivps), Some(19), "#XM");
    assert_eq!(ymm(&pc, 1), before);
    assert_ne!(pc.cpu.sse().mxcsr & (1 << 2), 0, "ZE recorded");
}

#[test]
fn the_shuffles_use_the_next_immediate_bits_in_the_upper_lane() {
    // `VSHUFPD ymm`: bits 0-1 for the low lane, 2-3 for the high one;
    // `VSHUFPS ymm`: the same byte in both (*Intel SDM* volume 2).
    let pc = avx();
    set_ymm(&pc, &[(2, pd([10, 11, 12, 13])), (3, pd([20, 21, 22, 23]))]);
    run(
        &pc,
        &vex(M0F, P66, true, false, 2, 0xc6, &[rr(1, 3), 0b0110]),
    );
    assert_eq!(ymm(&pc, 1), pd([10, 21, 13, 22]));
    set_ymm(
        &pc,
        &[
            (2, ps([0, 1, 2, 3, 4, 5, 6, 7])),
            (3, ps([8, 9, 10, 11, 12, 13, 14, 15])),
        ],
    );
    run(
        &pc,
        &vex(M0F, NP, true, false, 2, 0xc6, &[rr(1, 3), 0b00_01_10_11]),
    );
    assert_eq!(ymm(&pc, 1), ps([3, 2, 9, 8, 7, 6, 13, 12]));
    // `VUNPCKLPS ymm` interleaves within each lane.
    run(&pc, &vex(M0F, NP, true, false, 2, 0x14, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), ps([0, 8, 1, 9, 4, 12, 5, 13]));
    // `VBLENDPS ymm`: one immediate bit per single, across both lanes.
    run(
        &pc,
        &vex(M3A, P66, true, false, 2, 0x0c, &[rr(1, 3), 0b1000_0001]),
    );
    assert_eq!(ymm(&pc, 1), ps([8, 1, 2, 3, 4, 5, 6, 15]));
}

#[test]
fn the_horizontal_operations_and_the_dot_product_stay_in_their_lane() {
    let pc = avx();
    set_ymm(
        &pc,
        &[
            (2, ps([S1, S2, S3, S4, S5, S6, S7, S8])),
            (3, ps([S1, S1, S2, S2, S3, S3, S4, S4])),
        ],
    );
    // vhaddps ymm1, ymm2, ymm3: per lane, dst pairs then src pairs.
    run(&pc, &vex(M0F, PF2, true, false, 2, 0x7c, &[rr(1, 3)]));
    let s11 = 0x4130_0000; // 11.0
    let s15 = 0x4170_0000; // 15.0
    assert_eq!(ymm(&pc, 1), ps([S3, S7, S2, S4, s11, s15, S6, S8]));
    // vdpps ymm1, ymm2, ymm3, 0xf1: each lane's full dot product into its
    // element 0. Low: 1+2+6+8 = 17; high: 15+18+28+32 = 93.
    run(&pc, &vex(M3A, P66, true, false, 2, 0x40, &[rr(1, 3), 0xf1]));
    assert_eq!(
        ymm(&pc, 1),
        ps([0x4188_0000, 0, 0, 0, 0x42ba_0000, 0, 0, 0])
    );
}

#[test]
fn the_conversions_that_change_length() {
    // `VCVTPS2PD ymm, xmm/m128` widens four singles; `VCVTPD2PS xmm,
    // ymm/m256` narrows four doubles and zeroes bits 255:128 (*Intel SDM*
    // volume 2, `CVTPS2PD`, `CVTPD2PS`).
    let pc = avx();
    set_ymm(&pc, &[(2, ps([S1, S2, S3, S4, S9, S9, S9, S9]))]);
    run(&pc, &vex(M0F, NP, true, false, 0, 0x5a, &[rr(1, 2)]));
    assert_eq!(ymm(&pc, 1), pd([D1, D2, D3, D4]));
    set_ymm(&pc, &[(3, [[!0, !0], [!0, !0]])]);
    run(&pc, &vex(M0F, P66, true, false, 0, 0x5a, &[rr(3, 1)]));
    assert_eq!(ymm(&pc, 3), [ps([S1, S2, S3, S4, 0, 0, 0, 0])[0], [0, 0]]);
    // From memory, at both lengths: `vcvtdq2pd ymm0, [DATA]` reads sixteen
    // bytes, the 128-bit form eight.
    pc.write(
        DATA,
        &[1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0xff, 0xff, 0xff, 0xff],
    );
    run(&pc, &vex(M0F, PF3, true, false, 0, 0xe6, &m32(0, DATA)));
    assert_eq!(ymm(&pc, 0), pd([D1, D2, D3, 0xbff0_0000_0000_0000]));
    run(&pc, &vex(M0F, PF3, false, false, 0, 0xe6, &m32(0, DATA)));
    assert_eq!(ymm(&pc, 0), [[D1, D2], [0, 0]]);
    // `vcvttpd2dq xmm0, ymm1`: four truncated doublewords.
    set_ymm(
        &pc,
        &[(
            1,
            pd([0x3ff8_0000_0000_0000, D2, 0xc004_0000_0000_0000, D4]),
        )],
    );
    run(&pc, &vex(M0F, P66, true, false, 0, 0xe6, &[rr(0, 1)]));
    assert_eq!(
        ymm(&pc, 0),
        [[1 | (2 << 32), 0xffff_fffe | (4 << 32)], [0, 0]]
    );
}

// -- Moves --------------------------------------------------------------------

#[test]
fn aligned_moves_check_their_own_length_and_unaligned_ones_nothing() {
    // *Intel SDM* volume 2, `MOVAPS`: `VEX.256` must be 32-byte aligned;
    // `MOVUPS` need not be aligned at all.
    let pc = avx();
    let v = pd([1, 2, 3, 4]);
    write_ymm(&pc, DATA + 16, v);
    // vmovaps ymm0, [DATA+16]: 16-aligned is not enough.
    assert_eq!(
        exec(&pc, &vex(M0F, NP, true, false, 0, 0x28, &m32(0, DATA + 16))),
        Some(13)
    );
    // vmovups ymm0, [DATA+16].
    run(&pc, &vex(M0F, NP, true, false, 0, 0x10, &m32(0, DATA + 16)));
    assert_eq!(ymm(&pc, 0), v);
    // vmovaps xmm0, [DATA+16] is fine at 128 bits.
    run(
        &pc,
        &vex(M0F, NP, false, false, 0, 0x28, &m32(0, DATA + 16)),
    );
    assert_eq!(ymm(&pc, 0), [[1, 2], [0, 0]]);
    // vmovdqa [OUT], ymm1 stores all 32 bytes.
    set_ymm(&pc, &[(1, v)]);
    run(&pc, &vex(M0F, P66, true, false, 0, 0x7f, &m32(1, OUT)));
    assert_eq!(
        (0..4).map(|i| read64(&pc, OUT + 8 * i)).collect::<Vec<_>>(),
        [1, 2, 3, 4]
    );
    // vmovntps has no register form.
    assert_eq!(
        exec(&pc, &vex(M0F, NP, true, false, 0, 0x2b, &[rr(1, 0)])),
        Some(6)
    );
}

#[test]
fn vmovss_merges_from_vvvv_in_the_register_form_and_zeroes_on_a_load() {
    // *Intel SDM* volume 2, `MOVSS`.
    let pc = avx();
    set_ymm(
        &pc,
        &[
            (1, [[!0, !0], [!0, !0]]),
            (2, ps([10, 11, 12, 13, 14, 15, 16, 17])),
            (3, ps([20, 21, 22, 23, 24, 25, 26, 27])),
        ],
    );
    run(&pc, &vex(M0F, PF3, false, false, 2, 0x10, &[rr(1, 3)])); // vmovss xmm1, xmm2, xmm3
    assert_eq!(ymm(&pc, 1), [ps([20, 11, 12, 13, 0, 0, 0, 0])[0], [0, 0]]);
    pc.write(DATA, &[0x78, 0x56, 0x34, 0x12]);
    run(&pc, &vex(M0F, PF3, false, false, 0, 0x10, &m32(1, DATA))); // vmovss xmm1, [DATA]
    assert_eq!(ymm(&pc, 1), [[0x1234_5678, 0], [0, 0]]);
    // The store form writes four bytes and nothing else.
    write64(&pc, OUT, u64::MAX);
    run(&pc, &vex(M0F, PF3, false, false, 0, 0x11, &m32(3, OUT)));
    assert_eq!(read64(&pc, OUT), 0xffff_ffff_0000_0014);
}

#[test]
fn the_half_moves_take_the_other_half_from_vvvv() {
    let pc = avx();
    set_ymm(
        &pc,
        &[(2, [[0xaa, 0xbb], [1, 1]]), (3, [[0xcc, 0xdd], [1, 1]])],
    );
    write64(&pc, DATA, 0xee);
    // vmovhps xmm1, xmm2, [DATA]: low from xmm2, high from memory.
    run(&pc, &vex(M0F, NP, false, false, 2, 0x16, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), [[0xaa, 0xee], [0, 0]]);
    // vmovhlps xmm1, xmm2, xmm3: low = xmm3 high, high = xmm2 high.
    run(&pc, &vex(M0F, NP, false, false, 2, 0x12, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), [[0xdd, 0xbb], [0, 0]]);
    // vmovlhps xmm1, xmm2, xmm3: low = xmm2 low, high = xmm3 low.
    run(&pc, &vex(M0F, NP, false, false, 2, 0x16, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), [[0xaa, 0xcc], [0, 0]]);
}

#[test]
fn vmovddup_and_vmovsldup_duplicate_in_each_lane() {
    let pc = avx();
    write_ymm(&pc, DATA, pd([1, 2, 3, 4]));
    run(&pc, &vex(M0F, PF2, true, false, 0, 0x12, &m32(0, DATA))); // vmovddup ymm0, [DATA]
    assert_eq!(ymm(&pc, 0), pd([1, 1, 3, 3]));
    run(&pc, &vex(M0F, PF2, false, false, 0, 0x12, &m32(0, DATA))); // 128: one quadword
    assert_eq!(ymm(&pc, 0), [[1, 1], [0, 0]]);
    set_ymm(&pc, &[(2, ps([0, 1, 2, 3, 4, 5, 6, 7]))]);
    run(&pc, &vex(M0F, PF3, true, false, 0, 0x16, &[rr(1, 2)])); // vmovshdup
    assert_eq!(ymm(&pc, 1), ps([1, 1, 3, 3, 5, 5, 7, 7]));
}

#[test]
fn vmovmskps_takes_eight_sign_bits_at_256() {
    let pc = avx();
    let n = 0x8000_0000u32;
    set_ymm(&pc, &[(2, ps([n, 0, n, n, 0, 0, 0, n]))]);
    run(&pc, &vex(M0F, NP, true, false, 0, 0x50, &[rr(0, 2)]));
    assert_eq!(pc.regs().rax & 0xffff_ffff, 0b1000_1101);
    // vmovmskpd: the sign of each quadword is its upper single's.
    run(&pc, &vex(M0F, P66, true, false, 0, 0x50, &[rr(0, 2)]));
    assert_eq!(pc.regs().rax & 0xffff_ffff, 0b1010);
}

// -- AVX's own -----------------------------------------------------------------

#[test]
fn the_broadcasts_take_memory_and_only_memory() {
    // *Intel SDM* volume 2, `VBROADCAST`: memory sources in AVX; the register
    // forms are AVX2's and `#UD` here.
    let pc = avx();
    write_ymm(&pc, DATA, pd([0x1111_2222_3333_4444, 0x5555, 0, 0]));
    run(&pc, &vex(M38, P66, true, false, 0, 0x18, &m32(0, DATA)));
    assert_eq!(ymm(&pc, 0), [[0x3333_4444_3333_4444; 2]; 2]);
    run(&pc, &vex(M38, P66, false, false, 0, 0x18, &m32(0, DATA)));
    assert_eq!(ymm(&pc, 0), [[0x3333_4444_3333_4444; 2], [0, 0]]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x19, &m32(0, DATA)));
    assert_eq!(ymm(&pc, 0), [[0x1111_2222_3333_4444; 2]; 2]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x1a, &m32(0, DATA)));
    assert_eq!(ymm(&pc, 0), [[0x1111_2222_3333_4444, 0x5555]; 2]);
    assert_eq!(
        exec(&pc, &vex(M38, P66, true, false, 0, 0x18, &[rr(0, 1)])),
        Some(6)
    );
}

#[test]
fn vinsertf128_vextractf128_and_vperm2f128_move_whole_halves() {
    let pc = avx();
    let a = pd([1, 2, 3, 4]);
    let b = pd([5, 6, 7, 8]);
    set_ymm(&pc, &[(2, a), (3, b)]);
    // vinsertf128 ymm1, ymm2, xmm3, 1.
    run(&pc, &vex(M3A, P66, true, false, 2, 0x18, &[rr(1, 3), 1]));
    assert_eq!(ymm(&pc, 1), pd([1, 2, 5, 6]));
    // vinsertf128 ymm1, ymm2, [DATA], 0.
    write_ymm(&pc, DATA, pd([9, 10, 0, 0]));
    run(
        &pc,
        &vex(M3A, P66, true, false, 2, 0x18, &with(m32(1, DATA), 0)),
    );
    assert_eq!(ymm(&pc, 1), pd([9, 10, 3, 4]));
    // vextractf128 xmm4, ymm2, 1 — and the destination's upper half zeroed.
    set_ymm(&pc, &[(4, b)]);
    run(&pc, &vex(M3A, P66, true, false, 0, 0x19, &[rr(2, 4), 1]));
    assert_eq!(ymm(&pc, 4), [[3, 4], [0, 0]]);
    // vextractf128 [OUT], ymm3, 1: sixteen bytes.
    write64(&pc, OUT + 16, 0x77);
    run(
        &pc,
        &vex(M3A, P66, true, false, 0, 0x19, &with(m32(3, OUT), 1)),
    );
    assert_eq!(
        [
            read64(&pc, OUT),
            read64(&pc, OUT + 8),
            read64(&pc, OUT + 16)
        ],
        [7, 8, 0x77]
    );
    // vperm2f128 ymm1, ymm2, ymm3, imm: low half = b.hi (3), high half zero
    // (bit 3 of the upper nibble).
    run(&pc, &vex(M3A, P66, true, false, 2, 0x06, &[rr(1, 3), 0x83]));
    assert_eq!(ymm(&pc, 1), pd([7, 8, 0, 0]));
    run(&pc, &vex(M3A, P66, true, false, 2, 0x06, &[rr(1, 3), 0x20]));
    assert_eq!(ymm(&pc, 1), pd([1, 2, 5, 6]));
    // `VEX.L = 0` is not an encoding of any of the three.
    assert_eq!(
        exec(&pc, &vex(M3A, P66, false, false, 2, 0x06, &[rr(1, 3), 0])),
        Some(6)
    );
}

#[test]
fn vpermilps_and_vpermilpd_by_immediate_and_by_vector() {
    // *Intel SDM* volume 2, `VPERMILPS`/`VPERMILPD`.
    let pc = avx();
    set_ymm(&pc, &[(2, ps([0, 1, 2, 3, 4, 5, 6, 7]))]);
    // Immediate: the same selector in both lanes — reverse.
    run(
        &pc,
        &vex(M3A, P66, true, false, 0, 0x04, &[rr(1, 2), 0b00_01_10_11]),
    );
    assert_eq!(ymm(&pc, 1), ps([3, 2, 1, 0, 7, 6, 5, 4]));
    // Vector: data is vvvv, control r/m, bits 1:0 of each doubleword.
    set_ymm(&pc, &[(3, ps([1, 1, 0x10, 3, 2, 0xfffe, 0, 1]))]);
    run(&pc, &vex(M38, P66, true, false, 2, 0x0c, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), ps([1, 1, 0, 3, 6, 6, 4, 5]));
    // Doubles: one immediate bit per element, lane after lane.
    set_ymm(&pc, &[(2, pd([10, 11, 12, 13]))]);
    run(
        &pc,
        &vex(M3A, P66, true, false, 0, 0x05, &[rr(1, 2), 0b0110]),
    );
    assert_eq!(ymm(&pc, 1), pd([10, 11, 13, 12]));
    // Vector: bit 1 of each control quadword.
    set_ymm(&pc, &[(3, pd([2, 0, 1, 3]))]);
    run(&pc, &vex(M38, P66, true, false, 2, 0x0d, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), pd([11, 10, 12, 13]));
}

#[test]
fn vblendvps_takes_its_mask_from_the_register_the_immediate_names() {
    let pc = avx();
    let n = 0x8000_0000u32;
    set_ymm(
        &pc,
        &[
            (2, ps([0, 1, 2, 3, 4, 5, 6, 7])),
            (3, ps([10, 11, 12, 13, 14, 15, 16, 17])),
            (4, ps([n, 0, 0, n, 0, n, 0, 0])),
            (0, ps([n; 8])),
        ],
    );
    // vblendvps ymm1, ymm2, ymm3, ymm4 — not XMM0.
    run(&pc, &vex(M3A, P66, true, false, 2, 0x4a, &[rr(1, 3), 0x40]));
    assert_eq!(ymm(&pc, 1), ps([10, 1, 2, 13, 4, 15, 6, 7]));
}

#[test]
fn vmaskmovps_never_touches_a_masked_off_element() {
    // *Intel SDM* volume 2, `VMASKMOV`: "Faults occur only due to mask-bit
    // required memory accesses that caused the faults." `ES` here ends 16
    // bytes into the operand, so the upper four singles are past its limit.
    let pc = avx();
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::ES)].limit = (DATA + 15) as u32;
    pc.cpu.set_sys(sys);
    write_ymm(&pc, DATA, ps([1, 2, 3, 4, 5, 6, 7, 8]));
    let n = 0x8000_0000u32;
    set_ymm(
        &pc,
        &[(2, ps([n, 0, n, n, 0, 0, 0, 0])), (1, [[!0, !0], [!0, !0]])],
    );
    // es: vmaskmovps ymm1, ymm2, [DATA]
    let mut load = alloc::vec![0x26];
    load.extend(vex(M38, P66, true, false, 2, 0x2c, &m32(1, DATA)));
    run(&pc, &load);
    assert_eq!(ymm(&pc, 1), ps([1, 0, 3, 4, 0, 0, 0, 0]));
    // An element past the limit with its mask bit set does fault.
    set_ymm(&pc, &[(2, ps([0, 0, 0, 0, 0, n, 0, 0]))]);
    assert_eq!(exec(&pc, &load), Some(13));
    // The store writes exactly the selected elements.
    set_ymm(&pc, &[(2, ps([0, n, 0, 0, 0, 0, 0, 0])), (3, ps([9; 8]))]);
    let mut store = alloc::vec![0x26];
    store.extend(vex(M38, P66, true, false, 2, 0x2e, &m32(3, DATA)));
    run(&pc, &store);
    assert_eq!(read64(&pc, DATA), 1 | (9 << 32));
    assert_eq!(read64(&pc, DATA + 8), 3 | (4 << 32));
    // And it has no register form.
    assert_eq!(
        exec(&pc, &vex(M38, P66, true, false, 2, 0x2c, &[rr(1, 3)])),
        Some(6)
    );
}

#[test]
fn vptest_and_vtestps_set_zf_and_cf_over_the_whole_register() {
    let pc = avx();
    let flags_of = |pc: &Pc| {
        let f = pc.regs().eflags;
        (f & flags::ZF != 0, f & flags::CF != 0)
    };
    // vptest ymm1, ymm2: the AND is zero only if the upper lanes agree too.
    set_ymm(&pc, &[(1, pd([0xf0, 0, 0, 1])), (2, pd([0x0f, 0, 0, 0]))]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x17, &[rr(1, 2)]));
    assert_eq!(flags_of(&pc), (true, false));
    set_ymm(&pc, &[(2, pd([0x0f, 0, 0, 1]))]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x17, &[rr(1, 2)]));
    assert_eq!(flags_of(&pc), (false, false));
    // vtestps reads sign bits only: bit 30 set everywhere changes nothing.
    let n = 0x8000_0000u32;
    let m = 0x4000_0000u32;
    set_ymm(&pc, &[(1, ps([m; 8])), (2, ps([m; 8]))]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x0e, &[rr(1, 2)]));
    assert_eq!(flags_of(&pc), (true, true));
    set_ymm(&pc, &[(2, ps([m, m, m, m, m, m, m, n]))]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x0e, &[rr(1, 2)]));
    assert_eq!(
        flags_of(&pc),
        (true, false),
        "a sign in src not in dst: CF clear"
    );
}

// -- The VEX.128 integer and transfer forms -------------------------------------

#[test]
fn the_integer_forms_are_non_destructive_at_128_bits() {
    let pc = avx();
    set_ymm(
        &pc,
        &[
            (1, [[!0, !0], [!0, !0]]),
            (2, [[0x0102_0304_0506_0708, 0], [5, 5]]),
            (3, [[0x1010_1010_1010_1010, 1], [5, 5]]),
        ],
    );
    // vpaddb xmm1, xmm2, xmm3.
    run(&pc, &vex(M0F, P66, false, false, 2, 0xfc, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), [[0x1112_1314_1516_1718, 1], [0, 0]]);
    assert_eq!(
        ymm(&pc, 2)[0],
        [0x0102_0304_0506_0708, 0],
        "the source is intact"
    );
    // vpsrlw xmm4, xmm2, 4 — vvvv is the destination.
    run(&pc, &vex(M0F, P66, false, false, 4, 0x71, &[rr(2, 2), 4]));
    assert_eq!(ymm(&pc, 4), [[0x0010_0030_0050_0070, 0], [0, 0]]);
    // vpshufb xmm1, xmm2, xmm3 from the `0F 38` map.
    set_ymm(
        &pc,
        &[(3, [[0x0001_0203_0405_0607, 0x8080_8080_8080_8080], [0, 0]])],
    );
    run(&pc, &vex(M38, P66, false, false, 2, 0x00, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), [[0x0807_0605_0403_0201, 0], [0, 0]]);
    // vpinsrb xmm1, xmm2, eax, 15: the rest from xmm2.
    let mut regs = pc.regs();
    regs.rax = 0xab;
    pc.cpu.set_regs(regs);
    run(&pc, &vex(M3A, P66, false, false, 2, 0x20, &[rr(1, 0), 15]));
    assert_eq!(
        ymm(&pc, 1),
        [[0x0102_0304_0506_0708, 0xab00_0000_0000_0000], [0, 0]]
    );
    // vpextrb ecx, xmm1, 15.
    run(&pc, &vex(M3A, P66, false, false, 0, 0x14, &[rr(1, 1), 15]));
    assert_eq!(pc.regs().rcx & 0xffff_ffff, 0xab);
    // vcvtsi2ss xmm5, xmm2, eax: bits 127:32 from xmm2.
    set_ymm(&pc, &[(5, [[!0, !0], [!0, !0]])]);
    let mut regs = pc.regs();
    regs.rax = 3;
    pc.cpu.set_regs(regs);
    run(&pc, &vex(M0F, PF3, false, false, 2, 0x2a, &[rr(5, 0)]));
    assert_eq!(
        ymm(&pc, 5),
        [[0x0102_0304_0000_0000 | u64::from(S3), 0], [0, 0]]
    );
}

#[test]
fn vpcmpistrm_writes_xmm0_and_zeroes_its_upper_half() {
    // *Intel SDM* volume 2, `PCMPISTRM`: the VEX.128 form zeroes bits
    // 255:128 of `XMM0`.
    let pc = avx();
    set_ymm(
        &pc,
        &[
            (0, [[0, 0], [7, 7]]),
            (1, [[u64::from_le_bytes(*b"abcdefgh"), 0], [0, 0]]),
            (2, [[u64::from_le_bytes(*b"abXdefgh"), 0], [0, 0]]),
        ],
    );
    // vpcmpistrm xmm1, xmm2, 0x48: equal each, expanded to a byte mask.
    run(
        &pc,
        &vex(M3A, P66, false, false, 0, 0x62, &[rr(1, 2), 0x48]),
    );
    let x0 = ymm(&pc, 0);
    assert_eq!(x0[1], [0, 0]);
    assert_eq!(
        x0[0][0] & 0xff_ffff,
        0x00_ffff,
        "bytes 0 and 1 equal, 2 not"
    );
}

#[test]
fn vldmxcsr_and_vstmxcsr_move_mxcsr() {
    let pc = avx();
    pc.write32(DATA, u64::from(mxcsr::RESET | mxcsr::FTZ));
    run(&pc, &vex(M0F, NP, false, false, 0, 0xae, &m32(2, DATA)));
    assert_eq!(pc.cpu.sse().mxcsr, mxcsr::RESET | mxcsr::FTZ);
    run(&pc, &vex(M0F, NP, false, false, 0, 0xae, &m32(3, OUT)));
    assert_eq!(pc.read32(OUT), u64::from(mxcsr::RESET | mxcsr::FTZ));
    // `VEX.L = 1` is not an encoding of either.
    assert_eq!(
        exec(&pc, &vex(M0F, NP, true, false, 0, 0xae, &m32(3, OUT))),
        Some(6)
    );
}

#[test]
fn the_disassembler_and_the_interpreter_agree_on_every_length() {
    // A run of VEX instructions laid end to end executes to its `hlt`, which
    // it could not if the decoder's length for any of them were wrong.
    let pc = avx();
    let mut code = Vec::new();
    code.extend(vaddps256());
    code.extend(vex(M3A, P66, true, false, 2, 0x18, &with(m32(1, DATA), 1)));
    code.extend(vex(M3A, P66, true, false, 2, 0x4a, &[rr(1, 3), 0x40]));
    code.extend([0xc5, 0xf8, 0x77]);
    code.extend(vex(M0F, P66, false, false, 4, 0x71, &[rr(2, 2), 4]));
    run(&pc, &code);
    let mut at = 0usize;
    let mut count = 0;
    while at < code.len() {
        let d =
            super::super::disasm::disassemble_as(isa::Gen::I386, isa::Bits::B32, 0, 0, &code[at..]);
        assert!(!d.fields.truncated);
        assert_ne!(d.fields.insn.op, Op::UD, "{}", d);
        at += usize::from(d.len);
        count += 1;
    }
    assert_eq!((at, count), (code.len(), 5));
}
