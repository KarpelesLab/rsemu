//! AVX2, FMA and F16C, executed as a guest sees them.
//!
//! The same harness as `avx.rs` — flat 32-bit protected mode, `XCR0` with
//! the `YMM` state enabled — and the same rule for expectations: each is
//! either from the *Intel SDM* volume 2 instruction page named on the test,
//! or arithmetic that can be checked by hand. The floating-point values are
//! small integers and powers of two whose encodings can be read off.

use super::avx::*;
use super::*;
use crate::cpu::x86::fpu::mxcsr;
use crate::cpu::x86::prot::{cr4, xcr0};

/// A part with AVX2, FMA and F16C on top of AVX.
fn features2() -> Features {
    Features {
        avx2: true,
        fma: true,
        f16c: true,
        ..features()
    }
}

fn avx2() -> Pc {
    pc_with(features2())
}

/// Eight doublewords, low first.
fn dd(v: [u32; 8]) -> Ymm {
    ps(v)
}

fn set_regs(pc: &Pc, eax: u64) {
    let mut regs = pc.regs();
    regs.rax = eax;
    pc.cpu.set_regs(regs);
}

fn mxcsr_now(pc: &Pc) -> u32 {
    pc.cpu.sse().mxcsr
}

fn set_mxcsr(pc: &Pc, value: u32) {
    let mut sse = pc.cpu.sse();
    sse.mxcsr = value;
    pc.cpu.set_sse(sse);
}

// -- Features and gates ----------------------------------------------------------

#[test]
fn cpuid_reports_each_extension_and_validate_wants_avx() {
    // *Intel SDM* volume 2, `CPUID`: FMA is leaf 1 `ECX[12]`, F16C
    // `ECX[29]`, AVX2 leaf 7 sub-leaf 0 `EBX[5]`.
    let cpuid = |features: Features, leaf: u8| {
        let pc = pc_with(features);
        run(&pc, &[0xb8, leaf, 0, 0, 0, 0x31, 0xc9, 0x0f, 0xa2]);
        let r = pc.regs();
        (r.rbx & 0xffff_ffff, r.rcx & 0xffff_ffff)
    };
    let (_, ecx) = cpuid(features2(), 1);
    assert_eq!(ecx & (1 << 12 | 1 << 29), 1 << 12 | 1 << 29);
    let (ebx, _) = cpuid(features2(), 7);
    assert_eq!(ebx & (1 << 5), 1 << 5);
    let (_, ecx) = cpuid(features(), 1);
    assert_eq!(ecx & (1 << 12 | 1 << 29), 0);
    let (ebx, _) = cpuid(features(), 7);
    assert_eq!(ebx & (1 << 5), 0);
    assert!(features2().validate().is_ok());
    for one in [
        Features {
            avx2: true,
            ..Features::X86_64_V2
        },
        Features {
            fma: true,
            ..Features::X86_64_V2
        },
        Features {
            f16c: true,
            ..Features::X86_64_V2
        },
    ] {
        assert!(one.validate().is_err(), "{one:?}");
    }
}

#[test]
fn each_extension_is_undefined_opcode_without_its_bit() {
    // `vpaddb ymm1, ymm2, ymm3` is AVX2's; its `VEX.128` form is AVX's.
    let vpaddb = |l: bool| vex(M0F, P66, l, false, 2, 0xfc, &[rr(1, 3)]);
    let avx_only = pc_with(features());
    run(&avx_only, &vpaddb(false));
    assert_eq!(exec(&avx_only, &vpaddb(true)), Some(6));
    run(&avx2(), &vpaddb(true));
    // And without AVX2 it is `#UD` even with `CR0.TS` set, where an AVX
    // instruction would be `#NM`: there is no instruction to defer.
    let mut sys = avx_only.cpu.sys();
    sys.cr0 |= cr0::TS;
    avx_only.cpu.set_sys(sys);
    assert_eq!(exec(&avx_only, &vpaddb(true)), Some(6));
    assert_eq!(exec(&avx_only, &vpaddb(false)), Some(7));
    // The instructions AVX2 added, and the register broadcasts.
    let without = |f: Features, code: &[u8]| exec(&pc_with(f), code);
    let no_avx2 = Features {
        avx2: false,
        ..features2()
    };
    let vpermd = vex(M38, P66, true, false, 2, 0x36, &[rr(1, 3)]);
    assert_eq!(without(no_avx2, &vpermd), Some(6));
    let vbroadcastss_reg = vex(M38, P66, true, false, 0, 0x18, &[rr(1, 2)]);
    assert_eq!(without(no_avx2, &vbroadcastss_reg), Some(6));
    run(&avx2(), &vbroadcastss_reg);
    // FMA and F16C each on their own bit.
    let vfmadd = vex(M38, P66, true, false, 2, 0x98, &[rr(1, 3)]);
    let no_fma = Features {
        fma: false,
        ..features2()
    };
    assert_eq!(without(no_fma, &vfmadd), Some(6));
    run(&avx2(), &vfmadd);
    let vcvtph2ps = vex(M38, P66, true, false, 0, 0x13, &[rr(1, 2)]);
    let no_f16c = Features {
        f16c: false,
        ..features2()
    };
    assert_eq!(without(no_f16c, &vcvtph2ps), Some(6));
    run(&avx2(), &vcvtph2ps);
}

// -- The 256-bit integer forms -----------------------------------------------------

#[test]
fn the_256_bit_integer_forms_work_lane_by_lane() {
    let pc = avx2();
    let bytes = pd([
        0x0706_0504_0302_0100,
        0x0f0e_0d0c_0b0a_0908,
        0x1716_1514_1312_1110,
        0x1f1e_1d1c_1b1a_1918,
    ]);
    // vpaddb: plain arithmetic, wrapping per byte.
    set_ymm(
        &pc,
        &[
            (2, pd([0x0102_0304_0506_0708, 0, 0x10, 0xff])),
            (3, pd([0x1010_1010_1010_1010, 1, 0x01, 0x01])),
        ],
    );
    run(&pc, &vex(M0F, P66, true, false, 2, 0xfc, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), pd([0x1112_1314_1516_1718, 1, 0x11, 0]));
    // vpunpcklbw: each lane interleaves its own low halves (*Intel SDM*
    // volume 2, `PUNPCKLBW`, `VEX.256` operation).
    set_ymm(
        &pc,
        &[
            (2, bytes),
            (3, pd([0x8786_8584_8382_8180, 0, 0x9796_9594_9392_9190, 0])),
        ],
    );
    run(&pc, &vex(M0F, P66, true, false, 2, 0x60, &[rr(1, 3)]));
    assert_eq!(
        ymm(&pc, 1),
        pd([
            0x8303_8202_8101_8000,
            0x8707_8606_8505_8404,
            0x9313_9212_9111_9010,
            0x9717_9616_9515_9414
        ])
    );
    // vpshufb: an index selects within its own lane.
    set_ymm(&pc, &[(3, pd([0, 0, 0x0f01, 0]))]);
    run(&pc, &vex(M38, P66, true, false, 2, 0x00, &[rr(1, 3)]));
    assert_eq!(
        ymm(&pc, 1),
        pd([0, 0, 0x1010_1010_1010_1f11, 0x1010_1010_1010_1010])
    );
    // vpslldq ymm4, ymm2, 1: each lane shifts by a byte on its own.
    run(&pc, &vex(M0F, P66, true, false, 4, 0x73, &[rr(7, 2), 1]));
    assert_eq!(
        ymm(&pc, 4),
        pd([
            0x0605_0403_0201_0000,
            0x0e0d_0c0b_0a09_0807,
            0x1615_1413_1211_1000,
            0x1e1d_1c1b_1a19_1817
        ])
    );
    // vpalignr ymm1, ymm2, ymm3, 8: each lane is (a:b) >> 64, in-lane.
    set_ymm(&pc, &[(3, pd([1, 2, 3, 4]))]);
    run(&pc, &vex(M3A, P66, true, false, 2, 0x0f, &[rr(1, 3), 8]));
    assert_eq!(
        ymm(&pc, 1),
        pd([2, 0x0706_0504_0302_0100, 4, 0x1716_1514_1312_1110])
    );
    // vphaddd: pairs within each lane, the first source's first.
    set_ymm(
        &pc,
        &[
            (2, dd([1, 2, 3, 4, 10, 20, 30, 40])),
            (3, dd([5, 6, 7, 8, 50, 60, 70, 80])),
        ],
    );
    run(&pc, &vex(M38, P66, true, false, 2, 0x02, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), dd([3, 7, 11, 15, 30, 70, 110, 150]));
}

#[test]
fn a_shift_by_register_takes_one_count_for_both_lanes() {
    // *Intel SDM* volume 2, `PSLLW`: `VPSLLW ymm1, ymm2, xmm3/m128` — the
    // count is the low quadword of an `XMM` register or of sixteen bytes
    // of memory, and a count past the width clears (or fills with sign).
    let pc = avx2();
    let words = pd([0x0001_0002_0003_0004; 4]);
    set_ymm(&pc, &[(2, words), (3, pd([4, 0, 99, 99]))]);
    run(&pc, &vex(M0F, P66, true, false, 2, 0xf1, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), pd([0x0010_0020_0030_0040; 4]));
    set_ymm(&pc, &[(3, pd([16, 0, 0, 0]))]);
    run(&pc, &vex(M0F, P66, true, false, 2, 0xf1, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), pd([0; 4]));
    // vpsraw by 100: every word becomes its sign.
    set_ymm(
        &pc,
        &[(2, pd([0x8000_7fff_0001_ffff; 4])), (3, pd([100, 0, 0, 0]))],
    );
    run(&pc, &vex(M0F, P66, true, false, 2, 0xe1, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), pd([0xffff_0000_0000_ffff; 4]));
    // The memory form reads sixteen bytes; the next sixteen are no part of
    // it. vpsrlq ymm1, ymm2, [DATA].
    write_ymm(&pc, DATA, pd([3, 0, 1, 1]));
    set_ymm(&pc, &[(2, pd([0x80, 0x100, 0x200, 0x400]))]);
    run(&pc, &vex(M0F, P66, true, false, 2, 0xd3, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), pd([0x10, 0x20, 0x40, 0x80]));
}

#[test]
fn the_variable_shifts_take_a_count_per_element() {
    // *Intel SDM* volume 2, `VPSLLVD`/`VPSRLVD`/`VPSRAVD`: a count of the
    // element width or more gives zero, or for `VPSRAVD` the sign.
    let pc = avx2();
    let counts = dd([0, 1, 31, 32, 33, 0xffff_ffff, 4, 5]);
    set_ymm(&pc, &[(2, dd([1; 8])), (3, counts)]);
    run(&pc, &vex(M38, P66, true, false, 2, 0x47, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), dd([1, 2, 0x8000_0000, 0, 0, 0, 16, 32]));
    let n = 0x8000_0000;
    set_ymm(&pc, &[(2, dd([n; 8]))]);
    run(&pc, &vex(M38, P66, true, false, 2, 0x45, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), dd([n, n >> 1, 1, 0, 0, 0, n >> 4, n >> 5]));
    run(&pc, &vex(M38, P66, true, false, 2, 0x46, &[rr(1, 3)]));
    assert_eq!(
        ymm(&pc, 1),
        dd([n, 0xc000_0000, !0, !0, !0, !0, 0xf800_0000, 0xfc00_0000])
    );
    // `VEX.W1`: the quadword forms.
    set_ymm(&pc, &[(2, pd([1; 4])), (3, pd([0, 63, 64, u64::MAX]))]);
    run(&pc, &vex(M38, P66, true, true, 2, 0x47, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), pd([1, 1 << 63, 0, 0]));
    set_ymm(&pc, &[(2, pd([1 << 63; 4])), (3, pd([1, 63, 64, 1000]))]);
    run(&pc, &vex(M38, P66, true, true, 2, 0x45, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), pd([1 << 62, 1, 0, 0]));
    // `VPSRAVD` has no `W1` form.
    assert_eq!(
        exec(&pc, &vex(M38, P66, true, true, 2, 0x46, &[rr(1, 3)])),
        Some(6)
    );
}

#[test]
fn the_widening_moves_cross_lanes_at_256_bits() {
    // *Intel SDM* volume 2, `PMOVSX`: `VPMOVSXBW ymm1, xmm2/m128` widens
    // sixteen bytes into sixteen words — the upper lane's from bytes 8-15.
    let pc = avx2();
    set_ymm(
        &pc,
        &[(
            2,
            pd([0x0706_0504_0302_01ff, 0x8f0e_0d0c_0b0a_0908, !0, !0]),
        )],
    );
    run(&pc, &vex(M38, P66, true, false, 0, 0x20, &[rr(1, 2)]));
    assert_eq!(
        ymm(&pc, 1),
        pd([
            0x0003_0002_0001_ffff,
            0x0007_0006_0005_0004,
            0x000b_000a_0009_0008,
            0xff8f_000e_000d_000c
        ])
    );
    // vpmovzxbd ymm1, [DATA]: eight bytes of memory.
    write_ymm(&pc, DATA, pd([0x8807_0605_0403_0201, !0, !0, !0]));
    run(&pc, &vex(M38, P66, true, false, 0, 0x31, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), dd([1, 2, 3, 4, 5, 6, 7, 0x88]));
    // vpmovzxbq ymm1, [DATA]: four bytes.
    run(&pc, &vex(M38, P66, true, false, 0, 0x32, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), pd([1, 2, 3, 4]));
    // vpmovsxdq ymm1, xmm2.
    set_ymm(&pc, &[(2, dd([!0, 2, (-3i32) as u32, 4, 9, 9, 9, 9]))]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x25, &[rr(1, 2)]));
    assert_eq!(ymm(&pc, 1), pd([u64::MAX, 2, (-3i64) as u64, 4]));
    // The 128-bit form is unchanged: vpmovzxbw xmm1, [DATA] reads eight.
    run(&pc, &vex(M38, P66, false, false, 0, 0x30, &m32(1, DATA)));
    assert_eq!(
        ymm(&pc, 1),
        pd([0x0004_0003_0002_0001, 0x0088_0007_0006_0005, 0, 0])
    );
}

#[test]
fn vpmovmskb_and_vmpsadbw_at_256_bits() {
    let pc = avx2();
    // *Intel SDM* volume 2, `PMOVMSKB`: thirty-two mask bits.
    set_ymm(&pc, &[(2, pd([0x8000_0000_0000_0080, 0, 0, 1 << 63]))]);
    run(&pc, &vex(M0F, P66, true, false, 0, 0xd7, &[rr(0, 2)]));
    assert_eq!(pc.regs().rax, 0x8000_0081, "zero-extended");
    // *Intel SDM* volume 2, `MPSADBW`: the upper lane's block offsets are
    // imm8[5:3]. Against a zero source each sum is of the first operand's
    // bytes alone: the low lane from offset 0 sums i..i+3 for 4i + 6, the
    // upper from offset 4 sums 20+i..23+i for 4i + 86.
    let bytes = pd([
        0x0706_0504_0302_0100,
        0x0f0e_0d0c_0b0a_0908,
        0x1716_1514_1312_1110,
        0x1f1e_1d1c_1b1a_1918,
    ]);
    set_ymm(&pc, &[(2, bytes), (3, pd([0; 4]))]);
    run(
        &pc,
        &vex(M3A, P66, true, false, 2, 0x42, &[rr(1, 3), 0b101_000]),
    );
    let words = |base: u64| {
        let w = |i: u64| base + 4 * i;
        [
            w(0) | w(1) << 16 | w(2) << 32 | w(3) << 48,
            w(4) | w(5) << 16 | w(6) << 32 | w(7) << 48,
        ]
    };
    assert_eq!(ymm(&pc, 1), [words(6), words(86)]);
}

// -- AVX2's own -----------------------------------------------------------------

#[test]
fn the_broadcasts_from_a_register_or_one_element_of_memory() {
    // *Intel SDM* volume 2, `VPBROADCAST` and `VBROADCAST`.
    let pc = avx2();
    set_ymm(&pc, &[(2, pd([0x1122_3344_5566_77ab, 9, 9, 9]))]);
    run(&pc, &vex(M38, P66, true, false, 0, 0x78, &[rr(1, 2)]));
    assert_eq!(ymm(&pc, 1), pd([0xabab_abab_abab_abab; 4]));
    write_ymm(&pc, DATA, pd([0x1234, 0, 0, 0]));
    run(&pc, &vex(M38, P66, true, false, 0, 0x79, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), pd([0x1234_1234_1234_1234; 4]));
    // vpbroadcastd xmm1, xmm2: VEX.128, the upper half zeroed.
    run(&pc, &vex(M38, P66, false, false, 0, 0x58, &[rr(1, 2)]));
    assert_eq!(
        ymm(&pc, 1),
        pd([0x5566_77ab_5566_77ab, 0x5566_77ab_5566_77ab, 0, 0])
    );
    run(&pc, &vex(M38, P66, true, false, 0, 0x59, &[rr(1, 2)]));
    assert_eq!(ymm(&pc, 1), pd([0x1122_3344_5566_77ab; 4]));
    // The register forms of AVX's broadcasts.
    run(&pc, &vex(M38, P66, true, false, 0, 0x18, &[rr(1, 2)]));
    assert_eq!(ymm(&pc, 1), pd([0x5566_77ab_5566_77ab; 4]));
    run(&pc, &vex(M38, P66, true, false, 0, 0x19, &[rr(1, 2)]));
    assert_eq!(ymm(&pc, 1), pd([0x1122_3344_5566_77ab; 4]));
    // vbroadcasti128: memory only, even with AVX2.
    write_ymm(&pc, DATA, pd([5, 6, 7, 8]));
    run(&pc, &vex(M38, P66, true, false, 0, 0x5a, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), pd([5, 6, 5, 6]));
    assert_eq!(
        exec(&pc, &vex(M38, P66, true, false, 0, 0x5a, &[rr(1, 2)])),
        Some(6)
    );
}

#[test]
fn the_integer_128_bit_moves_and_permutes_cross_lanes() {
    let pc = avx2();
    set_ymm(&pc, &[(2, pd([1, 2, 3, 4])), (3, pd([5, 6, 7, 8]))]);
    // vinserti128 ymm1, ymm2, xmm3, 1.
    run(&pc, &vex(M3A, P66, true, false, 2, 0x38, &[rr(1, 3), 1]));
    assert_eq!(ymm(&pc, 1), pd([1, 2, 5, 6]));
    // vextracti128 xmm4, ymm2, 1.
    run(&pc, &vex(M3A, P66, true, false, 0, 0x39, &[rr(2, 4), 1]));
    assert_eq!(ymm(&pc, 4), pd([3, 4, 0, 0]));
    // vperm2i128 ymm1, ymm2, ymm3, 0x31: a's upper half, then b's.
    run(&pc, &vex(M3A, P66, true, false, 2, 0x46, &[rr(1, 3), 0x31]));
    assert_eq!(ymm(&pc, 1), pd([3, 4, 7, 8]));
    // vpermd ymm1, ymm2, ymm3: indices in vvvv, data in r/m, three bits of
    // each index (*Intel SDM* volume 2, `VPERMD`).
    set_ymm(
        &pc,
        &[
            (2, dd([7, 6, 5, 4, 3, 2, 1, 8])),
            (3, dd([10, 11, 12, 13, 14, 15, 16, 17])),
        ],
    );
    run(&pc, &vex(M38, P66, true, false, 2, 0x36, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), dd([17, 16, 15, 14, 13, 12, 11, 10]));
    run(&pc, &vex(M38, P66, true, false, 2, 0x16, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1), dd([17, 16, 15, 14, 13, 12, 11, 10]));
    // vpermq ymm1, ymm3, 0x1b reverses; vpermpd 0x0e picks 2, 3, 0, 0.
    set_ymm(&pc, &[(3, pd([10, 11, 12, 13]))]);
    run(&pc, &vex(M3A, P66, true, true, 0, 0x00, &[rr(1, 3), 0x1b]));
    assert_eq!(ymm(&pc, 1), pd([13, 12, 11, 10]));
    run(&pc, &vex(M3A, P66, true, true, 0, 0x01, &[rr(1, 3), 0x0e]));
    assert_eq!(ymm(&pc, 1), pd([12, 13, 10, 10]));
    // `VPERMQ` is `W1` and 256-bit only; `VPERMD` is 256-bit only.
    assert_eq!(
        exec(&pc, &vex(M3A, P66, true, false, 0, 0x00, &[rr(1, 3), 0])),
        Some(6)
    );
    assert_eq!(
        exec(&pc, &vex(M3A, P66, false, true, 0, 0x00, &[rr(1, 3), 0])),
        Some(6)
    );
    assert_eq!(
        exec(&pc, &vex(M38, P66, false, false, 2, 0x36, &[rr(1, 3)])),
        Some(6)
    );
    // vpblendd: one immediate bit per doubleword across both lanes.
    set_ymm(
        &pc,
        &[
            (2, dd([0, 1, 2, 3, 4, 5, 6, 7])),
            (3, dd([10, 11, 12, 13, 14, 15, 16, 17])),
        ],
    );
    run(
        &pc,
        &vex(M3A, P66, true, false, 2, 0x02, &[rr(1, 3), 0b1010_0101]),
    );
    assert_eq!(ymm(&pc, 1), dd([10, 1, 12, 3, 4, 15, 6, 17]));
}

#[test]
fn vpmaskmov_never_touches_a_masked_off_element() {
    // *Intel SDM* volume 2, `VPMASKMOV`: as `VMASKMOV`, faults only from
    // the elements the mask selects. `ES` ends sixteen bytes in.
    let pc = avx2();
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::ES)].limit = (DATA + 15) as u32;
    pc.cpu.set_sys(sys);
    write_ymm(&pc, DATA, pd([0x11, 0x22, 0x33, 0x44]));
    let n = 1u64 << 63;
    set_ymm(&pc, &[(2, pd([0, n, 0, 0])), (1, pd([!0; 4]))]);
    // es: vpmaskmovq ymm1, ymm2, [DATA]
    let mut load = alloc::vec![0x26];
    load.extend(vex(M38, P66, true, true, 2, 0x8c, &m32(1, DATA)));
    run(&pc, &load);
    assert_eq!(ymm(&pc, 1), pd([0, 0x22, 0, 0]));
    set_ymm(&pc, &[(2, pd([0, 0, n, 0]))]);
    assert_eq!(exec(&pc, &load), Some(13));
    // es: vpmaskmovd [DATA], ymm2, ymm3: doubleword masks.
    let d = 0x8000_0000;
    set_ymm(&pc, &[(2, dd([0, d, 0, 0, 0, 0, 0, 0])), (3, dd([9; 8]))]);
    let mut store = alloc::vec![0x26];
    store.extend(vex(M38, P66, true, false, 2, 0x8e, &m32(3, DATA)));
    run(&pc, &store);
    assert_eq!(read64(&pc, DATA), 0x11 | (9 << 32));
    assert_eq!(read64(&pc, DATA + 8), 0x22);
}

// -- The gathers ---------------------------------------------------------------

/// `[eax + idx*scale]` as a VSIB ModRM and SIB, `reg` the destination.
fn vsib(reg: u8, index: u8, scale: u8) -> Vec<u8> {
    alloc::vec![(reg << 3) | 4, (scale << 6) | (index << 3)]
}

/// `[DATA + idx*4]`, no base.
fn vsib_abs(reg: u8, index: u8) -> Vec<u8> {
    let mut out = alloc::vec![(reg << 3) | 4, (2 << 6) | (index << 3) | 5];
    out.extend_from_slice(&(DATA as u32).to_le_bytes());
    out
}

/// Doublewords `100 + i` at `DATA + 4i`, and quadwords `1000 + i` at
/// `OUT + 8i`.
fn gather_memory(pc: &Pc) {
    for i in 0..16u64 {
        pc.write32(DATA + 4 * i, 100 + i);
        write64(pc, OUT + 8 * i, 1000 + i);
    }
}

#[test]
fn a_gather_loads_each_selected_element_and_clears_the_mask() {
    // *Intel SDM* volume 2, `VPGATHERDD`: signed indices, scaled, added to
    // the base; an element whose mask sign is clear keeps its old value;
    // the mask is zero afterwards.
    let pc = avx2();
    gather_memory(&pc);
    set_regs(&pc, DATA + 16);
    let n = 0x8000_0000;
    let neg = |v: i32| v as u32;
    set_ymm(
        &pc,
        &[
            (1, dd([0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7])),
            (2, dd([0, neg(-4), 1, 2, neg(-1), 5, 3, 9])),
            (3, dd([n, n, 0x7fff_ffff, n, n | 1, n, n, n])),
        ],
    );
    // vpgatherdd ymm1, [eax + ymm2*4], ymm3
    run(&pc, &vex(M38, P66, true, false, 3, 0x90, &vsib(1, 2, 2)));
    assert_eq!(ymm(&pc, 1), dd([104, 100, 0xd2, 106, 103, 109, 107, 113]));
    assert_eq!(ymm(&pc, 3), pd([0; 4]));
    // vpgatherqd xmm1, [eax + ymm2*4], xmm3: four quadword indices, four
    // doublewords into an `XMM` register, everything above zeroed.
    set_ymm(
        &pc,
        &[
            (1, pd([!0; 4])),
            (2, pd([0, (-4i64) as u64, 9, 2])),
            (
                3,
                pd([0x8000_0000_8000_0000, 0x8000_0000_8000_0000, !0, !0]),
            ),
        ],
    );
    run(&pc, &vex(M38, P66, true, false, 3, 0x91, &vsib(1, 2, 2)));
    assert_eq!(ymm(&pc, 1), [[100 << 32 | 104, 106 << 32 | 113], [0, 0]]);
    assert_eq!(ymm(&pc, 3), pd([0; 4]));
    // vgatherdpd ymm1, [eax + xmm2*8], ymm3: four doubleword indices, four
    // quadwords.
    set_regs(&pc, OUT);
    set_ymm(
        &pc,
        &[
            (1, pd([0; 4])),
            (2, dd([3, 0, 1, 2, 99, 99, 99, 99])),
            (3, pd([1 << 63, 1 << 63, 0, 1 << 63])),
        ],
    );
    run(&pc, &vex(M38, P66, true, true, 3, 0x92, &vsib(1, 2, 3)));
    assert_eq!(ymm(&pc, 1), pd([1003, 1000, 0, 1002]));
    // vpgatherqq xmm1, [eax + xmm2*8], xmm3: two, and the upper half zeroed.
    set_ymm(
        &pc,
        &[
            (1, pd([!0; 4])),
            (2, pd([5, 7, 0, 0])),
            (3, pd([1 << 63, 1 << 63, !0, !0])),
        ],
    );
    run(&pc, &vex(M38, P66, false, true, 3, 0x91, &vsib(1, 2, 3)));
    assert_eq!(ymm(&pc, 1), pd([1005, 1007, 0, 0]));
    assert_eq!(ymm(&pc, 3), pd([0; 4]));
}

#[test]
fn in_64_bit_mode_vex_x_extends_the_vector_index() {
    // *Intel SDM* volume 2 §2.3.12: the VSIB index is `VEX.X̄` and SIB.index,
    // so ymm10 is reachable; `vvvv` reaches ymm11 and `VEX.R̄` ymm9.
    let pc = Pc::with_features(Variant::X86_64, features2());
    gather_memory(&pc);
    let n = 0x8000_0000;
    set_ymm(
        &pc,
        &[
            (9, dd([0; 8])),
            (10, dd([7, 6, 5, 4, 3, 2, 1, 0])),
            (11, dd([n; 8])),
        ],
    );
    // xsetbv with XCR0 = x87 | SSE | AVX; mov rax, DATA;
    // vpgatherdd ymm9, [rax + ymm10*4], ymm11; hlt
    let mut code = alloc::vec![0x31, 0xc9, 0x31, 0xd2, 0xb8];
    code.extend_from_slice(&((xcr0::X87 | xcr0::SSE | xcr0::AVX) as u32).to_le_bytes());
    code.extend([0x0f, 0x01, 0xd1, 0x48, 0xb8]);
    code.extend_from_slice(&DATA.to_le_bytes());
    code.extend([0xc4, 0x22, 0x25, 0x90, 0x0c, 0x90, 0xf4]);
    let pc = super::long_mode::run64_on(pc, cr4::OSFXSR | cr4::OSXMMEXCPT | cr4::OSXSAVE, &code);
    assert_eq!(ymm(&pc, 9), dd([107, 106, 105, 104, 103, 102, 101, 100]));
    assert_eq!(ymm(&pc, 11), dd([0; 8]));
}

#[test]
fn a_gather_is_undefined_with_shared_registers_or_no_vsib() {
    // *Intel SDM* volume 2, `VPGATHERDD`: any two of destination, index and
    // mask the same is `#UD`, and so is a memory operand without a SIB byte
    // or a register operand.
    let pc = avx2();
    gather_memory(&pc);
    set_regs(&pc, DATA);
    let n = 0x8000_0000;
    set_ymm(&pc, &[(2, dd([0; 8])), (3, dd([n; 8]))]);
    for (reg, vvvv, index) in [(2, 3, 2), (1, 1, 2), (1, 2, 2)] {
        assert_eq!(
            exec(
                &pc,
                &vex(M38, P66, true, false, vvvv, 0x90, &vsib(reg, index, 2))
            ),
            Some(6),
            "dest {reg}, mask {vvvv}, index {index}"
        );
    }
    assert_eq!(ymm(&pc, 3), dd([n; 8]), "nothing was done");
    assert_eq!(
        exec(&pc, &vex(M38, P66, true, false, 3, 0x90, &[0x08])),
        Some(6),
        "no SIB"
    );
    assert_eq!(
        exec(&pc, &vex(M38, P66, true, false, 3, 0x90, &[rr(1, 2)])),
        Some(6),
        "register"
    );
    run(&pc, &vex(M38, P66, true, false, 3, 0x90, &vsib(1, 2, 2)));
}

#[test]
fn a_fault_mid_gather_leaves_the_progress_in_the_mask() {
    // *Intel SDM* volume 2, `VPGATHERDD`: "This instruction can be suspended
    // by an exception if at least one element is already gathered … the
    // elements already gathered … have their mask cleared." `ES` ends at
    // `DATA + 31`, so the element at index 8 faults and those before it do
    // not; restarting after the limit is raised finishes only what is left.
    let pc = avx2();
    gather_memory(&pc);
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::ES)].limit = (DATA + 31) as u32;
    pc.cpu.set_sys(sys);
    let n = 0x8000_0000;
    set_ymm(
        &pc,
        &[
            (1, dd([0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7])),
            (2, dd([0, 1, 2, 3, 8, 4, 5, 6])),
            (3, dd([n, 0, n, n, n, n, 0x8000_0005, n])),
        ],
    );
    // es: vpgatherdd ymm1, [DATA + ymm2*4], ymm3
    let mut code = alloc::vec![0x26];
    code.extend(vex(M38, P66, true, false, 3, 0x90, &vsib_abs(1, 2)));
    assert_eq!(exec(&pc, &code), Some(13));
    assert_eq!(
        ymm(&pc, 1),
        dd([100, 0xd1, 102, 103, 0xd4, 0xd5, 0xd6, 0xd7]),
        "the elements below the fault, and only those"
    );
    assert_eq!(
        ymm(&pc, 3),
        dd([0, 0, 0, 0, !0, !0, !0, !0]),
        "completed elements cleared, the rest reduced to their signs"
    );
    // Restart: element 0's memory changes, and is not read again.
    pc.write32(DATA, 999);
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::ES)].limit = u32::MAX;
    pc.cpu.set_sys(sys);
    run(&pc, &code);
    assert_eq!(ymm(&pc, 1), dd([100, 0xd1, 102, 103, 108, 104, 105, 106]));
    assert_eq!(ymm(&pc, 3), dd([0; 8]));
}

// -- FMA -------------------------------------------------------------------------

const S13: u32 = 0x4150_0000;
const S11: u32 = 0x4130_0000;
const S17: u32 = 0x4188_0000;

#[test]
fn the_three_operand_orders() {
    // *Intel SDM* volume 2, `VFMADD132PS`: 132 is dst*src3 + src2, 213
    // src2*dst + src3, 231 src2*src3 + dst. With 2, 3 and 5: 13, 11, 17.
    let pc = avx2();
    let go = |opcode: u8| {
        set_ymm(&pc, &[(1, ps([S2; 8])), (2, ps([S3; 8])), (3, ps([S5; 8]))]);
        run(&pc, &vex(M38, P66, true, false, 2, opcode, &[rr(1, 3)]));
        ymm(&pc, 1)
    };
    assert_eq!(go(0x98), ps([S13; 8]), "132");
    assert_eq!(go(0xa8), ps([S11; 8]), "213");
    assert_eq!(go(0xb8), ps([S17; 8]), "231");
    // The signs, all at 231: 15 - 2, -15 + 2, -15 - 2.
    let neg = |v: u32| v | 0x8000_0000;
    assert_eq!(go(0xba), ps([S13; 8]), "vfmsub");
    assert_eq!(go(0xbc), ps([neg(S13); 8]), "vfnmadd");
    assert_eq!(go(0xbe), ps([neg(S17); 8]), "vfnmsub");
    // The alternating forms: `VFMADDSUB` subtracts in the even elements.
    assert_eq!(go(0xb6), ps([S13, S17, S13, S17, S13, S17, S13, S17]));
    assert_eq!(go(0xb7), ps([S17, S13, S17, S13, S17, S13, S17, S13]));
    // `VEX.W1`: doubles, at 128 bits with the upper half cleared.
    set_ymm(
        &pc,
        &[
            (1, pd([D2, D2, 7, 7])),
            (2, pd([D3, D3, 0, 0])),
            (3, pd([D4, D1, 0, 0])),
        ],
    );
    run(&pc, &vex(M38, P66, false, true, 2, 0xb8, &[rr(1, 3)]));
    // 3*4 + 2 = 14, 3*1 + 2 = 5.
    assert_eq!(
        ymm(&pc, 1),
        pd([0x402c_0000_0000_0000, 0x4014_0000_0000_0000, 0, 0])
    );
}

#[test]
fn a_scalar_fma_keeps_the_destination_above_its_element() {
    // *Intel SDM* volume 2, `VFMADD231SD`: DEST[127:64] unchanged, the rest
    // of the `YMM` register zeroed; the memory operand is eight bytes.
    let pc = avx2();
    set_ymm(&pc, &[(1, pd([D2, 0x1234, 9, 9])), (2, pd([D3, 7, 7, 7]))]);
    write_ymm(&pc, DATA, pd([D4, 0x5555, 0, 0]));
    run(&pc, &vex(M38, P66, false, true, 2, 0xb9, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), pd([0x402c_0000_0000_0000, 0x1234, 0, 0]));
    // `VEX.L` is ignored by the scalar forms.
    set_ymm(&pc, &[(1, pd([D2, 0x1234, 9, 9]))]);
    run(&pc, &vex(M38, P66, true, true, 2, 0xb9, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), pd([0x402c_0000_0000_0000, 0x1234, 0, 0]));
}

#[test]
fn an_fma_rounds_once() {
    // (1 + 2^-52)(1 - 2^-52) - 1 = -2^-104 exactly, where the product
    // rounded on its own is 1.0 and the difference zero.
    let pc = avx2();
    set_ymm(
        &pc,
        &[
            (1, pd([0xbff0_0000_0000_0000, 0, 0, 0])),
            (2, pd([0x3ff0_0000_0000_0001, 0, 0, 0])),
            (3, pd([0x3fef_ffff_ffff_fffe, 0, 0, 0])),
        ],
    );
    run(&pc, &vex(M38, P66, false, true, 2, 0xb9, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1)[0][0], 0xb970_0000_0000_0000);
    assert_eq!(mxcsr_now(&pc) & mxcsr::EXCEPTIONS, 0, "exact");
    // vmulsd then vaddsd: zero, and inexact.
    set_ymm(&pc, &[(4, pd([0x3ff0_0000_0000_0001, 0, 0, 0]))]);
    run(&pc, &vex(M0F, PF2, false, false, 4, 0x59, &[rr(4, 3)]));
    set_ymm(&pc, &[(5, pd([0xbff0_0000_0000_0000, 0, 0, 0]))]);
    run(&pc, &vex(M0F, PF2, false, false, 4, 0x58, &[rr(4, 5)]));
    assert_eq!(ymm(&pc, 4)[0][0], 0);
}

#[test]
fn fma_special_cases_follow_the_sdm_table() {
    // *Intel SDM* volume 1 table 14-17, "FMA Numeric Behavior": the first
    // NaN of x, y, z in x*y + z, quieted; invalid for any signaling NaN,
    // for inf * 0 unless z is a NaN, and for inf - inf.
    let pc = avx2();
    let inf = 0x7f80_0000u32;
    // vfmadd231ss xmm1, xmm2, xmm3: x = xmm2, y = xmm3, z = xmm1.
    let fma231 = |z: u32, x: u32, y: u32| {
        set_mxcsr(&pc, mxcsr::RESET);
        set_ymm(
            &pc,
            &[
                (1, ps([z, 0, 0, 0, 0, 0, 0, 0])),
                (2, ps([x, 0, 0, 0, 0, 0, 0, 0])),
                (3, ps([y, 0, 0, 0, 0, 0, 0, 0])),
            ],
        );
        run(&pc, &vex(M38, P66, false, false, 2, 0xb9, &[rr(1, 3)]));
        (ymm(&pc, 1)[0][0] as u32, mxcsr_now(&pc) & mxcsr::IE != 0)
    };
    assert_eq!(
        fma231(0x7fc0_1234, inf, 0),
        (0x7fc0_1234, false),
        "inf*0 + QNaN"
    );
    assert_eq!(fma231(S1, inf, 0), (0xffc0_0000, true), "inf*0 + 1");
    assert_eq!(
        fma231(0x7f80_0001, 0, inf),
        (0x7fc0_0001, true),
        "0*inf + SNaN"
    );
    assert_eq!(
        fma231(inf | 0x8000_0000, inf, S1),
        (0xffc0_0000, true),
        "inf - inf"
    );
    assert_eq!(
        fma231(0x7fc0_000c, 0x7fc0_000a, 0x7fc0_000b),
        (0x7fc0_000a, false)
    );
    assert_eq!(fma231(0x7fc0_000c, S1, 0x7f80_000b), (0x7fc0_000b, true));
    // In 132 order x is the destination and y the third source: a NaN in
    // the third source outranks one in the second, which is the reverse of
    // the encoding's order.
    set_ymm(
        &pc,
        &[
            (1, ps([S1, 0, 0, 0, 0, 0, 0, 0])),
            (2, ps([0x7fc0_000b, 0, 0, 0, 0, 0, 0, 0])),
            (3, ps([0x7fc0_000a, 0, 0, 0, 0, 0, 0, 0])),
        ],
    );
    run(&pc, &vex(M38, P66, false, false, 2, 0x99, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1)[0][0] as u32, 0x7fc0_000a);
    // A negated form returns a NaN operand as it was, sign included.
    set_ymm(
        &pc,
        &[
            (1, ps([S1, 0, 0, 0, 0, 0, 0, 0])),
            (2, ps([0x7fc0_000a, 0, 0, 0, 0, 0, 0, 0])),
            (3, ps([S1, 0, 0, 0, 0, 0, 0, 0])),
        ],
    );
    run(&pc, &vex(M38, P66, false, false, 2, 0xbd, &[rr(1, 3)]));
    assert_eq!(ymm(&pc, 1)[0][0] as u32, 0x7fc0_000a, "vfnmadd231ss");
    // Unmasked, invalid is `#XM` and the destination is left alone.
    set_mxcsr(&pc, mxcsr::RESET & !mxcsr::IM);
    set_ymm(&pc, &[(1, ps([S1; 8])), (2, ps([inf; 8])), (3, ps([0; 8]))]);
    assert_eq!(
        exec(&pc, &vex(M38, P66, true, false, 2, 0xb8, &[rr(1, 3)])),
        Some(19)
    );
    assert_eq!(ymm(&pc, 1), ps([S1; 8]));
}

// -- F16C ------------------------------------------------------------------------

#[test]
fn vcvtph2ps_is_exact_and_ignores_daz() {
    // *Intel SDM* volume 2, `VCVTPH2PS`: a denormal half gives the normal
    // single it equals, `MXCSR.DAZ` is treated as 0 and no denormal
    // exception is reported; a signaling NaN is quieted with invalid.
    let pc = avx2();
    set_mxcsr(&pc, mxcsr::RESET | mxcsr::DAZ);
    set_ymm(
        &pc,
        &[(
            2,
            pd([0x7c00_0001_c000_3c00, 0x8000_7c01_7e00_fc00, !0, !0]),
        )],
    );
    run(&pc, &vex(M38, P66, true, false, 0, 0x13, &[rr(1, 2)]));
    assert_eq!(
        ymm(&pc, 1),
        ps([
            S1,
            0xc000_0000,
            0x3380_0000,
            0x7f80_0000,
            0xff80_0000,
            0x7fc0_0000,
            0x7fc0_2000,
            0x8000_0000
        ])
    );
    assert_eq!(mxcsr_now(&pc) & mxcsr::EXCEPTIONS, mxcsr::IE);
    // VEX.128 from eight bytes of memory.
    write_ymm(&pc, DATA, pd([0x4400_4200_4000_3c00, !0, 0, 0]));
    run(&pc, &vex(M38, P66, false, false, 0, 0x13, &m32(1, DATA)));
    assert_eq!(ymm(&pc, 1), ps([S1, S2, S3, S4, 0, 0, 0, 0]));
}

#[test]
fn vcvtps2ph_rounds_by_the_immediate_or_by_mxcsr() {
    // *Intel SDM* volume 2, `VCVTPS2PH`: imm8[2] selects MXCSR.RC, else
    // imm8[1:0] is the rounding mode; FTZ is ignored; overflow, underflow,
    // inexact and invalid are reported.
    let pc = avx2();
    let src = ps([
        S1,
        0x3f80_1000, // 1 + 2^-11, halfway between two halves
        0x477f_f000, // 65520, halfway between 65504 and 65536
        0x47c3_5000, // 100000
        0x3580_0000, // 2^-20, exactly a denormal half
        0x3280_0000, // 2^-26, below the smallest half
        0x7fc0_2000, // a quiet NaN with a payload that survives
        0x7f80_0001, // a signaling NaN whose payload does not
    ]);
    let convert = |imm: u8, rc: u32| {
        set_mxcsr(&pc, mxcsr::RESET | mxcsr::FTZ | (rc << mxcsr::RC_SHIFT));
        set_ymm(&pc, &[(2, src), (1, pd([!0; 4]))]);
        run(&pc, &vex(M3A, P66, true, false, 0, 0x1d, &[rr(2, 1), imm]));
        let out = ymm(&pc, 1);
        assert_eq!(out[1], [0, 0], "the upper half is zeroed");
        (out[0], mxcsr_now(&pc) & mxcsr::EXCEPTIONS)
    };
    let halves = |h: [u64; 8]| {
        [
            h[0] | h[1] << 16 | h[2] << 32 | h[3] << 48,
            h[4] | h[5] << 16 | h[6] << 32 | h[7] << 48,
        ]
    };
    let (out, flags) = convert(0, 0);
    assert_eq!(
        out,
        halves([0x3c00, 0x3c00, 0x7c00, 0x7c00, 0x0010, 0, 0x7e01, 0x7e00])
    );
    assert_eq!(
        flags,
        mxcsr::IE | mxcsr::OE | mxcsr::UE | mxcsr::PE,
        "and no FTZ"
    );
    // Toward +inf by the immediate.
    let (out, _) = convert(2, 0);
    assert_eq!(
        out,
        halves([
            0x3c00, 0x3c01, 0x7c00, 0x7c00, 0x0010, 0x0001, 0x7e01, 0x7e00
        ])
    );
    // Toward zero: the largest finite half instead of infinity.
    let (out, _) = convert(3, 0);
    assert_eq!(
        out,
        halves([0x3c00, 0x3c00, 0x7bff, 0x7bff, 0x0010, 0, 0x7e01, 0x7e00])
    );
    // imm8[2]: MXCSR's toward-+inf, whatever imm8[1:0] says.
    let (out, _) = convert(4 | 3, 2);
    assert_eq!(out[0] >> 16 & 0xffff, 0x3c01);
    // The 128-bit memory form writes eight bytes and no more.
    write64(&pc, OUT + 8, 0x7777);
    set_mxcsr(&pc, mxcsr::RESET);
    run(
        &pc,
        &vex(M3A, P66, false, false, 0, 0x1d, &with(m32(2, OUT), 0)),
    );
    assert_eq!(
        read64(&pc, OUT),
        halves([0x3c00, 0x3c00, 0x7c00, 0x7c00, 0, 0, 0, 0])[0]
    );
    assert_eq!(read64(&pc, OUT + 8), 0x7777);
}

#[test]
fn the_disassembler_names_the_new_instructions() {
    // A gather prints its VSIB operand with a vector index, and its mask
    // last; the `W1` twins print their own mnemonics.
    let text = |code: &[u8]| {
        super::super::disasm::disassemble_as(isa::Gen::I386, isa::Bits::B32, 0, 0, code).to_string()
    };
    assert_eq!(
        text(&vex(M38, P66, true, false, 3, 0x90, &vsib(1, 2, 2))),
        "vpgatherdd ymm1, [ds:eax+ymm2*4], ymm3"
    );
    assert_eq!(
        text(&vex(M38, P66, true, false, 3, 0x91, &vsib(1, 2, 2))),
        "vpgatherqd xmm1, [ds:eax+ymm2*4], xmm3"
    );
    assert_eq!(
        text(&vex(M38, P66, true, true, 3, 0x92, &vsib(1, 2, 3))),
        "vgatherdpd ymm1, [ds:eax+xmm2*8], ymm3"
    );
    assert_eq!(
        text(&vex(M38, P66, true, true, 2, 0x98, &[rr(1, 3)])),
        "vfmadd132pd ymm1, ymm2, ymm3"
    );
    assert_eq!(
        text(&vex(M3A, P66, true, true, 0, 0x00, &[rr(1, 3), 0x1b])),
        "vpermq ymm1, ymm3, 0x1b"
    );
    assert_eq!(
        text(&vex(M38, P66, true, false, 0, 0x13, &[rr(1, 2)])),
        "vcvtph2ps ymm1, xmm2"
    );
    assert_eq!(
        text(&vex(M0F, P66, true, false, 2, 0xfc, &[rr(1, 3)])),
        "vpaddb ymm1, ymm2, ymm3"
    );
}
