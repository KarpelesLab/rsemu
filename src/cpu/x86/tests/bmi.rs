//! BMI1 and BMI2, executed as a guest sees them: the gate, the operand
//! sizes, the flags, and each instruction on hand-checked values.
//!
//! The encodings are built by [`vex`] from the fields each *Intel SDM*
//! volume 2 page's opcode line gives (`VEX.LZ.F2.0F38.W0 F6 /r` and so on),
//! so each test names the instruction it means. `TZCNT` is legacy-encoded
//! and is tested beside `LZCNT`, in the long-mode module.

use super::long_mode::{cpuid64, faults64, run64_on};
use super::*;
use crate::cpu::x86::prot::cr0;

/// Where a fault handler sits, and `RIP` once its `hlt` has retired.
const HANDLER: u64 = 0x3800;
/// Scratch memory for the memory forms.
const DATA: u64 = 0xb000;

/// The `VEX.mmmmm` maps.
const M38: u8 = 2;
const M3A: u8 = 3;
/// The `VEX.pp` columns.
const NP: u8 = 0;
const P66: u8 = 1;
const PF3: u8 = 2;
const PF2: u8 = 3;

/// A three-byte VEX instruction: `map`, `pp`, `L`, `W`, `vvvv` (0-15,
/// inverted here), the opcode, ModRM `reg` and `rm` as register numbers
/// (0-15, their fourth bits in `R̄` and `B̄`), and `rest` after the ModRM
/// byte. With `memory`, ModRM is `mod == 00`, `r/m == 101` and `rest` must
/// start with the displacement.
#[allow(clippy::too_many_arguments)]
fn vex(
    map: u8,
    pp: u8,
    l: bool,
    w: bool,
    vvvv: u8,
    opcode: u8,
    reg: u8,
    rm: u8,
    rest: &[u8],
) -> Vec<u8> {
    let r = u8::from(reg < 8) << 7;
    let b = u8::from(rm < 8) << 5;
    let mut out = alloc::vec![
        0xc4,
        r | 0x40 | b | map,
        (u8::from(w) << 7) | ((!vvvv & 15) << 3) | (u8::from(l) << 2) | pp,
        opcode,
        0xc0 | ((reg & 7) << 3) | (rm & 7),
    ];
    out.extend_from_slice(rest);
    out
}

/// The same with a `disp32` memory operand at `addr` in place of `rm`.
fn vex_mem(map: u8, pp: u8, w: bool, vvvv: u8, opcode: u8, reg: u8, addr: u64) -> Vec<u8> {
    let mut out = vex(map, pp, false, w, vvvv, opcode, reg, 0, &[]);
    let modrm = out.len() - 1;
    out[modrm] = ((reg & 7) << 3) | 0b101;
    out.extend_from_slice(&(addr as u32).to_le_bytes());
    out
}

fn bmi() -> Features {
    Features {
        bmi1: true,
        bmi2: true,
        ..Features::X86_64
    }
}

/// A part with `features` in flat 32-bit protected mode — `CR4` as reset
/// left it, so no `OSXSAVE` and no `OSFXSR` — with a `hlt` handler for
/// `#UD` and `#NM`.
fn pc_with(features: Features) -> Pc {
    let pc = Pc::with_features(Variant::X86_64, features);
    pc.start_protected();
    for v in [6u64, 7, 13, 14] {
        let at = HANDLER + v * 16;
        pc.idt(v, gate(0x08, at as u32, sys_type::INT_GATE32, 0));
        pc.write(at, &[0xf4]);
    }
    pc
}

/// The eight 32-bit registers, `EAX` to `EDI`, and the flags.
type Gprs = [u32; 8];

/// Run `code` and a `hlt` with the registers `regs` and the flags `flags`
/// in; the registers and flags out, or the vector it faulted with.
fn exec(pc: &Pc, regs: Gprs, flags: u32, code: &[u8]) -> Result<(Gprs, u32), u64> {
    let mut all = code.to_vec();
    all.push(0xf4);
    pc.write(at::CODE0, &all);
    let mut r = pc.regs();
    for (i, v) in regs.into_iter().enumerate() {
        r.set_qword(i as u8, u64::from(v));
    }
    r.rsp = at::STACK0;
    r.eflags = flags | 2;
    r.rip = at::CODE0;
    pc.cpu.set_regs(r);
    pc.cpu.session.lock().state.halted = false;
    assert!(pc.run(100) < 100, "the program halted");
    let r = pc.regs();
    if r.rip == at::CODE0 + all.len() as u64 {
        let mut out = [0u32; 8];
        for (i, v) in out.iter_mut().enumerate() {
            *v = r.qword(i as u8) as u32;
        }
        return Ok((out, r.eflags));
    }
    let vector = [6u64, 7, 13, 14]
        .into_iter()
        .find(|v| r.rip == HANDLER + v * 16 + 1);
    Err(vector.unwrap_or_else(|| panic!("stopped at {:#x}", r.rip)))
}

/// [`exec`] on a fresh BMI part with no flags in, which must not fault.
fn run(regs: Gprs, code: &[u8]) -> (Gprs, u32) {
    exec(&pc_with(bmi()), regs, 0, code).unwrap_or_else(|v| panic!("{code:02x?}: vector {v}"))
}

const EAX: u8 = 0;
const ECX: u8 = 1;
const EDX: u8 = 2;
const EBX: u8 = 3;
const ESI: u8 = 6;
const EDI: u8 = 7;

/// The arithmetic flags, by name, from an `EFLAGS` image.
fn fl(eflags: u32) -> [bool; 6] {
    [
        eflags & flags::CF != 0,
        eflags & flags::PF != 0,
        eflags & flags::AF != 0,
        eflags & flags::ZF != 0,
        eflags & flags::SF != 0,
        eflags & flags::OF != 0,
    ]
}

const ALL: u32 = flags::CF | flags::PF | flags::AF | flags::ZF | flags::SF | flags::OF;

/// Registers with only the named ones set.
fn regs(set: &[(u8, u32)]) -> Gprs {
    let mut out = [0u32; 8];
    out[usize::from(ESI)] = 0x5151_5151;
    for &(r, v) in set {
        out[usize::from(r)] = v;
    }
    out
}

// -- BMI1 ---------------------------------------------------------------------

/// `ANDN r32a, r32b, r/m32` — `VEX.LZ.0F38.W0 F2 /r`.
fn andn(dst: u8, first: u8, second: u8) -> Vec<u8> {
    vex(M38, NP, false, false, first, 0xf2, dst, second, &[])
}

#[test]
fn andn_inverts_its_first_source() {
    // *Intel SDM* volume 2, `ANDN`: `DEST := (NOT SRC1) BITWISE AND SRC2`;
    // SF and ZF from the result, OF and CF cleared, AF and PF undefined —
    // left as they were here.
    let pc = pc_with(bmi());
    let r = regs(&[(ECX, 0xf0f0_f0f0), (EDX, 0xffff_0000)]);
    let (out, f) = exec(&pc, r, ALL, &andn(EAX, ECX, EDX)).unwrap();
    assert_eq!(out[0], 0x0f0f_0000);
    assert_eq!(fl(f), [false, true, true, false, false, false]);
    let r = regs(&[(ECX, 0xffff_ffff), (EDX, 0x1234)]);
    let (out, f) = exec(&pc, r, 0, &andn(EAX, ECX, EDX)).unwrap();
    assert_eq!(out[0], 0);
    assert_eq!(fl(f), [false, false, false, true, false, false], "ZF");
    let r = regs(&[(ECX, 0), (EDX, 0x8000_0000)]);
    let (_, f) = exec(&pc, r, 0, &andn(EAX, ECX, EDX)).unwrap();
    assert!(fl(f)[4], "SF");
    // The memory form.
    pc.write(DATA, &0xff00_ff00u32.to_le_bytes());
    let code = vex_mem(M38, NP, false, ECX, 0xf2, EBX, DATA);
    let (out, _) = exec(&pc, regs(&[(ECX, 0x0ff0_0ff0)]), 0, &code).unwrap();
    assert_eq!(out[usize::from(EBX)], 0xf000_f000);
}

/// `BEXTR r32a, r/m32, r32b` — `VEX.LZ.0F38.W0 F7 /r`, the control in `vvvv`.
fn bextr(dst: u8, src: u8, control: u8) -> Vec<u8> {
    vex(M38, NP, false, false, control, 0xf7, dst, src, &[])
}

#[test]
fn bextr_takes_start_and_length_from_the_control_register() {
    // *Intel SDM* volume 2, `BEXTR`: start in bits 7:0, length in 15:8; ZF
    // from the result, CF and OF cleared, the others undefined.
    let pc = pc_with(bmi());
    let code = bextr(EAX, EBX, ECX);
    for (control, want) in [
        (0x0804, 0x67),
        (0x1010, 0x1234),
        (0x0000_0004, 0),   // length 0
        (0x0820, 0),        // start = width
        (0x08ff, 0),        // start past it
        (0xff1c, 0x1),      // runs off the top: bits 31:28
        (0xdead_0404, 0x7), // bits 31:16 of the control are not read
    ] {
        let r = regs(&[(EBX, 0x1234_5678), (ECX, control)]);
        let (out, f) = exec(&pc, r, ALL & !flags::ZF, &code).unwrap();
        assert_eq!(out[0], want, "control {control:#x}");
        assert_eq!(f & flags::ZF != 0, want == 0, "ZF for {control:#x}");
        assert_eq!(f & (flags::CF | flags::OF), 0);
        assert_eq!(
            f & (flags::SF | flags::AF | flags::PF),
            flags::SF | flags::AF | flags::PF,
            "undefined, so kept"
        );
    }
}

/// Group 17, `VEX.LZ.0F38.W0 F3 /n`: `BLSR` (1), `BLSMSK` (2), `BLSI` (3),
/// the destination in `vvvv`.
fn group17(n: u8, dst: u8, src: u8) -> Vec<u8> {
    vex(M38, NP, false, false, dst, 0xf3, n, src, &[])
}

#[test]
fn the_lowest_set_bit_group() {
    // *Intel SDM* volume 2: `BLSI` is `-SRC AND SRC`, CF set for a non-zero
    // source; `BLSMSK` is `(SRC-1) XOR SRC`, CF set for a zero source and ZF
    // cleared; `BLSR` is `(SRC-1) AND SRC`, CF set for a zero source. SF
    // from the result in all three (ZF too but for `BLSMSK`), OF cleared.
    // [CF, PF, AF, ZF, SF, OF]; PF and AF go in set and stay set.
    let cases: [(u8, u32, u32, [bool; 6]); 9] = [
        (
            3,
            0x0000_0b00,
            0x0000_0100,
            [true, true, true, false, false, false],
        ),
        (3, 0, 0, [false, true, true, true, false, false]),
        (
            3,
            0x8000_0000,
            0x8000_0000,
            [true, true, true, false, true, false],
        ),
        (
            2,
            0x0000_0b00,
            0x0000_01ff,
            [false, true, true, false, false, false],
        ),
        (2, 0, 0xffff_ffff, [true, true, true, false, true, false]),
        (
            2,
            0x8000_0000,
            0xffff_ffff,
            [false, true, true, false, true, false],
        ),
        (
            1,
            0x0000_0b00,
            0x0000_0a00,
            [false, true, true, false, false, false],
        ),
        (1, 0, 0, [true, true, true, true, false, false]),
        (1, 0x8000_0000, 0, [false, true, true, true, false, false]),
    ];
    let pc = pc_with(bmi());
    for (n, src, want, flags_out) in cases {
        let r = regs(&[(EBX, src), (EAX, 0x5555_5555)]);
        let (out, f) = exec(&pc, r, ALL & !flags::ZF, &group17(n, EAX, EBX)).unwrap();
        assert_eq!(out[0], want, "/{n} of {src:#x}");
        assert_eq!(fl(f), flags_out, "/{n} of {src:#x}");
        assert_eq!(out[usize::from(EBX)], src, "the source is not written");
    }
    // A memory source.
    pc.write(DATA, &0x0000_0b00u32.to_le_bytes());
    let mut code = vex_mem(M38, NP, false, EDI, 0xf3, 3, DATA);
    let (out, _) = exec(&pc, regs(&[]), 0, &code).unwrap();
    assert_eq!(out[usize::from(EDI)], 0x100, "blsi edi, [DATA]");
    // `/0` and `/4`-`/7` are no instruction.
    for n in [0, 4, 7] {
        code = group17(n, EAX, EBX);
        assert_eq!(exec(&pc, regs(&[]), 0, &code), Err(6), "/{n}");
    }
}

// -- BMI2 ---------------------------------------------------------------------

/// `BZHI r32a, r/m32, r32b` — `VEX.LZ.0F38.W0 F5 /r`.
fn bzhi(dst: u8, src: u8, index: u8) -> Vec<u8> {
    vex(M38, NP, false, false, index, 0xf5, dst, src, &[])
}

#[test]
fn bzhi_zeroes_from_the_index_and_says_when_it_is_past_the_width() {
    // *Intel SDM* volume 2, `BZHI`: `N := SRC2[7:0]`; bits N and up cleared
    // when N is below the operand size; CF "set if N > OperandSize - 1";
    // SF and ZF from the result; OF cleared.
    let pc = pc_with(bmi());
    for (index, want, cf) in [
        (4, 0xf, false),
        (0, 0, false),
        (31, 0x7fff_ffff, false),
        (32, 0xffff_ffff, true),
        (0xff, 0xffff_ffff, true),
        (0x1_0004, 0xf, false),
    ] {
        let r = regs(&[(EBX, 0xffff_ffff), (ECX, index)]);
        let (out, f) = exec(&pc, r, flags::OF, &bzhi(EAX, EBX, ECX)).unwrap();
        assert_eq!(out[0], want, "index {index:#x}");
        let [c, _, _, z, s, o] = fl(f);
        assert_eq!((c, z, s, o), (cf, want == 0, want >> 31 != 0, false));
    }
}

/// `PDEP` (`VEX.LZ.F2.0F38 F5`) and `PEXT` (`VEX.LZ.F3.0F38 F5`):
/// `r32a, r32b, r/m32`, the mask in r/m.
fn pdep(dst: u8, src: u8, mask: u8) -> Vec<u8> {
    vex(M38, PF2, false, false, src, 0xf5, dst, mask, &[])
}

fn pext(dst: u8, src: u8, mask: u8) -> Vec<u8> {
    vex(M38, PF3, false, false, src, 0xf5, dst, mask, &[])
}

#[test]
fn pdep_and_pext_touch_no_flag() {
    let pc = pc_with(bmi());
    let r = regs(&[(ECX, 0x1234_5678), (EDX, 0xff00_fff0)]);
    let (out, f) = exec(&pc, r, ALL, &pext(EAX, ECX, EDX)).unwrap();
    assert_eq!(out[0], 0x1_2567);
    assert_eq!(f & ALL, ALL);
    let r = regs(&[(ECX, 0x1_2567), (EDX, 0xff00_fff0)]);
    let (out, f) = exec(&pc, r, 0, &pdep(EAX, ECX, EDX)).unwrap();
    assert_eq!(out[0], 0x1200_5670);
    assert_eq!(f & ALL, 0);
    // Sparse and dense masks.
    let r = regs(&[(ECX, 0b1011), (EDX, 0x5555_5555)]);
    assert_eq!(run(r, &pdep(EAX, ECX, EDX)).0[0], 0b0100_0101);
    let r = regs(&[(ECX, 0xdead_beef), (EDX, 0xffff_ffff)]);
    assert_eq!(run(r, &pext(EAX, ECX, EDX)).0[0], 0xdead_beef);
    let r = regs(&[(ECX, 0xdead_beef), (EDX, 0)]);
    assert_eq!(run(r, &pext(EAX, ECX, EDX)).0[0], 0);
}

/// `MULX r32a, r32b, r/m32` — `VEX.LZ.F2.0F38.W0 F6 /r`: high half to
/// ModRM `reg`, low half to `vvvv`, the other factor `EDX`.
fn mulx(high: u8, low: u8, src: u8) -> Vec<u8> {
    vex(M38, PF2, false, false, low, 0xf6, high, src, &[])
}

#[test]
fn mulx_is_unsigned_two_destinations_and_no_flags() {
    let pc = pc_with(bmi());
    let r = regs(&[(EDX, 0xffff_ffff), (EBX, 0xffff_ffff)]);
    let (out, f) = exec(&pc, r, ALL, &mulx(EAX, ECX, EBX)).unwrap();
    assert_eq!((out[0], out[1]), (0xffff_fffe, 1));
    assert_eq!(f & ALL, ALL, "no flag is touched");
    // `EDX` as a destination: it is read before either is written.
    let r = regs(&[(EDX, 0x1_0000), (EBX, 0x3_0000)]);
    let (out, _) = exec(&pc, r, 0, &mulx(EDX, EAX, EBX)).unwrap();
    assert_eq!((out[usize::from(EDX)], out[0]), (3, 0));
    // The same register twice: "it will contain the high half".
    let r = regs(&[(EDX, 0x8000_0001), (EBX, 4)]);
    let (out, _) = exec(&pc, r, 0, &mulx(ECX, ECX, EBX)).unwrap();
    assert_eq!(out[1], 2);
}

/// `RORX r32, r/m32, imm8` — `VEX.LZ.F2.0F3A.W0 F0 /r ib`.
fn rorx(dst: u8, src: u8, imm: u8) -> Vec<u8> {
    vex(M3A, PF2, false, false, 0, 0xf0, dst, src, &[imm])
}

/// `SHLX` (`66`), `SHRX` (`F2`) and `SARX` (`F3`), `VEX.LZ.0F38.W0 F7 /r`:
/// `r32a, r/m32, r32b`, the count in `vvvv`.
fn shiftx(pp: u8, dst: u8, src: u8, count: u8) -> Vec<u8> {
    vex(M38, pp, false, false, count, 0xf7, dst, src, &[])
}

#[test]
fn the_flagless_shifts_and_rotate_mask_their_counts() {
    let pc = pc_with(bmi());
    // Counts are masked to five bits in a 32-bit operation: 33 is 1.
    for (pp, count, want) in [
        (P66, 4, 0x1234_5670),
        (P66, 33, 0x0246_8ace),
        (PF2, 4, 0x0812_3456),
        (PF2, 32, 0x8123_4567),
        (PF3, 4, 0xf812_3456),
        (PF3, 35, 0xf024_68ac),
        (PF3, 31, 0xffff_ffff),
    ] {
        let r = regs(&[(EBX, 0x8123_4567), (ECX, count)]);
        let (out, f) = exec(&pc, r, ALL, &shiftx(pp, EAX, EBX, ECX)).unwrap();
        assert_eq!(out[0], want, "pp {pp} count {count}");
        assert_eq!(f & ALL, ALL, "no flag is touched");
    }
    // `RORX`, with the immediate masked to five bits too.
    let r = regs(&[(EBX, 0x1234_5678)]);
    assert_eq!(run(r, &rorx(EAX, EBX, 8)).0[0], 0x7812_3456);
    assert_eq!(run(r, &rorx(EAX, EBX, 40)).0[0], 0x7812_3456);
    let (out, f) = exec(&pc, r, ALL, &rorx(EAX, EBX, 0)).unwrap();
    assert_eq!((out[0], f & ALL), (0x1234_5678, ALL));
}

// -- The gate -----------------------------------------------------------------

/// One encoding of each instruction, register forms.
fn one_of_each() -> [(Vec<u8>, bool); 13] {
    [
        (andn(EAX, ECX, EDX), true),
        (bextr(EAX, ECX, EDX), true),
        (group17(1, EAX, ECX), true),
        (group17(2, EAX, ECX), true),
        (group17(3, EAX, ECX), true),
        (bzhi(EAX, ECX, EDX), false),
        (pdep(EAX, ECX, EDX), false),
        (pext(EAX, ECX, EDX), false),
        (mulx(EAX, ECX, EBX), false),
        (rorx(EAX, ECX, 3), false),
        (shiftx(P66, EAX, ECX, EDX), false),
        (shiftx(PF2, EAX, ECX, EDX), false),
        (shiftx(PF3, EAX, ECX, EDX), false),
    ]
}

#[test]
fn each_needs_its_own_cpuid_bit() {
    let only1 = Features {
        bmi2: false,
        ..bmi()
    };
    let only2 = Features {
        bmi1: false,
        ..bmi()
    };
    for (code, is_bmi1) in one_of_each() {
        let none = exec(&pc_with(Features::X86_64), regs(&[]), 0, &code);
        assert_eq!(none.err(), Some(6), "{code:02x?} without either");
        let one = exec(&pc_with(only1), regs(&[]), 0, &code).err();
        let two = exec(&pc_with(only2), regs(&[]), 0, &code).err();
        if is_bmi1 {
            assert_eq!((one, two), (None, Some(6)), "{code:02x?}");
        } else {
            assert_eq!((one, two), (Some(6), None), "{code:02x?}");
        }
    }
}

#[test]
fn vex_l_is_undefined_opcode_and_avx_state_is_not_consulted() {
    // *Intel SDM* volume 2 §2.5.1, type 13: `#UD` for `VEX.L = 1`, and
    // nothing about `CR4.OSXSAVE`, `XCR0` or `CR0.TS` — which `pc_with`
    // leaves at reset, with no `OSXSAVE` and no `OSFXSR`, and `TS` is set
    // here.
    let pc = pc_with(bmi());
    let mut sys = pc.cpu.sys();
    sys.cr0 |= cr0::TS | cr0::EM;
    pc.cpu.set_sys(sys);
    for (code, _) in one_of_each() {
        assert_eq!(exec(&pc, regs(&[]), 0, &code).err(), None, "{code:02x?}");
        let mut long = code.clone();
        long[2] |= 4;
        assert_eq!(exec(&pc, regs(&[]), 0, &long).err(), Some(6), "{long:02x?}");
    }
    // `vvvv` must be 1111b where nothing reads it: `RORX`.
    let code = vex(M3A, PF2, false, false, 1, 0xf0, EAX, EBX, &[1]);
    assert_eq!(exec(&pc, regs(&[]), 0, &code).err(), Some(6));
    // And a legacy prefix in front of the VEX prefix is `#UD`.
    let mut code = alloc::vec![0x66];
    code.extend(andn(EAX, ECX, EDX));
    assert_eq!(exec(&pc, regs(&[]), 0, &code).err(), Some(6));
}

#[test]
fn the_legacy_encodings_of_the_cells_are_not_instructions() {
    // `0F 38 F2`, `0F 38 F3 /1`, `0F 38 F5`, `0F 38 F7`, `F2 0F 38 F6`,
    // `F2 0F 3A F0`: the `only` in the row. (`66 0F 38 F6` is ADX's
    // `ADCX`, not implemented, and `#UD` too.)
    let pc = pc_with(bmi());
    for code in [
        &[0x0f, 0x38, 0xf2, 0xc1][..],
        &[0x0f, 0x38, 0xf3, 0xc9],
        &[0x0f, 0x38, 0xf5, 0xc1],
        &[0x0f, 0x38, 0xf7, 0xc1],
        &[0xf2, 0x0f, 0x38, 0xf6, 0xc1],
        &[0xf2, 0x0f, 0x3a, 0xf0, 0xc1, 0x01],
    ] {
        assert_eq!(exec(&pc, regs(&[]), 0, code).err(), Some(6), "{code:02x?}");
    }
}

#[test]
fn in_32_bit_mode_vex_w_is_ignored() {
    // Each page's opcode table: the `W1` forms are 64-bit-mode only, and
    // outside it `VEX.W1` is the 32-bit instruction.
    let code = vex(M38, NP, false, true, ECX, 0xf2, EAX, EDX, &[]);
    let r = regs(&[(ECX, 0xf0f0_f0f0), (EDX, 0xffff_ffff)]);
    assert_eq!(run(r, &code).0[0], 0x0f0f_0f0f);
}

#[test]
fn in_a_16_bit_code_segment_the_operation_is_still_32_bits() {
    // There is no `66` to change it (a `66` before VEX is `#UD`), and the
    // operand size is `VEX.W`'s, not the code segment's.
    let pc = pc_with(bmi());
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::CS)].ar &= !ar::DB;
    pc.cpu.set_sys(sys);
    let r = regs(&[(ECX, 0xf0f0_f0f0), (EDX, 0xffff_ffff)]);
    let (out, _) = exec(&pc, r, 0, &andn(EAX, ECX, EDX)).unwrap();
    assert_eq!(out[0], 0x0f0f_0f0f);
}

#[test]
fn in_real_mode_vex_is_undefined_and_c4_with_memory_is_les() {
    // *Intel SDM* volume 2 §2.5.1, type 13: `#UD` in real-address mode,
    // where `C4` with a memory operand is `LES`.
    let pc = Pc::with_features(Variant::X86_64, bmi());
    pc.write(6 * 4, &[0x00, 0x05, 0x00, 0x00]);
    pc.write(0x500, &[0xf4]);
    pc.write(0x600, &[0x34, 0x12, 0x78, 0x56]);
    let mut code = alloc::vec![0xc4, 0x06, 0x00, 0x06]; // les ax, [0x600]
    code.extend(andn(EAX, ECX, EDX));
    code.push(0xf4);
    pc.write(0x7c00, &code);
    pc.start_real(0, 0x7c00);
    assert!(pc.run(20) < 20);
    let r = pc.regs();
    assert_eq!((r.rax & 0xffff, r.es), (0x1234, 0x5678));
    assert_eq!((r.cs, r.rip), (0, 0x501), "#UD at the VEX instruction");
}

#[test]
fn a_memory_operand_that_faults_leaves_both_destinations() {
    // `mulx eax, ecx, [DATA]` past a short `DS` limit: `#GP(0)`, and neither
    // half of the product is written.
    let pc = pc_with(bmi());
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::DS)].limit = 0xfff;
    pc.cpu.set_sys(sys);
    let code = vex_mem(M38, PF2, false, ECX, 0xf6, EAX, DATA);
    let r = regs(&[(EAX, 0x1111), (ECX, 0x2222), (EDX, 3)]);
    assert_eq!(exec(&pc, r, 0, &code).err(), Some(13));
    let r = pc.regs();
    assert_eq!((r.rax, r.rcx), (0x1111, 0x2222));
}

// -- 64-bit mode --------------------------------------------------------------

/// `mov r64, imm64` for each of `set`, `push imm32 ; popfq` for `flags_in`,
/// then `insn` and `hlt`, in 64-bit mode on a part with `features`.
fn run64(features: Features, set: &[(u8, u64)], flags_in: u32, insn: &[u8]) -> Regs {
    let mut code = Vec::new();
    for &(r, v) in set {
        code.push(0x48 | u8::from(r >= 8));
        code.push(0xb8 + (r & 7));
        code.extend_from_slice(&v.to_le_bytes());
    }
    code.push(0x68);
    code.extend_from_slice(&(flags_in | 2).to_le_bytes());
    code.push(0x9d);
    code.extend_from_slice(insn);
    code.push(0xf4);
    run64_on(Pc::with_features(Variant::X86_64, features), 0, &code).regs()
}

/// A 64-bit register by number.
fn q(r: &Regs, n: u8) -> u64 {
    r.qword(n)
}

const R8: u8 = 8;
const R9: u8 = 9;
const R10: u8 = 10;
const R15: u8 = 15;

#[test]
fn vex_w_selects_64_bits_and_reaches_every_register() {
    // andn r9, r15, r10 — `VEX.LZ.0F38.W1 F2 /r`, with `vvvv[3]`, `R̄` and
    // `B̄` naming the upper eight.
    let r = run64(
        bmi(),
        &[(R15, 0xf0f0_f0f0_0000_ffff), (R10, u64::MAX)],
        0,
        &vex(M38, NP, false, true, R15, 0xf2, R9, R10, &[]),
    );
    assert_eq!(q(&r, R9), 0x0f0f_0f0f_ffff_0000);
    assert_eq!(r.eflags & flags::SF, 0);
    // The W0 form is 32 bits, and its write zero-extends.
    let r = run64(
        bmi(),
        &[
            (R15, 0xf0f0_f0f0_0000_ffff),
            (R10, u64::MAX),
            (R9, u64::MAX),
        ],
        0,
        &vex(M38, NP, false, false, R15, 0xf2, R9, R10, &[]),
    );
    assert_eq!(q(&r, R9), 0xffff_0000);
    assert_ne!(r.eflags & flags::SF, 0, "bit 31 is the sign");
}

#[test]
fn the_64_bit_forms() {
    let w = |map, pp, vvvv, op, reg, rm, rest: &[u8]| {
        vex(map, pp, false, true, vvvv, op, reg, rm, rest)
    };
    let x: u64 = 0x8123_4567_89ab_cdef;
    // bextr rax, rbx, rcx: start 60, length 8 — runs off the top.
    let r = run64(
        bmi(),
        &[(3, x), (1, 0x083c)],
        0,
        &w(M38, NP, 1, 0xf7, 0, 3, &[]),
    );
    assert_eq!(q(&r, 0), 0x8);
    // blsr r8, rbx ; blsmsk / blsi.
    let r = run64(bmi(), &[(3, 1 << 63)], 0, &w(M38, NP, R8, 0xf3, 1, 3, &[]));
    assert_eq!(q(&r, R8), 0);
    assert_eq!(r.eflags & (flags::ZF | flags::CF), flags::ZF);
    let r = run64(bmi(), &[(3, 0x30)], 0, &w(M38, NP, R8, 0xf3, 2, 3, &[]));
    assert_eq!(q(&r, R8), 0x1f);
    let r = run64(bmi(), &[(3, 0x30)], 0, &w(M38, NP, R8, 0xf3, 3, 3, &[]));
    assert_eq!(q(&r, R8), 0x10);
    // bzhi rax, rbx, rcx: index 63 keeps 63 bits; 64 is CF and all of them.
    let r = run64(
        bmi(),
        &[(3, u64::MAX), (1, 63)],
        0,
        &w(M38, NP, 1, 0xf5, 0, 3, &[]),
    );
    assert_eq!((q(&r, 0), r.eflags & flags::CF), (u64::MAX >> 1, 0));
    let r = run64(
        bmi(),
        &[(3, u64::MAX), (1, 64)],
        0,
        &w(M38, NP, 1, 0xf5, 0, 3, &[]),
    );
    assert_eq!((q(&r, 0), r.eflags & flags::CF), (u64::MAX, flags::CF));
    // pdep / pext rax, rcx, rbx at 64 bits.
    let mask = 0xf000_0000_0000_000f;
    let r = run64(
        bmi(),
        &[(1, 0xa5), (3, mask)],
        0,
        &w(M38, PF2, 1, 0xf5, 0, 3, &[]),
    );
    assert_eq!(q(&r, 0), 0xa000_0000_0000_0005);
    let r = run64(
        bmi(),
        &[(1, x), (3, mask)],
        0,
        &w(M38, PF3, 1, 0xf5, 0, 3, &[]),
    );
    assert_eq!(q(&r, 0), 0x8f);
    // mulx r9, r8, rbx: RDX × RBX, unsigned, 128 bits.
    let r = run64(
        bmi(),
        &[(2, u64::MAX), (3, u64::MAX)],
        ALL,
        &w(M38, PF2, R8, 0xf6, R9, 3, &[]),
    );
    assert_eq!((q(&r, R9), q(&r, R8)), (u64::MAX - 1, 1));
    assert_eq!(r.eflags & ALL, ALL, "no flag is touched");
    // rorx rax, rbx, 68: the immediate masked to six bits, 4.
    let r = run64(bmi(), &[(3, x)], 0, &w(M3A, PF2, 0, 0xf0, 0, 3, &[68]));
    assert_eq!(q(&r, 0), x.rotate_right(4));
    // shlx / shrx / sarx rax, rbx, rcx with 65: masked to six bits, 1.
    for (pp, want) in [
        (P66, x << 1),
        (PF2, x >> 1),
        (PF3, ((x as i64) >> 1) as u64),
    ] {
        let r = run64(
            bmi(),
            &[(3, x), (1, 65)],
            0,
            &w(M38, pp, 1, 0xf7, 0, 3, &[]),
        );
        assert_eq!(q(&r, 0), want, "pp {pp}");
    }
}

#[test]
fn in_64_bit_mode_the_gate_is_the_same() {
    // Without the bit, and with `VEX.L = 1`.
    faults64(
        Features::X86_64,
        6,
        &vex(M38, NP, false, true, 1, 0xf2, 0, 2, &[]),
    );
    faults64(bmi(), 6, &vex(M38, NP, true, true, 1, 0xf2, 0, 2, &[]));
}

#[test]
fn cpuid_reports_bmi1_and_bmi2_in_leaf_7() {
    // *Intel SDM* volume 2, `CPUID` leaf 07H: EBX[3] is BMI1, EBX[8] BMI2.
    let ebx = |features| cpuid64(features, 7, 0).1;
    assert_eq!(ebx(bmi()), (1 << 3) | (1 << 8));
    assert_eq!(
        ebx(Features {
            bmi2: false,
            ..bmi()
        }),
        1 << 3
    );
    assert_eq!(
        ebx(Features {
            bmi1: false,
            ..bmi()
        }),
        1 << 8
    );
    // A part with BMI and nothing else in leaf 7 raises the maximum to 7.
    assert_eq!(cpuid64(bmi(), 0, 0).0, 7);
    // They need `CPUID`, and nothing more: not AVX, not long mode.
    assert!(
        Features {
            bmi1: true,
            ..Features::I80486
        }
        .validate()
        .is_ok()
    );
    assert!(
        Features {
            bmi2: true,
            ..Features::NONE
        }
        .validate()
        .is_err()
    );
}
