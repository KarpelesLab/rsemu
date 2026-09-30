//! The 32-bit Thumb-2 (T32) encodings, described once.
//!
//! A 32-bit Thumb instruction decodes to the **same semantic
//! [`Insn`]** an A32 word does, wrapped in the same [`Decoded`] — so
//! `ADD.W r0, r1, r2, LSL #3` and A32's `ADD r0, r1, r2, LSL #3` are one value,
//! executed by one arm of the interpreter with one barrel shifter. That is the
//! whole design: T32 is a second *encoding* of an instruction set the core
//! already executes, and the places where the two genuinely differ are few and
//! named:
//!
//! - the modified immediate ([`crate::cpu::arm::t32::thumb_expand_imm`]),
//!   which A32's even-rotation immediate cannot represent and which decodes
//!   to [`Operand::Const`];
//! - `ORN`, which A32 lacks ([`DpOp::Orn`]);
//! - `LDRT` and friends with an offset and no writeback
//!   ([`Index::Unprivileged`]);
//! - `LDRD`/`STRD` with independent registers ([`Insn::LoadStoreDual`]),
//!   `LDREX` with an offset and `LDREXD` with any pair (the `imm` and `rt2`
//!   fields of [`Insn::LoadExclusive`]), and `TBB`/`TBH`
//!   ([`Insn::TableBranch`]).
//!
//! Everything else — the parallel arithmetic, the multiplies, the bitfield
//! and saturate instructions, the exclusives, the barriers, `CPS`, `SRS`,
//! `RFE`, `MRS`/`MSR` — is an A32 [`Insn`] with its fields filled from T32's
//! bit positions. The coprocessor space goes further still: T32's bits 27..0
//! *are* the A32 encoding there (DDI 0406C A6.3.18), so it is decoded by
//! [`super::isa::decode_for`] itself and reaches the same CP15 and VFP paths.
//!
//! The one T32 rule that has no A32 counterpart at all is `IT`, which is a
//! 16-bit instruction ([`super::thumb::Thumb::It`]) and whose state the
//! interpreter keeps in `CPSR` (DDI 0406C A2.5.2).
//!
//! # Disassembly
//!
//! [`Ual`] prints a Thumb instruction of either width in UAL, the way GNU
//! `objdump` prints it — `.W` where a 16-bit encoding of the same mnemonic
//! exists, the `IT` block's condition where there is one, and no `S` on a
//! 16-bit data-processing instruction inside an `IT` block, because there it
//! does not set the flags. A 16-bit instruction is printed from the same
//! [`Thumb`] value the interpreter executes; this module adds only the UAL
//! spelling, which differs from the pre-UAL one [`Thumb`]'s own `Display`
//! keeps for ARMv5 listings.
//!
//! # Sources
//!
//! ARM DDI 0406C: A6.3 ("32-bit Thumb instruction encoding") and every table
//! under it — A6.3.1 (modified immediate), A6.3.3 (plain binary immediate),
//! A6.3.4 (branches and miscellaneous control), A6.3.5 (load/store
//! multiple), A6.3.6 (dual, exclusive, table branch), A6.3.7–A6.3.10 (load
//! and store single, memory hints), A6.3.11 (shifted register), A6.3.12–
//! A6.3.15 (register data processing, parallel arithmetic, miscellaneous),
//! A6.3.16/A6.3.17 (multiplies and divide), A6.3.18 (coprocessor); A8.4.3
//! (`DecodeImmShift`); and A8.8's per-instruction pages for fields, the
//! UNPREDICTABLE cases and the assembler syntax. Encodings were cross-checked
//! by assembling with GNU `as -mthumb -march=armv7-a` and reading the words
//! back — running a tool, not reading its source. No emulator source of any
//! licence was consulted (`ROADMAP.md` §1).

use core::fmt;

use super::arch::{Arch, Extensions};
use super::isa::{
    Addressing, BarrierKind, BitfieldOp, Cond, Decoded, DpOp, ExSize, ExtendSize, ExtraOp,
    HalfMulOp, HintOp, Index, Insn, Offset, Operand, ParKind, ParOp, ParShape, RegName, RevOp,
    SatOp, Shift, ShiftType, bit, field,
};
use super::thumb::{AluOp, ImmOp, Thumb};
use crate::cpu::arm::t32::{ItState, thumb_expand_imm};

pub use crate::cpu::arm::t32::is_32bit;

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

/// Decode a 32-bit Thumb instruction for the part `arch` describes.
///
/// `hw1` is the halfword at the lower address. The returned [`Decoded`]
/// carries `hw1:hw2` as its `raw` word and [`Cond::AL`] as its condition,
/// except for the conditional branch `B<c>.W`, whose condition is in the
/// encoding — inside an `IT` block the block's condition applies instead,
/// which is the interpreter's business, not the decoder's.
///
/// Never fails: an encoding the part does not define, and the UNPREDICTABLE
/// ones that are cheap to recognise, decode to [`Insn::Undefined`] — the
/// same policy as the A32 decoder (see the ledger's
/// `unpredictable-as-undefined`). A part without Thumb-2 has no 32-bit
/// encodings at all, and gets `Undefined` for everything.
#[must_use]
pub fn decode_for(arch: &Arch, hw1: u16, hw2: u16) -> Decoded {
    let a = u32::from(hw1);
    let b = u32::from(hw2);
    let raw = (a << 16) | b;
    let mut decoded = Decoded {
        raw,
        cond: Cond::AL,
        insn: Insn::Undefined,
    };
    let ext = &arch.ext;
    if !ext.thumb2 || !is_32bit(hw1) {
        return decoded;
    }
    decoded.insn = match field(a, 12, 11) {
        0b01 => {
            if bit(a, 10) {
                coprocessor(arch, raw)
            } else if bit(a, 9) {
                dp_shifted(a, b)
            } else if bit(a, 6) {
                dual_exclusive(a, b, ext)
            } else {
                load_store_multiple(a, b)
            }
        }
        0b10 => {
            if bit(b, 15) {
                let (cond, insn) = branch_misc(a, b, ext);
                decoded.cond = cond;
                insn
            } else if bit(a, 9) {
                dp_plain_imm(a, b)
            } else {
                dp_modified_imm(a, b)
            }
        }
        _ => {
            if bit(a, 10) {
                coprocessor(arch, raw)
            } else if field(a, 10, 9) == 0 {
                if !bit(a, 4) {
                    // Stores: `hw1[8]` set is the Advanced SIMD element and
                    // structure space, which this core does not implement.
                    if bit(a, 8) {
                        Insn::Undefined
                    } else {
                        load_store_single(a, b, ext)
                    }
                } else {
                    load_store_single(a, b, ext)
                }
            } else if field(a, 10, 8) == 0b010 {
                dp_register(a, b)
            } else if field(a, 10, 7) == 0b0110 {
                multiply(a, b)
            } else if field(a, 10, 7) == 0b0111 {
                long_multiply(a, b, ext)
            } else {
                Insn::Undefined
            }
        }
    };
    decoded
}

/// A6.3.18: the coprocessor space, whose low 28 bits are the A32 encoding.
fn coprocessor(arch: &Arch, raw: u32) -> Insn {
    let op1 = field(raw, 25, 20);
    // `11xxxx` is Advanced SIMD data processing (`111U 1111`), and `00000x`
    // is UNDEFINED; neither is a coprocessor instruction.
    if op1 & 0b11_0000 == 0b11_0000 || op1 & 0b11_1110 == 0 {
        return Insn::Undefined;
    }
    // `hw1[12]` is T32's `T` bit, which puts the instruction in A32's
    // unconditional (`1111`) space — the `2` forms — exactly as the top
    // nibble of `raw` already says.
    super::isa::decode_for(arch, raw).insn
}

/// The `op` field A6.3.1 and A6.3.11 share, with the aliasing rules that
/// turn `Rd == PC` with `S` into a test and `Rn == PC` into a move.
///
/// Returns `None` for an unallocated `op` and for a write to the PC outside
/// those aliases, which T32 does not define.
fn dp_op(op: u32, s: bool, rd: u8, rn: u8) -> Option<(DpOp, u8, u8)> {
    let (base, test, unary) = match op {
        0b0000 => (DpOp::And, Some(DpOp::Tst), None),
        0b0001 => (DpOp::Bic, None, None),
        0b0010 => (DpOp::Orr, None, Some(DpOp::Mov)),
        0b0011 => (DpOp::Orn, None, Some(DpOp::Mvn)),
        0b0100 => (DpOp::Eor, Some(DpOp::Teq), None),
        0b1000 => (DpOp::Add, Some(DpOp::Cmn), None),
        0b1010 => (DpOp::Adc, None, None),
        0b1011 => (DpOp::Sbc, None, None),
        0b1101 => (DpOp::Sub, Some(DpOp::Cmp), None),
        0b1110 => (DpOp::Rsb, None, None),
        _ => return None,
    };
    if rd == 15
        && s
        && let Some(test) = test
    {
        return Some((test, 0, rn));
    }
    if rn == 15
        && let Some(unary) = unary
    {
        return Some((unary, rd, 0));
    }
    if rd == 15 || rn == 15 {
        return None;
    }
    Some((base, rd, rn))
}

/// A6.3.1: data processing with a modified immediate.
fn dp_modified_imm(a: u32, b: u32) -> Insn {
    let s = bit(a, 4);
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let imm12 = (field(a, 10, 10) << 11) | (field(b, 14, 12) << 8) | field(b, 7, 0);
    let (value, carry) = thumb_expand_imm(imm12);
    match dp_op(field(a, 8, 5), s, rd, rn) {
        Some((op, rd, rn)) => Insn::DataProc {
            op,
            s,
            rd,
            rn,
            operand: Operand::Const { value, carry },
        },
        None => Insn::Undefined,
    }
}

/// A6.3.3: data processing with a plain binary immediate.
fn dp_plain_imm(a: u32, b: u32) -> Insn {
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let imm12 = (field(a, 10, 10) << 11) | (field(b, 14, 12) << 8) | field(b, 7, 0);
    // `imm3:imm2`, the shift and `lsb` of the saturate and bitfield forms.
    let imm5 = ((field(b, 14, 12) << 2) | field(b, 7, 6)) as u8;
    let high = field(b, 4, 0) as u8;
    let plain = |op| Insn::DataProc {
        op,
        s: false,
        rd,
        rn,
        operand: Operand::Const {
            value: imm12,
            carry: None,
        },
    };
    match field(a, 8, 4) {
        // `ADDW` and `SUBW`; with `Rn == PC` they are the two `ADR` forms,
        // which the interpreter reads as `Align(PC, 4)` (A8.8.12).
        0b00000 => plain(DpOp::Add),
        0b01010 => plain(DpOp::Sub),
        0b00100 | 0b01100 => Insn::MovWide {
            top: bit(a, 7),
            rd,
            imm: ((field(a, 3, 0) << 12) | imm12) as u16,
        },
        // `SSAT`/`USAT`; with `sh == 1` and a zero shift, the halfword forms.
        0b10000 | 0b10010 | 0b11000 | 0b11010 => {
            let unsigned = bit(a, 7);
            let asr = bit(a, 5);
            if asr && imm5 == 0 {
                let bits = field(b, 3, 0) as u8;
                Insn::Saturate16 {
                    unsigned,
                    bits: if unsigned { bits } else { bits + 1 },
                    rd,
                    rn,
                }
            } else {
                Insn::Saturate {
                    unsigned,
                    bits: if unsigned { high } else { high + 1 },
                    rd,
                    rn,
                    asr,
                    amount: imm5,
                }
            }
        }
        0b10100 | 0b11100 => {
            // An extract running off bit 31 is UNPREDICTABLE (A8.8.164).
            if u32::from(imm5) + u32::from(high) > 31 {
                return Insn::Undefined;
            }
            Insn::Bitfield {
                op: if bit(a, 7) {
                    BitfieldOp::Ubfx
                } else {
                    BitfieldOp::Sbfx
                },
                rd,
                rn,
                lsb: imm5,
                width: high + 1,
            }
        }
        0b10110 => {
            // `msb < lsb` is UNPREDICTABLE (A8.8.19, A8.8.20).
            if high < imm5 {
                return Insn::Undefined;
            }
            Insn::Bitfield {
                op: if rn == 15 {
                    BitfieldOp::Bfc
                } else {
                    BitfieldOp::Bfi
                },
                rd,
                rn,
                lsb: imm5,
                width: high - imm5 + 1,
            }
        }
        _ => Insn::Undefined,
    }
}

/// A6.3.11: data processing with a constant-shifted register.
fn dp_shifted(a: u32, b: u32) -> Insn {
    let s = bit(a, 4);
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let amount = ((field(b, 14, 12) << 2) | field(b, 7, 6)) as u8;
    let op = field(a, 8, 5);
    if op == 0b0110 {
        // PKHBT/PKHTB: `hw2` is `(0) imm3 Rd imm2 tb T Rm`, where `T` must be
        // zero and `S` must be clear (A8.8.125).
        if s || bit(b, 4) {
            return Insn::Undefined;
        }
        return Insn::Pack {
            tb: bit(b, 5),
            rd,
            rn,
            rm,
            amount,
        };
    }
    // `DecodeImmShift` (A8.4.3) is exactly A32's reading of the raw `type`
    // and `imm5`, which is what `Shift::Imm` keeps.
    let shift = Shift::Imm {
        ty: ShiftType::from_bits(field(b, 5, 4)),
        amount,
    };
    match dp_op(op, s, rd, rn) {
        Some((op, rd, rn)) => Insn::DataProc {
            op,
            s,
            rd,
            rn,
            operand: Operand::Reg { rm, shift },
        },
        None => Insn::Undefined,
    }
}

/// Sign-extend the low `bits` of `value`.
const fn sign_extend(value: u32, bits: u32) -> i32 {
    let shift = 32 - bits;
    ((value << shift) as i32) >> shift
}

/// A6.3.4: branches and miscellaneous control. Returns the condition too,
/// because `B<c>.W` carries one.
fn branch_misc(a: u32, b: u32, ext: &Extensions) -> (Cond, Insn) {
    let op = field(a, 10, 4);
    let op1 = field(b, 14, 12);
    let s = field(a, 10, 10);
    let j1 = field(b, 13, 13);
    let j2 = field(b, 11, 11);
    let imm11 = field(b, 10, 0);
    // `I1 = NOT(J1 EOR S)`, `I2 = NOT(J2 EOR S)` (A8.8.18, encoding T4).
    let i1 = (j1 ^ s) ^ 1;
    let i2 = (j2 ^ s) ^ 1;
    let long = (s << 24) | (i1 << 23) | (i2 << 22) | (field(a, 9, 0) << 12);
    let al = Cond::AL;
    match op1 & 0b101 {
        0b000 => {
            if op & 0b011_1000 != 0b011_1000 {
                // B<c>.W (T3): ±1 MiB. `hw1[9:6]` is the condition; `111x`
                // is the system space below instead.
                let imm =
                    (s << 20) | (j2 << 19) | (j1 << 18) | (field(a, 5, 0) << 12) | (imm11 << 1);
                return (
                    Cond(field(a, 9, 6) as u8),
                    Insn::Branch {
                        link: false,
                        offset: sign_extend(imm, 21),
                    },
                );
            }
            (al, system(a, b, op, op1, ext))
        }
        // B.W (T4): ±16 MiB.
        0b001 => (
            al,
            Insn::Branch {
                link: false,
                offset: sign_extend(long | (imm11 << 1), 25),
            },
        ),
        // BLX (immediate, T2): the target is word-aligned, so `hw2[0]` (`H`)
        // must be zero; one is UNPREDICTABLE (A8.8.25).
        0b100 => {
            if bit(b, 0) {
                return (al, Insn::Undefined);
            }
            (
                al,
                Insn::BlxImm {
                    offset: sign_extend(long | (field(b, 10, 1) << 2), 25),
                },
            )
        }
        // BL (T1).
        _ => (
            al,
            Insn::Branch {
                link: true,
                offset: sign_extend(long | (imm11 << 1), 25),
            },
        ),
    }
}

/// The `0111xxx` corner of A6.3.4 and the two `1111111` encodings beside it.
fn system(a: u32, b: u32, op: u32, op1: u32, ext: &Extensions) -> Insn {
    let rn = field(a, 3, 0) as u8;
    match op {
        // MSR (register). `hw2[5]` set is the banked form, which needs the
        // Virtualization Extensions this core does not have.
        0b011_1000 | 0b011_1001 => {
            if bit(b, 5) {
                return Insn::Undefined;
            }
            Insn::Msr {
                spsr: bit(a, 4),
                mask: field(b, 11, 8) as u8,
                operand: Operand::Reg {
                    rm: rn,
                    shift: Shift::Imm {
                        ty: ShiftType::Lsl,
                        amount: 0,
                    },
                },
            }
        }
        // CPS, and the hints where its `imod:M` would be zero (A6.3.4's
        // "Change Processor State, and hints" table).
        0b011_1010 => {
            if field(b, 10, 8) == 0 {
                let n = field(b, 7, 0) as u8;
                return Insn::Hint {
                    op: match n {
                        0 => HintOp::Nop,
                        1 if ext.v6k => HintOp::Yield,
                        2 if ext.v6k => HintOp::Wfe,
                        3 if ext.v6k => HintOp::Wfi,
                        4 if ext.v6k => HintOp::Sev,
                        0xf0..=0xff if ext.v7 => HintOp::Dbg(n & 0xf),
                        _ => HintOp::Other(n),
                    },
                };
            }
            let imod = field(b, 10, 9);
            let change_mode = bit(b, 8);
            // imod 01 is reserved; imod 00 without M changes nothing
            // (A8.8.31 / B9.3.2, `CPS`).
            if imod == 0b01 || (imod == 0 && !change_mode) {
                return Insn::Undefined;
            }
            Insn::Cps {
                enable: match imod {
                    0b10 => Some(true),
                    0b11 => Some(false),
                    _ => None,
                },
                a: bit(b, 7),
                i: bit(b, 6),
                f: bit(b, 5),
                mode: change_mode.then_some(field(b, 4, 0) as u8),
            }
        }
        // Miscellaneous control: CLREX, DSB, DMB, ISB.
        0b011_1011 => {
            let option = field(b, 3, 0) as u8;
            match field(b, 7, 4) {
                0b0010 if ext.v6k || ext.v7 => Insn::Clrex,
                0b0100 if ext.v7 => Insn::Barrier {
                    kind: BarrierKind::Dsb,
                    option,
                },
                0b0101 if ext.v7 => Insn::Barrier {
                    kind: BarrierKind::Dmb,
                    option,
                },
                0b0110 if ext.v7 => Insn::Barrier {
                    kind: BarrierKind::Isb,
                    option,
                },
                _ => Insn::Undefined,
            }
        }
        0b011_1100 => Insn::Bxj { rm: rn },
        // SUBS PC, LR, #imm8 (B9.3.19): the Thumb exception return. It *is*
        // `SUBS pc, lr, #imm8`, which the interpreter already executes as an
        // exception return; `Rn` is fixed at LR.
        0b011_1101 => Insn::DataProc {
            op: DpOp::Sub,
            s: true,
            rd: 15,
            rn: 14,
            operand: Operand::Const {
                value: field(b, 7, 0),
                carry: None,
            },
        },
        // MRS; banked form as for MSR.
        0b011_1110 | 0b011_1111 => {
            if bit(b, 5) {
                return Insn::Undefined;
            }
            Insn::Mrs {
                rd: field(b, 11, 8) as u8,
                spsr: bit(a, 4),
            }
        }
        // SMC with `op1 == 000`; `op1 == 010` is the permanently undefined
        // `UDF.W`.
        0b111_1111 if op1 == 0 && ext.security => Insn::Smc { imm: rn },
        _ => Insn::Undefined,
    }
}

/// A6.3.5: load and store multiple, `SRS` and `RFE`.
fn load_store_multiple(a: u32, b: u32) -> Insn {
    let load = bit(a, 4);
    let writeback = bit(a, 5);
    let rn = field(a, 3, 0) as u8;
    match field(a, 8, 7) {
        // 00: the DB forms of SRS/RFE; 11: the IA forms.
        op @ (0b00 | 0b11) => {
            let up = op == 0b11;
            if load {
                Insn::Rfe {
                    before: !up,
                    up,
                    writeback,
                    rn,
                }
            } else {
                Insn::Srs {
                    before: !up,
                    up,
                    writeback,
                    mode: field(b, 4, 0) as u8,
                }
            }
        }
        op => {
            // A base of PC is UNPREDICTABLE, and so are an empty list and `SP`
            // — or, for a store, `PC` — in the list (A8.8.58, A8.8.199).
            let list = b as u16;
            if rn == 15 || list == 0 || list & (1 << 13) != 0 || (!load && list & 0x8000 != 0) {
                return Insn::Undefined;
            }
            Insn::BlockTransfer {
                load,
                before: op == 0b10,
                up: op == 0b01,
                user: false,
                writeback,
                rn,
                list,
            }
        }
    }
}

/// A6.3.6: load/store dual, load/store exclusive, table branch.
fn dual_exclusive(a: u32, b: u32, ext: &Extensions) -> Insn {
    let p = bit(a, 8);
    let u = bit(a, 7);
    let w = bit(a, 5);
    let load = bit(a, 4);
    let rn = field(a, 3, 0) as u8;
    let rt = field(b, 15, 12) as u8;
    let rt2 = field(b, 11, 8) as u8;
    let imm8x4 = (field(b, 7, 0) * 4) as u16;
    if !p && !w {
        if !u {
            // LDREX/STREX with a word-scaled offset.
            return if load {
                Insn::LoadExclusive {
                    size: ExSize::Word,
                    rt,
                    rt2: 15,
                    rn,
                    imm: imm8x4,
                }
            } else {
                Insn::StoreExclusive {
                    size: ExSize::Word,
                    rd: rt2,
                    rt,
                    rt2: 15,
                    rn,
                    imm: imm8x4,
                }
            };
        }
        let rm = field(b, 3, 0) as u8;
        let size = match field(b, 7, 4) {
            0b0100 => Some(ExSize::Byte),
            0b0101 => Some(ExSize::Half),
            0b0111 => Some(ExSize::Double),
            _ => None,
        };
        return match (load, field(b, 7, 4), size) {
            (true, 0b0000 | 0b0001, _) => Insn::TableBranch {
                half: bit(b, 4),
                rn,
                rm,
            },
            (_, _, Some(size)) if ext.v6k || ext.v7 => {
                if load {
                    Insn::LoadExclusive {
                        size,
                        rt,
                        rt2,
                        rn,
                        imm: 0,
                    }
                } else {
                    Insn::StoreExclusive {
                        size,
                        rd: rm,
                        rt,
                        rt2,
                        rn,
                        imm: 0,
                    }
                }
            }
            _ => Insn::Undefined,
        };
    }
    // LDRD/STRD. Writeback with a PC base is UNPREDICTABLE, and so is a
    // store with a PC base at all (A8.8.72, A8.8.210).
    if rn == 15 && (w || !load) {
        return Insn::Undefined;
    }
    Insn::LoadStoreDual {
        load,
        rt,
        rt2,
        rn,
        up: u,
        index: if p {
            Index::Pre { writeback: w }
        } else {
            Index::Post {
                unprivileged: false,
            }
        },
        imm: imm8x4,
    }
}

/// A6.3.7–A6.3.10: load and store a single item, and the memory hints that
/// live where a byte or halfword load of the PC would be.
fn load_store_single(a: u32, b: u32, ext: &Extensions) -> Insn {
    let load = bit(a, 4);
    let signed = bit(a, 8);
    let size = field(a, 6, 5);
    let rn = field(a, 3, 0) as u8;
    let rt = field(b, 15, 12) as u8;
    if size == 0b11 || (signed && (size == 0b10 || !load)) {
        return Insn::Undefined;
    }
    let (up, index, offset) = if rn == 15 {
        // The literal forms: `U` is `hw1[7]`, and the base is `Align(PC, 4)`.
        if !load {
            return Insn::Undefined;
        }
        (
            bit(a, 7),
            Index::Pre { writeback: false },
            Offset::Imm(field(b, 11, 0) as u16),
        )
    } else if bit(a, 7) {
        (
            true,
            Index::Pre { writeback: false },
            Offset::Imm(field(b, 11, 0) as u16),
        )
    } else if field(b, 11, 6) == 0 {
        (
            true,
            Index::Pre { writeback: false },
            Offset::Reg {
                rm: field(b, 3, 0) as u8,
                shift: Shift::Imm {
                    ty: ShiftType::Lsl,
                    amount: field(b, 5, 4) as u8,
                },
            },
        )
    } else if bit(b, 11) {
        let (p, u, w) = (bit(b, 10), bit(b, 9), bit(b, 8));
        let imm = Offset::Imm(field(b, 7, 0) as u16);
        match (p, u, w) {
            // `P == 1, U == 1, W == 0` is the unprivileged `T` form.
            (true, true, false) => (true, Index::Unprivileged, imm),
            (false, _, false) => return Insn::Undefined,
            (true, u, w) => (u, Index::Pre { writeback: w }, imm),
            (false, u, true) => (
                u,
                Index::Post {
                    unprivileged: false,
                },
                imm,
            ),
        }
    } else {
        return Insn::Undefined;
    };
    if load && rt == 15 && size != 0b10 {
        return memory_hint(size, signed, rn, up, index, offset, ext);
    }
    match (size, signed) {
        (0b10, _) | (0b00, false) => Insn::LoadStore {
            load,
            byte: size == 0,
            up,
            index,
            rn,
            rd: rt,
            offset,
        },
        _ => Insn::LoadStoreExtra {
            op: match (load, size, signed) {
                (false, _, _) => ExtraOp::Strh,
                (true, 0b01, false) => ExtraOp::Ldrh,
                (true, 0b00, true) => ExtraOp::Ldrsb,
                _ => ExtraOp::Ldrsh,
            },
            up,
            index,
            rn,
            rd: rt,
            offset,
        },
    }
}

/// A byte or halfword "load" of the PC: `PLD`, `PLDW`, `PLI`, or an
/// unallocated hint that executes as `NOP` (A6.3.7, A6.3.8).
///
/// Only the offset forms are hints; a writeback or `T` form with `Rt == PC`
/// is UNPREDICTABLE.
fn memory_hint(
    size: u32,
    signed: bool,
    rn: u8,
    up: bool,
    index: Index,
    offset: Offset,
    ext: &Extensions,
) -> Insn {
    if index != (Index::Pre { writeback: false }) {
        return Insn::Undefined;
    }
    let nop = Insn::Hint {
        op: HintOp::Other(0),
    };
    match (size, signed) {
        (0b00, false) => Insn::Pld { rn, up, offset },
        // The halfword slot is `PLDW` (`hw1[5]` is its `W`), which has no
        // literal form, and exists only with the Multiprocessing Extensions.
        (0b01, false) if rn != 15 && ext.mp => Insn::Pldw { rn, up, offset },
        (0b00, true) if ext.v7 => Insn::Pli { rn, up, offset },
        _ => nop,
    }
}

/// A6.3.12–A6.3.15: data processing with register operands.
fn dp_register(a: u32, b: u32) -> Insn {
    if field(b, 15, 12) != 0b1111 {
        return Insn::Undefined;
    }
    let rn = field(a, 3, 0) as u8;
    let rd = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let op1 = field(a, 7, 4);
    let op2 = field(b, 7, 4);
    if op1 & 0b1000 == 0 && op2 == 0 {
        // LSL/LSR/ASR/ROR (register): `MOV Rd, Rn, <type> Rm`, the A32
        // register-shifted move.
        return Insn::DataProc {
            op: DpOp::Mov,
            s: bit(a, 4),
            rd,
            rn: 0,
            operand: Operand::Reg {
                rm: rn,
                shift: Shift::Reg {
                    ty: ShiftType::from_bits(field(a, 6, 5)),
                    rs: rm,
                },
            },
        };
    }
    if op1 & 0b1000 == 0 && op2 & 0b1000 != 0 {
        let (signed, size) = match op1 {
            0b0000 => (true, ExtendSize::Half),
            0b0001 => (false, ExtendSize::Half),
            0b0010 => (true, ExtendSize::Byte16),
            0b0011 => (false, ExtendSize::Byte16),
            0b0100 => (true, ExtendSize::Byte),
            0b0101 => (false, ExtendSize::Byte),
            _ => return Insn::Undefined,
        };
        return Insn::Extend {
            signed,
            size,
            rd,
            rn: if rn == 15 { None } else { Some(rn) },
            rm,
            rotate: field(b, 5, 4) as u8,
        };
    }
    if op1 & 0b1000 != 0 && op2 & 0b1000 == 0 {
        // A6.3.13/A6.3.14: `hw2[6]` picks unsigned, `hw2[5:4]` the kind.
        let kind = match field(b, 6, 4) {
            0b000 => ParKind::S,
            0b001 => ParKind::Q,
            0b010 => ParKind::Sh,
            0b100 => ParKind::U,
            0b101 => ParKind::Uq,
            0b110 => ParKind::Uh,
            _ => return Insn::Undefined,
        };
        let shape = match field(a, 6, 4) {
            0b000 => ParShape::Add8,
            0b001 => ParShape::Add16,
            0b010 => ParShape::Asx,
            0b100 => ParShape::Sub8,
            0b101 => ParShape::Sub16,
            0b110 => ParShape::Sax,
            _ => return Insn::Undefined,
        };
        return Insn::Parallel {
            op: ParOp { kind, shape },
            rd,
            rn,
            rm,
        };
    }
    if field(a, 7, 6) == 0b10 && field(b, 7, 6) == 0b10 {
        // A6.3.15. The one-operand forms repeat `Rm` in `hw1`'s `Rn`
        // field; that it matches is not checked (a mismatch is
        // UNPREDICTABLE, and `hw2`'s copy is the one the manual reads).
        let sat = |op| Insn::Saturating { op, rd, rm, rn };
        let rev = |op| Insn::Reverse { op, rd, rm };
        return match (field(a, 5, 4), field(b, 5, 4)) {
            (0b00, 0b00) => sat(SatOp::QAdd),
            (0b00, 0b01) => sat(SatOp::QDAdd),
            (0b00, 0b10) => sat(SatOp::QSub),
            (0b00, 0b11) => sat(SatOp::QDSub),
            (0b01, 0b00) => rev(RevOp::Rev),
            (0b01, 0b01) => rev(RevOp::Rev16),
            (0b01, 0b10) => rev(RevOp::Rbit),
            (0b01, 0b11) => rev(RevOp::Revsh),
            (0b10, 0b00) => Insn::Sel { rd, rn, rm },
            (0b11, 0b00) => Insn::Clz { rd, rm },
            _ => Insn::Undefined,
        };
    }
    Insn::Undefined
}

/// `Ra`-style fields where `0b1111` means the non-accumulating form.
const fn optional(r: u8) -> Option<u8> {
    if r == 15 { None } else { Some(r) }
}

/// A6.3.16: multiply, multiply-accumulate, absolute difference.
///
/// A32's multiply variants name their operands `Rm` and `Rs` where T32 says
/// `Rn` and `Rm`; the mapping below is positional (first operand, second
/// operand), so the disassembly and the arithmetic come out the same.
fn multiply(a: u32, b: u32) -> Insn {
    if field(b, 7, 6) != 0 {
        return Insn::Undefined;
    }
    let rn = field(a, 3, 0) as u8;
    let ra = field(b, 15, 12) as u8;
    let rd = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let op2 = field(b, 5, 4);
    let half = |on| {
        if on {
            super::isa::Half::Top
        } else {
            super::isa::Half::Bottom
        }
    };
    match (field(a, 6, 4), op2) {
        (0b000, 0b00) => Insn::Mul {
            accumulate: ra != 15,
            s: false,
            rd,
            rn: ra,
            rm: rn,
            rs: rm,
        },
        (0b000, 0b01) => Insn::Mls { rd, rn, rm, ra },
        (0b001, _) => Insn::HalfMul {
            op: if ra == 15 {
                HalfMulOp::Smul
            } else {
                HalfMulOp::Smla
            },
            rd,
            rn: ra,
            rm: rn,
            rs: rm,
            x: half(bit(b, 5)),
            y: half(bit(b, 4)),
        },
        (0b010 | 0b100, 0b00 | 0b01) => Insn::DualMul {
            sub: field(a, 6, 4) == 0b100,
            exchange: bit(b, 4),
            rd,
            rn,
            rm,
            ra: optional(ra),
        },
        (0b011, 0b00 | 0b01) => Insn::HalfMul {
            op: if ra == 15 {
                HalfMulOp::Smulw
            } else {
                HalfMulOp::Smlaw
            },
            rd,
            rn: ra,
            rm: rn,
            rs: rm,
            x: super::isa::Half::Bottom,
            y: half(bit(b, 4)),
        },
        (0b101, 0b00 | 0b01) => Insn::MulHigh {
            sub: false,
            round: bit(b, 4),
            rd,
            rn,
            rm,
            ra: optional(ra),
        },
        (0b110, 0b00 | 0b01) => Insn::MulHigh {
            sub: true,
            round: bit(b, 4),
            rd,
            rn,
            rm,
            ra: Some(ra),
        },
        (0b111, 0b00) => Insn::Usad8 {
            rd,
            rn,
            rm,
            ra: optional(ra),
        },
        _ => Insn::Undefined,
    }
}

/// A6.3.17: long multiply, long multiply-accumulate, divide.
fn long_multiply(a: u32, b: u32, ext: &Extensions) -> Insn {
    let rn = field(a, 3, 0) as u8;
    let rdlo = field(b, 15, 12) as u8;
    let rdhi = field(b, 11, 8) as u8;
    let rm = field(b, 3, 0) as u8;
    let long = |signed, accumulate| Insn::MulLong {
        signed,
        accumulate,
        s: false,
        rdhi,
        rdlo,
        rm: rn,
        rs: rm,
    };
    let op1 = field(a, 6, 4);
    match (op1, field(b, 7, 4)) {
        (0b000, 0b0000) => long(true, false),
        (0b010, 0b0000) => long(false, false),
        (0b100, 0b0000) => long(true, true),
        (0b110, 0b0000) => long(false, true),
        // SDIV/UDIV: `hw2[15:12]` is `(1)(1)(1)(1)`.
        (0b001 | 0b011, 0b1111) if ext.idiv_thumb => Insn::Divide {
            signed: op1 == 0b001,
            rd: rdhi,
            rn,
            rm,
        },
        (0b100, 0b1000..=0b1011) => Insn::HalfMul {
            op: HalfMulOp::Smlal,
            rd: rdhi,
            rn: rdlo,
            rm: rn,
            rs: rm,
            x: if bit(b, 5) {
                super::isa::Half::Top
            } else {
                super::isa::Half::Bottom
            },
            y: if bit(b, 4) {
                super::isa::Half::Top
            } else {
                super::isa::Half::Bottom
            },
        },
        (0b100 | 0b101, 0b1100 | 0b1101) => Insn::DualMulLong {
            sub: op1 == 0b101,
            exchange: bit(b, 4),
            rdhi,
            rdlo,
            rn,
            rm,
        },
        (0b110, 0b0110) => Insn::Umaal { rdhi, rdlo, rn, rm },
        _ => Insn::Undefined,
    }
}

// ---------------------------------------------------------------------------
// One instruction of either width
// ---------------------------------------------------------------------------

/// One Thumb instruction on a part with Thumb-2: a 16-bit [`Thumb`], or a
/// 32-bit T32 instruction decoded to [`Decoded`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum T32 {
    /// A 16-bit encoding.
    Narrow {
        /// The halfword.
        raw: u16,
        /// What it decodes to.
        insn: Thumb,
    },
    /// A 32-bit encoding; `raw` is `hw1:hw2`.
    Wide(Decoded),
}

impl T32 {
    /// Decode one instruction from its first halfword and, when
    /// [`is_32bit`] says it needs one, its second.
    #[must_use]
    pub fn decode_for(arch: &Arch, hw1: u16, hw2: u16) -> T32 {
        if arch.ext.thumb2 && is_32bit(hw1) {
            T32::Wide(decode_for(arch, hw1, hw2))
        } else {
            T32::Narrow {
                raw: hw1,
                insn: super::thumb::decode_for(arch, hw1),
            }
        }
    }

    /// Two bytes or four.
    #[must_use]
    pub const fn byte_len(&self) -> u32 {
        match self {
            T32::Narrow { .. } => 2,
            T32::Wide(_) => 4,
        }
    }

    /// The absolute target of a branch whose target is a constant, given
    /// where the instruction lives. The PC reads as the instruction plus
    /// four in Thumb state; `BLX` to ARM state aligns it first (A8.8.25).
    #[must_use]
    pub const fn branch_target(&self, addr: u32) -> Option<u32> {
        let pc = addr.wrapping_add(4);
        match *self {
            T32::Narrow { insn, .. } => match insn {
                Thumb::Branch { offset } | Thumb::BranchCond { offset, .. } => {
                    Some(pc.wrapping_add(offset as u32))
                }
                Thumb::CompareBranch { offset, .. } => Some(pc.wrapping_add(offset as u32)),
                _ => None,
            },
            T32::Wide(d) => match d.insn {
                Insn::Branch { offset, .. } => Some(pc.wrapping_add(offset as u32)),
                Insn::BlxImm { offset } => Some((pc & !3).wrapping_add(offset as u32)),
                _ => None,
            },
        }
    }

    /// Whether this is an `IT` instruction, and the state it leaves.
    #[must_use]
    pub const fn it_state(&self) -> Option<ItState> {
        match *self {
            T32::Narrow {
                insn: Thumb::It { firstcond, mask },
                ..
            } => Some(ItState::from_it(firstcond, mask)),
            _ => None,
        }
    }

    /// Print as UAL. `it` is the `IT` block condition this instruction
    /// executes under, if it is inside one; `addr` resolves branch targets.
    #[must_use]
    pub const fn ual(&self, addr: Option<u32>, it: Option<Cond>) -> Ual {
        Ual {
            insn: *self,
            addr,
            it,
        }
    }
}

/// A Thumb instruction printed in UAL; see the module docs.
#[derive(Debug, Clone, Copy)]
pub struct Ual {
    insn: T32,
    addr: Option<u32>,
    it: Option<Cond>,
}

impl fmt::Display for Ual {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.insn {
            T32::Narrow { raw, insn } => fmt_narrow(raw, insn, self.target(), self.it, f),
            T32::Wide(d) => fmt_wide(d, self.target(), self.it, f),
        }
    }
}

impl Ual {
    fn target(&self) -> Option<u32> {
        self.addr.and_then(|a| self.insn.branch_target(a))
    }
}

/// A condition suffix: the `IT` block's if there is one, and nothing for
/// `AL`.
fn suffix(c: Option<Cond>) -> &'static str {
    match c {
        Some(c) => c.name(),
        None => "",
    }
}

/// A branch operand: the absolute target when the address is known, the
/// signed offset otherwise.
struct Target(Option<u32>, i32);

impl fmt::Display for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(t) => write!(f, "0x{t:08x}"),
            None => write!(f, "{:+}", self.1),
        }
    }
}

/// A register list with every register named, as GNU `objdump` writes a
/// Thumb one: `{r4, r5, lr}`.
struct List(u16);

impl fmt::Display for List {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("{")?;
        let mut first = true;
        for i in 0..16u8 {
            if self.0 & (1 << i) != 0 {
                if !first {
                    f.write_str(", ")?;
                }
                first = false;
                write!(f, "{}", RegName(i))?;
            }
        }
        f.write_str("}")
    }
}

/// A 16-bit instruction in UAL.
#[allow(clippy::too_many_lines)] // One arm per format.
fn fmt_narrow(
    raw: u16,
    insn: Thumb,
    target: Option<u32>,
    it: Option<Cond>,
    f: &mut fmt::Formatter<'_>,
) -> fmt::Result {
    let r = RegName;
    let c = suffix(it);
    // Outside an IT block the flag-setting forms are the `S` ones; inside,
    // the same encodings leave the flags alone and drop the `S`.
    let s = if it.is_some() { "" } else { "S" };
    match insn {
        Thumb::ShiftImm { ty, rd, rm, imm } => {
            if ty == ShiftType::Lsl && imm == 0 {
                write!(f, "MOV{s}{c} {}, {}", r(rd), r(rm))
            } else {
                let n = if imm == 0 { 32 } else { u32::from(imm) };
                write!(f, "{ty}{s}{c} {}, {}, #{n}", r(rd), r(rm))
            }
        }
        Thumb::AddSub {
            sub,
            rd,
            rn,
            operand,
        } => {
            let op = if sub { "SUB" } else { "ADD" };
            write!(f, "{op}{s}{c} {}, {}, {operand}", r(rd), r(rn))
        }
        Thumb::AluImm { op, rd, imm } => {
            let flag = if op == ImmOp::Cmp { "" } else { s };
            write!(f, "{}{flag}{c} {}, #{imm}", op.mnemonic(), r(rd))
        }
        Thumb::Alu { op, rd, rm } => match op {
            AluOp::Tst | AluOp::Cmp | AluOp::Cmn => {
                write!(f, "{}{c} {}, {}", op.mnemonic(), r(rd), r(rm))
            }
            AluOp::Neg => write!(f, "NEG{s}{c} {}, {}", r(rd), r(rm)),
            // `MULS <Rdm>, <Rn>, <Rdm>`, written in its two-operand form.
            AluOp::Mul => write!(f, "MUL{s}{c} {}, {}", r(rd), r(rm)),
            _ => write!(f, "{}{s}{c} {}, {}", op.mnemonic(), r(rd), r(rm)),
        },
        Thumb::HiReg { op, rd, rm } => {
            write!(f, "{}{c} {}, {}", op.mnemonic(), r(rd), r(rm))
        }
        Thumb::BranchExchange { link, rm } => {
            let name = if link { "BLX" } else { "BX" };
            write!(f, "{name}{c} {}", r(rm))
        }
        Thumb::LoadLiteral { rd, imm } => {
            write!(f, "LDR{c} {}, [pc, #{}]", r(rd), u32::from(imm) * 4)
        }
        Thumb::MemReg { op, rd, rn, rm } => {
            write!(f, "{}{c} {}, [{}, {}]", op.mnemonic(), r(rd), r(rn), r(rm))
        }
        Thumb::MemImm {
            load,
            size,
            rd,
            rn,
            imm,
        } => {
            let name = if load { "LDR" } else { "STR" };
            write!(
                f,
                "{name}{}{c} {}, [{}, #{}]",
                size.suffix(),
                r(rd),
                r(rn),
                u32::from(imm) * size.bytes()
            )
        }
        Thumb::MemStack { load, rd, imm } => {
            let name = if load { "LDR" } else { "STR" };
            write!(f, "{name}{c} {}, [sp, #{}]", r(rd), u32::from(imm) * 4)
        }
        Thumb::AddPcSp { sp, rd, imm } => {
            let base = if sp { "sp" } else { "pc" };
            write!(f, "ADD{c} {}, {base}, #{}", r(rd), u32::from(imm) * 4)
        }
        Thumb::AdjustStack { sub, imm } => {
            let name = if sub { "SUB" } else { "ADD" };
            write!(f, "{name}{c} sp, #{}", u32::from(imm) * 4)
        }
        Thumb::PushPop { load, extra, list } => {
            let name = if load { "POP" } else { "PUSH" };
            let mut full = u16::from(list);
            if extra {
                full |= if load { 0x8000 } else { 0x4000 };
            }
            write!(f, "{name}{c} {}", List(full))
        }
        Thumb::BlockTransfer { load, rn, list } => {
            let name = if load { "LDMIA" } else { "STMIA" };
            // A load that includes its base does not write it back.
            let w = if load && list & (1 << rn) != 0 {
                ""
            } else {
                "!"
            };
            write!(f, "{name}{c} {}{w}, {}", r(rn), List(u16::from(list)))
        }
        Thumb::BranchCond { cond, offset } => {
            write!(f, "B{cond}.N {}", Target(target, offset))
        }
        Thumb::Swi { imm } => write!(f, "SVC{c} #{imm}"),
        Thumb::Bkpt { imm } => write!(f, "BKPT #{imm}"),
        Thumb::Branch { offset } => write!(f, "B{c}.N {}", Target(target, offset)),
        Thumb::CompareBranch {
            nonzero,
            rn,
            offset,
        } => {
            let name = if nonzero { "CBNZ" } else { "CBZ" };
            write!(f, "{name} {}, {}", r(rn), Target(target, i32::from(offset)))
        }
        Thumb::It { .. } => write!(f, "{insn}"),
        Thumb::Common(insn) => write!(
            f,
            "{}",
            Decoded {
                raw: u32::from(raw),
                cond: it.unwrap_or(Cond::AL),
                insn,
            }
        ),
        Thumb::Undefined if raw >> 8 == 0xde => write!(f, "UDF #{}", raw & 0xff),
        // The ARMv5T `BL`/`BLX` halves, which a Thumb-2 part never decodes
        // alone, and anything else unallocated.
        _ => write!(f, "UNDEFINED ; 0x{raw:04x}"),
    }
}

/// The `MSR` field letters as UAL orders them: `f`, `s`, `x`, `c`.
fn msr_fields(mask: u8) -> [&'static str; 4] {
    [
        if mask & 0b1000 != 0 { "f" } else { "" },
        if mask & 0b0100 != 0 { "s" } else { "" },
        if mask & 0b0010 != 0 { "x" } else { "" },
        if mask & 0b0001 != 0 { "c" } else { "" },
    ]
}

/// Whether a data-processing mnemonic also has a 16-bit encoding, which is
/// when UAL spells the 32-bit one with `.W`. `RSB`'s 16-bit form is spelled
/// `NEG`, so `RSB` itself has none.
const fn has_narrow(op: DpOp) -> bool {
    !matches!(op, DpOp::Teq | DpOp::Orn | DpOp::Rsc | DpOp::Rsb)
}

/// A 32-bit instruction in UAL.
#[allow(clippy::too_many_lines)] // One arm per encoding group.
fn fmt_wide(
    d: Decoded,
    target: Option<u32>,
    it: Option<Cond>,
    f: &mut fmt::Formatter<'_>,
) -> fmt::Result {
    let r = RegName;
    // Inside an IT block the block's condition; otherwise the encoding's,
    // which is `AL` for everything but `B<c>.W`.
    let cond = it.unwrap_or(d.cond);
    let c = cond.name();
    let hw1 = d.raw >> 16;
    #[cfg(feature = "cpu-arm-aprofile-vfp")]
    if d.raw >> 28 != 0xf
        && let Some(v) = super::vfpisa::decode(d.raw)
    {
        return write!(f, "{}", v.display(cond));
    }
    match d.insn {
        Insn::DataProc {
            op,
            s,
            rd,
            rn,
            operand,
        } => {
            if rd == 15 && rn == 14 && op == DpOp::Sub {
                return write!(f, "SUBS{c} pc, lr, {operand}");
            }
            // `ADDW`/`SUBW` and the `ADR` forms: the plain-immediate group.
            if hw1 >> 11 == 0b11110 && bit(hw1, 9) {
                let name = if op == DpOp::Add { "ADDW" } else { "SUBW" };
                return write!(f, "{name}{c} {}, {}, {operand}", r(rd), r(rn));
            }
            let w = if has_narrow(op) { ".W" } else { "" };
            if !op.writes_result() {
                return write!(f, "{op}{c}{w} {}, {operand}", r(rn));
            }
            let flag = if s { "S" } else { "" };
            if let Operand::Reg {
                rm,
                shift: Shift::Reg { ty, rs },
            } = operand
            {
                return write!(f, "{ty}{flag}{c}.W {}, {}, {}", r(rd), r(rm), r(rs));
            }
            if op.reads_rn() {
                write!(f, "{op}{flag}{c}{w} {}, {}, {operand}", r(rd), r(rn))
            } else {
                write!(f, "{op}{flag}{c}{w} {}, {operand}", r(rd))
            }
        }
        Insn::LoadStore {
            load,
            byte,
            up,
            index,
            rn,
            rd,
            offset,
        } => {
            let name = if load { "LDR" } else { "STR" };
            let b = if byte { "B" } else { "" };
            let (t, w) = if index.is_unprivileged() {
                ("T", "")
            } else {
                ("", ".W")
            };
            write!(
                f,
                "{name}{b}{t}{c}{w} {}, {}",
                r(rd),
                Addressing {
                    rn,
                    up,
                    index,
                    offset
                }
            )
        }
        Insn::LoadStoreExtra {
            op,
            up,
            index,
            rn,
            rd,
            offset,
        } => {
            let (t, w) = if index.is_unprivileged() {
                ("T", "")
            } else {
                ("", ".W")
            };
            write!(
                f,
                "{}{t}{c}{w} {}, {}",
                op.mnemonic(),
                r(rd),
                Addressing {
                    rn,
                    up,
                    index,
                    offset
                }
            )
        }
        Insn::BlockTransfer {
            load,
            before,
            writeback,
            rn,
            list,
            ..
        } => {
            let name = if load { "LDM" } else { "STM" };
            let (mode, w) = if before { ("DB", "") } else { ("IA", ".W") };
            let bang = if writeback { "!" } else { "" };
            write!(f, "{name}{mode}{c}{w} {}{bang}, {}", r(rn), List(list))
        }
        Insn::Branch { link, offset } => {
            if link {
                write!(f, "BL{c} {}", Target(target, offset))
            } else {
                write!(f, "B{c}.W {}", Target(target, offset))
            }
        }
        Insn::BlxImm { offset } => write!(f, "BLX{c} {}", Target(target, offset)),
        Insn::Msr {
            spsr,
            mask,
            operand,
        } => {
            let dst = if spsr { "SPSR" } else { "CPSR" };
            let [a, b, cc, e] = msr_fields(mask);
            write!(f, "MSR{c} {dst}_{a}{b}{cc}{e}, {operand}")
        }
        Insn::Mul {
            accumulate: false,
            rd,
            rm,
            rs,
            ..
        } => write!(f, "MUL{c}.W {}, {}, {}", r(rd), r(rm), r(rs)),
        Insn::Hint { op } => match op {
            HintOp::Nop | HintOp::Other(_) => write!(f, "NOP{c}.W"),
            HintOp::Yield => write!(f, "YIELD{c}.W"),
            HintOp::Wfe => write!(f, "WFE{c}.W"),
            HintOp::Wfi => write!(f, "WFI{c}.W"),
            HintOp::Sev => write!(f, "SEV{c}.W"),
            HintOp::Dbg(n) => write!(f, "DBG{c} #{n}"),
        },
        Insn::Cps {
            enable: Some(enable),
            a,
            i,
            f: fiq,
            mode: None,
        } => write!(
            f,
            "CPSI{}.W {}{}{}",
            if enable { "E" } else { "D" },
            if a { "a" } else { "" },
            if i { "i" } else { "" },
            if fiq { "f" } else { "" }
        ),
        Insn::Extend {
            signed,
            size,
            rd,
            rn: None,
            rm,
            rotate,
        } if size != ExtendSize::Byte16 => {
            let s = if signed { "S" } else { "U" };
            write!(f, "{s}XT{}{c}.W {}, {}", size.suffix(), r(rd), r(rm))?;
            if rotate != 0 {
                write!(f, ", ROR #{}", u32::from(rotate) * 8)?;
            }
            Ok(())
        }
        Insn::Reverse { op, rd, rm } if op != RevOp::Rbit => {
            write!(f, "{}{c}.W {}, {}", op.mnemonic(), r(rd), r(rm))
        }
        Insn::Undefined if d.raw & 0xfff0_f000 == 0xf7f0_a000 => {
            write!(f, "UDF.W #{}", ((d.raw >> 4) & 0xf000) | (d.raw & 0xfff))
        }
        Insn::Undefined => write!(f, "UNDEFINED ; 0x{:04x} 0x{:04x}", hw1, d.raw & 0xffff),
        insn => write!(
            f,
            "{}",
            Decoded {
                raw: d.raw,
                cond,
                insn
            }
        ),
    }
}
