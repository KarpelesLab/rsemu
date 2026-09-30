//! The A32 encodings ARMv6, ARMv6K, ARMv6T2 and ARMv7 added, described once.
//!
//! This is the second half of [`super::isa`], split out only for length: the
//! variants live in the same [`Insn`] enum, [`super::isa::decode_for`] routes
//! into the functions here, and the disassembly below is the same `Display`
//! impl continued. There is still one description of each encoding.
//!
//! # Gating
//!
//! Every function here takes the part's [`Extensions`] and answers for *that
//! part*: an encoding the part lacks decodes exactly as it did on ARMv5TE —
//! usually [`Insn::Undefined`], which is how a guest probing for a feature
//! sees it absent. Three places are not UNDEFINED on the older part and keep
//! their older meaning instead, which is why decode rather than the
//! interpreter owns the gate:
//!
//! - the hint space (`NOP`, `WFI`, …) is `MSR CPSR_, #imm` with an empty mask
//!   before ARMv6K — an instruction that writes nothing;
//! - `0000 01xx … 1001` (`UMAAL`, `MLS`) is ignored bit 22 of `MUL` before
//!   ARMv6;
//! - the halfword and signed-byte `P == 0, W == 1` forms are post-indexed
//!   accesses before v6T2 made them `LDRHT` and friends.
//!
//! # Sources
//!
//! ARM DDI 0406C: A5.2 ("Data-processing and miscellaneous instructions")
//! and its sub-tables A5.2.5 (multiply), A5.2.8/A5.2.9 (extra load/store,
//! including the unprivileged forms), A5.2.10 (synchronization primitives),
//! A5.2.11 (`MSR` immediate and hints), A5.2.12 (miscellaneous); A5.4 (media
//! instructions) with A5.4.1–A5.4.4; A5.7 and A5.7.1 (unconditional
//! instructions, memory hints and barriers); and A8.8's per-instruction pages
//! for the assembler syntax. Encodings were cross-checked by assembling with
//! GNU `as` and comparing words (running a tool, not reading its source). No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

use core::fmt;

use super::arch::Extensions;
use super::isa::{Addressing, Decoded, Index, Insn, RegName, bit, field, mode2_offset};
use super::media::{ExtendSize, ParKind, ParOp, ParShape, RevOp};

/// The access size of a load/store exclusive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExSize {
    /// `LDREX`/`STREX` (ARMv6).
    Word,
    /// `LDREXB`/`STREXB` (ARMv6K).
    Byte,
    /// `LDREXH`/`STREXH` (ARMv6K).
    Half,
    /// `LDREXD`/`STREXD` (ARMv6K): a register pair.
    Double,
}

impl ExSize {
    /// Bytes transferred, which is also the required alignment.
    #[must_use]
    pub const fn bytes(self) -> u32 {
        match self {
            ExSize::Byte => 1,
            ExSize::Half => 2,
            ExSize::Word => 4,
            ExSize::Double => 8,
        }
    }

    /// The mnemonic suffix.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            ExSize::Word => "",
            ExSize::Byte => "B",
            ExSize::Half => "H",
            ExSize::Double => "D",
        }
    }
}

/// One of the hints in the `MSR`-immediate space (DDI 0406C A5.2.11).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HintOp {
    /// No operation.
    Nop,
    /// A hint that this thread could yield to another.
    Yield,
    /// Wait for an event, or an interrupt.
    Wfe,
    /// Wait for an interrupt.
    Wfi,
    /// Signal an event to every core.
    Sev,
    /// `DBG #option` (ARMv7): a hint to the debug system.
    Dbg(u8),
    /// An unallocated hint number, which executes as `NOP`.
    Other(u8),
}

/// The three barriers (ARMv7; DDI 0406C A8.8, the `DMB`, `DSB` and `ISB` pages).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BarrierKind {
    /// Data memory barrier.
    Dmb,
    /// Data synchronization barrier.
    Dsb,
    /// Instruction synchronization barrier.
    Isb,
}

/// The four bitfield operations (v6T2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BitfieldOp {
    /// Clear bits `lsb..lsb + width` of `Rd`.
    Bfc,
    /// Insert the low `width` bits of `Rn` at `lsb` of `Rd`.
    Bfi,
    /// Extract `width` bits of `Rn` from `lsb`, sign-extended.
    Sbfx,
    /// Extract `width` bits of `Rn` from `lsb`, zero-extended.
    Ubfx,
}

impl BitfieldOp {
    const fn mnemonic(self) -> &'static str {
        match self {
            BitfieldOp::Bfc => "BFC",
            BitfieldOp::Bfi => "BFI",
            BitfieldOp::Sbfx => "SBFX",
            BitfieldOp::Ubfx => "UBFX",
        }
    }
}

const fn reg(raw: u32, hi: u32) -> u8 {
    field(raw, hi, hi - 3) as u8
}

/// `Rn`/`Ra` fields where `0b1111` means "no register" (the non-accumulating
/// form of the same encoding).
const fn optional_reg(raw: u32, hi: u32) -> Option<u8> {
    match reg(raw, hi) {
        15 => None,
        r => Some(r),
    }
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

/// `cond 011x xxxx xxxx xxxx xxxx xxx1 xxxx`: the media space (A5.4).
pub(super) fn decode_media(raw: u32, ext: &Extensions) -> Insn {
    if !ext.v6 {
        return Insn::Undefined;
    }
    match field(raw, 24, 23) {
        0b00 => decode_parallel(raw),
        0b01 => decode_pack_sat_rev(raw, ext),
        0b10 => decode_signed_multiply(raw, ext),
        _ => decode_media_misc(raw, ext),
    }
}

/// A5.4.1 and A5.4.2: the parallel add/subtract family.
fn decode_parallel(raw: u32) -> Insn {
    let unsigned = bit(raw, 22);
    let kind = match (unsigned, field(raw, 21, 20)) {
        (false, 0b01) => ParKind::S,
        (false, 0b10) => ParKind::Q,
        (false, 0b11) => ParKind::Sh,
        (true, 0b01) => ParKind::U,
        (true, 0b10) => ParKind::Uq,
        (true, 0b11) => ParKind::Uh,
        _ => return Insn::Undefined,
    };
    let shape = match field(raw, 7, 5) {
        0b000 => ParShape::Add16,
        0b001 => ParShape::Asx,
        0b010 => ParShape::Sax,
        0b011 => ParShape::Sub16,
        0b100 => ParShape::Add8,
        0b111 => ParShape::Sub8,
        _ => return Insn::Undefined,
    };
    Insn::Parallel {
        op: ParOp { kind, shape },
        rd: reg(raw, 15),
        rn: reg(raw, 19),
        rm: reg(raw, 3),
    }
}

/// A5.4.3: packing, unpacking, saturation and reversal.
fn decode_pack_sat_rev(raw: u32, ext: &Extensions) -> Insn {
    let op1 = field(raw, 22, 20);
    let op2 = field(raw, 7, 5);
    let rd = reg(raw, 15);
    let rm = reg(raw, 3);
    let extend = |signed, size| Insn::Extend {
        signed,
        size,
        rd,
        rn: optional_reg(raw, 19),
        rm,
        rotate: field(raw, 11, 10) as u8,
    };
    match (op1, op2) {
        (0b000, _) if op2 & 1 == 0 => Insn::Pack {
            tb: bit(raw, 6),
            rd,
            rn: reg(raw, 19),
            rm,
            amount: field(raw, 11, 7) as u8,
        },
        (0b000, 0b011) => extend(true, ExtendSize::Byte16),
        (0b000, 0b101) => Insn::Sel {
            rd,
            rn: reg(raw, 19),
            rm,
        },
        (0b010 | 0b011, _) if op2 & 1 == 0 => Insn::Saturate {
            unsigned: false,
            bits: field(raw, 20, 16) as u8 + 1,
            rd,
            rn: rm,
            asr: bit(raw, 6),
            amount: field(raw, 11, 7) as u8,
        },
        (0b110 | 0b111, _) if op2 & 1 == 0 => Insn::Saturate {
            unsigned: true,
            bits: field(raw, 20, 16) as u8,
            rd,
            rn: rm,
            asr: bit(raw, 6),
            amount: field(raw, 11, 7) as u8,
        },
        (0b010, 0b001) => Insn::Saturate16 {
            unsigned: false,
            bits: field(raw, 19, 16) as u8 + 1,
            rd,
            rn: rm,
        },
        (0b110, 0b001) => Insn::Saturate16 {
            unsigned: true,
            bits: field(raw, 19, 16) as u8,
            rd,
            rn: rm,
        },
        (0b010, 0b011) => extend(true, ExtendSize::Byte),
        (0b011, 0b011) => extend(true, ExtendSize::Half),
        (0b100, 0b011) => extend(false, ExtendSize::Byte16),
        (0b110, 0b011) => extend(false, ExtendSize::Byte),
        (0b111, 0b011) => extend(false, ExtendSize::Half),
        (0b011, 0b001) => Insn::Reverse {
            op: RevOp::Rev,
            rd,
            rm,
        },
        (0b011, 0b101) => Insn::Reverse {
            op: RevOp::Rev16,
            rd,
            rm,
        },
        (0b111, 0b101) => Insn::Reverse {
            op: RevOp::Revsh,
            rd,
            rm,
        },
        (0b111, 0b001) if ext.thumb2 => Insn::Reverse {
            op: RevOp::Rbit,
            rd,
            rm,
        },
        _ => Insn::Undefined,
    }
}

/// A5.4.4: the signed multiplies, and the hardware divide beside them.
fn decode_signed_multiply(raw: u32, ext: &Extensions) -> Insn {
    let op2 = field(raw, 7, 5);
    // In this group `Rd` is at 19:16 and `Ra` at 15:12 — the reverse of the
    // data-processing layout, as for `MUL`.
    let rd = reg(raw, 19);
    let rn = reg(raw, 3);
    let rm = reg(raw, 11);
    let exchange = bit(raw, 5);
    match (field(raw, 22, 20), op2 >> 1) {
        (0b000, 0b00 | 0b01) => Insn::DualMul {
            sub: op2 >> 1 == 0b01,
            exchange,
            rd,
            rn,
            rm,
            ra: optional_reg(raw, 15),
        },
        (0b100, 0b00 | 0b01) => Insn::DualMulLong {
            sub: op2 >> 1 == 0b01,
            exchange,
            rdhi: rd,
            rdlo: reg(raw, 15),
            rn,
            rm,
        },
        (0b101, 0b00) => Insn::MulHigh {
            sub: false,
            round: bit(raw, 5),
            rd,
            rn,
            rm,
            ra: optional_reg(raw, 15),
        },
        (0b101, 0b11) => Insn::MulHigh {
            sub: true,
            round: bit(raw, 5),
            rd,
            rn,
            rm,
            ra: Some(reg(raw, 15)),
        },
        (0b001 | 0b011, 0b00) if op2 == 0 && ext.idiv_arm => Insn::Divide {
            signed: field(raw, 22, 20) == 0b001,
            rd,
            rn,
            rm,
        },
        _ => Insn::Undefined,
    }
}

/// The rest of A5.4: `USAD8`, the bitfield instructions and `UDF`.
fn decode_media_misc(raw: u32, ext: &Extensions) -> Insn {
    let op1 = field(raw, 24, 20);
    let op2 = field(raw, 7, 5);
    if op1 == 0b11000 && op2 == 0 {
        return Insn::Usad8 {
            rd: reg(raw, 19),
            rn: reg(raw, 3),
            rm: reg(raw, 11),
            ra: optional_reg(raw, 15),
        };
    }
    if !ext.thumb2 {
        return Insn::Undefined;
    }
    let rd = reg(raw, 15);
    let rn = reg(raw, 3);
    let lsb = field(raw, 11, 7) as u8;
    let high = field(raw, 20, 16) as u8;
    match (op1 >> 1, op2 & 0b011) {
        (0b1101 | 0b1111, 0b010) => {
            // `widthm1` in 20:16. An extract running off the top of the
            // register is UNPREDICTABLE (A8.8, `SBFX`); refusing it is the one
            // reading that cannot silently compute something.
            if u32::from(lsb) + u32::from(high) > 31 {
                return Insn::Undefined;
            }
            Insn::Bitfield {
                op: if op1 >> 1 == 0b1101 {
                    BitfieldOp::Sbfx
                } else {
                    BitfieldOp::Ubfx
                },
                rd,
                rn,
                lsb,
                width: high + 1,
            }
        }
        (0b1110, 0b000) => {
            // `msb` in 20:16; `msb < lsb` is UNPREDICTABLE (A8.8, `BFC` and `BFI`).
            if high < lsb {
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
                lsb,
                width: high - lsb + 1,
            }
        }
        // `UDF` (11111 / 111) and everything unallocated.
        _ => Insn::Undefined,
    }
}

/// A5.2.5: the multiplies with bit 22 set, `UMAAL` and `MLS`.
///
/// Before ARMv6 the decoder ignored that bit and produced a `MUL`/`MLA`;
/// that reading is kept for those parts (see the module docs).
pub(super) fn decode_multiply_v6(raw: u32, ext: &Extensions) -> Option<Insn> {
    if !ext.v6 || !bit(raw, 22) {
        return None;
    }
    Some(match field(raw, 22, 20) {
        0b100 => Insn::Umaal {
            rdhi: reg(raw, 19),
            rdlo: reg(raw, 15),
            rn: reg(raw, 3),
            rm: reg(raw, 11),
        },
        0b110 if ext.thumb2 => Insn::Mls {
            rd: reg(raw, 19),
            rn: reg(raw, 3),
            rm: reg(raw, 11),
            ra: reg(raw, 15),
        },
        _ => Insn::Undefined,
    })
}

/// A5.2.10: `cond 0001 1xxx … 1001`, the load/store exclusives.
pub(super) fn decode_exclusive(raw: u32, ext: &Extensions) -> Insn {
    let size = match field(raw, 22, 21) {
        0b00 if ext.v6 => ExSize::Word,
        0b01 if ext.v6k => ExSize::Double,
        0b10 if ext.v6k => ExSize::Byte,
        0b11 if ext.v6k => ExSize::Half,
        _ => return Insn::Undefined,
    };
    let rn = reg(raw, 19);
    if bit(raw, 20) {
        Insn::LoadExclusive {
            size,
            rt: reg(raw, 15),
            rn,
        }
    } else {
        Insn::StoreExclusive {
            size,
            rd: reg(raw, 15),
            rt: reg(raw, 3),
            rn,
        }
    }
}

/// A5.2.11: an `MSR CPSR, #imm` with an empty mask is a hint on ARMv6K.
pub(super) fn decode_hint(raw: u32, ext: &Extensions) -> Option<Insn> {
    if !ext.v6k || bit(raw, 22) || field(raw, 19, 16) != 0 {
        return None;
    }
    let op = field(raw, 7, 0) as u8;
    Some(Insn::Hint {
        op: match op {
            0 => HintOp::Nop,
            1 => HintOp::Yield,
            2 => HintOp::Wfe,
            3 => HintOp::Wfi,
            4 => HintOp::Sev,
            0xf0..=0xff if ext.v7 => HintOp::Dbg(op & 0xf),
            _ => HintOp::Other(op),
        },
    })
}

/// `MOVW`/`MOVT` (v6T2): the `10x00` holes in the immediate data-processing
/// space that A3.4's `MSR` note left undefined.
pub(super) fn decode_move_wide(raw: u32, ext: &Extensions) -> Insn {
    if !ext.thumb2 {
        return Insn::Undefined;
    }
    Insn::MovWide {
        top: bit(raw, 22),
        rd: reg(raw, 15),
        imm: ((field(raw, 19, 16) << 12) | field(raw, 11, 0)) as u16,
    }
}

/// A5.7 and A5.7.1: the unconditional encodings ARMv6 and later define.
///
/// Returns `None` for anything not new, so [`super::isa`]'s ARMv5 table
/// answers as it always did.
pub(super) fn decode_unconditional_v6(raw: u32, ext: &Extensions) -> Option<Insn> {
    if !ext.v6 {
        return None;
    }
    let op1 = field(raw, 27, 20);
    Some(match field(raw, 27, 25) {
        0b000 if op1 == 0b0001_0000 => {
            if bit(raw, 16) {
                // SETEND: 1111 0001 0000 0001 0000 00E0 0000 0000.
                if field(raw, 7, 4) != 0 {
                    return None;
                }
                Insn::Setend { big: bit(raw, 9) }
            } else {
                if bit(raw, 5) {
                    return None;
                }
                let imod = field(raw, 19, 18);
                let change_mode = bit(raw, 17);
                // imod 01 is reserved; imod 00 without M changes nothing
                // and is UNPREDICTABLE (B9.3, `CPS`).
                if imod == 0b01 || (imod == 0 && !change_mode) {
                    return Some(Insn::Undefined);
                }
                Insn::Cps {
                    enable: match imod {
                        0b10 => Some(true),
                        0b11 => Some(false),
                        _ => None,
                    },
                    a: bit(raw, 8),
                    i: bit(raw, 7),
                    f: bit(raw, 6),
                    mode: change_mode.then_some(field(raw, 4, 0) as u8),
                }
            }
        }
        0b100 if bit(raw, 22) && !bit(raw, 20) => Insn::Srs {
            before: bit(raw, 24),
            up: bit(raw, 23),
            writeback: bit(raw, 21),
            mode: field(raw, 4, 0) as u8,
        },
        0b100 if !bit(raw, 22) && bit(raw, 20) => Insn::Rfe {
            before: bit(raw, 24),
            up: bit(raw, 23),
            writeback: bit(raw, 21),
            rn: reg(raw, 19),
        },
        0b010 | 0b011 => {
            if op1 == 0b0101_0111 {
                let option = field(raw, 3, 0) as u8;
                return Some(match field(raw, 7, 4) {
                    0b0001 if ext.v6k => Insn::Clrex,
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
                });
            }
            // A register-offset hint needs bit 4 clear; set, it is the
            // architecturally UNDEFINED media-like space.
            if bit(raw, 25) && bit(raw, 4) {
                return None;
            }
            let hint = bit(raw, 20) && !bit(raw, 21);
            match (bit(raw, 24), bit(raw, 22)) {
                (false, true) if hint && ext.v7 => Insn::Pli {
                    rn: reg(raw, 19),
                    up: bit(raw, 23),
                    offset: mode2_offset(raw),
                },
                (true, false) if hint && ext.mp => Insn::Pldw {
                    rn: reg(raw, 19),
                    up: bit(raw, 23),
                    offset: mode2_offset(raw),
                },
                _ => return None,
            }
        }
        _ => return None,
    })
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

/// The barrier option names (DDI 0406C A8.8, `DMB`).
const fn barrier_option(option: u8) -> Option<&'static str> {
    match option {
        0b1111 => Some("SY"),
        0b1110 => Some("ST"),
        0b1011 => Some("ISH"),
        0b1010 => Some("ISHST"),
        0b0111 => Some("NSH"),
        0b0110 => Some("NSHST"),
        0b0011 => Some("OSH"),
        0b0010 => Some("OSHST"),
        _ => None,
    }
}

/// `IA`, `IB`, `DA` or `DB`.
const fn block_mode(before: bool, up: bool) -> &'static str {
    match (before, up) {
        (false, true) => "IA",
        (true, true) => "IB",
        (false, false) => "DA",
        (true, false) => "DB",
    }
}

/// The UAL text for the variants this module added. The condition goes
/// after the whole mnemonic, as UAL writes it.
#[allow(clippy::too_many_lines)] // One arm per variant, as in `isa`.
pub(super) fn fmt_insn(d: &Decoded, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let c = d.cond;
    let r = RegName;
    match d.insn {
        Insn::Parallel { op, rd, rn, rm } => {
            write!(f, "{op}{c} {}, {}, {}", r(rd), r(rn), r(rm))
        }
        Insn::Extend {
            signed,
            size,
            rd,
            rn,
            rm,
            rotate,
        } => {
            let s = if signed { "S" } else { "U" };
            match rn {
                Some(rn) => write!(
                    f,
                    "{s}XTA{}{c} {}, {}, {}",
                    size.suffix(),
                    r(rd),
                    r(rn),
                    r(rm)
                )?,
                None => write!(f, "{s}XT{}{c} {}, {}", size.suffix(), r(rd), r(rm))?,
            }
            if rotate != 0 {
                write!(f, ", ROR #{}", u32::from(rotate) * 8)?;
            }
            Ok(())
        }
        Insn::Sel { rd, rn, rm } => write!(f, "SEL{c} {}, {}, {}", r(rd), r(rn), r(rm)),
        Insn::Saturate {
            unsigned,
            bits,
            rd,
            rn,
            asr,
            amount,
        } => {
            let u = if unsigned { "U" } else { "S" };
            write!(f, "{u}SAT{c} {}, #{bits}, {}", r(rd), r(rn))?;
            match (asr, amount) {
                (false, 0) => Ok(()),
                (false, n) => write!(f, ", LSL #{n}"),
                (true, 0) => f.write_str(", ASR #32"),
                (true, n) => write!(f, ", ASR #{n}"),
            }
        }
        Insn::Saturate16 {
            unsigned,
            bits,
            rd,
            rn,
        } => {
            let u = if unsigned { "U" } else { "S" };
            write!(f, "{u}SAT16{c} {}, #{bits}, {}", r(rd), r(rn))
        }
        Insn::Pack {
            tb,
            rd,
            rn,
            rm,
            amount,
        } => {
            if tb {
                let n = if amount == 0 { 32 } else { amount };
                write!(f, "PKHTB{c} {}, {}, {}, ASR #{n}", r(rd), r(rn), r(rm))
            } else if amount == 0 {
                write!(f, "PKHBT{c} {}, {}, {}", r(rd), r(rn), r(rm))
            } else {
                write!(f, "PKHBT{c} {}, {}, {}, LSL #{amount}", r(rd), r(rn), r(rm))
            }
        }
        Insn::Reverse { op, rd, rm } => write!(f, "{}{c} {}, {}", op.mnemonic(), r(rd), r(rm)),
        Insn::DualMul {
            sub,
            exchange,
            rd,
            rn,
            rm,
            ra,
        } => {
            let x = if exchange { "X" } else { "" };
            let s = if sub { "S" } else { "A" };
            match ra {
                Some(ra) => write!(
                    f,
                    "SML{s}D{x}{c} {}, {}, {}, {}",
                    r(rd),
                    r(rn),
                    r(rm),
                    r(ra)
                ),
                None => write!(f, "SMU{s}D{x}{c} {}, {}, {}", r(rd), r(rn), r(rm)),
            }
        }
        Insn::DualMulLong {
            sub,
            exchange,
            rdhi,
            rdlo,
            rn,
            rm,
        } => {
            let x = if exchange { "X" } else { "" };
            let s = if sub { "S" } else { "A" };
            write!(
                f,
                "SML{s}LD{x}{c} {}, {}, {}, {}",
                r(rdlo),
                r(rdhi),
                r(rn),
                r(rm)
            )
        }
        Insn::MulHigh {
            sub,
            round,
            rd,
            rn,
            rm,
            ra,
        } => {
            let rr = if round { "R" } else { "" };
            match (sub, ra) {
                (false, None) => write!(f, "SMMUL{rr}{c} {}, {}, {}", r(rd), r(rn), r(rm)),
                (sub, Some(ra)) => {
                    let op = if sub { "SMMLS" } else { "SMMLA" };
                    write!(f, "{op}{rr}{c} {}, {}, {}, {}", r(rd), r(rn), r(rm), r(ra))
                }
                (true, None) => write!(f, "SMMLS{rr}{c} {}, {}, {}", r(rd), r(rn), r(rm)),
            }
        }
        Insn::Usad8 { rd, rn, rm, ra } => match ra {
            Some(ra) => write!(f, "USADA8{c} {}, {}, {}, {}", r(rd), r(rn), r(rm), r(ra)),
            None => write!(f, "USAD8{c} {}, {}, {}", r(rd), r(rn), r(rm)),
        },
        Insn::Umaal { rdhi, rdlo, rn, rm } => {
            write!(f, "UMAAL{c} {}, {}, {}, {}", r(rdlo), r(rdhi), r(rn), r(rm))
        }
        Insn::Mls { rd, rn, rm, ra } => {
            write!(f, "MLS{c} {}, {}, {}, {}", r(rd), r(rn), r(rm), r(ra))
        }
        Insn::Divide { signed, rd, rn, rm } => {
            let s = if signed { "S" } else { "U" };
            write!(f, "{s}DIV{c} {}, {}, {}", r(rd), r(rn), r(rm))
        }
        Insn::MovWide { top, rd, imm } => {
            let t = if top { "T" } else { "W" };
            write!(f, "MOV{t}{c} {}, #{imm}", r(rd))
        }
        Insn::Bitfield {
            op,
            rd,
            rn,
            lsb,
            width,
        } => match op {
            BitfieldOp::Bfc => write!(f, "BFC{c} {}, #{lsb}, #{width}", r(rd)),
            _ => write!(
                f,
                "{}{c} {}, {}, #{lsb}, #{width}",
                op.mnemonic(),
                r(rd),
                r(rn)
            ),
        },
        Insn::Cps {
            enable,
            a,
            i,
            f: fiq,
            mode,
        } => {
            let op = match enable {
                Some(true) => "CPSIE",
                Some(false) => "CPSID",
                None => "CPS",
            };
            f.write_str(op)?;
            if enable.is_some() {
                write!(
                    f,
                    " {}{}{}",
                    if a { "a" } else { "" },
                    if i { "i" } else { "" },
                    if fiq { "f" } else { "" }
                )?;
                if let Some(m) = mode {
                    write!(f, ", #{m}")?;
                }
                Ok(())
            } else {
                write!(f, " #{}", mode.unwrap_or(0))
            }
        }
        Insn::Setend { big } => f.write_str(if big { "SETEND BE" } else { "SETEND LE" }),
        Insn::Srs {
            before,
            up,
            writeback,
            mode,
        } => {
            let w = if writeback { "!" } else { "" };
            write!(f, "SRS{} sp{w}, #{mode}", block_mode(before, up))
        }
        Insn::Rfe {
            before,
            up,
            writeback,
            rn,
        } => {
            let w = if writeback { "!" } else { "" };
            write!(f, "RFE{} {}{w}", block_mode(before, up), r(rn))
        }
        Insn::LoadExclusive { size, rt, rn } => {
            write!(f, "LDREX{}{c} {}, ", size.suffix(), r(rt))?;
            if size == ExSize::Double {
                write!(f, "{}, ", r(rt.wrapping_add(1)))?;
            }
            write!(f, "[{}]", r(rn))
        }
        Insn::StoreExclusive { size, rd, rt, rn } => {
            write!(f, "STREX{}{c} {}, {}, ", size.suffix(), r(rd), r(rt))?;
            if size == ExSize::Double {
                write!(f, "{}, ", r(rt.wrapping_add(1)))?;
            }
            write!(f, "[{}]", r(rn))
        }
        Insn::Clrex => f.write_str("CLREX"),
        Insn::Hint { op } => match op {
            HintOp::Nop => write!(f, "NOP{c}"),
            HintOp::Yield => write!(f, "YIELD{c}"),
            HintOp::Wfe => write!(f, "WFE{c}"),
            HintOp::Wfi => write!(f, "WFI{c}"),
            HintOp::Sev => write!(f, "SEV{c}"),
            HintOp::Dbg(n) => write!(f, "DBG{c} #{n}"),
            HintOp::Other(n) => write!(f, "NOP{c} ; hint #{n}"),
        },
        Insn::Barrier { kind, option } => {
            let name = match kind {
                BarrierKind::Dmb => "DMB",
                BarrierKind::Dsb => "DSB",
                BarrierKind::Isb => "ISB",
            };
            match barrier_option(option) {
                Some(o) => write!(f, "{name} {o}"),
                None => write!(f, "{name} #{option}"),
            }
        }
        Insn::Pli { rn, up, offset } | Insn::Pldw { rn, up, offset } => {
            let name = if matches!(d.insn, Insn::Pli { .. }) {
                "PLI"
            } else {
                "PLDW"
            };
            write!(
                f,
                "{name} {}",
                Addressing {
                    rn,
                    up,
                    index: Index::Pre { writeback: false },
                    offset,
                }
            )
        }
        Insn::Smc { imm } => write!(f, "SMC{c} #{imm}"),
        Insn::Bxj { rm } => write!(f, "BXJ{c} {}", r(rm)),
        // Everything else is ARMv5's and printed by `isa`.
        _ => write!(f, "UNDEFINED ; 0x{:08x}", d.raw),
    }
}
