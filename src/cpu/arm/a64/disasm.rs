//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! CLAUDE.md is explicit that the disassembler is not a side project: gdb and
//! the monitor both need it, and writing the instruction set out twice is how
//! the two drift. So nothing here knows an opcode: every mnemonic comes from
//! [`isa::Op::mnemonic`], every operand layout from [`isa::Fmt`], and every
//! system-register name from [`sysreg::SysReg::name`]. Adding a row to
//! [`isa::TABLE`] makes it disassemble with no edit here.
//!
//! # Aliases are not printed
//!
//! A64's assembly syntax is full of aliases — `MOV` is `ORR` with `XZR`, `CMP`
//! is `SUBS` with `XZR`, `LSL` is a `UBFM`, `TLBI` is a `SYS`. This
//! disassembler prints the *encoding's* instruction and not its preferred
//! alias, because the table names encodings and inventing a second naming
//! layer here would be the duplication the rule exists to prevent. It is a
//! readability cost paid deliberately, and it is the one place this output
//! differs from `objdump`.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;
use core::fmt::Write as _;

use super::isa::{self, Fmt, Suffix};
use super::{fp, simd, sysreg};

/// Why a listing has a hole in it.
///
/// A listing does not stop at a hole and does not shorten: it carries the hole
/// as a value and keeps going, because "the first ten instructions were fine"
/// is exactly the case a monitor is looking at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Missing {
    /// The translation tables map nothing at that virtual address.
    Untranslated,
    /// Nothing answered at that physical address: the bus refused the read.
    Unmapped,
}

impl fmt::Display for Missing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Missing::Untranslated => f.write_str("not mapped"),
            Missing::Unmapped => f.write_str("no memory"),
        }
    }
}

/// One disassembled instruction, or one hole where an instruction could not be
/// read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Disassembled {
    /// Where it starts.
    pub addr: u64,
    /// The raw encoding. Zero for a hole.
    pub encoding: u32,
    /// The assembly text.
    pub text: String,
    /// What was missing, when this entry is a hole rather than an instruction.
    pub hole: Option<Missing>,
}

impl Disassembled {
    /// How many bytes it occupies. Always four: A64 has one instruction
    /// length, which is the whole reason this is a constant and not a field.
    pub const LEN: u64 = 4;

    /// A hole in a listing: nothing could be read at `addr`, for this reason.
    #[must_use]
    pub fn missing(addr: u64, why: Missing) -> Disassembled {
        Disassembled {
            addr,
            encoding: 0,
            text: format!("?? <{why}>"),
            hole: Some(why),
        }
    }
}

impl fmt::Display for Disassembled {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.hole.is_some() {
            return write!(f, "{:016x}:           {}", self.addr, self.text);
        }
        write!(
            f,
            "{:016x}: {:08x}  {}",
            self.addr, self.encoding, self.text
        )
    }
}

/// A general-purpose register's name.
///
/// The `SP`/`XZR` distinction is the caller's — it comes from
/// [`Fmt::rd_is_sp`] and [`Fmt::rn_is_sp`], the same source the interpreter
/// reads — because there is no way to tell from the register number alone, and
/// a disassembler that guessed would disagree with the core beside it.
fn reg(index: u32, width: u32, is_sp: bool) -> String {
    let sixty_four = width == 64;
    if index == 31 {
        return match (is_sp, sixty_four) {
            (true, true) => "sp".to_string(),
            (true, false) => "wsp".to_string(),
            (false, true) => "xzr".to_string(),
            (false, false) => "wzr".to_string(),
        };
    }
    format!("{}{}", if sixty_four { 'x' } else { 'w' }, index)
}

/// A SIMD&FP register's name at a precision.
///
/// The letter *is* the width — `s0`, `d0`, `h0` — which is why A64's
/// floating-point mnemonics carry no width suffix and this function exists.
/// An unallocated `ptype` prints the register as `v<n>.?`, which is what a
/// listing of a word that will raise `UNDEFINED` should say.
fn vreg_prec(index: u32, prec: Option<fp::Prec>) -> String {
    match prec {
        Some(p) => format!("{}{index}", p.letter()),
        None => format!("v{index}.?"),
    }
}

/// A SIMD&FP register's name, at the precision this word's `ptype` names.
fn vreg(index: u32, word: u32) -> String {
    vreg_prec(index, fp::Prec::from_ptype(isa::ptype(word)))
}

/// A SIMD&FP register's name at a load or store's access width, as the
/// base-2 logarithm of its size in bytes.
///
/// Five widths here rather than three: a load or store can move a single byte
/// or a whole 128-bit register, neither of which is a floating-point format
/// and neither of which [`fp::Prec`] therefore names.
fn vreg_scale(index: u32, scale: Option<u32>) -> String {
    let letter = match scale {
        Some(0) => 'b',
        Some(1) => 'h',
        Some(2) => 's',
        Some(3) => 'd',
        Some(4) => 'q',
        _ => return format!("v{index}.?"),
    };
    format!("{letter}{index}")
}

/// A signed immediate, in the form an assembler would accept.
fn imm(value: i64) -> String {
    if value.abs() < 10 {
        format!("#{value}")
    } else if value < 0 {
        format!("#-0x{:x}", value.unsigned_abs())
    } else {
        format!("#0x{value:x}")
    }
}

/// A branch target, printed as the absolute address it resolves to.
fn target(pc: u64, offset: i64) -> String {
    format!("0x{:x}", pc.wrapping_add(offset as u64))
}

/// The `{, shift #amount}` tail of a shifted-register operand, empty when the
/// shift is the default `LSL #0`.
fn shift_tail(word: u32) -> String {
    let amount = isa::shift_amount(word);
    if amount == 0 && isa::shift_type(word) == 0 {
        return String::new();
    }
    format!(
        ", {} #{amount}",
        isa::ShiftKind::from_bits(isa::shift_type(word)).name()
    )
}

/// The `{, extend {#amount}}` tail of an extended-register operand.
fn extend_tail(word: u32, amount: u32, default_option: u32) -> String {
    let option = isa::extend_option(word);
    if option == default_option && amount == 0 {
        return String::new();
    }
    let name = isa::extend_name(option);
    if amount == 0 {
        format!(", {name}")
    } else {
        format!(", {name} #{amount}")
    }
}

/// A vector register and its arrangement — `v3.4s`.
///
/// An arrangement the architecture reserves prints as `v3.?`, which is what a
/// listing of a word that will raise `UNDEFINED` should say: the row decoded,
/// and the shape it names does not exist.
fn varr(index: u32, arr: Option<simd::Arrangement>) -> String {
    match arr {
        Some(a) => format!("v{index}.{}", a.name()),
        None => format!("v{index}.?"),
    }
}

/// One lane of a vector register — `v3.s[2]`.
fn vlane(index: u32, esize: u32, lane: u32) -> String {
    if esize > 3 {
        return format!("v{index}.?[{lane}]");
    }
    format!("v{index}.{}[{lane}]", simd::elem_letter(esize))
}

/// The element width and lane index an `imm5` field names.
///
/// A zero low nibble names no width, so this reports `4` — which every caller
/// prints as `?` rather than guessing, since the interpreter will raise
/// `UNDEFINED` on the same word.
fn lane_of(word: u32) -> (u32, u32) {
    let imm5 = isa::simd_imm5(word);
    if imm5 & 0xf == 0 {
        return (4, 0);
    }
    let esize = imm5.trailing_zeros();
    (esize, imm5 >> (esize + 1))
}

/// The `8B`/`16B` arrangement of the operations that have no element width.
fn byte_arr(q: bool) -> Option<simd::Arrangement> {
    Some(simd::Arrangement {
        esize: 0,
        lanes: if q { 16 } else { 8 },
    })
}

/// The arrangement of a three-register operation, which comes from a
/// different field in each of the three groups.
fn three_same_arr(word: u32, fmt: Fmt) -> Option<simd::Arrangement> {
    match fmt {
        Fmt::VecThreeSameLog => byte_arr(isa::q(word)),
        Fmt::VecThreeSameFp => simd::Arrangement::from_sz(isa::simd_sz(word), isa::q(word)),
        _ => simd::Arrangement::from_size(isa::simd_size(word), isa::q(word)),
    }
}

/// The same for a two-register operation.
///
/// `NOT`, `RBIT` and `CNT` are the exception: their `size` field selects the
/// operation rather than an element width, and all three print `8B` or `16B`.
fn two_misc_arr(word: u32, fmt: Fmt, op: isa::Op) -> Option<simd::Arrangement> {
    if matches!(op, isa::Op::NotVec | isa::Op::RbitVec | isa::Op::CntVec) {
        return byte_arr(isa::q(word));
    }
    if fmt == Fmt::VecTwoMiscFp {
        simd::Arrangement::from_sz(isa::simd_sz(word), isa::q(word))
    } else {
        simd::Arrangement::from_size(isa::simd_size(word), isa::q(word))
    }
}

/// The element width and the `immh`:`immb` value of a shift by an immediate.
fn shift_of(word: u32) -> (u32, u32) {
    let immhb = isa::simd_immhb(word);
    let immh = immhb >> 3;
    if immh == 0 {
        return (0, immhb);
    }
    (31 - immh.leading_zeros(), immhb)
}

/// The `, lsl #8` or `, msl #16` tail a modified immediate's `cmode` names.
fn mod_imm_shift(cmode: u32) -> String {
    match (cmode >> 1) & 7 {
        0b000 | 0b100 => String::new(),
        0b001 | 0b101 => ", lsl #8".to_string(),
        0b010 => ", lsl #16".to_string(),
        0b011 => ", lsl #24".to_string(),
        0b110 => {
            if cmode & 1 == 0 {
                ", msl #8".to_string()
            } else {
                ", msl #16".to_string()
            }
        }
        _ => String::new(),
    }
}

/// Whether a shift by an immediate shifts *left*, and so reads its amount as
/// `immh:immb - esize` rather than `2 * esize - immh:immb`.
///
/// The two directions share an encoding field and differ only in how it is
/// read, so a row that lands on the wrong side of this prints a shift of the
/// right magnitude in the wrong place — which is exactly the kind of defect
/// that survives a decode-only cross-check.
fn left_shift(op: isa::Op) -> bool {
    matches!(
        op,
        isa::Op::ShlVec
            | isa::Op::SliVec
            | isa::Op::SqshlImmVec
            | isa::Op::UqshlImmVec
            | isa::Op::SqshluImmVec
            | isa::Op::SqshlImmScalar
            | isa::Op::UqshlImmScalar
            | isa::Op::SqshluImmScalar
    )
}

/// Whether a scalar SIMD operation is a floating-point one, and therefore
/// takes its width from `sz` rather than being a doubleword.
fn fp_scalar(op: isa::Op) -> bool {
    matches!(
        op,
        isa::Op::FcmeqScalar
            | isa::Op::FcmgeScalar
            | isa::Op::FcmgtScalar
            | isa::Op::FabdScalar
            | isa::Op::FcmeqZeroScalar
            | isa::Op::FcmgeZeroScalar
            | isa::Op::FcmgtZeroScalar
            | isa::Op::FcmleZeroScalar
            | isa::Op::FcmltZeroScalar
            | isa::Op::FaddpScalar
            | isa::Op::FmaxpScalar
            | isa::Op::FminpScalar
    )
}

/// The register letter a scalar SIMD operation prints.
fn scalar_letter(word: u32, op: isa::Op) -> char {
    if fp_scalar(op) {
        simd::elem_letter(2 + u32::from(isa::simd_sz(word)))
    } else {
        'd'
    }
}

/// The `{ v0.4s, v1.4s }` list of a multiple-structures access.
fn struct_multi_list(word: u32, t: u32) -> String {
    let Some((repeats, selem)) = isa::struct_shape(isa::field(word, 15, 12)) else {
        return "{ ? }".to_string();
    };
    let arr = simd::Arrangement::whole(isa::field(word, 11, 10), isa::q(word));
    let mut out = String::from("{ ");
    for i in 0..repeats * selem {
        if i > 0 {
            out.push_str(", ");
        }
        out.push_str(&varr((t + i) % 32, arr));
    }
    out.push_str(" }");
    out
}

/// The `{ v0.s, v1.s }[2]` list of a single-element access, or the
/// `{ v0.4s }` of a replicating load.
fn struct_single_list(word: u32, t: u32) -> String {
    let selem = isa::struct_single_selem(word);
    let mut out = String::from("{ ");
    if isa::field(word, 15, 14) == 0b11 {
        let arr = simd::Arrangement::whole(isa::field(word, 11, 10), isa::q(word));
        for i in 0..selem {
            if i > 0 {
                out.push_str(", ");
            }
            out.push_str(&varr((t + i) % 32, arr));
        }
        out.push_str(" }");
        return out;
    }
    let shape = isa::struct_single_shape(word);
    for i in 0..selem {
        if i > 0 {
            out.push_str(", ");
        }
        match shape {
            Some((esize, _)) => {
                let _ = write!(out, "v{}.{}", (t + i) % 32, simd::elem_letter(esize));
            }
            None => {
                let _ = write!(out, "v{}.?", (t + i) % 32);
            }
        }
    }
    out.push_str(" }");
    match shape {
        Some((_, index)) => {
            let _ = write!(out, "[{index}]");
        }
        None => out.push_str("[?]"),
    }
    out
}

/// How many bytes a structure access moves, which is also the immediate its
/// post-indexed form adds.
fn struct_bytes(word: u32, single: bool) -> u64 {
    if single {
        let selem = u64::from(isa::struct_single_selem(word));
        let ebytes = if isa::field(word, 15, 14) == 0b11 {
            1u64 << isa::field(word, 11, 10)
        } else {
            match isa::struct_single_shape(word) {
                Some((esize, _)) => 1u64 << esize,
                None => 0,
            }
        };
        return selem * ebytes;
    }
    let Some((repeats, selem)) = isa::struct_shape(isa::field(word, 15, 12)) else {
        return 0;
    };
    let bytes = if isa::q(word) { 16u64 } else { 8 };
    u64::from(repeats * selem) * bytes
}

/// Disassemble one instruction word at `pc`.
///
/// `features` is the core's, because an encoding whose feature is absent is
/// not an instruction on that core — the disassembler is honest about which
/// part it is disassembling for, which is the other half of `ROADMAP.md`
/// §6.1.1's per-entry gating.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn disassemble(word: u32, pc: u64, features: isa::Features) -> Disassembled {
    let Some(insn) = isa::decode(word, features) else {
        return Disassembled {
            addr: pc,
            encoding: word,
            text: format!(".word\t0x{word:08x}"),
            hole: None,
        };
    };

    let mut text = String::from(insn.op.mnemonic());
    match insn.fmt.suffix_kind() {
        Suffix::None => {}
        Suffix::Cond => {
            let _ = write!(text, ".{}", isa::cond_lo(word));
        }
        Suffix::Wide => {
            // The `2` of `XTN2`, `FCVTL2`, `UMULL2`: `Q` says which half of
            // the wide operand the instruction touches, and the mnemonic
            // column cannot carry a bit.
            if isa::q(word) {
                text.push('2');
            }
        }
        Suffix::Order => {
            // The acquire and release bits sit at different places on the two
            // atomic encodings: bits 23:22 on a compare-and-swap, and bit 23
            // with bit 22 on the read-modify-writes.
            let (acquire, release) = if matches!(insn.op, isa::Op::CasW | isa::Op::CasX) {
                (isa::bit(word, 22), isa::bit(word, 15))
            } else {
                (isa::bit(word, 23), isa::bit(word, 22))
            };
            if acquire {
                text.push('a');
            }
            if release {
                text.push('l');
            }
        }
    }

    let width = isa::datasize(word);
    let d = isa::rd(word);
    let n = isa::rn(word);
    let m = isa::rm(word);
    let rd_sp = insn.fmt.rd_is_sp();
    let rn_sp = insn.fmt.rn_is_sp();
    let mut ops = String::new();

    match insn.fmt {
        Fmt::PcRel => {
            let raw = isa::sext(
                ((isa::field(word, 23, 5) as u64) << 2) | u64::from(isa::field(word, 30, 29)),
                21,
            );
            let (base, offset) = if insn.op == isa::Op::Adrp {
                (pc & !0xfff, raw << 12)
            } else {
                (pc, raw)
            };
            let _ = write!(ops, "{}, {}", reg(d, 64, false), target(base, offset));
        }
        Fmt::AddSubImm | Fmt::AddSubImmS => {
            let shift = isa::field(word, 23, 22);
            let _ = write!(
                ops,
                "{}, {}, #0x{:x}",
                reg(d, width, rd_sp),
                reg(n, width, rn_sp),
                isa::imm12(word)
            );
            if shift == 1 {
                ops.push_str(", lsl #12");
            }
        }
        Fmt::LogImm | Fmt::LogImmS => {
            let value = isa::decode_bit_masks(
                isa::n_bit(word),
                isa::imms(word),
                isa::immr(word),
                true,
                width,
            )
            .map_or(0, |(w, _)| w);
            let _ = write!(
                ops,
                "{}, {}, #0x{value:x}",
                reg(d, width, rd_sp),
                reg(n, width, false)
            );
        }
        Fmt::MoveWide => {
            let hw = isa::field(word, 22, 21);
            let _ = write!(ops, "{}, #0x{:x}", reg(d, width, false), isa::imm16(word));
            if hw != 0 {
                let _ = write!(ops, ", lsl #{}", hw * 16);
            }
        }
        Fmt::Bitfield => {
            let _ = write!(
                ops,
                "{}, {}, #{}, #{}",
                reg(d, width, false),
                reg(n, width, false),
                isa::immr(word),
                isa::imms(word)
            );
        }
        Fmt::Extr => {
            let _ = write!(
                ops,
                "{}, {}, {}, #{}",
                reg(d, width, false),
                reg(n, width, false),
                reg(m, width, false),
                isa::imms(word)
            );
        }
        Fmt::BranchImm => ops.push_str(&target(pc, isa::imm26(word))),
        Fmt::CondBranch => ops.push_str(&target(pc, isa::imm19(word))),
        Fmt::CmpBranch => {
            let _ = write!(
                ops,
                "{}, {}",
                reg(d, width, false),
                target(pc, isa::imm19(word))
            );
        }
        Fmt::TestBranch => {
            let pos = (u32::from(isa::sf(word)) << 5) | isa::field(word, 23, 19);
            let _ = write!(
                ops,
                "{}, #{pos}, {}",
                reg(d, if pos >= 32 { 64 } else { 32 }, false),
                target(pc, isa::imm14(word))
            );
        }
        Fmt::BranchReg => ops.push_str(&reg(n, 64, false)),
        Fmt::NoOperands => {}
        Fmt::Exception => {
            let _ = write!(ops, "#0x{:x}", isa::imm16(word));
        }
        Fmt::Barrier => ops.push_str(isa::barrier_domain(isa::field(word, 11, 8))),
        Fmt::PstateImm => {
            let field = match insn.op {
                isa::Op::MsrSpsel => "spsel",
                isa::Op::MsrDaifset => "daifset",
                _ => "daifclr",
            };
            let _ = write!(ops, "{field}, #0x{:x}", isa::field(word, 11, 8));
        }
        Fmt::SysRead | Fmt::SysWrite => {
            let key = isa::field(word, 20, 5) as u16;
            let name = sysreg::lookup(key).map_or_else(
                || {
                    format!(
                        "s{}_{}_c{}_c{}_{}",
                        isa::field(word, 20, 19),
                        isa::field(word, 18, 16),
                        isa::field(word, 15, 12),
                        isa::field(word, 11, 8),
                        isa::field(word, 7, 5)
                    )
                },
                |spec| spec.reg.name().to_string(),
            );
            if insn.fmt == Fmt::SysRead {
                let _ = write!(ops, "{}, {name}", reg(d, 64, false));
            } else {
                let _ = write!(ops, "{name}, {}", reg(d, 64, false));
            }
        }
        Fmt::SysOp => {
            let _ = write!(
                ops,
                "#{}, c{}, c{}, #{}, {}",
                isa::field(word, 18, 16),
                isa::field(word, 15, 12),
                isa::field(word, 11, 8),
                isa::field(word, 7, 5),
                reg(d, 64, false)
            );
        }
        Fmt::LoadLiteral => {
            let dest_width = if matches!(insn.op, isa::Op::LdrLitW) {
                32
            } else {
                64
            };
            if insn.op == isa::Op::PrfmLit {
                let _ = write!(ops, "#0x{:x}, {}", d, target(pc, isa::imm19(word)));
            } else {
                let _ = write!(
                    ops,
                    "{}, {}",
                    reg(d, dest_width, false),
                    target(pc, isa::imm19(word))
                );
            }
        }
        Fmt::LdStUImm
        | Fmt::LdStUnscaled
        | Fmt::LdStUnpriv
        | Fmt::LdStPost
        | Fmt::LdStPre
        | Fmt::LdStRegOff => {
            let size = isa::ls_size(word);
            let dest = match isa::ls_access(size, isa::ls_opc(word)) {
                Some(isa::LsAccess::Store { bytes }) => {
                    reg(d, if bytes == 8 { 64 } else { 32 }, false)
                }
                Some(isa::LsAccess::Load { wide, .. } | isa::LsAccess::LoadSigned { wide, .. }) => {
                    reg(d, if wide { 64 } else { 32 }, false)
                }
                _ => format!("#0x{d:x}"),
            };
            let base = reg(n, 64, rn_sp);
            match insn.fmt {
                Fmt::LdStUImm => {
                    let offset = u64::from(isa::imm12(word)) << size;
                    if offset == 0 {
                        let _ = write!(ops, "{dest}, [{base}]");
                    } else {
                        let _ = write!(ops, "{dest}, [{base}, #0x{offset:x}]");
                    }
                }
                Fmt::LdStUnscaled | Fmt::LdStUnpriv => {
                    let offset = isa::imm9(word);
                    if offset == 0 {
                        let _ = write!(ops, "{dest}, [{base}]");
                    } else {
                        let _ = write!(ops, "{dest}, [{base}, {}]", imm(offset));
                    }
                }
                Fmt::LdStPost => {
                    let _ = write!(ops, "{dest}, [{base}], {}", imm(isa::imm9(word)));
                }
                Fmt::LdStPre => {
                    let _ = write!(ops, "{dest}, [{base}, {}]!", imm(isa::imm9(word)));
                }
                _ => {
                    let option = isa::extend_option(word);
                    let amount = if isa::bit(word, 12) { size } else { 0 };
                    let index_width = if option & 1 == 0 { 32 } else { 64 };
                    let _ = write!(
                        ops,
                        "{dest}, [{base}, {}{}]",
                        reg(m, index_width, false),
                        extend_tail(word, amount, 3)
                    );
                }
            }
        }
        Fmt::LdStPairOff | Fmt::LdStPairPost | Fmt::LdStPairPre => {
            let opc = isa::field(word, 31, 30);
            let scale = if opc == 0b10 { 3 } else { 2 };
            let regs_width = if opc == 0b00 { 32 } else { 64 };
            let offset = isa::imm7(word) << scale;
            let base = reg(n, 64, rn_sp);
            let pair = format!(
                "{}, {}",
                reg(d, regs_width, false),
                reg(isa::ra(word), regs_width, false)
            );
            match insn.fmt {
                Fmt::LdStPairOff if offset == 0 => {
                    let _ = write!(ops, "{pair}, [{base}]");
                }
                Fmt::LdStPairOff => {
                    let _ = write!(ops, "{pair}, [{base}, {}]", imm(offset));
                }
                Fmt::LdStPairPost => {
                    let _ = write!(ops, "{pair}, [{base}], {}", imm(offset));
                }
                _ => {
                    let _ = write!(ops, "{pair}, [{base}, {}]!", imm(offset));
                }
            }
        }
        Fmt::LdStExclusive => {
            let bytes = 1u64 << isa::ls_size(word);
            let _ = write!(
                ops,
                "{}, [{}]",
                reg(d, if bytes == 8 { 64 } else { 32 }, false),
                reg(n, 64, rn_sp)
            );
        }
        Fmt::StoreExclusive => {
            let bytes = 1u64 << isa::ls_size(word);
            let _ = write!(
                ops,
                "{}, {}, [{}]",
                reg(m, 32, false),
                reg(d, if bytes == 8 { 64 } else { 32 }, false),
                reg(n, 64, rn_sp)
            );
        }
        // The pair exclusives. `Rt2` is bits 14:10 — the field an ordinary
        // pair load spends on `Rt2` as well, which is the one thing about
        // these encodings that is not surprising. The status register of
        // `STXP` is always 32 bits whatever the pair's width is, because it
        // holds a zero or a one.
        Fmt::LoadExclusivePair => {
            let w = if isa::ls_size(word) == 3 { 64 } else { 32 };
            let _ = write!(
                ops,
                "{}, {}, [{}]",
                reg(d, w, false),
                reg(isa::ra(word), w, false),
                reg(n, 64, rn_sp)
            );
        }
        Fmt::StoreExclusivePair => {
            let w = if isa::ls_size(word) == 3 { 64 } else { 32 };
            let _ = write!(
                ops,
                "{}, {}, {}, [{}]",
                reg(m, 32, false),
                reg(d, w, false),
                reg(isa::ra(word), w, false),
                reg(n, 64, rn_sp)
            );
        }
        Fmt::Atomic => {
            let bytes = 1u64 << isa::ls_size(word);
            let w = if bytes == 8 { 64 } else { 32 };
            let _ = write!(
                ops,
                "{}, {}, [{}]",
                reg(m, w, false),
                reg(d, w, false),
                reg(n, 64, rn_sp)
            );
        }
        Fmt::AddSubExt | Fmt::AddSubExtS => {
            let amount = isa::field(word, 12, 10);
            let option = isa::extend_option(word);
            let index_width = if option & 1 == 0 { 32 } else { 64 };
            let _ = write!(
                ops,
                "{}, {}, {}{}",
                reg(d, width, rd_sp),
                reg(n, width, rn_sp),
                reg(m, index_width, false),
                extend_tail(word, amount, if width == 64 { 3 } else { 2 })
            );
        }
        Fmt::ShiftedReg => {
            let _ = write!(
                ops,
                "{}, {}, {}{}",
                reg(d, width, false),
                reg(n, width, false),
                reg(m, width, false),
                shift_tail(word)
            );
        }
        Fmt::ThreeReg => {
            // `SMULH` and `UMULH` are always 64-bit; the rest follow `sf`.
            let w = if matches!(insn.op, isa::Op::Smulh | isa::Op::Umulh) {
                64
            } else {
                width
            };
            let _ = write!(
                ops,
                "{}, {}, {}",
                reg(d, w, false),
                reg(n, w, false),
                reg(m, w, false)
            );
        }
        Fmt::CondCmpReg => {
            let _ = write!(
                ops,
                "{}, {}, #0x{:x}, {}",
                reg(n, width, false),
                reg(m, width, false),
                word & 0xf,
                isa::cond_hi(word)
            );
        }
        Fmt::CondCmpImm => {
            let _ = write!(
                ops,
                "{}, #0x{:x}, #0x{:x}, {}",
                reg(n, width, false),
                m,
                word & 0xf,
                isa::cond_hi(word)
            );
        }
        Fmt::CondSel => {
            let _ = write!(
                ops,
                "{}, {}, {}, {}",
                reg(d, width, false),
                reg(n, width, false),
                reg(m, width, false),
                isa::cond_hi(word)
            );
        }
        Fmt::FourReg => {
            // The long multiplies take word sources and a doubleword
            // accumulator, which is the whole point of their existing.
            let long = matches!(
                insn.op,
                isa::Op::Smaddl | isa::Op::Smsubl | isa::Op::Umaddl | isa::Op::Umsubl
            );
            let src = if long { 32 } else { width };
            let dst = if long { 64 } else { width };
            let _ = write!(
                ops,
                "{}, {}, {}, {}",
                reg(d, dst, false),
                reg(n, src, false),
                reg(m, src, false),
                reg(isa::ra(word), dst, false)
            );
        }
        Fmt::CrcReg => {
            // `CRC32X`/`CRC32CX` take a doubleword source; the rest are all
            // 32-bit, and the accumulator and result always are.
            let src = if matches!(insn.op, isa::Op::Crc32x | isa::Op::Crc32cx) {
                64
            } else {
                32
            };
            let _ = write!(
                ops,
                "{}, {}, {}",
                reg(d, 32, false),
                reg(n, 32, false),
                reg(m, src, false)
            );
        }
        Fmt::TwoReg => {
            let w = if matches!(insn.op, isa::Op::Rev32 | isa::Op::RevX) {
                64
            } else if insn.op == isa::Op::RevW {
                32
            } else {
                width
            };
            let _ = write!(ops, "{}, {}", reg(d, w, false), reg(n, w, false));
        }
        // -- Scalar floating point ---------------------------------------
        Fmt::FpOneSrc => {
            let _ = write!(ops, "{}, {}", vreg(d, word), vreg(n, word));
        }
        Fmt::FpCvt => {
            // The destination's precision is `opc` (bits 16:15) and the
            // source's is `ptype`, which is the one place in the whole
            // encoding where the two ends of an instruction disagree.
            let dst = fp::Prec::from_ptype(isa::field(word, 16, 15));
            let _ = write!(
                ops,
                "{}, {}",
                vreg_prec(d, dst),
                vreg_prec(n, fp::Prec::from_ptype(isa::ptype(word)))
            );
        }
        Fmt::FpTwoSrc => {
            let _ = write!(
                ops,
                "{}, {}, {}",
                vreg(d, word),
                vreg(n, word),
                vreg(m, word)
            );
        }
        Fmt::FpThreeSrc => {
            let _ = write!(
                ops,
                "{}, {}, {}, {}",
                vreg(d, word),
                vreg(n, word),
                vreg(m, word),
                vreg(isa::ra(word), word)
            );
        }
        Fmt::FpCmp => {
            // Bit 3 of `opcode2` selects the compare-with-zero form, whose
            // second operand is written out rather than named.
            if isa::bit(word, 3) {
                let _ = write!(ops, "{}, #0.0", vreg(n, word));
            } else {
                let _ = write!(ops, "{}, {}", vreg(n, word), vreg(m, word));
            }
        }
        Fmt::FpCondCmp => {
            let _ = write!(
                ops,
                "{}, {}, #0x{:x}, {}",
                vreg(n, word),
                vreg(m, word),
                word & 0xf,
                isa::cond_hi(word)
            );
        }
        Fmt::FpCondSel => {
            let _ = write!(
                ops,
                "{}, {}, {}, {}",
                vreg(d, word),
                vreg(n, word),
                vreg(m, word),
                isa::cond_hi(word)
            );
        }
        Fmt::FpImm => {
            // The eight bits are printed as the value they expand to, in hex:
            // a decimal here would need a float formatter, and this crate has
            // no host floating point to write one with.
            let value = fp::Prec::from_ptype(isa::ptype(word))
                .map_or(0, |p| fp::expand_imm(isa::fp_imm8(word), p));
            let _ = write!(ops, "{}, #0x{value:x}", vreg(d, word));
        }
        Fmt::FpIntCvt => {
            let prec = fp::Prec::from_ptype(isa::ptype(word));
            // `SCVTF`, `UCVTF` and `FMOV` from a general register write the
            // floating-point side; every other opcode in the group writes the
            // general one. The `rmode == 01` forms name the *top half* of a
            // vector register, which no other encoding can reach.
            let opcode = isa::cvt_opcode(word);
            let fp_is_dest = matches!(opcode, 0b010 | 0b011 | 0b111);
            // `rmode == 0b01` is the *top half* pair only in company with an
            // `FMOV` opcode: on `FCVTPS` the same `rmode` means "round toward
            // +infinity", and reading it as a register half would print a
            // vector operand for an ordinary conversion.
            let high = isa::cvt_rmode(word) == 0b01 && matches!(opcode, 0b110 | 0b111);
            let fp_side = |index: u32| {
                if high {
                    format!("v{index}.d[1]")
                } else {
                    vreg_prec(index, prec)
                }
            };
            if fp_is_dest {
                let _ = write!(ops, "{}, {}", fp_side(d), reg(n, width, false));
            } else {
                let _ = write!(ops, "{}, {}", reg(d, width, false), fp_side(n));
            }
        }
        Fmt::FpFixCvt => {
            let prec = fp::Prec::from_ptype(isa::ptype(word));
            let fp_is_dest = matches!(isa::cvt_opcode(word), 0b010 | 0b011);
            if fp_is_dest {
                let _ = write!(
                    ops,
                    "{}, {}, #{}",
                    vreg_prec(d, prec),
                    reg(n, width, false),
                    isa::fbits(word)
                );
            } else {
                let _ = write!(
                    ops,
                    "{}, {}, #{}",
                    reg(d, width, false),
                    vreg_prec(n, prec),
                    isa::fbits(word)
                );
            }
        }
        Fmt::LoadFpLiteral => {
            let _ = write!(
                ops,
                "{}, {}",
                vreg_scale(d, isa::fp_opc_scale(word)),
                target(pc, isa::imm19(word))
            );
        }
        Fmt::LdStFpUImm
        | Fmt::LdStFpUnscaled
        | Fmt::LdStFpPost
        | Fmt::LdStFpPre
        | Fmt::LdStFpRegOff => {
            let scale = isa::fp_ls_scale(word);
            let t = vreg_scale(d, scale);
            let base = reg(n, 64, rn_sp);
            match insn.fmt {
                Fmt::LdStFpUImm => {
                    let offset = u64::from(isa::imm12(word)) << scale.unwrap_or(0);
                    if offset == 0 {
                        let _ = write!(ops, "{t}, [{base}]");
                    } else {
                        let _ = write!(ops, "{t}, [{base}, #0x{offset:x}]");
                    }
                }
                Fmt::LdStFpUnscaled => {
                    let offset = isa::imm9(word);
                    if offset == 0 {
                        let _ = write!(ops, "{t}, [{base}]");
                    } else {
                        let _ = write!(ops, "{t}, [{base}, {}]", imm(offset));
                    }
                }
                Fmt::LdStFpPost => {
                    let _ = write!(ops, "{t}, [{base}], {}", imm(isa::imm9(word)));
                }
                Fmt::LdStFpPre => {
                    let _ = write!(ops, "{t}, [{base}, {}]!", imm(isa::imm9(word)));
                }
                _ => {
                    let option = isa::extend_option(word);
                    let amount = if isa::bit(word, 12) {
                        scale.unwrap_or(0)
                    } else {
                        0
                    };
                    let index_width = if option & 1 == 0 { 32 } else { 64 };
                    let _ = write!(
                        ops,
                        "{t}, [{base}, {}{}]",
                        reg(m, index_width, false),
                        extend_tail(word, amount, 3)
                    );
                }
            }
        }
        Fmt::LdStFpPairOff | Fmt::LdStFpPairPost | Fmt::LdStFpPairPre => {
            let scale = isa::fp_opc_scale(word);
            let offset = isa::imm7(word) << scale.unwrap_or(0);
            let base = reg(n, 64, rn_sp);
            let pair = format!(
                "{}, {}",
                vreg_scale(d, scale),
                vreg_scale(isa::ra(word), scale)
            );
            match insn.fmt {
                Fmt::LdStFpPairOff if offset == 0 => {
                    let _ = write!(ops, "{pair}, [{base}]");
                }
                Fmt::LdStFpPairOff => {
                    let _ = write!(ops, "{pair}, [{base}, {}]", imm(offset));
                }
                Fmt::LdStFpPairPost => {
                    let _ = write!(ops, "{pair}, [{base}], {}", imm(offset));
                }
                _ => {
                    let _ = write!(ops, "{pair}, [{base}, {}]!", imm(offset));
                }
            }
        }

        // -- Advanced SIMD ------------------------------------------------
        //
        // A vector operand prints its *arrangement*, and the arrangement is
        // not always in the same field — so each arm asks `isa` for the field
        // its encoding group uses and `simd` for the letters, which is the
        // same split the interpreter makes.
        Fmt::VecModImm => {
            let esize = match insn.op {
                isa::Op::MoviByte => 0,
                isa::Op::MoviShiftH
                | isa::Op::MvniShiftH
                | isa::Op::OrrVecImmH
                | isa::Op::BicVecImmH => 1,
                isa::Op::MoviWide | isa::Op::FmovVecD => 3,
                _ => 2,
            };
            let cmode = isa::simd_cmode(word);
            let imm8 = isa::simd_imm8(word);
            // `MOVI Dd, #imm` is the one modified-immediate form with a
            // scalar destination — and it is the encoding LLVM reaches for to
            // materialise a floating-point zero.
            let dest = if insn.op == isa::Op::MoviWide && !isa::q(word) {
                format!("d{d}")
            } else {
                varr(d, simd::Arrangement::from_size(esize, isa::q(word)))
            };
            // Three of the fourteen forms print the *expanded* value rather
            // than the eight bits: the two `FMOV`s, whose immediate is a
            // floating-point number the encoding compresses, and the 64-bit
            // `MOVI`, whose eight bits are a bytemask. The other eleven print
            // the byte and its shift, which is what an assembler takes back.
            let expanded = matches!(
                insn.op,
                isa::Op::FmovVecS | isa::Op::FmovVecD | isa::Op::MoviWide
            );
            if expanded {
                // In hex rather than as a decimal: a float here would need a
                // formatter this crate has no host floating point to write.
                let whole = simd::expand_imm(isa::bit(word, 29), cmode, imm8).unwrap_or(0);
                let value = if esize >= 3 {
                    whole
                } else {
                    whole & u64::from(u32::MAX)
                };
                let _ = write!(ops, "{dest}, #0x{value:x}");
            } else {
                let _ = write!(ops, "{dest}, #0x{imm8:x}{}", mod_imm_shift(cmode));
            }
        }
        Fmt::VecDupElem => {
            let (esize, index) = lane_of(word);
            let _ = write!(
                ops,
                "{}, {}",
                varr(d, simd::Arrangement::from_size(esize, isa::q(word))),
                vlane(n, esize, index)
            );
        }
        Fmt::VecDupGen => {
            let (esize, _) = lane_of(word);
            let _ = write!(
                ops,
                "{}, {}",
                varr(d, simd::Arrangement::from_size(esize, isa::q(word))),
                reg(n, if esize == 3 { 64 } else { 32 }, false)
            );
        }
        Fmt::VecToGp => {
            let (esize, index) = lane_of(word);
            let _ = write!(
                ops,
                "{}, {}",
                reg(d, if isa::q(word) { 64 } else { 32 }, false),
                vlane(n, esize, index)
            );
        }
        Fmt::VecInsGen => {
            let (esize, index) = lane_of(word);
            let _ = write!(
                ops,
                "{}, {}",
                vlane(d, esize, index),
                reg(n, if esize == 3 { 64 } else { 32 }, false)
            );
        }
        Fmt::VecInsElem => {
            let (esize, index) = lane_of(word);
            let _ = write!(
                ops,
                "{}, {}",
                vlane(d, esize, index),
                vlane(n, esize, isa::simd_imm4(word) >> esize)
            );
        }
        Fmt::VecThreeSame | Fmt::VecThreeSameFp | Fmt::VecThreeSameLog => {
            let arr = three_same_arr(word, insn.fmt);
            let _ = write!(ops, "{}, {}, {}", varr(d, arr), varr(n, arr), varr(m, arr));
        }
        Fmt::VecTwoMisc | Fmt::VecTwoMiscFp => {
            let arr = two_misc_arr(word, insn.fmt, insn.op);
            let _ = write!(ops, "{}, {}", varr(d, arr), varr(n, arr));
        }
        Fmt::VecCmpZero => {
            let arr = simd::Arrangement::from_size(isa::simd_size(word), isa::q(word));
            let _ = write!(ops, "{}, {}, #0", varr(d, arr), varr(n, arr));
        }
        Fmt::VecCmpZeroFp => {
            let arr = simd::Arrangement::from_sz(isa::simd_sz(word), isa::q(word));
            let _ = write!(ops, "{}, {}, #0.0", varr(d, arr), varr(n, arr));
        }
        Fmt::VecNarrow => {
            // The integer narrows — `XTN` and the three saturating ones —
            // take the destination width from `size`, and `FCVTN` from `sz`,
            // one field lower. The two groups share an encoding but not a
            // field, which is why the destination is computed here and not in
            // a shared helper.
            let dst = if insn.op == isa::Op::FcvtnVec {
                1 + u32::from(isa::simd_sz(word))
            } else {
                isa::simd_size(word)
            };
            let lanes = 64 / (8 << dst);
            let _ = write!(
                ops,
                "{}, {}",
                varr(
                    d,
                    simd::Arrangement::from_size(dst, isa::q(word)).map(|_| {
                        simd::Arrangement {
                            esize: dst,
                            lanes: lanes * (1 + u32::from(isa::q(word))),
                        }
                    })
                ),
                varr(
                    n,
                    Some(simd::Arrangement {
                        esize: dst + 1,
                        lanes,
                    })
                )
            );
        }
        Fmt::VecWiden => {
            let src = 1 + u32::from(isa::simd_sz(word));
            let lanes = 64 / (8 << src);
            let _ = write!(
                ops,
                "{}, {}",
                varr(
                    d,
                    Some(simd::Arrangement {
                        esize: src + 1,
                        lanes,
                    })
                ),
                varr(
                    n,
                    Some(simd::Arrangement {
                        esize: src,
                        lanes: lanes * (1 + u32::from(isa::q(word))),
                    })
                )
            );
        }
        Fmt::VecAcross | Fmt::VecAcrossFp => {
            let (arr, dst) = if insn.fmt == Fmt::VecAcrossFp {
                (
                    simd::Arrangement::from_sz(isa::simd_sz(word), isa::q(word)),
                    2,
                )
            } else {
                let size = isa::simd_size(word);
                let widening = matches!(insn.op, isa::Op::SaddlvVec | isa::Op::UaddlvVec);
                (
                    simd::Arrangement::from_size(size, isa::q(word)),
                    size + u32::from(widening),
                )
            };
            let _ = write!(ops, "{}{d}, {}", simd::elem_letter(dst), varr(n, arr));
        }
        Fmt::VecExt => {
            let arr = byte_arr(isa::q(word));
            let _ = write!(
                ops,
                "{}, {}, {}, #0x{:x}",
                varr(d, arr),
                varr(n, arr),
                varr(m, arr),
                isa::simd_imm4(word)
            );
        }
        Fmt::VecTable => {
            let arr = byte_arr(isa::q(word));
            let mut table = String::from("{ ");
            for i in 0..=isa::field(word, 14, 13) {
                if i > 0 {
                    table.push_str(", ");
                }
                let _ = write!(table, "v{}.16b", (n + i) % 32);
            }
            table.push_str(" }");
            let _ = write!(ops, "{}, {table}, {}", varr(d, arr), varr(m, arr));
        }
        Fmt::VecShiftImm => {
            let (esize, immhb) = shift_of(word);
            // The fixed-point conversions exist only at the two widths that
            // are floating-point formats; `immh` naming a byte or a halfword
            // is reserved there even though it is a shift everywhere else.
            let fixed = matches!(
                insn.op,
                isa::Op::ScvtfFixVec
                    | isa::Op::UcvtfFixVec
                    | isa::Op::FcvtzsFixVec
                    | isa::Op::FcvtzuFixVec
            );
            let arr = if fixed && esize < 2 {
                None
            } else {
                simd::Arrangement::from_size(esize, isa::q(word))
            };
            let bits = 8 << esize;
            let amount = if left_shift(insn.op) {
                immhb.wrapping_sub(bits)
            } else {
                (2 * bits).wrapping_sub(immhb)
            };
            let _ = write!(ops, "{}, {}, #{amount}", varr(d, arr), varr(n, arr));
        }
        Fmt::VecShiftLong => {
            let (esize, immhb) = shift_of(word);
            let lanes = 64 / (8 << esize);
            let _ = write!(
                ops,
                "{}, {}, #{}",
                varr(
                    d,
                    Some(simd::Arrangement {
                        esize: esize + 1,
                        lanes
                    })
                ),
                varr(
                    n,
                    Some(simd::Arrangement {
                        esize,
                        lanes: lanes * (1 + u32::from(isa::q(word)))
                    })
                ),
                immhb.wrapping_sub(8 << esize)
            );
        }
        Fmt::VecShiftNarrow => {
            let (esize, immhb) = shift_of(word);
            let lanes = 64 / (8 << esize);
            let _ = write!(
                ops,
                "{}, {}, #{}",
                varr(
                    d,
                    Some(simd::Arrangement {
                        esize,
                        lanes: lanes * (1 + u32::from(isa::q(word)))
                    })
                ),
                varr(
                    n,
                    Some(simd::Arrangement {
                        esize: esize + 1,
                        lanes
                    })
                ),
                (16u32 << esize).wrapping_sub(immhb)
            );
        }
        Fmt::VecThreeDiff | Fmt::VecThreeWide => {
            let src = isa::simd_size(word);
            let lanes = 64 / (8 << src);
            let wide = simd::Arrangement {
                esize: src + 1,
                lanes,
            };
            let narrow = simd::Arrangement {
                esize: src,
                lanes: lanes * (1 + u32::from(isa::q(word))),
            };
            let first = if insn.fmt == Fmt::VecThreeWide {
                wide
            } else {
                narrow
            };
            let _ = write!(
                ops,
                "{}, {}, {}",
                varr(d, Some(wide)),
                varr(n, Some(first)),
                varr(m, Some(narrow))
            );
        }
        Fmt::VecByElem => {
            let floating = matches!(
                insn.op,
                isa::Op::FmulElem | isa::Op::FmlaElem | isa::Op::FmlsElem
            );
            let l = u32::from(isa::bit(word, 21));
            let h = u32::from(isa::bit(word, 11));
            let m_bit = u32::from(isa::bit(word, 20));
            let (arr, esize, index, source) = if floating {
                let arr = simd::Arrangement::from_sz(isa::simd_sz(word), isa::q(word));
                let esize = 2 + u32::from(isa::simd_sz(word));
                let index = if isa::simd_sz(word) { h } else { (h << 1) | l };
                (arr, esize, index, m)
            } else {
                let size = isa::simd_size(word);
                // Only a halfword or a word element has an indexed form; the
                // other two `size` values are reserved, so there is no
                // arrangement to print.
                let arr = if matches!(size, 1 | 2) {
                    simd::Arrangement::from_size(size, isa::q(word))
                } else {
                    None
                };
                if size == 1 {
                    (
                        arr,
                        size,
                        (h << 2) | (l << 1) | m_bit,
                        isa::field(word, 19, 16),
                    )
                } else {
                    (arr, size, (h << 1) | l, m)
                }
            };
            let _ = write!(
                ops,
                "{}, {}, {}",
                varr(d, arr),
                varr(n, arr),
                vlane(source, esize, index)
            );
        }
        Fmt::SimdScalarTwo | Fmt::SimdScalarThree | Fmt::SimdScalarCmpZero => {
            let letter = scalar_letter(word, insn.op);
            if insn.op == isa::Op::DupElemScalar {
                let (esize, index) = lane_of(word);
                let _ = write!(
                    ops,
                    "{}{d}, {}",
                    simd::elem_letter(esize),
                    vlane(n, esize, index)
                );
            } else if insn.fmt == Fmt::SimdScalarThree {
                let _ = write!(ops, "{letter}{d}, {letter}{n}, {letter}{m}");
            } else if insn.fmt == Fmt::SimdScalarCmpZero {
                let zero = if letter == 'd' && !fp_scalar(insn.op) {
                    "#0"
                } else {
                    "#0.0"
                };
                let _ = write!(ops, "{letter}{d}, {letter}{n}, {zero}");
            } else {
                let _ = write!(ops, "{letter}{d}, {letter}{n}");
            }
        }
        Fmt::SimdScalarThreeSz | Fmt::SimdScalarTwoSz => {
            let letter = simd::elem_letter(isa::simd_size(word));
            if insn.fmt == Fmt::SimdScalarThreeSz {
                let _ = write!(ops, "{letter}{d}, {letter}{n}, {letter}{m}");
            } else {
                let _ = write!(ops, "{letter}{d}, {letter}{n}");
            }
        }
        Fmt::SimdScalarNarrow => {
            let dst = isa::simd_size(word);
            let _ = write!(
                ops,
                "{}{d}, {}{n}",
                simd::elem_letter(dst),
                simd::elem_letter(dst + 1)
            );
        }
        Fmt::SimdScalarDiff => {
            let src = isa::simd_size(word);
            let _ = write!(
                ops,
                "{}{d}, {}{n}, {}{m}",
                simd::elem_letter(src + 1),
                simd::elem_letter(src),
                simd::elem_letter(src)
            );
        }
        Fmt::SimdScalarShift => {
            let (esize, immhb) = shift_of(word);
            let letter = simd::elem_letter(esize);
            let _ = write!(
                ops,
                "{letter}{d}, {letter}{n}, #{}",
                immhb.wrapping_sub(8 << esize)
            );
        }
        Fmt::SimdScalarShiftNarrow => {
            let (esize, immhb) = shift_of(word);
            let _ = write!(
                ops,
                "{}{d}, {}{n}, #{}",
                simd::elem_letter(esize),
                simd::elem_letter(esize + 1),
                (16u32 << esize).wrapping_sub(immhb)
            );
        }
        Fmt::SimdScalarPair => {
            let letter = scalar_letter(word, insn.op);
            let esize = if insn.op == isa::Op::AddpScalar {
                3
            } else {
                2 + u32::from(isa::simd_sz(word))
            };
            let _ = write!(
                ops,
                "{letter}{d}, {}",
                varr(n, Some(simd::Arrangement { esize, lanes: 2 }))
            );
        }
        Fmt::LdStStruct
        | Fmt::LdStStructPost
        | Fmt::LdStStructSingle
        | Fmt::LdStStructSinglePost => {
            let base = reg(n, 64, rn_sp);
            let single = matches!(insn.fmt, Fmt::LdStStructSingle | Fmt::LdStStructSinglePost);
            let list = if single {
                struct_single_list(word, d)
            } else {
                struct_multi_list(word, d)
            };
            let _ = write!(ops, "{list}, [{base}]");
            if matches!(insn.fmt, Fmt::LdStStructPost | Fmt::LdStStructSinglePost) {
                if m == 31 {
                    // The immediate is always the transfer size, so the
                    // disassembler prints what the instruction will actually
                    // add rather than a field it does not have.
                    let _ = write!(ops, ", #0x{:x}", struct_bytes(word, single));
                } else {
                    let _ = write!(ops, ", {}", reg(m, 64, false));
                }
            }
        }
    }

    if !ops.is_empty() {
        text.push('\t');
        text.push_str(&ops);
    }
    Disassembled {
        addr: pc,
        encoding: word,
        text,
        hole: None,
    }
}

/// Disassemble `count` instructions starting at `pc`, reading words through
/// `read`.
///
/// A word that cannot be read becomes a hole and the listing carries on, so
/// the count is always what was asked for.
pub fn disassemble_run<F>(
    pc: u64,
    count: usize,
    features: isa::Features,
    mut read: F,
) -> Vec<Disassembled>
where
    F: FnMut(u64) -> Result<u32, Missing>,
{
    let mut out = Vec::with_capacity(count);
    let mut at = pc;
    for _ in 0..count {
        match read(at) {
            Ok(word) => out.push(disassemble(word, at, features)),
            Err(why) => out.push(Disassembled::missing(at, why)),
        }
        at = at.wrapping_add(Disassembled::LEN);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn text(word: u32, pc: u64) -> String {
        disassemble(word, pc, isa::Features::ALL).text
    }

    #[test]
    fn common_encodings_print_as_the_manual_spells_them() {
        assert_eq!(text(0xd503201f, 0), "nop");
        assert_eq!(text(0xd65f03c0, 0), "ret\tx30");
        assert_eq!(text(0x14000004, 0x1000), "b\t0x1010");
        assert_eq!(text(0x54000040, 0x1000), "b.eq\t0x1008");
        // `mov x29, sp` really is `add x29, sp, #0`, and the disassembler
        // prints the encoding rather than the alias.
        assert_eq!(text(0x910003fd, 0), "add\tx29, sp, #0x0");
        assert_eq!(text(0xf9400000, 0), "ldr\tx0, [x0]");
        assert_eq!(text(0xa9bf7bfd, 0), "stp\tx29, x30, [sp, #-0x10]!");
        assert_eq!(text(0xa8c17bfd, 0), "ldp\tx29, x30, [sp], #0x10");
        assert_eq!(text(0xd2800540, 0), "movz\tx0, #0x2a");
        assert_eq!(text(0xd4000001, 0), "svc\t#0x0");
        assert_eq!(text(0x8b010000, 0), "add\tx0, x0, x1");
        assert_eq!(text(0x9ac10800, 0), "udiv\tx0, x0, x1");
    }

    /// The `SP`/`XZR` distinction, which is the one the format owns.
    #[test]
    fn register_31_prints_as_sp_or_zr_by_format() {
        // `add x0, sp, #0`: register 31 in `Rn` of an add-immediate is `SP`.
        assert_eq!(text(0x910003e0, 0), "add\tx0, sp, #0x0");
        // `adds x0, x1, #0` with Rd = 31 is `cmn`, whose destination is `XZR`.
        assert_eq!(text(0x3100003f, 0), "adds\twzr, w1, #0x0");
        // `orr x0, xzr, x1` — the logical shifted-register `Rn` is never `SP`.
        assert_eq!(text(0xaa0103e0, 0), "orr\tx0, xzr, x1");
    }

    #[test]
    fn system_registers_print_by_name() {
        assert_eq!(text(0xd5381000, 0), "mrs\tx0, sctlr_el1");
        assert_eq!(text(0xd5182000, 0), "msr\tttbr0_el1, x0");
        // An encoding with no row prints its raw fields rather than lying.
        assert!(text(0xd53f0000, 0).starts_with("mrs\tx0, s"));
    }

    #[test]
    fn atomics_carry_their_ordering_suffix() {
        // `ldaddal x1, x2, [x3]`: A at bit 23 and R at bit 22.
        assert_eq!(text(0xf8e10062, 0), "ldaddal\tx1, x2, [x3]");
        assert_eq!(text(0xf8610062, 0), "ldaddl\tx1, x2, [x3]");
        assert_eq!(text(0xf8a10062, 0), "ldadda\tx1, x2, [x3]");
        assert_eq!(text(0xf8210062, 0), "ldadd\tx1, x2, [x3]");
        assert_eq!(text(0xc8a17c62, 0), "cas\tx1, x2, [x3]");
    }

    #[test]
    fn an_undecodable_word_prints_as_data() {
        assert_eq!(text(0x0000_0000, 0), ".word\t0x00000000");
    }

    /// A feature the core does not have is not an instruction on that core,
    /// and the disassembler says so rather than decoding it anyway.
    #[test]
    fn a_missing_feature_is_not_disassembled() {
        let d = disassemble(0xc8a17c62, 0, isa::Features::NONE);
        assert!(d.text.starts_with(".word"));
    }

    #[test]
    fn a_hole_does_not_shorten_a_listing() {
        let run = disassemble_run(0x1000, 3, isa::Features::ALL, |addr| {
            if addr == 0x1004 {
                Err(Missing::Unmapped)
            } else {
                Ok(0xd503201f)
            }
        });
        assert_eq!(run.len(), 3);
        assert_eq!(run[1].hole, Some(Missing::Unmapped));
        assert_eq!(run[2].addr, 0x1008);
    }
}
