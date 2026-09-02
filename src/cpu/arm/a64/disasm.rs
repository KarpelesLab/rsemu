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
use super::sysreg;

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
        Fmt::LdStUImm | Fmt::LdStUnscaled | Fmt::LdStPost | Fmt::LdStPre | Fmt::LdStRegOff => {
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
                Fmt::LdStUnscaled => {
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
