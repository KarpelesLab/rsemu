//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! CLAUDE.md is explicit that the disassembler is not a side project: gdb and
//! the monitor both need it, and writing the instruction set out twice is how
//! the two drift. So nothing here knows an opcode: every mnemonic comes from
//! [`isa::Op::mnemonic`] and every operand layout from [`isa::Fmt`]. Adding an
//! instruction to [`isa::TABLE`] makes it disassemble with no edit here.
//!
//! Branch and jump targets are resolved to absolute addresses, because that is
//! what a listing is for. Note that a MIPS displacement is relative to the
//! **delay slot** rather than to the branch, so the arithmetic here is not
//! `pc + imm * 4` and a reader who assumes it is will be one word out.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

use super::cp0;
use super::isa::{self, Fmt, Op, REG_NAMES};

/// One disassembled instruction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Disassembled {
    /// Where it starts.
    pub addr: u64,
    /// How many bytes it occupies. Always four — every MIPS I instruction is
    /// one word — and present so a caller can walk a listing the same way it
    /// walks a variable-length one.
    pub len: u64,
    /// The raw encoding.
    pub encoding: u32,
    /// The assembly text.
    pub text: String,
    /// Whether the instruction that follows is in this one's **delay slot**.
    ///
    /// A listing that does not say so is misleading: the next line executes
    /// before control transfers, which is the opposite of what a reader used
    /// to any other architecture will assume.
    pub delay_slot: bool,
}

impl fmt::Display for Disassembled {
    /// The one-line form a monitor listing wants.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:08x}: {:08x}  {}", self.addr, self.encoding, self.text)
    }
}

/// A general register's ABI name.
fn r(i: u32) -> &'static str {
    REG_NAMES[(i & 31) as usize]
}

/// A signed immediate, in the form an assembler would accept.
fn simm(word: u32) -> String {
    let v = (word & 0xffff) as u16 as i16;
    if v.unsigned_abs() < 10 {
        format!("{v}")
    } else if v < 0 {
        format!("-0x{:x}", v.unsigned_abs())
    } else {
        format!("0x{v:x}")
    }
}

/// A CP0 register, by name where it has one.
fn cp0_reg(n: u32) -> String {
    match cp0::reg::name(n) {
        Some(name) => format!("${name}"),
        None => format!("$c0_{n}"),
    }
}

/// Format one instruction word.
///
/// `pc` is the instruction's own address, which is what branch and jump
/// targets are resolved against.
#[must_use]
pub fn format_word(word: u32, pc: u32) -> String {
    // The canonical `nop`. Every MIPS assembler emits `sll $zero, $zero, 0`
    // for it, and printing the long form would make a listing unreadable.
    if word == 0 {
        return "nop".to_string();
    }
    let Some(insn) = isa::decode(word) else {
        return format!(".word 0x{word:08x}");
    };
    let m = insn.op.mnemonic();
    let rs = isa::rs(word);
    let rt = isa::rt(word);
    let rd = isa::rd(word);
    let delay_pc = pc.wrapping_add(4);

    match insn.fmt {
        Fmt::R => format!("{m} {}, {}, {}", r(rd), r(rs), r(rt)),
        Fmt::Shift => format!("{m} {}, {}, {}", r(rd), r(rt), isa::sa(word)),
        Fmt::ShiftV => format!("{m} {}, {}, {}", r(rd), r(rt), r(rs)),
        Fmt::I => {
            // The logical immediates are zero-extended and read better in hex;
            // the arithmetic ones are sign-extended and read better signed.
            match insn.op {
                Op::Andi | Op::Ori | Op::Xori => {
                    format!("{m} {}, {}, 0x{:x}", r(rt), r(rs), isa::imm(word))
                }
                _ => format!("{m} {}, {}, {}", r(rt), r(rs), simm(word)),
            }
        }
        Fmt::Mem => format!("{m} {}, {}({})", r(rt), simm(word), r(rs)),
        Fmt::Lui => format!("{m} {}, 0x{:x}", r(rt), isa::imm(word)),
        Fmt::Branch => format!(
            "{m} {}, {}, 0x{:08x}",
            r(rs),
            r(rt),
            isa::branch_target(delay_pc, word)
        ),
        Fmt::BranchZ => format!(
            "{m} {}, 0x{:08x}",
            r(rs),
            isa::branch_target(delay_pc, word)
        ),
        Fmt::Jump => format!("{m} 0x{:08x}", isa::jump_target(delay_pc, word)),
        Fmt::Rs | Fmt::MoveTo => format!("{m} {}", r(rs)),
        Fmt::JumpLink => {
            // `jalr $ra, rs` is the common form and the assembler lets it be
            // written with one operand, so print it that way. `$zero` is *not*
            // the same thing — it discards the link — so it prints in full.
            if rd == 31 {
                format!("{m} {}", r(rs))
            } else {
                format!("{m} {}, {}", r(rd), r(rs))
            }
        }
        Fmt::Rd => format!("{m} {}", r(rd)),
        Fmt::HiLo => format!("{m} {}, {}", r(rs), r(rt)),
        Fmt::Code => {
            let code = isa::code(word);
            if code == 0 {
                m.to_string()
            } else {
                format!("{m} 0x{code:x}")
            }
        }
        Fmt::Cop0Move => format!("{m} {}, {}", r(rt), cp0_reg(rd)),
        Fmt::None => m.to_string(),
        Fmt::CopFun => format!("{m} 0x{:07x}", isa::cofun(word)),
        Fmt::CopMem => format!("{m} $c{rt}, {}({})", simm(word), r(rs)),
    }
}

/// Disassemble one instruction, reading its word through `read`.
///
/// `read` returns `None` for an address that cannot be read at all, which is
/// what a listing that walks off the end of a mapped region gets.
#[must_use]
pub fn disassemble_one(pc: u32, read: &mut impl FnMut(u64) -> Option<u32>) -> Option<Disassembled> {
    let word = read(u64::from(pc))?;
    Some(Disassembled {
        addr: u64::from(pc),
        len: 4,
        encoding: word,
        text: format_word(word, pc),
        delay_slot: isa::decode(word).is_some_and(|i| i.is_branch()),
    })
}

/// Disassemble a run of instructions.
#[must_use]
pub fn disassemble_run(
    pc: u32,
    count: usize,
    mut read: impl FnMut(u64) -> Option<u32>,
) -> Vec<Disassembled> {
    let mut out = Vec::with_capacity(count);
    let mut at = pc;
    for _ in 0..count {
        let Some(one) = disassemble_one(at, &mut read) else {
            break;
        };
        at = at.wrapping_add(4);
        out.push(one);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_zero_word_is_a_nop() {
        assert_eq!(format_word(0, 0x1000), "nop");
    }

    #[test]
    fn the_common_forms_read_the_way_an_assembler_writes_them() {
        // addu $v0, $a0, $a1
        assert_eq!(
            format_word(0x0085_1021, 0),
            "addu v0, a0, a1",
            "{:08x}",
            0x0085_1021u32
        );
        // addiu $sp, $sp, -32
        assert_eq!(format_word(0x27bd_ffe0, 0), "addiu sp, sp, -0x20");
        // lw $ra, 28($sp)
        assert_eq!(format_word(0x8fbf_001c, 0), "lw ra, 0x1c(sp)");
        // sw $s0, 16($sp)
        assert_eq!(format_word(0xafb0_0010, 0), "sw s0, 0x10(sp)");
        // lui $at, 0x8000
        assert_eq!(format_word(0x3c01_8000, 0), "lui at, 0x8000");
        // ori $a0, $zero, 0x1234
        assert_eq!(format_word(0x3404_1234, 0), "ori a0, zero, 0x1234");
        // sll $t0, $t1, 3
        assert_eq!(format_word(0x0009_40c0, 0), "sll t0, t1, 3");
    }

    #[test]
    fn a_branch_target_is_relative_to_the_delay_slot() {
        // `beq $zero, $zero, -1` at 0x1000. The displacement counts from the
        // delay slot at 0x1004, so the target is the branch itself.
        assert_eq!(
            format_word(0x1000_ffff, 0x1000),
            "beq zero, zero, 0x00001000"
        );
        // Forward by two words from 0x2000: 0x2004 + 8 = 0x200c.
        assert_eq!(
            format_word(0x1000_0002, 0x2000),
            "beq zero, zero, 0x0000200c"
        );
    }

    #[test]
    fn a_jump_target_takes_its_high_bits_from_the_delay_slot() {
        // `j 0x0400` from 0x0fff_fffc: the delay slot is in the next 256 MB
        // region, so the target is 0x1000_0400 rather than 0x0000_0400.
        assert_eq!(
            format_word(0x0800_0100, 0x0fff_fffc),
            "j 0x10000400",
            "the region comes from the delay slot"
        );
    }

    #[test]
    fn a_branch_marks_the_line_after_it_as_a_delay_slot() {
        let program = [0x1000_ffffu32, 0x0000_0000, 0x2402_0001];
        let out = disassemble_run(0x1000, 3, |addr| {
            let i = ((addr - 0x1000) / 4) as usize;
            program.get(i).copied()
        });
        assert_eq!(out.len(), 3);
        assert!(out[0].delay_slot, "beq has a delay slot");
        assert!(!out[1].delay_slot, "nop does not");
        assert!(!out[2].delay_slot);
        assert_eq!(out[1].addr, 0x1004);
    }

    #[test]
    fn cop0_registers_print_by_name() {
        // mfc0 $k0, $sr  — 0x4000_0000 | (26 << 16) | (12 << 11)
        let word = 0x4000_0000 | (26 << 16) | (12 << 11);
        assert_eq!(format_word(word, 0), "mfc0 k0, $sr");
        // mtc0 $k0, $c0_20 — a register an R3000 does not have.
        let word = 0x4080_0000 | (26 << 16) | (20 << 11);
        assert_eq!(format_word(word, 0), "mtc0 k0, $c0_20");
    }

    #[test]
    fn the_processor_control_instructions_print_bare() {
        assert_eq!(format_word(0x4200_0010, 0), "rfe");
        assert_eq!(format_word(0x4200_0002, 0), "tlbwi");
        assert_eq!(format_word(0x0000_000c, 0), "syscall");
        assert_eq!(format_word(0x0004_100d, 0), "break 0x1040");
    }

    #[test]
    fn an_unknown_encoding_prints_as_a_word_rather_than_a_guess() {
        // Primary opcode 0x1f is not assigned on MIPS I.
        assert_eq!(format_word(0x7c00_0000, 0), ".word 0x7c000000");
    }

    #[test]
    fn every_table_entry_disassembles_to_something_starting_with_its_mnemonic() {
        // The strongest thing the generated disassembler can assert about
        // itself: no row prints as `.word`, and none prints another row's
        // name.
        for insn in isa::TABLE {
            let text = format_word(insn.bits, 0x1000);
            if insn.bits == 0 {
                // `sll $zero, $zero, 0` really is `nop`.
                assert_eq!(text, "nop");
                continue;
            }
            assert!(
                text.starts_with(insn.op.mnemonic()),
                "{} printed as `{text}`",
                insn.op.mnemonic()
            );
        }
    }
}
