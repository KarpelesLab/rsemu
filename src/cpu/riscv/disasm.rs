//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! CLAUDE.md is explicit that the disassembler is not a side project: gdb and
//! the monitor both need it, and writing the instruction set out twice is how
//! the two drift. So nothing here knows an opcode: every mnemonic comes from
//! [`isa::Op::mnemonic`] and every operand layout from [`isa::Fmt`]. Adding an
//! instruction to [`isa::TABLE`] makes it disassemble with no edit here.
//!
//! Compressed instructions print their own mnemonic — a reader wants to see
//! that the encoding was 16 bits — followed by the operands of the 32-bit
//! instruction they expand to, which is the same instruction spelled in full.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

use super::csr;
use super::isa::{self, Fmt, Op, Xlen};
use super::{F_NAMES, X_NAMES};

/// One disassembled instruction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Disassembled {
    /// Where it starts.
    pub addr: u64,
    /// How many bytes it occupies: 2 or 4.
    pub len: u64,
    /// The raw encoding, 16 or 32 bits wide as `len` says.
    pub encoding: u32,
    /// The assembly text.
    pub text: String,
}

impl fmt::Display for Disassembled {
    /// The one-line form a monitor listing wants.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.len == 2 {
            write!(
                f,
                "{:016x}:     {:04x}  {}",
                self.addr, self.encoding, self.text
            )
        } else {
            write!(
                f,
                "{:016x}: {:08x}  {}",
                self.addr, self.encoding, self.text
            )
        }
    }
}

/// An integer register's ABI name.
fn xn(i: u32) -> &'static str {
    X_NAMES[(i & 31) as usize]
}

/// A floating-point register's ABI name.
fn fnm(i: u32) -> &'static str {
    F_NAMES[(i & 31) as usize]
}

/// A signed immediate, in the form an assembler would accept.
fn imm(v: i64) -> String {
    if v.abs() < 10 {
        format!("{v}")
    } else if v < 0 {
        format!("-0x{:x}", v.unsigned_abs())
    } else {
        format!("0x{v:x}")
    }
}

/// The `iorw` letters of one half of a `FENCE`'s ordering set.
fn fence_set(bits: u32) -> String {
    let mut s = String::new();
    for (bit, letter) in [(8, 'i'), (4, 'o'), (2, 'r'), (1, 'w')] {
        if bits & bit != 0 {
            s.push(letter);
        }
    }
    if s.is_empty() {
        s.push_str("none");
    }
    s
}

/// The rounding-mode suffix, or nothing for the dynamic mode.
///
/// Printing `, rtz` only when the instruction actually names a static mode
/// keeps the common case readable and still round-trips the unusual one.
fn rm_suffix(word: u32) -> &'static str {
    match isa::funct3(word) {
        0 => ", rne",
        1 => ", rtz",
        2 => ", rdn",
        3 => ", rup",
        4 => ", rmm",
        7 => "",
        _ => ", <reserved>",
    }
}

/// The `.aq`/`.rl` suffix an atomic carries.
fn ordering_suffix(word: u32) -> &'static str {
    match (isa::aq(word), isa::rl(word)) {
        (false, false) => "",
        (true, false) => ".aq",
        (false, true) => ".rl",
        (true, true) => ".aqrl",
    }
}

/// Format one 32-bit instruction.
///
/// `pc` is used to resolve branch and jump targets to absolute addresses,
/// which is what a listing needs; `xlen` decides which encodings exist.
#[must_use]
pub fn format_word(word: u32, pc: u64, xlen: Xlen) -> String {
    let Some(insn) = isa::decode(word, xlen) else {
        return format!(".word 0x{word:08x}");
    };
    format_decoded(insn.op, insn.fmt, insn.op.mnemonic(), word, pc)
}

/// Format an instruction whose row is already known.
fn format_decoded(op: Op, fmt: Fmt, mnemonic: &str, word: u32, pc: u64) -> String {
    let rd = isa::rd(word);
    let rs1 = isa::rs1(word);
    let rs2 = isa::rs2(word);
    let rs3 = isa::rs3(word);
    match fmt {
        Fmt::R => format!("{mnemonic} {}, {}, {}", xn(rd), xn(rs1), xn(rs2)),
        Fmt::I => format!(
            "{mnemonic} {}, {}, {}",
            xn(rd),
            xn(rs1),
            imm(isa::imm_i(word))
        ),
        Fmt::Shift => format!("{mnemonic} {}, {}, {}", xn(rd), xn(rs1), isa::shamt(word)),
        Fmt::Load => format!(
            "{mnemonic} {}, {}({})",
            xn(rd),
            imm(isa::imm_i(word)),
            xn(rs1)
        ),
        Fmt::Store => format!(
            "{mnemonic} {}, {}({})",
            xn(rs2),
            imm(isa::imm_s(word)),
            xn(rs1)
        ),
        Fmt::Branch => format!(
            "{mnemonic} {}, {}, 0x{:x}",
            xn(rs1),
            xn(rs2),
            pc.wrapping_add(isa::imm_b(word) as u64)
        ),
        Fmt::U => format!("{mnemonic} {}, 0x{:x}", xn(rd), isa::imm_u(word) as u64),
        Fmt::Jump => format!(
            "{mnemonic} {}, 0x{:x}",
            xn(rd),
            pc.wrapping_add(isa::imm_j(word) as u64)
        ),
        Fmt::Fence => format!(
            "{mnemonic} {}, {}",
            fence_set((word >> 24) & 0xf),
            fence_set((word >> 20) & 0xf)
        ),
        Fmt::None => mnemonic.to_string(),
        Fmt::Sfence => format!("{mnemonic} {}, {}", xn(rs1), xn(rs2)),
        Fmt::Csr => format!(
            "{mnemonic} {}, {}, {}",
            xn(rd),
            csr_operand(isa::csr(word)),
            xn(rs1)
        ),
        Fmt::CsrImm => format!(
            "{mnemonic} {}, {}, {}",
            xn(rd),
            csr_operand(isa::csr(word)),
            rs1
        ),
        Fmt::AmoLoad => format!(
            "{mnemonic}{} {}, ({})",
            ordering_suffix(word),
            xn(rd),
            xn(rs1)
        ),
        Fmt::Amo => format!(
            "{mnemonic}{} {}, {}, ({})",
            ordering_suffix(word),
            xn(rd),
            xn(rs2),
            xn(rs1)
        ),
        Fmt::FpLoad => format!(
            "{mnemonic} {}, {}({})",
            fnm(rd),
            imm(isa::imm_i(word)),
            xn(rs1)
        ),
        Fmt::FpStore => format!(
            "{mnemonic} {}, {}({})",
            fnm(rs2),
            imm(isa::imm_s(word)),
            xn(rs1)
        ),
        Fmt::FpR => {
            // The sign-injection, minimum and maximum instructions use funct3
            // as an opcode field rather than as a rounding mode, so they must
            // not print one.
            let suffix = if fp_has_rounding(op) {
                rm_suffix(word)
            } else {
                ""
            };
            format!("{mnemonic} {}, {}, {}{suffix}", fnm(rd), fnm(rs1), fnm(rs2))
        }
        Fmt::FpUnary => format!("{mnemonic} {}, {}{}", fnm(rd), fnm(rs1), rm_suffix(word)),
        Fmt::FpR4 => format!(
            "{mnemonic} {}, {}, {}, {}{}",
            fnm(rd),
            fnm(rs1),
            fnm(rs2),
            fnm(rs3),
            rm_suffix(word)
        ),
        Fmt::FpCmp => format!("{mnemonic} {}, {}, {}", xn(rd), fnm(rs1), fnm(rs2)),
        Fmt::FpToInt => {
            let suffix = if fp_has_rounding(op) {
                rm_suffix(word)
            } else {
                ""
            };
            format!("{mnemonic} {}, {}{suffix}", xn(rd), fnm(rs1))
        }
        Fmt::FpFromInt => {
            let suffix = if fp_has_rounding(op) {
                rm_suffix(word)
            } else {
                ""
            };
            format!("{mnemonic} {}, {}{suffix}", fnm(rd), xn(rs1))
        }
    }
}

/// Whether an FP instruction's `funct3` field is a rounding mode.
///
/// The moves, the classifications and the sign injections use it as an opcode
/// field instead, and printing a rounding mode for those would be a lie.
fn fp_has_rounding(op: Op) -> bool {
    !matches!(
        op,
        Op::FmvXW
            | Op::FmvWX
            | Op::FmvXD
            | Op::FmvDX
            | Op::FclassS
            | Op::FclassD
            | Op::FsgnjS
            | Op::FsgnjnS
            | Op::FsgnjxS
            | Op::FsgnjD
            | Op::FsgnjnD
            | Op::FsgnjxD
            | Op::FminS
            | Op::FmaxS
            | Op::FminD
            | Op::FmaxD
    )
}

/// A CSR's name, or its number when it has none this build knows.
fn csr_operand(number: u32) -> String {
    csr::csr_name(number).map_or_else(|| format!("0x{number:x}"), ToString::to_string)
}

/// Disassemble one instruction from a halfword reader.
///
/// Returns `None` when the first halfword cannot be read at all — an unmapped
/// address, which a listing should stop at rather than fill with noise.
#[must_use]
pub fn disassemble_one(
    addr: u64,
    xlen: Xlen,
    read: &mut impl FnMut(u64) -> Option<u16>,
) -> Option<Disassembled> {
    let low = read(addr)?;
    if !isa::is_32bit(low) {
        let text = match isa::decode_compressed(low, xlen) {
            Some(c) => match isa::expand(low, xlen) {
                Some(word) => {
                    // The compressed mnemonic with the expanded operands: the
                    // reader sees both that it was 16 bits and what it does.
                    match isa::decode(word, xlen) {
                        Some(insn) => {
                            format_decoded(insn.op, insn.fmt, c.op.mnemonic(), word, addr)
                        }
                        None => c.op.mnemonic().to_string(),
                    }
                }
                None => format!("{} <reserved>", c.op.mnemonic()),
            },
            None => format!(".half 0x{low:04x}"),
        };
        return Some(Disassembled {
            addr,
            len: 2,
            encoding: u32::from(low),
            text,
        });
    }
    let high = read(addr.wrapping_add(2))?;
    let word = u32::from(low) | (u32::from(high) << 16);
    Some(Disassembled {
        addr,
        len: 4,
        encoding: word,
        text: format_word(word, addr, xlen),
    })
}

/// Disassemble `count` instructions starting at `addr`.
///
/// Stops early if the reader refuses, which is what makes it safe to point at
/// the end of a mapping.
#[must_use]
pub fn disassemble_run(
    addr: u64,
    count: usize,
    xlen: Xlen,
    mut read: impl FnMut(u64) -> Option<u16>,
) -> Vec<Disassembled> {
    let mut out = Vec::with_capacity(count);
    let mut at = addr;
    for _ in 0..count {
        let Some(one) = disassemble_one(at, xlen, &mut read) else {
            break;
        };
        at = at.wrapping_add(one.len);
        out.push(one);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(word: u32) -> String {
        format_word(word, 0x8000_0000, Xlen::Rv64)
    }

    #[test]
    fn integer_instructions_print_abi_names() {
        assert_eq!(d(0x00c5_8533), "add a0, a1, a2");
        assert_eq!(d(0xffb5_0513), "addi a0, a0, -5");
        assert_eq!(d(0x0080_a503), "lw a0, 8(ra)");
        assert_eq!(d(0x00b1_2423), "sw a1, 8(sp)");
        assert_eq!(d(0x0000_0013), "addi zero, zero, 0");
    }

    #[test]
    fn branches_and_jumps_resolve_their_targets() {
        // beq a0, a1, +8 from 0x80000000.
        let word = 0x00b5_0463;
        assert_eq!(d(word), "beq a0, a1, 0x80000008");
        // jal ra, +0x10
        let word = 0x0100_00ef;
        assert_eq!(d(word), "jal ra, 0x80000010");
    }

    #[test]
    fn system_instructions_name_their_csrs() {
        assert_eq!(d(0x0000_0073), "ecall");
        assert_eq!(d(0x3020_0073), "mret");
        assert_eq!(d(0x3400_2573), "csrrs a0, mscratch, zero");
        assert_eq!(d(0x3000_5073), "csrrwi zero, mstatus, 0");
    }

    #[test]
    fn atomics_show_their_ordering_bits() {
        // amoadd.w a0, a1, (a2)
        assert_eq!(d(0x00b6_252f), "amoadd.w a0, a1, (a2)");
        // amoadd.w.aqrl
        assert_eq!(d(0x06b6_252f), "amoadd.w.aqrl a0, a1, (a2)");
        // lr.w a0, (a1)
        assert_eq!(d(0x1005_a52f), "lr.w a0, (a1)");
    }

    #[test]
    fn floating_point_prints_its_rounding_mode_only_when_static() {
        // fadd.d fa0, fa1, fa2, dynamic rounding
        assert_eq!(d(0x02c5_f553), "fadd.d fa0, fa1, fa2");
        // the same with round-towards-zero
        assert_eq!(d(0x02c5_9553), "fadd.d fa0, fa1, fa2, rtz");
        // fsgnj.d uses funct3 as an opcode and must not print a mode
        assert_eq!(d(0x22c5_8553), "fsgnj.d fa0, fa1, fa2");
        assert_eq!(d(0xe205_9553), "fclass.d a0, fa1");
    }

    #[test]
    fn fences_decode_their_ordering_sets() {
        // fence rw, rw
        assert_eq!(d(0x0ff0_000f), "fence iorw, iorw");
        assert_eq!(d(0x0330_000f), "fence rw, rw");
    }

    #[test]
    fn an_unknown_encoding_is_printed_as_data() {
        assert!(d(0xffff_ffff).starts_with(".word"));
    }

    #[test]
    fn compressed_instructions_keep_their_own_mnemonic() {
        let mut halves = [0x0505u16, 0x8082].into_iter();
        let out = disassemble_run(0x8000_0000, 2, Xlen::Rv64, |_| halves.next());
        assert_eq!(out[0].len, 2);
        assert_eq!(out[0].text, "c.addi a0, a0, 1");
        assert_eq!(out[1].text, "c.jr zero, 0(ra)");
    }

    #[test]
    fn a_run_stops_where_memory_does() {
        let out = disassemble_run(
            0,
            4,
            Xlen::Rv64,
            |addr| {
                if addr < 4 { Some(0x0013) } else { None }
            },
        );
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].len, 4);
    }

    #[test]
    fn the_display_form_lines_up() {
        let one = Disassembled {
            addr: 0x8000_0000,
            len: 4,
            encoding: 0x0000_0013,
            text: "addi zero, zero, 0".into(),
        };
        assert_eq!(
            one.to_string(),
            "0000000080000000: 00000013  addi zero, zero, 0"
        );
    }

    #[test]
    fn every_table_row_disassembles_without_panicking() {
        // A cheap completeness check: build the canonical encoding of every
        // row and make sure the formatter has an arm for its layout.
        for insn in isa::TABLE {
            let word = insn.bits;
            let text = format_decoded(insn.op, insn.fmt, insn.op.mnemonic(), word, 0);
            assert!(
                text.starts_with(insn.op.mnemonic()),
                "{} formatted as {text}",
                insn.op.mnemonic()
            );
        }
    }
}
