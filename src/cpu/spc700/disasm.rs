//! The disassembler, generated from the same table the interpreter decodes
//! with ([`isa::TABLE`](super::isa::TABLE)); there is no second opcode list.
//!
//! ```
//! use rsemu::cpu::spc700::disasm::disassemble;
//!
//! assert_eq!(format!("{}", disassemble(0x0200, &[0xe8, 0x42])), "MOV A,#$42");
//! assert_eq!(format!("{}", disassemble(0x0200, &[0x8f, 0xaa, 0xf4])), "MOV $f4,#$aa");
//! assert_eq!(format!("{}", disassemble(0x0200, &[0xd7, 0x00])), "MOV [$00]+Y,A");
//! assert_eq!(format!("{}", disassemble(0x0200, &[0x2f, 0xfe])), "BRA $0200");
//! ```
//!
//! # Syntax
//!
//! The SNESdev wiki's spelling: `!` marks an absolute address, `#` an
//! immediate, `dp.b` a bit, `/m.b` a complemented bit; hex is lowercase with a
//! `$` sigil, and a branch prints its **target**.

use core::fmt;

use super::isa::{Insn, Op, Operand, decode};

/// One decoded instruction at a known address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disassembled {
    /// Address the instruction was decoded at.
    pub pc: u16,
    /// The opcode byte.
    pub opcode: u8,
    /// The table row for this encoding.
    pub insn: Insn,
    /// The operand bytes, in stream order. Unused bytes are zero.
    pub operand: [u8; 2],
    /// How many bytes the instruction occupies.
    pub len: u8,
    /// Whether every byte the instruction needs was available.
    pub truncated: bool,
}

impl Disassembled {
    /// The value of the operand in `slot` (0 = destination, 1 = source),
    /// little-endian, located by [`Insn::byte_order`].
    fn value_at(&self, slot: usize) -> u16 {
        let (dst, src) = (self.insn.dst, self.insn.src);
        // Source bytes first, unless the source is a displacement.
        let src_first = src != Operand::Rel;
        let (op, start) = if slot == 0 {
            (dst, if src_first { src.bytes() } else { 0 })
        } else {
            (src, if src_first { 0 } else { dst.bytes() })
        };
        let mut v = 0u16;
        for i in 0..op.bytes() {
            v |= u16::from(self.operand[(start + i) as usize]) << (8 * i);
        }
        v
    }

    /// The address a branch would go to, if this is a branch.
    #[must_use]
    pub fn branch_target(&self) -> Option<u16> {
        if self.insn.src != Operand::Rel {
            return None;
        }
        let rel = self.operand[(self.len as usize).saturating_sub(2)] as i8;
        Some(
            self.pc
                .wrapping_add(u16::from(self.len))
                .wrapping_add(rel as u16),
        )
    }
}

fn write_operand(f: &mut fmt::Formatter<'_>, d: &Disassembled, slot: usize) -> fmt::Result {
    let operand = if slot == 0 { d.insn.dst } else { d.insn.src };
    let v = d.value_at(slot);
    match operand {
        Operand::None => Ok(()),
        Operand::A => f.write_str("A"),
        Operand::X => f.write_str("X"),
        Operand::Y => f.write_str("Y"),
        Operand::Sp => f.write_str("SP"),
        Operand::Psw => f.write_str("PSW"),
        Operand::Ya => f.write_str("YA"),
        Operand::C => f.write_str("C"),
        Operand::Imm => write!(f, "#${v:02x}"),
        Operand::Dp => write!(f, "${v:02x}"),
        Operand::DpX => write!(f, "${v:02x}+X"),
        Operand::DpY => write!(f, "${v:02x}+Y"),
        Operand::Abs => write!(f, "!${v:04x}"),
        Operand::AbsX => write!(f, "!${v:04x}+X"),
        Operand::AbsY => write!(f, "!${v:04x}+Y"),
        Operand::IndX => f.write_str("(X)"),
        Operand::IndXInc => f.write_str("(X)+"),
        Operand::IndY => f.write_str("(Y)"),
        Operand::DpIndX => write!(f, "[${v:02x}+X]"),
        Operand::DpIndY => write!(f, "[${v:02x}]+Y"),
        Operand::AbsIndX => write!(f, "[!${v:04x}+X]"),
        Operand::Rel => write!(f, "${:04x}", d.branch_target().unwrap_or(0)),
        Operand::DpBit(b) => write!(f, "${v:02x}.{b}"),
        Operand::MemBit => write!(f, "${:04x}.{}", v & 0x1fff, v >> 13),
        Operand::MemBitNot => write!(f, "/${:04x}.{}", v & 0x1fff, v >> 13),
        Operand::Upage => write!(f, "${v:02x}"),
        Operand::Vector(n) => write!(f, "{n}"),
    }
}

impl fmt::Display for Disassembled {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.insn.op.mnemonic())?;
        // Branches with no tested operand put the displacement in `src`; print
        // it as the only operand.
        let has_dst = self.insn.dst != Operand::None;
        let has_src = self.insn.src != Operand::None;
        if !has_dst && !has_src {
            return Ok(());
        }
        f.write_str(" ")?;
        if has_dst {
            write_operand(f, self, 0)?;
        }
        if has_src {
            if has_dst {
                f.write_str(",")?;
            }
            write_operand(f, self, 1)?;
        }
        Ok(())
    }
}

/// Decode the instruction at `pc` from `bytes`, which start at `pc`. Never
/// fails; a short buffer yields zeroed operand bytes and `truncated`.
#[must_use]
pub fn disassemble(pc: u16, bytes: &[u8]) -> Disassembled {
    let opcode = bytes.first().copied().unwrap_or(0);
    let mut truncated = bytes.is_empty();
    let insn = decode(opcode);
    let len = insn.bytes() as usize;
    let mut operand = [0u8; 2];
    for (i, slot) in operand.iter_mut().enumerate().take(len - 1) {
        match bytes.get(1 + i) {
            Some(b) => *slot = *b,
            None => truncated = true,
        }
    }
    Disassembled {
        pc,
        opcode,
        insn,
        operand,
        len: len as u8,
        truncated,
    }
}

/// Decode `count` instructions starting at `pc`, calling `fetch` for each
/// byte — the closure is how a debugger passes a side-effect-free read.
pub fn disassemble_run(
    pc: u16,
    count: usize,
    mut fetch: impl FnMut(u16) -> Option<u8>,
) -> alloc::vec::Vec<Disassembled> {
    let mut out = alloc::vec::Vec::with_capacity(count);
    let mut at = pc;
    for _ in 0..count {
        let mut window = [0u8; 3];
        let mut got = 0usize;
        for (i, slot) in window.iter_mut().enumerate() {
            match fetch(at.wrapping_add(i as u16)) {
                Some(b) => {
                    *slot = b;
                    got += 1;
                }
                None => break,
            }
        }
        if got == 0 {
            break;
        }
        let d = disassemble(at, &window[..got]);
        at = at.wrapping_add(u16::from(d.len));
        out.push(d);
        if d.truncated {
            break;
        }
    }
    out
}

/// A one-line-per-opcode dump of the instruction set, for `rsemu describe`.
#[must_use]
pub fn describe_isa() -> alloc::string::String {
    use core::fmt::Write as _;
    let mut out = alloc::string::String::new();
    for opcode in 0..=255u8 {
        let d = disassemble(0x0000, &[opcode, 0, 0]);
        let _ = writeln!(
            out,
            "{opcode:02x} {:<20} {}",
            alloc::format!("{d}"),
            d.insn.op.summary()
        );
    }
    out
}

/// `Op` is named in this module's public surface through [`Insn`].
const _: fn(Op) = |_| {};

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;
    use alloc::string::String;

    fn text(bytes: &[u8]) -> String {
        format!("{}", disassemble(0x0200, bytes))
    }

    #[test]
    fn operands_print_in_assembler_order() {
        assert_eq!(text(&[0x00]), "NOP");
        assert_eq!(text(&[0xe4, 0x12]), "MOV A,$12");
        assert_eq!(text(&[0xf4, 0x12]), "MOV A,$12+X");
        assert_eq!(text(&[0xe5, 0x34, 0x12]), "MOV A,!$1234");
        assert_eq!(text(&[0xd5, 0x34, 0x12]), "MOV !$1234+X,A");
        assert_eq!(text(&[0xe7, 0x12]), "MOV A,[$12+X]");
        assert_eq!(text(&[0xbf]), "MOV A,(X)+");
        assert_eq!(text(&[0xfa, 0x11, 0x22]), "MOV $22,$11");
        assert_eq!(text(&[0x89, 0x11, 0x22]), "ADC $22,$11");
        assert_eq!(text(&[0x8f, 0xaa, 0xf4]), "MOV $f4,#$aa");
        assert_eq!(text(&[0x99]), "ADC (X),(Y)");
        assert_eq!(text(&[0xba, 0xf4]), "MOVW YA,$f4");
        assert_eq!(text(&[0xda, 0x00]), "MOVW $00,YA");
        assert_eq!(text(&[0x9e]), "DIV YA,X");
    }

    #[test]
    fn bits_and_branches() {
        assert_eq!(text(&[0xe2, 0x10]), "SET1 $10.7");
        assert_eq!(text(&[0x13, 0x10, 0x00]), "BBC $10.0,$0203");
        assert_eq!(text(&[0x2e, 0xf4, 0xfd]), "CBNE $f4,$0200");
        assert_eq!(text(&[0xfe, 0xfe]), "DBNZ Y,$0200");
        assert_eq!(text(&[0xd0, 0xfc]), "BNE $01fe");
        assert_eq!(text(&[0x2a, 0x34, 0xb2]), "OR1 C,/$1234.5");
        assert_eq!(text(&[0xca, 0x34, 0x12]), "MOV1 $1234.0,C");
        assert_eq!(text(&[0x1f, 0x00, 0x00]), "JMP [!$0000+X]");
        assert_eq!(text(&[0xc1]), "TCALL 12");
        assert_eq!(text(&[0x4f, 0x80]), "PCALL $80");
    }

    #[test]
    fn every_encoding_formats_and_the_dump_has_256_lines() {
        for op in 0..=255u8 {
            let _ = text(&[op, 0x34, 0x12]);
        }
        assert_eq!(describe_isa().lines().count(), 256);
    }

    #[test]
    fn a_truncated_stream_is_flagged() {
        assert!(disassemble(0, &[0xe5, 0x00]).truncated);
        assert!(disassemble(0, &[]).truncated);
    }
}
