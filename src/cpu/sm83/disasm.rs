//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! Not a side project: gdb's `disassemble`, the monitor's single-step display
//! and any trace log need it, and CLAUDE.md forbids describing the instruction
//! set twice. Everything here reads [`isa::BASE`](super::isa::BASE) and
//! [`isa::PREFIXED`](super::isa::PREFIXED); there is no second opcode list to
//! keep in step.
//!
//! ```
//! use rsemu::cpu::sm83::disasm::disassemble;
//!
//! assert_eq!(format!("{}", disassemble(0x0100, &[0x21, 0x34, 0x12])), "LD HL,$1234");
//! assert_eq!(format!("{}", disassemble(0x0100, &[0xe0, 0x40])), "LDH ($ff40),A");
//! assert_eq!(format!("{}", disassemble(0x0100, &[0x20, 0xfe])), "JR NZ,$0100");
//! assert_eq!(format!("{}", disassemble(0x0100, &[0xcb, 0x7e])), "BIT 7,(HL)");
//! ```
//!
//! # Syntax
//!
//! The gbdev/RGBDS spelling, which is what every Game Boy assembler and every
//! disassembly on the internet uses: `LD A,(HL+)`, `LDH ($ff00),A`, `JR NZ,$…`.
//! Hex is lowercase with a `$` sigil, and a relative branch prints its
//! **target** rather than its displacement, because that is the number a person
//! reading a trace wants. The raw byte is still in
//! [`Disassembled::operand`] for anyone who wants the other one.

use core::fmt;

use super::isa::{Class, Cond, Insn, Op, Operand, Reg8, Reg16, decode, decode_cb};

/// One decoded instruction at a known address.
///
/// Carries the raw operand bytes as well as the decoded row, so a monitor can
/// print `0150: 21 34 12  LD HL,$1234` without decoding twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disassembled {
    /// Address the instruction was decoded at.
    pub pc: u16,
    /// The opcode byte — the `$CB` prefix itself, for a prefixed instruction.
    pub opcode: u8,
    /// The second opcode byte, for a `$CB`-prefixed instruction.
    pub prefixed_opcode: Option<u8>,
    /// The table row for this encoding.
    pub insn: Insn,
    /// The operand bytes, low byte first. Unused bytes are zero.
    pub operand: [u8; 2],
    /// How many bytes the instruction occupies, prefix and opcode included.
    pub len: u8,
    /// Whether every byte the instruction needs was available.
    ///
    /// A monitor disassembling to the end of a buffer, or across an unmapped
    /// page, gets a best-effort decode with the missing bytes read as zero
    /// rather than a panic — but it is told.
    pub truncated: bool,
}

impl Disassembled {
    /// The 16-bit operand, low byte first. Meaningless for shorter operands.
    #[must_use]
    pub const fn word(&self) -> u16 {
        (self.operand[0] as u16) | ((self.operand[1] as u16) << 8)
    }

    /// The address a `JR` would jump to.
    ///
    /// The displacement is signed and counts from the *next* instruction, so
    /// this is `pc + 2 + e8`. `None` for everything else.
    #[must_use]
    pub const fn branch_target(&self) -> Option<u16> {
        match self.insn.op {
            // Guest arithmetic wraps: a branch near $ffff wraps to $0000, and
            // that is the address the CPU jumps to.
            Op::JR => Some(
                self.pc
                    .wrapping_add(2)
                    .wrapping_add(self.operand[0] as i8 as u16),
            ),
            _ => None,
        }
    }

    /// The address the instruction reads, writes or jumps to, where that can be
    /// known without the register file.
    #[must_use]
    pub const fn static_target(&self) -> Option<u16> {
        match self.insn.op {
            Op::JR => self.branch_target(),
            Op::RST => match self.insn.dst {
                Operand::Vector(v) => Some(v as u16),
                _ => None,
            },
            _ => match (self.insn.dst, self.insn.src) {
                (Operand::MemImm16, _) | (_, Operand::MemImm16) => Some(self.word()),
                (Operand::MemHighImm8, _) | (_, Operand::MemHighImm8) => {
                    Some(0xff00 | self.operand[0] as u16)
                }
                (Operand::Imm16, _) | (_, Operand::Imm16)
                    if matches!(self.insn.op, Op::JP | Op::CALL) =>
                {
                    Some(self.word())
                }
                _ => None,
            },
        }
    }

    /// Whether this encoding is one of the eleven the chip does not implement.
    #[must_use]
    pub const fn is_unimplemented(&self) -> bool {
        !matches!(self.insn.class, Class::Documented)
    }
}

/// Format one operand, given the instruction it belongs to.
///
/// A free function rather than a `Display` impl on [`Operand`], because three
/// of the forms need the operand *bytes* and one needs the program counter, and
/// none of those live in the enum.
fn write_operand(f: &mut fmt::Formatter<'_>, d: &Disassembled, operand: Operand) -> fmt::Result {
    let lo = d.operand[0];
    match operand {
        Operand::None => Ok(()),
        Operand::Reg(r) => f.write_str(r.name()),
        Operand::MemHl => f.write_str("(HL)"),
        Operand::Reg16(r) => f.write_str(r.name()),
        Operand::MemReg16(r) => write!(f, "({})", r.name()),
        Operand::MemHlInc => f.write_str("(HL+)"),
        Operand::MemHlDec => f.write_str("(HL-)"),
        Operand::MemHighC => f.write_str("(C)"),
        Operand::Imm8 => write!(f, "${lo:02x}"),
        Operand::Imm16 => write!(f, "${:04x}", d.word()),
        // For a branch, the target rather than the displacement: it is what a
        // person reading a trace is actually after. `ADD SP,e8` is the other
        // user of this operand and there is no target to print, so it keeps the
        // raw byte.
        Operand::Rel8 => match d.branch_target() {
            Some(target) => write!(f, "${target:04x}"),
            None => write!(f, "${lo:02x}"),
        },
        Operand::MemImm16 => write!(f, "(${:04x})", d.word()),
        Operand::MemHighImm8 => write!(f, "($ff{lo:02x})"),
        Operand::SpRel8 => {
            let e = lo as i8;
            if e < 0 {
                write!(f, "SP-${:02x}", e.unsigned_abs())
            } else {
                write!(f, "SP+${:02x}", e)
            }
        }
        Operand::Cond(c) => f.write_str(c.name()),
        Operand::Bit(b) => write!(f, "{b}"),
        Operand::Vector(v) => write!(f, "${v:02x}"),
    }
}

impl fmt::Display for Disassembled {
    /// RGBDS syntax: uppercase mnemonic, lowercase hex, `$` sigil.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let insn = self.insn;
        f.write_str(insn.op.mnemonic())?;

        // The accumulator is written out for `ADD A,B` but not for `SUB B`,
        // because that is the convention every Game Boy assembler uses and a
        // disassembly nobody can reassemble is a worse tool. `ADD`/`ADC`/`SBC`
        // keep it; `SUB`/`AND`/`OR`/`XOR`/`CP` drop it.
        let implicit_a = matches!(insn.op, Op::SUB | Op::AND | Op::OR | Op::XOR | Op::CP)
            && insn.dst == Operand::Reg(Reg8::A);

        let has_dst = insn.dst != Operand::None && !implicit_a;
        let has_src = insn.src != Operand::None;
        if !has_dst && !has_src {
            return Ok(());
        }
        f.write_str(" ")?;
        if has_dst {
            write_operand(f, self, insn.dst)?;
        }
        if has_src {
            if has_dst {
                f.write_str(",")?;
            }
            write_operand(f, self, insn.src)?;
        }
        Ok(())
    }
}

/// Decode the instruction at `pc` from `bytes`, which start at `pc`.
///
/// Never fails: all 512 encodings are defined, the eleven holes included. A
/// buffer too short for the operand yields a decode with zeroed operand bytes
/// and [`Disassembled::truncated`] set.
#[must_use]
pub fn disassemble(pc: u16, bytes: &[u8]) -> Disassembled {
    let opcode = bytes.first().copied().unwrap_or(0);
    let mut truncated = bytes.is_empty();
    let mut prefixed_opcode = None;
    let mut insn = decode(opcode);
    // `$CB` is not an instruction, it selects the second page. A stream that
    // ends on the prefix byte decodes as `RLC B` and says it was truncated,
    // which is the least misleading thing available.
    let mut operand_at = 1usize;
    if insn.op == Op::PREFIX {
        let second = match bytes.get(1) {
            Some(b) => *b,
            None => {
                truncated = true;
                0
            }
        };
        prefixed_opcode = Some(second);
        insn = decode_cb(second);
        operand_at = 2;
    }

    let len = insn.bytes() as usize;
    let mut operand = [0u8; 2];
    if bytes.len() < len {
        truncated = true;
    }
    for (i, slot) in operand
        .iter_mut()
        .enumerate()
        .take(len.saturating_sub(operand_at))
    {
        match bytes.get(operand_at + i) {
            Some(b) => *slot = *b,
            None => truncated = true,
        }
    }
    Disassembled {
        pc,
        opcode,
        prefixed_opcode,
        insn,
        operand,
        len: len as u8,
        truncated,
    }
}

/// Decode `count` instructions starting at `pc`, calling `fetch` for each byte.
///
/// The closure is how a monitor reaches guest memory without this module knowing
/// what an address space is — and how a debugger passes a side-effect-free read
/// (`MemAttrs::debug`, `ROADMAP.md` §15 invariant 5). Iteration stops early if
/// `fetch` returns `None`.
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
            // Guest arithmetic wraps: a run that reaches $ffff continues at
            // $0000, which is what the CPU would do.
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
        if got < 3 && d.truncated {
            break;
        }
    }
    out
}

/// A one-line dump of the whole instruction set, for `rsemu describe`.
///
/// Built from the tables, so it cannot drift from what the interpreter runs.
#[must_use]
pub fn describe_isa() -> alloc::string::String {
    use core::fmt::Write as _;
    let mut out = alloc::string::String::new();
    for opcode in 0..=255u8 {
        let d = disassemble(0x0000, &[opcode, 0, 0]);
        let mark = if d.is_unimplemented() { '!' } else { ' ' };
        let _ = writeln!(out, "{opcode:02x} {mark}{d:<18} {}", d.insn.op.summary());
    }
    for opcode in 0..=255u8 {
        let d = disassemble(0x0000, &[0xcb, opcode]);
        let _ = writeln!(out, "cb{opcode:02x} {d:<18} {}", d.insn.op.summary());
    }
    out
}

/// Silence the unused-import warning in builds that do not exercise every
/// operand form; `Cond` and `Reg16` are named in this module's signatures.
const _: fn(Cond, Reg16) = |_, _| {};

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;
    use alloc::vec::Vec;

    fn text(bytes: &[u8]) -> alloc::string::String {
        format!("{}", disassemble(0x0100, bytes))
    }

    #[test]
    fn the_loads_read_the_way_an_assembler_writes_them() {
        assert_eq!(text(&[0x00]), "NOP");
        assert_eq!(text(&[0x41]), "LD B,C");
        assert_eq!(text(&[0x46]), "LD B,(HL)");
        assert_eq!(text(&[0x70]), "LD (HL),B");
        assert_eq!(text(&[0x36, 0x42]), "LD (HL),$42");
        assert_eq!(text(&[0x01, 0x34, 0x12]), "LD BC,$1234");
        assert_eq!(text(&[0x2a]), "LD A,(HL+)");
        assert_eq!(text(&[0x32]), "LD (HL-),A");
        assert_eq!(text(&[0x08, 0x00, 0xc0]), "LD ($c000),SP");
        assert_eq!(text(&[0xea, 0x00, 0xc0]), "LD ($c000),A");
    }

    #[test]
    fn the_ff00_page_gets_its_own_mnemonic() {
        assert_eq!(text(&[0xe0, 0x40]), "LDH ($ff40),A");
        assert_eq!(text(&[0xf0, 0x44]), "LDH A,($ff44)");
        assert_eq!(text(&[0xe2]), "LDH (C),A");
        assert_eq!(text(&[0xf2]), "LDH A,(C)");
    }

    #[test]
    fn the_accumulator_is_implicit_exactly_where_convention_says() {
        assert_eq!(text(&[0x80]), "ADD A,B");
        assert_eq!(text(&[0x88]), "ADC A,B");
        assert_eq!(text(&[0x98]), "SBC A,B");
        assert_eq!(text(&[0x90]), "SUB B");
        assert_eq!(text(&[0xa0]), "AND B");
        assert_eq!(text(&[0xb0]), "OR B");
        assert_eq!(text(&[0xa8]), "XOR B");
        assert_eq!(text(&[0xb8]), "CP B");
        assert_eq!(text(&[0xfe, 0x90]), "CP $90");
    }

    #[test]
    fn a_relative_branch_prints_its_target() {
        // $fe is -2, so this is the classic self-loop.
        assert_eq!(text(&[0x18, 0xfe]), "JR $0100");
        assert_eq!(text(&[0x20, 0x05]), "JR NZ,$0107");
        assert_eq!(
            disassemble(0x0100, &[0x18, 0x05]).branch_target(),
            Some(0x0107)
        );
    }

    #[test]
    fn the_stack_pointer_forms_show_their_sign() {
        assert_eq!(text(&[0xf8, 0x05]), "LD HL,SP+$05");
        assert_eq!(text(&[0xf8, 0xfb]), "LD HL,SP-$05");
        assert_eq!(text(&[0xe8, 0xfb]), "ADD SP,$fb");
    }

    #[test]
    fn the_cb_page_decodes_through_the_prefix() {
        assert_eq!(text(&[0xcb, 0x00]), "RLC B");
        assert_eq!(text(&[0xcb, 0x36]), "SWAP (HL)");
        assert_eq!(text(&[0xcb, 0x7e]), "BIT 7,(HL)");
        assert_eq!(text(&[0xcb, 0x87]), "RES 0,A");
        assert_eq!(text(&[0xcb, 0xff]), "SET 7,A");
        assert_eq!(disassemble(0x0100, &[0xcb, 0x7e]).len, 2);
        assert_eq!(
            disassemble(0x0100, &[0xcb, 0x7e]).prefixed_opcode,
            Some(0x7e)
        );
    }

    #[test]
    fn control_flow_reads_correctly() {
        assert_eq!(text(&[0xc3, 0x50, 0x01]), "JP $0150");
        assert_eq!(text(&[0xca, 0x50, 0x01]), "JP Z,$0150");
        assert_eq!(text(&[0xe9]), "JP HL");
        assert_eq!(text(&[0xcd, 0x50, 0x01]), "CALL $0150");
        assert_eq!(text(&[0xc9]), "RET");
        assert_eq!(text(&[0xd8]), "RET C");
        assert_eq!(text(&[0xd9]), "RETI");
        assert_eq!(text(&[0xff]), "RST $38");
        assert_eq!(text(&[0xc5]), "PUSH BC");
        assert_eq!(text(&[0xf1]), "POP AF");
    }

    #[test]
    fn the_eleven_holes_say_what_they_are() {
        for opcode in [
            0xd3u8, 0xdb, 0xdd, 0xe3, 0xe4, 0xeb, 0xec, 0xed, 0xf4, 0xfc, 0xfd,
        ] {
            let d = disassemble(0x0100, &[opcode]);
            assert!(d.is_unimplemented(), "{opcode:#04x}");
            assert_eq!(format!("{d}"), "LOCK");
        }
    }

    #[test]
    fn a_truncated_stream_is_flagged_rather_than_fatal() {
        let d = disassemble(0x0100, &[0x01]);
        assert!(d.truncated);
        assert_eq!(d.len, 3);
        let d = disassemble(0x0100, &[0xcb]);
        assert!(d.truncated);
        let d = disassemble(0x0100, &[]);
        assert!(d.truncated);
    }

    #[test]
    fn a_run_walks_the_right_number_of_bytes() {
        let code = [0x21u8, 0x34, 0x12, 0xcb, 0x7e, 0x00];
        let out = disassemble_run(0, 3, |a| code.get(a as usize).copied());
        let text: Vec<alloc::string::String> = out.iter().map(|d| format!("{d}")).collect();
        assert_eq!(text, ["LD HL,$1234", "BIT 7,(HL)", "NOP"]);
        assert_eq!(out[1].pc, 3);
        assert_eq!(out[2].pc, 5);
    }

    #[test]
    fn every_encoding_formats_without_panicking() {
        // Total by construction, but the formatter has a lot of arms and a
        // missing one would be a panic in a monitor rather than a compile error.
        for opcode in 0..=255u8 {
            let _ = text(&[opcode, 0x34, 0x12]);
            let _ = text(&[0xcb, opcode]);
        }
    }

    #[test]
    fn the_isa_dump_covers_both_pages() {
        let dump = describe_isa();
        assert_eq!(dump.lines().count(), 512);
        assert!(dump.contains("cb7e BIT 7,(HL)"), "{dump}");
    }
}
