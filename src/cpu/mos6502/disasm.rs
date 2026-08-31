//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! Not a side project: gdb's `disassemble`, the monitor's single-step display
//! and any trace log need it, and CLAUDE.md forbids describing the instruction
//! set twice. Everything here reads [`isa::TABLE`](super::isa::TABLE); there is
//! no second opcode list to keep in step.
//!
//! ```
//! use rsemu::cpu::mos6502::disasm::disassemble;
//!
//! let d = disassemble(0xc000, &[0xbd, 0x34, 0x12]);
//! assert_eq!(format!("{d}"), "LDA $1234,X");
//! assert_eq!(d.len, 3);
//! ```

use core::fmt;

use super::isa::{Class, Insn, Mode, Variant, decode_as};

/// One decoded instruction at a known address.
///
/// Carries the raw bytes as well as the decoded row so a monitor can print
/// `c000: bd 34 12  LDA $1234,X` without decoding twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disassembled {
    /// Address the instruction was decoded at.
    pub pc: u16,
    /// The opcode byte.
    pub opcode: u8,
    /// The table row for [`Disassembled::opcode`].
    pub insn: Insn,
    /// The operand bytes, low byte first. Unused bytes are zero.
    pub operand: [u8; 2],
    /// How many bytes the instruction occupies, opcode included.
    pub len: u8,
    /// Whether every byte the instruction needs was available.
    ///
    /// A monitor disassembling to the end of a buffer, or across an unmapped
    /// page, gets a best-effort decode with the missing bytes read as zero
    /// rather than a panic — but it is told.
    pub truncated: bool,
}

impl Disassembled {
    /// The 16-bit operand, low byte first. Meaningless for shorter modes.
    #[must_use]
    pub const fn word(&self) -> u16 {
        (self.operand[0] as u16) | ((self.operand[1] as u16) << 8)
    }

    /// The address a branch would jump to.
    ///
    /// The displacement is signed and counts from the *next* instruction, so
    /// this is `pc + len + offset`. `BBR`/`BBS` carry theirs in the second
    /// operand byte, after the page-zero address. Returns `None` for every
    /// non-branching mode.
    #[must_use]
    pub const fn branch_target(&self) -> Option<u16> {
        // Guest arithmetic wraps: a branch near $ffff wraps to page zero, and
        // that is the address the CPU jumps to.
        match self.insn.mode {
            Mode::Relative => Some(
                self.pc
                    .wrapping_add(2)
                    .wrapping_add(self.operand[0] as i8 as u16),
            ),
            Mode::ZeroPageRelative => Some(
                self.pc
                    .wrapping_add(3)
                    .wrapping_add(self.operand[1] as i8 as u16),
            ),
            _ => None,
        }
    }

    /// The address the instruction ultimately reads or writes, where that can
    /// be known without the register file.
    ///
    /// Absolute and page-zero modes only: an indexed or indirect operand
    /// depends on state a static disassembler does not have.
    #[must_use]
    pub const fn static_target(&self) -> Option<u16> {
        match self.insn.mode {
            Mode::ZeroPage | Mode::ZeroPageRelative => Some(self.operand[0] as u16),
            Mode::Absolute => Some(self.word()),
            Mode::Relative => self.branch_target(),
            _ => None,
        }
    }

    /// Whether the encoding is undocumented, in either flavour.
    #[must_use]
    pub const fn is_undocumented(&self) -> bool {
        !matches!(self.insn.class, Class::Documented)
    }
}

impl fmt::Display for Disassembled {
    /// Standard 6502 assembler syntax, uppercase mnemonic, lowercase hex.
    ///
    /// Undocumented encodings are printed with no marker: the caller has
    /// [`Insn::class`] and can decide whether its audience wants a `*`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.insn.op.mnemonic())?;
        let lo = self.operand[0];
        match self.insn.mode {
            Mode::Implied | Mode::Single | Mode::Break => Ok(()),
            Mode::Accumulator => f.write_str(" A"),
            Mode::Immediate => write!(f, " #${lo:02x}"),
            Mode::ZeroPage => write!(f, " ${lo:02x}"),
            Mode::ZeroPageX => write!(f, " ${lo:02x},X"),
            Mode::ZeroPageY => write!(f, " ${lo:02x},Y"),
            Mode::Absolute => write!(f, " ${:04x}", self.word()),
            Mode::AbsoluteX => write!(f, " ${:04x},X", self.word()),
            Mode::AbsoluteY => write!(f, " ${:04x},Y", self.word()),
            Mode::Indirect => write!(f, " (${:04x})", self.word()),
            Mode::IndirectX => write!(f, " (${lo:02x},X)"),
            Mode::IndirectY => write!(f, " (${lo:02x}),Y"),
            Mode::ZeroPageIndirect => write!(f, " (${lo:02x})"),
            Mode::AbsoluteIndirectX => write!(f, " (${:04x},X)", self.word()),
            // The page-zero byte first, then the branch target — the order the
            // WDC datasheet writes them and the order an assembler accepts.
            Mode::ZeroPageRelative => write!(
                f,
                " ${lo:02x},${:04x}",
                self.branch_target()
                    .expect("zero-page relative has a target")
            ),
            // A branch is far more useful as a target than as a displacement,
            // which is why the raw byte is kept in `operand` for anyone who
            // wants it.
            Mode::Relative => write!(
                f,
                " ${:04x}",
                self.branch_target().expect("relative mode has a target")
            ),
        }
    }
}

/// Decode the instruction at `pc` from `bytes`, as an NMOS 6502 sees it.
///
/// Never fails: all 256 encodings are defined. A buffer too short for the
/// operand yields a decode with zeroed operand bytes and
/// [`Disassembled::truncated`] set.
#[must_use]
pub fn disassemble(pc: u16, bytes: &[u8]) -> Disassembled {
    disassemble_as(Variant::Nmos6502, pc, bytes)
}

/// Decode the instruction at `pc` from `bytes`, as `variant` sees it.
///
/// The variant is not decoration: `$3a` is a one-byte NOP on the NMOS part and
/// `DEC A` on the CMOS one, so a listing produced with the wrong table
/// desynchronises after the first such byte and everything below it is
/// nonsense.
#[must_use]
pub fn disassemble_as(variant: Variant, pc: u16, bytes: &[u8]) -> Disassembled {
    let opcode = bytes.first().copied().unwrap_or(0);
    let insn = decode_as(variant, opcode);
    let len = insn.bytes() as usize;
    let mut operand = [0u8; 2];
    let mut truncated = bytes.len() < len;
    for (i, slot) in operand.iter_mut().enumerate().take(len.saturating_sub(1)) {
        match bytes.get(i + 1) {
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

/// Decode `count` instructions starting at `pc`, calling `fetch` for each byte.
///
/// The closure is how a monitor reaches guest memory without this module
/// knowing what an address space is — and how a debugger passes a
/// side-effect-free read (`MemAttrs::debug`, `ROADMAP.md` §15 invariant 5).
/// Iteration stops early if `fetch` returns `None`.
pub fn disassemble_run(
    pc: u16,
    count: usize,
    fetch: impl FnMut(u16) -> Option<u8>,
) -> alloc::vec::Vec<Disassembled> {
    disassemble_run_as(Variant::Nmos6502, pc, count, fetch)
}

/// The same, decoding as `variant` sees it.
pub fn disassemble_run_as(
    variant: Variant,
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
        let d = disassemble_as(variant, at, &window[..got]);
        at = at.wrapping_add(u16::from(d.len));
        out.push(d);
        if got < 3 && d.truncated {
            break;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::super::isa::decode;
    use super::*;
    use alloc::format;
    use alloc::string::String;
    use alloc::vec::Vec;

    /// Re-encode a disassembly line back into an opcode and operand bytes.
    ///
    /// The point of a round-trip test is that it goes *back* through the text,
    /// so this parses the syntax rather than consulting the table by opcode.
    /// It returns the first encoding with that mnemonic and mode, which is the
    /// canonical one an assembler would pick.
    fn reassemble(text: &str) -> Option<(u8, Vec<u8>)> {
        let (mnemonic, operand) = match text.split_once(' ') {
            Some((m, o)) => (m, o.trim()),
            None => (text, ""),
        };
        let hex = |s: &str| u16::from_str_radix(s, 16).ok();
        let (mode, value): (Mode, u16) = if operand.is_empty() {
            (Mode::Implied, 0)
        } else if operand == "A" {
            (Mode::Accumulator, 0)
        } else if let Some(v) = operand.strip_prefix("#$") {
            (Mode::Immediate, hex(v)?)
        } else if let Some(v) = operand
            .strip_prefix("($")
            .and_then(|v| v.strip_suffix(",X)"))
        {
            (Mode::IndirectX, hex(v)?)
        } else if let Some(v) = operand
            .strip_prefix("($")
            .and_then(|v| v.strip_suffix("),Y"))
        {
            (Mode::IndirectY, hex(v)?)
        } else if let Some(v) = operand.strip_prefix("($").and_then(|v| v.strip_suffix(')')) {
            (Mode::Indirect, hex(v)?)
        } else if let Some(v) = operand.strip_prefix('$').and_then(|v| v.strip_suffix(",X")) {
            let m = if v.len() == 2 {
                Mode::ZeroPageX
            } else {
                Mode::AbsoluteX
            };
            (m, hex(v)?)
        } else if let Some(v) = operand.strip_prefix('$').and_then(|v| v.strip_suffix(",Y")) {
            let m = if v.len() == 2 {
                Mode::ZeroPageY
            } else {
                Mode::AbsoluteY
            };
            (m, hex(v)?)
        } else {
            let v = operand.strip_prefix('$')?;
            let m = if v.len() == 2 {
                Mode::ZeroPage
            } else {
                Mode::Absolute
            };
            (m, hex(v)?)
        };

        for opcode in 0..=255u8 {
            let insn = decode(opcode);
            // `Break` and `Relative` print in a shape the grammar above reads
            // as something else; the caller handles them.
            if insn.op.mnemonic() == mnemonic && insn.mode == mode {
                let mut bytes = Vec::new();
                match insn.bytes() {
                    2 => bytes.push(value as u8),
                    3 => {
                        bytes.push(value as u8);
                        bytes.push((value >> 8) as u8);
                    }
                    _ => {}
                }
                return Some((opcode, bytes));
            }
        }
        None
    }

    #[test]
    fn operand_syntax_matches_the_assembler_convention() {
        let cases: &[(&[u8], &str)] = &[
            (&[0xea], "NOP"),
            (&[0x0a], "ASL A"),
            (&[0xa9, 0x42], "LDA #$42"),
            (&[0xa5, 0x42], "LDA $42"),
            (&[0xb5, 0x42], "LDA $42,X"),
            (&[0xb6, 0x42], "LDX $42,Y"),
            (&[0xad, 0x34, 0x12], "LDA $1234"),
            (&[0xbd, 0x34, 0x12], "LDA $1234,X"),
            (&[0xb9, 0x34, 0x12], "LDA $1234,Y"),
            (&[0x6c, 0x34, 0x12], "JMP ($1234)"),
            (&[0xa1, 0x42], "LDA ($42,X)"),
            (&[0xb1, 0x42], "LDA ($42),Y"),
            (&[0x00, 0x00], "BRK"),
            (&[0x03, 0x42], "SLO ($42,X)"),
            (&[0xeb, 0x42], "USBC #$42"),
        ];
        for (bytes, want) in cases {
            assert_eq!(format!("{}", disassemble(0xc000, bytes)), *want);
        }
    }

    #[test]
    fn a_branch_prints_its_target_not_its_displacement() {
        // Forwards, backwards, and across a page boundary.
        assert_eq!(
            format!("{}", disassemble(0xc000, &[0xd0, 0x05])),
            "BNE $c007"
        );
        assert_eq!(
            format!("{}", disassemble(0xc000, &[0xd0, 0xfe])),
            "BNE $c000"
        );
        assert_eq!(
            format!("{}", disassemble(0xc0f0, &[0x10, 0x40])),
            "BPL $c132"
        );
        // And it wraps, because the guest's PC does.
        assert_eq!(
            format!("{}", disassemble(0xfffe, &[0xd0, 0x10])),
            "BNE $0010"
        );
    }

    #[test]
    fn every_opcode_round_trips_through_its_text() {
        // Text -> (mnemonic, mode) -> opcode. Several opcodes share a
        // mnemonic and mode (there are three `NOP zpg` encodings), so the
        // check is that re-assembling lands on an encoding that decodes back
        // to the same row, not necessarily the same byte.
        for opcode in 0..=255u8 {
            let insn = decode(opcode);
            let bytes = [opcode, 0x34, 0x12];
            let d = disassemble(0xc000, &bytes[..insn.bytes() as usize]);
            let text: String = format!("{d}");
            if insn.mode == Mode::Relative || insn.mode == Mode::Break {
                // A relative operand prints as a target and a BRK signature
                // byte does not print at all; both are checked separately.
                continue;
            }
            let (back, operand) =
                reassemble(&text).unwrap_or_else(|| panic!("{opcode:02x}: cannot parse {text:?}"));
            let round = decode(back);
            assert_eq!(round.op, insn.op, "{opcode:02x} {text}");
            assert_eq!(round.mode, insn.mode, "{opcode:02x} {text}");
            assert_eq!(operand, bytes[1..insn.bytes() as usize], "{opcode:02x}");
        }
    }

    #[test]
    fn a_short_buffer_decodes_as_truncated_rather_than_panicking() {
        let d = disassemble(0xc000, &[0xad, 0x34]);
        assert!(d.truncated);
        assert_eq!(d.len, 3);
        assert_eq!(d.word(), 0x0034);
        let empty = disassemble(0xc000, &[]);
        assert!(empty.truncated);
        assert_eq!(empty.opcode, 0x00);
    }

    #[test]
    fn a_run_walks_instruction_by_instruction() {
        // LDA #$01 / STA $0200 / BNE back / NOP
        let program: [u8; 8] = [0xa9, 0x01, 0x8d, 0x00, 0x02, 0xd0, 0xf9, 0xea];
        let run = disassemble_run(0xc000, 4, |a| {
            program.get(a.wrapping_sub(0xc000) as usize).copied()
        });
        let text: Vec<String> = run.iter().map(|d| format!("{d}")).collect();
        assert_eq!(text, ["LDA #$01", "STA $0200", "BNE $c000", "NOP"]);
        assert_eq!(run[2].branch_target(), Some(0xc000));
        assert_eq!(run[1].static_target(), Some(0x0200));
    }

    #[test]
    fn undocumented_encodings_are_flagged() {
        assert!(disassemble(0, &[0x03, 0]).is_undocumented());
        assert!(!disassemble(0, &[0xa9, 0]).is_undocumented());
    }
}
