//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! The one thing a 65C816 disassembler cannot know from the bytes alone is how
//! long an immediate is: `A9 34 12` is `LDA #$1234` with a 16-bit accumulator
//! and `LDA #$34` followed by a stray `$12` with an 8-bit one. So every entry
//! point takes the M and X widths ([`Widths`]), and [`disassemble_run`]
//! follows `REP`, `SEP` and `XCE` through a listing the way an assembler's
//! `.a16`/`.i16` tracking does — which is right for straight-line code and is
//! the best any static listing can do.
//!
//! ```
//! use rsemu::cpu::w65c816::disasm::{Widths, disassemble};
//!
//! let d = disassemble(0x00_8000, &[0xa9, 0x34, 0x12], Widths::NATIVE16);
//! assert_eq!(format!("{d}"), "LDA #$1234");
//! let d = disassemble(0x00_8000, &[0xa9, 0x34, 0x12], Widths::EMULATION);
//! assert_eq!(format!("{d}"), "LDA #$34");
//! ```

use alloc::vec::Vec;
use core::fmt;

use super::isa::{Insn, Mode, Op, decode};

/// The register widths an instruction is decoded under.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Widths {
    /// The accumulator is eight bits (`M` set, or emulation mode).
    pub m8: bool,
    /// The index registers are eight bits (`X` set, or emulation mode).
    pub x8: bool,
    /// Emulation mode, which pins both of the above.
    pub e: bool,
}

impl Widths {
    /// Emulation mode: everything eight bits, as after reset.
    pub const EMULATION: Widths = Widths {
        m8: true,
        x8: true,
        e: true,
    };
    /// Native mode with both widths sixteen bits.
    pub const NATIVE16: Widths = Widths {
        m8: false,
        x8: false,
        e: false,
    };

    /// The widths a status byte and an emulation flag describe.
    #[must_use]
    pub const fn from_p(p: u8, e: bool) -> Widths {
        Widths {
            m8: e || p & super::flags::M != 0,
            x8: e || p & super::flags::X != 0,
            e,
        }
    }

    /// What executing `d` does to the widths, as far as a static listing can
    /// tell: `REP`/`SEP` with an immediate, and `XCE` after a visible
    /// `CLC`/`SEC` is beyond it, so `XCE` leaves them alone.
    #[must_use]
    pub const fn after(self, d: &Disassembled) -> Widths {
        let imm = d.operand[0];
        match d.insn.op {
            Op::REP if !self.e => Widths {
                m8: self.m8 && imm & super::flags::M == 0,
                x8: self.x8 && imm & super::flags::X == 0,
                e: false,
            },
            Op::SEP => Widths {
                m8: self.m8 || imm & super::flags::M != 0,
                x8: self.x8 || imm & super::flags::X != 0,
                e: self.e,
            },
            _ => self,
        }
    }
}

/// One decoded instruction at a known 24-bit address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disassembled {
    /// Address the instruction was decoded at: `PBR:PC`.
    pub pc: u32,
    /// The opcode byte.
    pub opcode: u8,
    /// The table row for [`Disassembled::opcode`].
    pub insn: Insn,
    /// The operand bytes, low byte first. Unused bytes are zero.
    pub operand: [u8; 3],
    /// How many bytes the instruction occupies, opcode included.
    pub len: u8,
    /// Whether every byte the instruction needs was available.
    pub truncated: bool,
}

impl Disassembled {
    /// The 16-bit operand, low byte first.
    #[must_use]
    pub const fn word(&self) -> u16 {
        (self.operand[0] as u16) | ((self.operand[1] as u16) << 8)
    }

    /// The 24-bit operand, low byte first.
    #[must_use]
    pub const fn long(&self) -> u32 {
        (self.operand[0] as u32)
            | ((self.operand[1] as u32) << 8)
            | ((self.operand[2] as u32) << 16)
    }

    /// The address a branch (or `PER`) refers to: inside the program bank,
    /// because the program counter wraps there.
    #[must_use]
    pub const fn branch_target(&self) -> Option<u32> {
        let bank = self.pc & 0xff_0000;
        let next = (self.pc as u16).wrapping_add(self.len as u16);
        match self.insn.mode {
            Mode::Relative => Some(bank | next.wrapping_add(self.operand[0] as i8 as u16) as u32),
            Mode::RelativeLong => Some(bank | next.wrapping_add(self.word()) as u32),
            _ => None,
        }
    }
}

impl fmt::Display for Disassembled {
    /// WDC assembler syntax: uppercase mnemonic, lowercase hex, `[...]` for a
    /// long indirection, and source-then-destination for a block move.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.insn.op.mnemonic())?;
        let lo = self.operand[0];
        let w = self.word();
        match self.insn.mode {
            Mode::Implied => Ok(()),
            Mode::Accumulator => f.write_str(" A"),
            Mode::ImmediateM | Mode::ImmediateX => {
                if self.len == 3 {
                    write!(f, " #${w:04x}")
                } else {
                    write!(f, " #${lo:02x}")
                }
            }
            Mode::Immediate8 => write!(f, " #${lo:02x}"),
            Mode::Absolute => write!(f, " ${w:04x}"),
            Mode::AbsoluteX => write!(f, " ${w:04x},X"),
            Mode::AbsoluteY => write!(f, " ${w:04x},Y"),
            Mode::AbsoluteLong => write!(f, " ${:06x}", self.long()),
            Mode::AbsoluteLongX => write!(f, " ${:06x},X", self.long()),
            Mode::AbsoluteIndirect => write!(f, " (${w:04x})"),
            Mode::AbsoluteIndirectLong => write!(f, " [${w:04x}]"),
            Mode::AbsoluteIndexedIndirect => write!(f, " (${w:04x},X)"),
            Mode::Direct => write!(f, " ${lo:02x}"),
            Mode::DirectX => write!(f, " ${lo:02x},X"),
            Mode::DirectY => write!(f, " ${lo:02x},Y"),
            Mode::DirectIndirect => write!(f, " (${lo:02x})"),
            Mode::DirectIndirectLong => write!(f, " [${lo:02x}]"),
            Mode::DirectIndexedIndirect => write!(f, " (${lo:02x},X)"),
            Mode::DirectIndirectIndexed => write!(f, " (${lo:02x}),Y"),
            Mode::DirectIndirectLongIndexed => write!(f, " [${lo:02x}],Y"),
            Mode::StackRelative => write!(f, " ${lo:02x},S"),
            Mode::StackRelativeIndirectIndexed => write!(f, " (${lo:02x},S),Y"),
            Mode::Relative | Mode::RelativeLong => write!(
                f,
                " ${:06x}",
                self.branch_target().expect("a relative mode has a target")
            ),
            // Object code is destination bank then source bank; the assembler
            // writes them the other way round.
            Mode::BlockMove => write!(f, " ${:02x},${lo:02x}", self.operand[1]),
        }
    }
}

/// Disassemble one instruction from `bytes`, decoded at `pc` under `widths`.
///
/// Missing bytes read as zero and set [`Disassembled::truncated`].
#[must_use]
pub fn disassemble(pc: u32, bytes: &[u8], widths: Widths) -> Disassembled {
    let opcode = bytes.first().copied().unwrap_or(0);
    let insn = decode(opcode);
    let len = insn.bytes(widths.m8, widths.x8);
    let mut operand = [0u8; 3];
    for (i, slot) in operand.iter_mut().enumerate().take(usize::from(len) - 1) {
        *slot = bytes.get(i + 1).copied().unwrap_or(0);
    }
    Disassembled {
        pc: pc & 0xff_ffff,
        opcode,
        insn,
        operand,
        len,
        truncated: bytes.len() < usize::from(len),
    }
}

/// Disassemble `count` instructions starting at `pc`, fetching bytes through
/// `read` and following `REP`/`SEP` as it goes.
///
/// The program counter wraps inside its bank, as the processor's does. A
/// `read` that returns `None` marks the instruction truncated.
pub fn disassemble_run(
    pc: u32,
    count: usize,
    mut widths: Widths,
    mut read: impl FnMut(u32) -> Option<u8>,
) -> Vec<Disassembled> {
    let mut out = Vec::with_capacity(count);
    let bank = pc & 0xff_0000;
    let mut at = pc as u16;
    for _ in 0..count {
        let mut bytes = [0u8; 4];
        let mut have = 0usize;
        let mut missing = false;
        for (i, slot) in bytes.iter_mut().enumerate() {
            match read(bank | u32::from(at.wrapping_add(i as u16))) {
                Some(b) => {
                    *slot = b;
                    if !missing {
                        have += 1;
                    }
                }
                None => missing = true,
            }
        }
        let d = disassemble(bank | u32::from(at), &bytes[..have], widths);
        widths = widths.after(&d);
        at = at.wrapping_add(u16::from(d.len));
        out.push(d);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;
    use alloc::string::ToString;

    fn one(bytes: &[u8], w: Widths) -> alloc::string::String {
        disassemble(0x12_8000, bytes, w).to_string()
    }

    #[test]
    fn every_mode_prints_in_wdc_syntax() {
        let e = Widths::EMULATION;
        assert_eq!(one(&[0xea], e), "NOP");
        assert_eq!(one(&[0x0a], e), "ASL A");
        assert_eq!(one(&[0xc2, 0x30], e), "REP #$30");
        assert_eq!(one(&[0xad, 0x34, 0x12], e), "LDA $1234");
        assert_eq!(one(&[0xbf, 0x56, 0x34, 0x12], e), "LDA $123456,X");
        assert_eq!(one(&[0xdc, 0x34, 0x12], e), "JML [$1234]");
        assert_eq!(one(&[0xfc, 0x34, 0x12], e), "JSR ($1234,X)");
        assert_eq!(one(&[0xb7, 0x12], e), "LDA [$12],Y");
        assert_eq!(one(&[0xb3, 0x05], e), "LDA ($05,S),Y");
        assert_eq!(one(&[0x54, 0x7e, 0x7f], e), "MVN $7f,$7e");
        // A branch prints its target, which stays in the program bank.
        assert_eq!(one(&[0x80, 0xfe], e), "BRA $128000");
        assert_eq!(one(&[0x82, 0x00, 0x80], e), "BRL $120003");
    }

    #[test]
    fn a_run_follows_rep_and_sep() {
        // REP #$30; LDA #$1234; LDX #$5678; SEP #$20; LDA #$9a
        let code = [
            0xc2, 0x30, 0xa9, 0x34, 0x12, 0xa2, 0x78, 0x56, 0xe2, 0x20, 0xa9, 0x9a,
        ];
        let listing = disassemble_run(0x00_8000, 5, Widths::from_p(0x30, false), |a| {
            code.get((a - 0x8000) as usize).copied()
        });
        let text: alloc::vec::Vec<_> = listing.iter().map(|d| format!("{d}")).collect();
        assert_eq!(
            text,
            [
                "REP #$30",
                "LDA #$1234",
                "LDX #$5678",
                "SEP #$20",
                "LDA #$9a"
            ]
        );
        assert!(listing.iter().all(|d| !d.truncated));
    }

    #[test]
    fn rep_cannot_widen_anything_in_emulation_mode() {
        let d = disassemble(0, &[0xc2, 0x30], Widths::EMULATION);
        assert_eq!(Widths::EMULATION.after(&d), Widths::EMULATION);
    }

    #[test]
    fn a_short_buffer_is_reported_not_a_panic() {
        let d = disassemble(0, &[0xaf, 0x00], Widths::EMULATION);
        assert!(d.truncated);
        assert_eq!(d.len, 4);
    }
}
