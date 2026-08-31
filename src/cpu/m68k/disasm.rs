//! The disassembler, generated from the same description the interpreter
//! decodes with.
//!
//! Not a side project: gdb's `disassemble`, the monitor's single-step display
//! and any trace log need it, and CLAUDE.md forbids describing the instruction
//! set twice. Everything here reads [`isa::TABLE`](super::isa::TABLE) through
//! [`decode`]; there is no second opcode list to keep in step, and the
//! extension words are walked in the order the interpreter consumes them.
//!
//! Motorola syntax, which is what every 68000 assembler and every listing in
//! the manual uses: `$1234(A0)`, `$12(A0,D1.w)`, `#$42`, `-(A7)`.
//!
//! ```
//! use rsemu::cpu::m68k::disasm::disassemble;
//!
//! // MOVE.W $1234(A0),D3
//! let d = disassemble(0x400, &[0x3628, 0x1234]);
//! assert_eq!(format!("{d}"), "MOVE.W $1234(A0),D3");
//! assert_eq!(d.len, 4);
//! ```

use alloc::vec::Vec;
use core::fmt;

use super::isa::{Arg, Cond, ILLEGAL_OPCODE, Insn, Mode, Op, Size, decode, ea_of};

/// The most extension words any 68000 instruction can carry.
///
/// `MOVE.L ($12345678).L,($12345678).L` is the worst case: two absolute long
/// operands, four words, and the register-list forms never exceed it.
pub const MAX_EXT_WORDS: usize = 4;

/// One decoded instruction at a known address.
///
/// Carries the raw words as well as the decoded row so a monitor can print
/// `000400: 3628 1234  MOVE.W $1234(A0),D3` without decoding twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disassembled {
    /// Address the instruction was decoded at.
    pub pc: u32,
    /// The opcode word.
    pub opcode: u16,
    /// The row [`decode`] returned for [`Disassembled::opcode`].
    pub insn: Insn,
    /// The resolved operand size, if the row has one.
    pub size: Option<Size>,
    /// The extension words, in the order they appear after the opcode.
    /// Unused slots are zero.
    pub ext: [u16; MAX_EXT_WORDS],
    /// How many of [`Disassembled::ext`] are used.
    pub ext_len: u8,
    /// How many bytes the instruction occupies, opcode included.
    pub len: u8,
    /// Whether every word the instruction needs was available.
    ///
    /// A monitor disassembling to the end of a buffer, or across an unmapped
    /// page, gets a best-effort decode with the missing words read as zero
    /// rather than a panic — but it is told.
    pub truncated: bool,
}

impl Disassembled {
    /// The address a branch would jump to, for `Bcc`, `BRA`, `BSR` and `DBcc`.
    ///
    /// Returns `None` for every other instruction. The displacement counts
    /// from the word *after* the opcode, which is why this is not `pc + len`.
    #[must_use]
    pub fn branch_target(&self) -> Option<u32> {
        let base = self.pc.wrapping_add(2);
        match self.insn.op {
            Op::Bra | Op::Bsr | Op::Bcc => {
                let byte = self.opcode as i8;
                if byte == 0 {
                    Some(base.wrapping_add(i32::from(self.ext[0] as i16) as u32))
                } else {
                    Some(base.wrapping_add(i32::from(byte) as u32))
                }
            }
            Op::Dbcc => Some(base.wrapping_add(i32::from(self.ext[0] as i16) as u32)),
            _ => None,
        }
    }

    /// The condition this instruction tests, if it tests one.
    #[must_use]
    pub fn condition(&self) -> Option<Cond> {
        if self.insn.op.is_conditional() {
            Some(Cond::from_opcode(self.opcode))
        } else {
            None
        }
    }

    /// The mnemonic with its condition and size suffixes, as an assembler
    /// would write it.
    #[must_use]
    pub fn mnemonic(&self) -> MnemonicOf<'_> {
        MnemonicOf(self)
    }
}

/// The mnemonic of a [`Disassembled`], with suffixes, as a `Display`.
#[derive(Debug, Clone, Copy)]
pub struct MnemonicOf<'a>(&'a Disassembled);

impl fmt::Display for MnemonicOf<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let d = self.0;
        f.write_str(d.insn.op.mnemonic())?;
        if let Some(cond) = d.condition() {
            // Bcc with cc = T or F is BRA or BSR, which have their own rows,
            // so a condition printed here is always a real one.
            f.write_str(cond.name())?;
        }
        // The size suffix is noise on instructions that have exactly one.
        let suffixed = matches!(
            d.insn.size,
            super::isa::SizeSpec::Bits76
                | super::isa::SizeSpec::Bit6
                | super::isa::SizeSpec::Bit8
                | super::isa::SizeSpec::Move
        );
        if suffixed && let Some(size) = d.size {
            write!(f, ".{}", size.suffix().to_ascii_uppercase())?;
        }
        Ok(())
    }
}

impl fmt::Display for Disassembled {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // A word that decodes to nothing is almost always data, and printing
        // it as `ILLEGAL` turns a table of constants into a page of plausible
        // instructions. Only the one encoding that *is* the ILLEGAL
        // instruction gets the mnemonic.
        if self.insn.op == Op::Illegal && self.opcode != ILLEGAL_OPCODE {
            return write!(f, "DC.W ${:04x}", self.opcode);
        }
        write!(f, "{}", self.mnemonic())?;
        // The whole point of a line-A or line-F trap is the twelve bits the
        // handler reads out of the opcode, so they are not dropped.
        if matches!(self.insn.op, Op::LineA | Op::LineF) {
            return write!(f, " ${:04x}", self.opcode);
        }
        // Two orders are in play and they are not the same one. Extension
        // words must be *consumed* in the order the instruction encodes them —
        // MOVEM's register-list word comes before the address it reads from,
        // whichever direction the transfer goes — while the operands must be
        // *printed* source first. Resolving them in encoding order and then
        // printing by slot is what keeps both true.
        let mut cursor = Cursor::new(self);
        let mut resolved: [Option<Operand>; 2] = [None, None];
        for (slot, arg) in self.operand_order() {
            resolved[slot as usize] = cursor.operand(arg);
        }
        for (printed, operand) in resolved.into_iter().flatten().enumerate() {
            f.write_str(if printed == 0 { " " } else { "," })?;
            operand.fmt(f)?;
        }
        Ok(())
    }
}

/// Which operand slot an [`Arg`] came from, so a `MOVE`'s destination reads
/// the right half of the opcode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Slot {
    Src = 0,
    Dst = 1,
}

impl Disassembled {
    /// The operands in the order their extension words appear.
    ///
    /// Source before destination everywhere except `MOVEM` into registers,
    /// whose register-list word precedes the effective address it reads from.
    fn operand_order(&self) -> [(Slot, Arg); 2] {
        let insn = self.insn;
        if insn.op == Op::Movem && insn.dst == Arg::RegList {
            [(Slot::Dst, insn.dst), (Slot::Src, insn.src)]
        } else {
            [(Slot::Src, insn.src), (Slot::Dst, insn.dst)]
        }
    }
}

/// Walks the extension words while formatting.
struct Cursor<'a> {
    d: &'a Disassembled,
    at: usize,
}

impl<'a> Cursor<'a> {
    fn new(d: &'a Disassembled) -> Cursor<'a> {
        Cursor { d, at: 0 }
    }

    fn next(&mut self) -> u16 {
        let word = self.d.ext.get(self.at).copied().unwrap_or(0);
        self.at += 1;
        word
    }

    /// Format one operand, consuming the extension words it owns.
    fn operand(&mut self, arg: Arg) -> Option<Operand> {
        let opcode = self.d.opcode;
        let size = self.d.size.unwrap_or(Size::Word);
        Some(match arg {
            Arg::None => return None,
            Arg::DnHi => Operand::Data(((opcode >> 9) & 7) as u8),
            Arg::DnLo => Operand::Data((opcode & 7) as u8),
            Arg::AnHi => Operand::Addr(((opcode >> 9) & 7) as u8),
            Arg::AnLo => Operand::Addr((opcode & 7) as u8),
            Arg::RmLo => {
                if opcode & 8 == 0 {
                    Operand::Data((opcode & 7) as u8)
                } else {
                    Operand::PreDec((opcode & 7) as u8)
                }
            }
            Arg::RmHi => {
                if opcode & 8 == 0 {
                    Operand::Data(((opcode >> 9) & 7) as u8)
                } else {
                    Operand::PreDec(((opcode >> 9) & 7) as u8)
                }
            }
            Arg::PostLo => Operand::PostInc((opcode & 7) as u8),
            Arg::PostHi => Operand::PostInc(((opcode >> 9) & 7) as u8),
            Arg::Quick => {
                let q = (opcode >> 9) & 7;
                Operand::Imm(if q == 0 { 8 } else { u32::from(q) })
            }
            Arg::QuickByte => Operand::Imm(i32::from(opcode as i8) as u32),
            Arg::Vector => Operand::Imm(u32::from(opcode & 0xf)),
            Arg::Imm => Operand::Imm(match size {
                Size::Byte => u32::from(self.next() & 0xff),
                Size::Word => u32::from(self.next()),
                Size::Long => {
                    let hi = self.next();
                    let lo = self.next();
                    (u32::from(hi) << 16) | u32::from(lo)
                }
            }),
            Arg::BitNumber => Operand::Imm(u32::from(self.next() & 0xff)),
            Arg::Disp8 => {
                if opcode as i8 == 0 {
                    self.next();
                }
                Operand::Target(self.d.branch_target().unwrap_or(self.d.pc))
            }
            Arg::Disp16 => {
                let word = self.next();
                match self.d.insn.op {
                    Op::Dbcc => Operand::Target(self.d.branch_target().unwrap_or(self.d.pc)),
                    // LINK's displacement is a signed stack adjustment, not a
                    // target — and it is nearly always negative.
                    _ => Operand::SignedImm(i32::from(word as i16)),
                }
            }
            Arg::Ccr => Operand::Ccr,
            Arg::Sr => Operand::Sr,
            Arg::Usp => Operand::Usp,
            Arg::ShiftCount => {
                if opcode & 0x0020 == 0 {
                    let q = (opcode >> 9) & 7;
                    Operand::Imm(if q == 0 { 8 } else { u32::from(q) })
                } else {
                    Operand::Data(((opcode >> 9) & 7) as u8)
                }
            }
            Arg::RegList => Operand::RegList(self.next(), self.d.predecrement_list()),
            Arg::MovepEa => Operand::Disp16(i32::from(self.next() as i16), (opcode & 7) as u8),
            Arg::Ea | Arg::EaDst => {
                let (mode, reg) = ea_of(arg, opcode)?;
                self.effective(mode, reg, size)
            }
        })
    }

    fn effective(&mut self, mode: Mode, reg: u8, size: Size) -> Operand {
        match mode {
            Mode::DataReg => Operand::Data(reg),
            Mode::AddrReg => Operand::Addr(reg),
            Mode::Indirect => Operand::Indirect(reg),
            Mode::PostInc => Operand::PostInc(reg),
            Mode::PreDec => Operand::PreDec(reg),
            Mode::Disp16 => Operand::Disp16(i32::from(self.next() as i16), reg),
            Mode::Index8 => {
                let ext = self.next();
                Operand::Index(i32::from(ext as i8), Some(reg), Index::from_ext(ext))
            }
            // Sign-extended, because $ff00.w addresses $ffff00 and a monitor
            // that prints the raw word sends the reader to the wrong place.
            Mode::AbsShort => Operand::AbsShort(i32::from(self.next() as i16) as u32),
            Mode::AbsLong => {
                let hi = self.next();
                let lo = self.next();
                Operand::AbsLong((u32::from(hi) << 16) | u32::from(lo))
            }
            Mode::PcDisp16 => Operand::PcDisp(i32::from(self.next() as i16)),
            Mode::PcIndex8 => {
                let ext = self.next();
                Operand::Index(i32::from(ext as i8), None, Index::from_ext(ext))
            }
            Mode::Imm => Operand::Imm(match size {
                Size::Byte => u32::from(self.next() & 0xff),
                Size::Word => u32::from(self.next()),
                Size::Long => {
                    let hi = self.next();
                    let lo = self.next();
                    (u32::from(hi) << 16) | u32::from(lo)
                }
            }),
        }
    }
}

impl Disassembled {
    /// Whether a `MOVEM` register list is written in predecrement order.
    fn predecrement_list(&self) -> bool {
        self.insn.op == Op::Movem && matches!(ea_of(Arg::Ea, self.opcode), Some((Mode::PreDec, _)))
    }
}

/// The index register named by a brief extension word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Index {
    addr: bool,
    reg: u8,
    long: bool,
}

impl Index {
    const fn from_ext(ext: u16) -> Index {
        Index {
            addr: ext & 0x8000 != 0,
            reg: ((ext >> 12) & 7) as u8,
            long: ext & 0x0800 != 0,
        }
    }
}

impl fmt::Display for Index {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}{}.{}",
            if self.addr { 'A' } else { 'D' },
            self.reg,
            if self.long { 'l' } else { 'w' }
        )
    }
}

/// One formatted operand.
#[derive(Debug, Clone, Copy)]
enum Operand {
    Data(u8),
    Addr(u8),
    Indirect(u8),
    PostInc(u8),
    PreDec(u8),
    Disp16(i32, u8),
    Index(i32, Option<u8>, Index),
    AbsShort(u32),
    AbsLong(u32),
    PcDisp(i32),
    Imm(u32),
    SignedImm(i32),
    Target(u32),
    RegList(u16, bool),
    Ccr,
    Sr,
    Usp,
}

impl fmt::Display for Operand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Operand::Data(n) => write!(f, "D{n}"),
            Operand::Addr(n) => write!(f, "A{n}"),
            Operand::Indirect(n) => write!(f, "(A{n})"),
            Operand::PostInc(n) => write!(f, "(A{n})+"),
            Operand::PreDec(n) => write!(f, "-(A{n})"),
            Operand::Disp16(d, n) => write!(f, "{}(A{n})", Signed(d)),
            Operand::Index(d, base, index) => match base {
                Some(n) => write!(f, "{}(A{n},{index})", Signed(d)),
                None => write!(f, "{}(PC,{index})", Signed(d)),
            },
            Operand::AbsShort(v) => write!(f, "${v:08x}.w"),
            Operand::AbsLong(v) => write!(f, "${v:08x}.l"),
            Operand::PcDisp(d) => write!(f, "{}(PC)", Signed(d)),
            Operand::Imm(v) => write!(f, "#${v:x}"),
            Operand::SignedImm(v) => write!(f, "#{}", Signed(v)),
            Operand::Target(v) => write!(f, "${v:06x}"),
            Operand::RegList(mask, predec) => write_reg_list(f, mask, predec),
            Operand::Ccr => f.write_str("CCR"),
            Operand::Sr => f.write_str("SR"),
            Operand::Usp => f.write_str("USP"),
        }
    }
}

/// A displacement, printed the way an assembler listing does.
struct Signed(i32);

impl fmt::Display for Signed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.0 < 0 {
            write!(f, "-${:x}", self.0.unsigned_abs())
        } else {
            write!(f, "${:x}", self.0)
        }
    }
}

/// Print a `MOVEM` register list as ranges: `D0-D3/D7/A0-A6`.
///
/// The mask is read the other way round for a predecrement destination, which
/// is the source of a great deal of hand-written-assembler grief and is worth
/// the disassembler getting right (M68000PRM, *MOVEM*).
fn write_reg_list(f: &mut fmt::Formatter<'_>, mask: u16, predecrement: bool) -> fmt::Result {
    let mut names = [false; 16];
    for bit in 0..16u32 {
        if mask & (1 << bit) != 0 {
            let index = if predecrement { 15 - bit } else { bit };
            names[index as usize] = true;
        }
    }
    let mut first = true;
    let mut i = 0usize;
    while i < 16 {
        if !names[i] {
            i += 1;
            continue;
        }
        let start = i;
        // A range never crosses from the data file into the address file.
        while i + 1 < 16 && names[i + 1] && (i + 1 != 8) {
            i += 1;
        }
        if !first {
            f.write_str("/")?;
        }
        first = false;
        write_reg_name(f, start)?;
        if i != start {
            f.write_str("-")?;
            write_reg_name(f, i)?;
        }
        i += 1;
    }
    if first {
        f.write_str("#$0")?;
    }
    Ok(())
}

fn write_reg_name(f: &mut fmt::Formatter<'_>, index: usize) -> fmt::Result {
    if index < 8 {
        write!(f, "D{index}")
    } else {
        write!(f, "A{}", index - 8)
    }
}

/// Disassemble one instruction from a slice of words.
///
/// Words the instruction needs but the slice does not have are read as zero
/// and [`Disassembled::truncated`] is set, so a monitor listing to the end of
/// a buffer degrades rather than panics.
#[must_use]
pub fn disassemble(pc: u32, words: &[u16]) -> Disassembled {
    let opcode = words.first().copied().unwrap_or(0);
    let insn = decode(opcode);
    let size = insn.size.resolve(opcode);
    let needed = ext_words(insn, opcode, size.unwrap_or(Size::Word));
    let mut ext = [0u16; MAX_EXT_WORDS];
    let mut truncated = words.is_empty();
    for (i, slot) in ext.iter_mut().enumerate().take(needed) {
        match words.get(i + 1) {
            Some(word) => *slot = *word,
            None => truncated = true,
        }
    }
    Disassembled {
        pc,
        opcode,
        insn,
        size,
        ext,
        ext_len: needed as u8,
        len: 2 + 2 * needed as u8,
        truncated,
    }
}

/// How many extension words an encoding carries.
///
/// Derived from the row and the addressing modes, which is the same
/// calculation the interpreter's prefetch slides perform — a length the
/// disassembler computed independently would drift.
fn ext_words(insn: Insn, opcode: u16, size: Size) -> usize {
    let mut count = 0usize;
    for arg in [insn.src, insn.dst] {
        count += match arg {
            Arg::Imm => {
                if size == Size::Long {
                    2
                } else {
                    1
                }
            }
            Arg::Disp16 | Arg::RegList | Arg::MovepEa | Arg::BitNumber => 1,
            Arg::Disp8 => usize::from(opcode as i8 == 0),
            Arg::Ea | Arg::EaDst => match ea_of(arg, opcode) {
                Some((mode, _)) => mode.ext_words(size) as usize,
                None => 0,
            },
            _ => 0,
        };
    }
    count.min(MAX_EXT_WORDS)
}

/// Disassemble `count` instructions starting at `pc`, reading guest memory
/// through `read_word`.
///
/// `read_word` returns `None` for a word that cannot be read, which stops the
/// run rather than inventing instructions out of a hole in the memory map.
pub fn disassemble_run(
    pc: u32,
    count: usize,
    mut read_word: impl FnMut(u32) -> Option<u16>,
) -> Vec<Disassembled> {
    let mut out = Vec::with_capacity(count);
    let mut at = pc;
    for _ in 0..count {
        let mut words = [0u16; MAX_EXT_WORDS + 1];
        let mut have = 0;
        for (i, slot) in words.iter_mut().enumerate() {
            match read_word(at.wrapping_add(2 * i as u32)) {
                Some(word) => {
                    *slot = word;
                    have += 1;
                }
                None => break,
            }
        }
        if have == 0 {
            break;
        }
        let d = disassemble(at, &words[..have]);
        at = at.wrapping_add(u32::from(d.len));
        out.push(d);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;

    fn text(pc: u32, words: &[u16]) -> alloc::string::String {
        format!("{}", disassemble(pc, words))
    }

    #[test]
    fn moves() {
        assert_eq!(text(0x400, &[0x3628, 0x1234]), "MOVE.W $1234(A0),D3");
        assert_eq!(text(0x400, &[0x2000]), "MOVE.L D0,D0");
        assert_eq!(text(0x400, &[0x7042]), "MOVEQ #$42,D0");
        assert_eq!(text(0x400, &[0x3040]), "MOVEA.W D0,A0");
        assert_eq!(
            text(0x400, &[0x23fc, 0x1234, 0x5678, 0x0000, 0x0100]),
            "MOVE.L #$12345678,$00000100.l"
        );
    }

    #[test]
    fn addressing_modes() {
        assert_eq!(text(0x400, &[0x1010]), "MOVE.B (A0),D0");
        assert_eq!(text(0x400, &[0x1018]), "MOVE.B (A0)+,D0");
        assert_eq!(text(0x400, &[0x1020]), "MOVE.B -(A0),D0");
        assert_eq!(text(0x400, &[0x1030, 0x1004]), "MOVE.B $4(A0,D1.w),D0");
        assert_eq!(text(0x400, &[0x1038, 0x0100]), "MOVE.B $00000100.w,D0");
        // The short form is sign-extended, so a negative one addresses the top
        // of the map rather than the bottom.
        assert_eq!(text(0x400, &[0x1038, 0xff00]), "MOVE.B $ffffff00.w,D0");
        assert_eq!(text(0x400, &[0x103a, 0x0010]), "MOVE.B $10(PC),D0");
        assert_eq!(text(0x400, &[0x103b, 0x8002]), "MOVE.B $2(PC,A0.w),D0");
        assert_eq!(text(0x400, &[0x103c, 0x0042]), "MOVE.B #$42,D0");
    }

    #[test]
    fn branches_show_their_target() {
        // BRA.S +$10 from $400 lands at $412.
        let d = disassemble(0x400, &[0x6010]);
        assert_eq!(d.branch_target(), Some(0x412));
        assert_eq!(format!("{d}"), "BRA $000412");
        // BNE with a word displacement.
        let d = disassemble(0x400, &[0x6600, 0x0100]);
        assert_eq!(d.branch_target(), Some(0x502));
        assert_eq!(d.len, 4);
        assert_eq!(format!("{d}"), "BNE $000502");
    }

    #[test]
    fn register_lists_collapse_to_ranges() {
        // MOVEM.L D0-D3/A0,-(A7)
        assert_eq!(text(0x400, &[0x48e7, 0xf080]), "MOVEM.L D0-D3/A0,-(A7)");
        // MOVEM.L (A7)+,D0-D3/A0
        assert_eq!(text(0x400, &[0x4cdf, 0x010f]), "MOVEM.L (A7)+,D0-D3/A0");
    }

    #[test]
    fn control_and_privileged() {
        assert_eq!(text(0x400, &[0x4e71]), "NOP");
        assert_eq!(text(0x400, &[0x4e75]), "RTS");
        assert_eq!(text(0x400, &[0x4e4f]), "TRAP #$f");
        assert_eq!(text(0x400, &[0x46fc, 0x2700]), "MOVE #$2700,SR");
        assert_eq!(text(0x400, &[0x4e68]), "MOVE USP,A0");
        assert_eq!(text(0x400, &[0x4e72, 0x2700]), "STOP #$2700");
        assert_eq!(text(0x400, &[0x4e50, 0xfff0]), "LINK A0,#-$10");
        // A word that decodes to nothing is data, and only $4afc is the
        // instruction that asks for the trap on purpose.
        assert_eq!(text(0x400, &[0x4afd]), "DC.W $4afd");
        assert_eq!(text(0x400, &[0xa123]), "LINEA $a123");
        assert_eq!(text(0x400, &[0xf123]), "LINEF $f123");
        assert_eq!(text(0x400, &[0x4afc]), "ILLEGAL");
    }

    #[test]
    fn conditionals_carry_their_suffix() {
        assert_eq!(text(0x400, &[0x57c0]), "SEQ D0");
        assert_eq!(text(0x400, &[0x51c8, 0xfffc]), "DBF D0,$0003fe");
        assert_eq!(text(0x400, &[0xe148]), "LSL.W #$8,D0");
    }

    #[test]
    fn a_static_bit_number_is_one_word_whatever_the_operand_size() {
        // The operand size is long when the destination is a data register,
        // but the bit number is a byte in one extension word either way — so
        // both of these are four bytes long, not six and four.
        let d = disassemble(0x400, &[0x0800, 0x0005]);
        assert_eq!(format!("{d}"), "BTST #$5,D0");
        assert_eq!(d.len, 4);
        let d = disassemble(0x400, &[0x08d0, 0x0005]);
        assert_eq!(format!("{d}"), "BSET #$5,(A0)");
        assert_eq!(d.len, 4);
    }

    #[test]
    fn a_truncated_read_is_reported_not_panicked() {
        let d = disassemble(0x400, &[0x3628]);
        assert!(d.truncated);
        assert_eq!(d.len, 4);
        let d = disassemble(0x400, &[]);
        assert!(d.truncated);
    }

    #[test]
    fn a_run_stops_at_a_hole() {
        let words = [0x4e71u16, 0x4e71];
        let run = disassemble_run(0x400, 8, |addr| {
            let index = (addr - 0x400) / 2;
            words.get(index as usize).copied()
        });
        assert_eq!(run.len(), 2);
        assert_eq!(run[1].pc, 0x402);
    }
}
