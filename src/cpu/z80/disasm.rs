//! The disassembler, generated from the same tables the interpreter decodes
//! with.
//!
//! Not a side project: gdb's `disassemble`, the monitor's single-step display
//! and any trace log need it, and CLAUDE.md forbids describing the instruction
//! set twice. Everything here reads [`isa`](super::isa); there is no second
//! opcode list to keep in step, and the index pages are derived by the same
//! two functions the interpreter uses.
//!
//! ```
//! use rsemu::cpu::z80::disasm::disassemble;
//!
//! let d = disassemble(0x8000, &[0xdd, 0x36, 0x05, 0x42]);
//! assert_eq!(format!("{d}"), "LD (IX+$05),$42");
//! assert_eq!(d.len, 4);
//!
//! // The undocumented DDCB forms write a register as well as memory, and the
//! // disassembly says so rather than hiding it.
//! let d = disassemble(0x8000, &[0xdd, 0xcb, 0xfe, 0x00]);
//! assert_eq!(format!("{d}"), "RLC (IX-$02),B");
//! ```

use alloc::string::String;
use core::fmt;

use super::isa::{Class, Cond, Index, Insn, Operand, decode, decode_cb, decode_ddcb};
use super::isa::{decode_ed, index_substitute};

/// Which opcode page an encoding ended up on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Page {
    /// No prefix.
    Base,
    /// The `$cb` page.
    Cb,
    /// The `$ed` page.
    Ed,
    /// The `$dd`/`$fd` view of the base page.
    Index,
    /// The `$dd $cb d` / `$fd $cb d` page.
    IndexCb,
}

/// How many bytes [`disassemble`] will look at.
///
/// Four is the longest real Z80 instruction (`DD CB d op`). The extra room is
/// for a run of redundant index prefixes, which is legal and which the CPU
/// treats as part of the same instruction.
pub const WINDOW: usize = 6;

/// One decoded instruction at a known address.
///
/// Carries the decoded row *and* the raw fields, so a monitor can print
/// `8000: dd 36 05 42  LD (IX+$05),$42` without decoding twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disassembled {
    /// Address the instruction was decoded at.
    pub pc: u16,
    /// The opcode byte of the page the instruction finally landed on.
    pub opcode: u8,
    /// Which page that was.
    pub page: Page,
    /// The index register a `$dd`/`$fd` prefix selected, if any.
    pub index: Option<Index>,
    /// The table row for this encoding, index substitution applied.
    pub insn: Insn,
    /// The signed displacement of an `(IX+d)` operand. Zero when there is
    /// none.
    pub displacement: i8,
    /// The immediate operand, whether one byte or two. Zero when there is
    /// none.
    pub immediate: u16,
    /// How many bytes the instruction occupies, every prefix included.
    pub len: u8,
    /// Whether every byte the instruction needs was available.
    ///
    /// A monitor disassembling to the end of a buffer, or across an unmapped
    /// page, gets a best-effort decode with the missing bytes read as zero
    /// rather than a panic — but it is told.
    pub truncated: bool,
}

impl Disassembled {
    /// The address a [`Operand::Rel`] branch would jump to.
    ///
    /// The displacement is signed and counts from the *next* instruction.
    /// Returns `None` for everything but `JR` and `DJNZ`.
    #[must_use]
    pub const fn branch_target(&self) -> Option<u16> {
        if matches!(self.insn.src, Operand::Rel) {
            // Guest arithmetic wraps: a branch near $ffff wraps to $0000, and
            // that is the address the CPU jumps to.
            Some(
                self.pc
                    .wrapping_add(self.len as u16)
                    .wrapping_add(self.immediate as u8 as i8 as u16),
            )
        } else {
            None
        }
    }

    /// The address the instruction ultimately reads or writes, where that can
    /// be known without the register file.
    #[must_use]
    pub const fn static_target(&self) -> Option<u16> {
        if matches!(self.insn.dst, Operand::Abs) || matches!(self.insn.src, Operand::Abs) {
            Some(self.immediate)
        } else {
            self.branch_target()
        }
    }

    /// Whether the encoding is undocumented.
    #[must_use]
    pub const fn is_undocumented(&self) -> bool {
        matches!(self.insn.class, Class::Undocumented)
    }
}

/// Render one operand with the instruction's actual bytes filled in.
fn write_operand(f: &mut fmt::Formatter<'_>, d: &Disassembled, operand: Operand) -> fmt::Result {
    match operand {
        Operand::None => Ok(()),
        Operand::Reg(r) => f.write_str(r.name()),
        Operand::Reg16(r) => f.write_str(r.name()),
        Operand::Ind(r) => write!(f, "({})", r.name()),
        Operand::Ptr(r) => write!(f, "({})", r.name()),
        Operand::Idx(r) => {
            // Assembler convention writes the sign, so an unsigned magnitude
            // reads correctly either way.
            if d.displacement < 0 {
                write!(f, "({}-${:02x})", r.name(), d.displacement.unsigned_abs())
            } else {
                write!(f, "({}+${:02x})", r.name(), d.displacement)
            }
        }
        Operand::Imm8 => write!(f, "${:02x}", d.immediate as u8),
        Operand::Imm16 => write!(f, "${:04x}", d.immediate),
        Operand::Abs => write!(f, "(${:04x})", d.immediate),
        // A branch is far more useful as a target than as a displacement,
        // which is why the raw byte stays in `immediate` for anyone who wants
        // it.
        Operand::Rel => write!(
            f,
            "${:04x}",
            d.branch_target().expect("a relative operand has a target")
        ),
        Operand::Bit(n) => write!(f, "{n}"),
        Operand::Rst(n) => write!(f, "${n:02x}"),
        Operand::Mode(n) => write!(f, "{n}"),
        Operand::PortC => f.write_str("(C)"),
        Operand::PortImm => write!(f, "(${:02x})", d.immediate as u8),
        Operand::Zero => f.write_str("0"),
    }
}

impl fmt::Display for Disassembled {
    /// Standard Z80 assembler syntax: uppercase mnemonic, lowercase hex.
    ///
    /// Undocumented encodings are printed with no marker; the caller has
    /// [`Insn::class`] and can decide whether its audience wants a `*`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.insn.op.mnemonic())?;
        let mut wrote = false;
        if self.insn.cond != Cond::Always {
            write!(f, " {}", self.insn.cond)?;
            wrote = true;
        }
        for operand in [self.insn.dst, self.insn.src] {
            if operand == Operand::None {
                continue;
            }
            f.write_str(if wrote { "," } else { " " })?;
            wrote = true;
            write_operand(f, self, operand)?;
        }
        // The undocumented DDCB forms write a register as well as memory. It
        // is the third operand in everything but name, so it prints like one.
        if let Some(also) = self.insn.also {
            write!(f, ",{}", also.name())?;
        }
        Ok(())
    }
}

/// Decode the instruction at `pc` from `bytes`, which start at `pc`.
///
/// Never fails: every encoding on every page is defined, and the `ED` page's
/// holes decode as an undocumented `NOP` because that is what they do. A
/// buffer too short for the operand yields a decode with zeroed operand bytes
/// and [`Disassembled::truncated`] set.
#[must_use]
pub fn disassemble(pc: u16, bytes: &[u8]) -> Disassembled {
    let mut at = 0usize;
    let mut truncated = false;
    let take = |at: &mut usize, truncated: &mut bool| -> u8 {
        match bytes.get(*at) {
            Some(b) => {
                *at += 1;
                *b
            }
            None => {
                *truncated = true;
                *at += 1;
                0
            }
        }
    };

    let mut index: Option<Index> = None;
    let mut page = Page::Base;
    let mut opcode;
    let insn;
    loop {
        opcode = take(&mut at, &mut truncated);
        match opcode {
            0xdd if at < WINDOW => index = Some(Index::Ix),
            0xfd if at < WINDOW => index = Some(Index::Iy),
            0xcb => {
                match index {
                    Some(i) => {
                        page = Page::IndexCb;
                        let d = take(&mut at, &mut truncated);
                        opcode = take(&mut at, &mut truncated);
                        insn = decode_ddcb(opcode, i);
                        let mut out = finish(pc, opcode, page, index, insn, at, truncated);
                        out.displacement = d as i8;
                        return out;
                    }
                    None => {
                        page = Page::Cb;
                        opcode = take(&mut at, &mut truncated);
                        insn = decode_cb(opcode);
                    }
                }
                break;
            }
            0xed => {
                page = Page::Ed;
                opcode = take(&mut at, &mut truncated);
                insn = decode_ed(opcode);
                break;
            }
            other => {
                insn = match index {
                    Some(i) => {
                        page = Page::Index;
                        index_substitute(decode(other), i)
                    }
                    None => decode(other),
                };
                break;
            }
        }
        if at >= WINDOW {
            // A run of index prefixes longer than the window: report what was
            // read rather than looping, and say it was cut short.
            insn = decode(opcode);
            truncated = true;
            break;
        }
    }

    // The trailing operand bytes, in encoding order: an index displacement
    // comes before an immediate, which is why `LD (IX+d),n` reads `d` first.
    let mut displacement = 0i8;
    let mut immediate = 0u16;
    for operand in [insn.dst, insn.src] {
        match operand {
            Operand::Idx(_) => displacement = take(&mut at, &mut truncated) as i8,
            Operand::Imm8 | Operand::Rel | Operand::PortImm => {
                immediate = u16::from(take(&mut at, &mut truncated));
            }
            Operand::Imm16 | Operand::Abs => {
                let lo = take(&mut at, &mut truncated);
                let hi = take(&mut at, &mut truncated);
                immediate = u16::from(lo) | (u16::from(hi) << 8);
            }
            _ => {}
        }
    }
    let mut out = finish(pc, opcode, page, index, insn, at, truncated);
    out.displacement = displacement;
    out.immediate = immediate;
    out
}

fn finish(
    pc: u16,
    opcode: u8,
    page: Page,
    index: Option<Index>,
    insn: Insn,
    len: usize,
    truncated: bool,
) -> Disassembled {
    Disassembled {
        pc,
        opcode,
        page,
        index,
        insn,
        displacement: 0,
        immediate: 0,
        len: len as u8,
        truncated,
    }
}

/// Decode `count` instructions starting at `pc`, calling `fetch` for each
/// byte.
///
/// The closure is how a monitor reaches guest memory without this module
/// knowing what an address space is — and how a debugger passes a
/// side-effect-free read (`MemAttrs::debug`, `ROADMAP.md` §15 invariant 5).
/// Iteration stops early if `fetch` returns `None`.
pub fn disassemble_run(
    pc: u16,
    count: usize,
    mut fetch: impl FnMut(u16) -> Option<u8>,
) -> alloc::vec::Vec<Disassembled> {
    let mut out = alloc::vec::Vec::with_capacity(count);
    let mut at = pc;
    for _ in 0..count {
        let mut window = [0u8; WINDOW];
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
        let stop = d.truncated;
        out.push(d);
        if stop {
            break;
        }
    }
    out
}

/// The symbolic form of a row: mnemonic and operand *shapes*, with `n`, `nn`
/// and `d` standing in for bytes there is no instruction stream to read.
///
/// This is what `rsemu describe cpu.z80` prints, so the description of the
/// instruction set and the thing that executes it come from one table.
#[must_use]
pub fn mnemonic_and_operands(insn: Insn) -> String {
    use core::fmt::Write as _;
    let mut out = String::from(insn.op.mnemonic());
    let mut wrote = false;
    if insn.cond != Cond::Always {
        let _ = write!(out, " {}", insn.cond);
        wrote = true;
    }
    for operand in [insn.dst, insn.src] {
        if operand == Operand::None {
            continue;
        }
        out.push(if wrote { ',' } else { ' ' });
        wrote = true;
        let _ = match operand {
            Operand::Reg(r) => write!(out, "{}", r.name()),
            Operand::Reg16(r) => write!(out, "{}", r.name()),
            Operand::Ind(r) | Operand::Ptr(r) => write!(out, "({})", r.name()),
            Operand::Idx(r) => write!(out, "({}+d)", r.name()),
            Operand::Imm8 => write!(out, "n"),
            Operand::Imm16 => write!(out, "nn"),
            Operand::Abs => write!(out, "(nn)"),
            Operand::Rel => write!(out, "e"),
            Operand::Bit(n) => write!(out, "{n}"),
            Operand::Rst(n) => write!(out, "${n:02x}"),
            Operand::Mode(n) => write!(out, "{n}"),
            Operand::PortC => write!(out, "(C)"),
            Operand::PortImm => write!(out, "(n)"),
            Operand::Zero => write!(out, "0"),
            Operand::None => Ok(()),
        };
    }
    if let Some(also) = insn.also {
        let _ = write!(out, ",{}", also.name());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::super::isa::Op;
    use super::*;
    use alloc::format;
    use alloc::vec::Vec;

    fn text(bytes: &[u8]) -> String {
        format!("{}", disassemble(0x8000, bytes))
    }

    #[test]
    fn the_base_page_prints_assembler_syntax() {
        assert_eq!(text(&[0x00]), "NOP");
        assert_eq!(text(&[0x3e, 0x42]), "LD A,$42");
        assert_eq!(text(&[0x21, 0x34, 0x12]), "LD HL,$1234");
        assert_eq!(text(&[0x32, 0x34, 0x12]), "LD ($1234),A");
        assert_eq!(text(&[0x77]), "LD (HL),A");
        assert_eq!(text(&[0x86]), "ADD A,(HL)");
        assert_eq!(text(&[0xd6, 0x10]), "SUB $10");
        assert_eq!(text(&[0xe9]), "JP (HL)");
        assert_eq!(text(&[0xeb]), "EX DE,HL");
        assert_eq!(text(&[0x08]), "EX AF,AF'");
        assert_eq!(text(&[0xc7]), "RST $00");
        assert_eq!(text(&[0xff]), "RST $38");
        assert_eq!(text(&[0xdb, 0xfe]), "IN A,($fe)");
        assert_eq!(text(&[0xd3, 0xfe]), "OUT ($fe),A");
    }

    #[test]
    fn conditions_print_before_the_operands() {
        assert_eq!(text(&[0xc0]), "RET NZ");
        assert_eq!(text(&[0xc2, 0x00, 0x40]), "JP NZ,$4000");
        assert_eq!(text(&[0xdc, 0x00, 0x40]), "CALL C,$4000");
    }

    #[test]
    fn relative_branches_print_their_target() {
        // JR $+2 with e = 0 lands right after the instruction.
        assert_eq!(text(&[0x18, 0x00]), "JR $8002");
        assert_eq!(text(&[0x18, 0xfe]), "JR $8000");
        assert_eq!(text(&[0x20, 0x05]), "JR NZ,$8007");
        assert_eq!(text(&[0x10, 0xfb]), "DJNZ $7ffd");
    }

    #[test]
    fn the_prefix_pages_print_the_registers_they_really_use() {
        assert_eq!(text(&[0xcb, 0x00]), "RLC B");
        assert_eq!(text(&[0xcb, 0x46]), "BIT 0,(HL)");
        assert_eq!(text(&[0xcb, 0x30]), "SLL B");
        assert_eq!(text(&[0xed, 0xb0]), "LDIR");
        assert_eq!(text(&[0xed, 0x40]), "IN B,(C)");
        assert_eq!(text(&[0xed, 0x71]), "OUT (C),0");
        assert_eq!(text(&[0xed, 0x5e]), "IM 2");
        assert_eq!(text(&[0xed, 0x57]), "LD A,I");
        assert_eq!(text(&[0xdd, 0x21, 0x34, 0x12]), "LD IX,$1234");
        assert_eq!(text(&[0xdd, 0x7e, 0x05]), "LD A,(IX+$05)");
        assert_eq!(text(&[0xfd, 0x7e, 0xfb]), "LD A,(IY-$05)");
        // The half registers only exist through a prefix, and only when no
        // displacement is in play.
        assert_eq!(text(&[0xdd, 0x65]), "LD IXH,IXL");
        assert_eq!(text(&[0xdd, 0x66, 0x02]), "LD H,(IX+$02)");
        assert_eq!(text(&[0xdd, 0xcb, 0x02, 0x46]), "BIT 0,(IX+$02)");
        assert_eq!(text(&[0xdd, 0xcb, 0x02, 0x06]), "RLC (IX+$02)");
        assert_eq!(text(&[0xfd, 0xcb, 0x02, 0xc1]), "SET 0,(IY+$02),C");
    }

    #[test]
    fn lengths_match_the_encodings() {
        for (bytes, len) in [
            (&[0x00u8][..], 1u8),
            (&[0x3e, 0x42], 2),
            (&[0x21, 0x34, 0x12], 3),
            (&[0xcb, 0x00], 2),
            (&[0xed, 0xb0], 2),
            (&[0xdd, 0x7e, 0x05], 3),
            (&[0xdd, 0x36, 0x05, 0x42], 4),
            (&[0xdd, 0xcb, 0x02, 0x06], 4),
        ] {
            let d = disassemble(0x8000, bytes);
            assert_eq!(d.len, len, "{bytes:02x?}");
            assert!(!d.truncated, "{bytes:02x?}");
        }
    }

    #[test]
    fn a_short_buffer_is_reported_rather_than_panicking() {
        let d = disassemble(0x8000, &[0x21]);
        assert!(d.truncated);
        assert_eq!(d.insn.op, Op::LD);

        let d = disassemble(0x8000, &[0xdd, 0xcb]);
        assert!(d.truncated);
    }

    #[test]
    fn a_run_walks_forward_by_each_instructions_length() {
        let code = [0x00u8, 0x3e, 0x42, 0xcb, 0x00, 0xdd, 0x36, 0x01, 0x02];
        let run = disassemble_run(0x8000, 4, |addr| {
            code.get(addr.wrapping_sub(0x8000) as usize).copied()
        });
        let text: Vec<String> = run.iter().map(|d| format!("{d}")).collect();
        assert_eq!(text, ["NOP", "LD A,$42", "RLC B", "LD (IX+$01),$02"]);
        assert_eq!(run[3].pc, 0x8005);
    }

    #[test]
    fn every_base_encoding_disassembles_to_something_nonempty() {
        // The prefix bytes are the only rows with no instruction of their own,
        // and even those must not print an empty line.
        for opcode in 0..=255u8 {
            let d = disassemble(0x8000, &[opcode, 0, 0, 0, 0, 0]);
            assert!(!format!("{d}").is_empty(), "opcode {opcode:02x}");
        }
    }

    #[test]
    fn the_symbolic_form_names_operand_shapes() {
        assert_eq!(mnemonic_and_operands(decode(0x3e)), "LD A,n");
        assert_eq!(mnemonic_and_operands(decode(0x21)), "LD HL,nn");
        assert_eq!(mnemonic_and_operands(decode(0x18)), "JR e");
        assert_eq!(
            mnemonic_and_operands(index_substitute(decode(0x36), Index::Ix)),
            "LD (IX+d),n"
        );
    }
}
