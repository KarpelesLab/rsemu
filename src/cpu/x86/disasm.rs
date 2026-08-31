//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! Not a side project: gdb's `disassemble`, the monitor's single-step display
//! and any trace log need it, and CLAUDE.md forbids describing the instruction
//! set twice. Everything here reads [`isa::TABLE`] through
//! [`isa::decode_stream`] — the same function the
//! interpreter fetches with — so there is no second opcode list and no second
//! idea of where an instruction ends.
//!
//! # Syntax
//!
//! Intel order (destination first), lower case, `0x`-prefixed hex, and the
//! segment written inside the brackets it applies to:
//!
//! ```
//! use rsemu::cpu::x86::disasm::disassemble;
//!
//! let d = disassemble(0xf000, 0xfff0, &[0x26, 0x03, 0x46, 0xfe]);
//! assert_eq!(format!("{d}"), "add ax, [es:bp-0x2]");
//! assert_eq!(d.len, 4);
//! ```
//!
//! An operand size is printed only when nothing else in the instruction fixes
//! it — `inc word [bx]` needs it, `add ax, [bx]` does not.

use alloc::vec::Vec;
use core::fmt;

use super::isa::{self, Arg, Class, Fields, ModRm, Op, Rep, seg};

/// The 8-bit register names, in ModRM `reg` order.
pub const REG8: [&str; 8] = ["al", "cl", "dl", "bl", "ah", "ch", "dh", "bh"];

/// The 16-bit register names, in ModRM `reg` order.
pub const REG16: [&str; 8] = ["ax", "cx", "dx", "bx", "sp", "bp", "si", "di"];

/// The address expression each `rm` value selects, for `md != 3`.
///
/// `rm == 6` is the odd one out: with `md == 0` it is a direct 16-bit address
/// instead, which is why the entry here is only reached for `md` of 1 or 2.
const RM_TERMS: [&str; 8] = ["bx+si", "bx+di", "bp+si", "bp+di", "si", "di", "bp", "bx"];

/// How long an instruction this type keeps the raw bytes of.
///
/// Long enough for every encoding an assembler produces (up to seven bytes)
/// with room for a prefix run; a pathological run of prefixes is still decoded
/// correctly, it just is not echoed in full.
pub const MAX_KEPT_BYTES: usize = 16;

/// One decoded instruction at a known `CS:IP`.
///
/// Carries the raw bytes as well as the decoded fields so a monitor can print
/// `f000:fff0  ea 5b e0 00 f0   jmpf 0xf000:0xe05b` without decoding twice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Disassembled {
    /// Code segment the instruction was decoded in.
    pub cs: u16,
    /// Offset the instruction starts at.
    pub ip: u16,
    /// Everything the decoder extracted, including the resolved table row.
    pub fields: Fields,
    /// The raw bytes, up to [`MAX_KEPT_BYTES`].
    pub bytes: [u8; MAX_KEPT_BYTES],
    /// How many bytes the instruction occupies, prefixes included.
    pub len: u8,
    /// Whether every byte the instruction needs was available.
    ///
    /// A monitor disassembling to the end of a buffer, or across an unmapped
    /// page, gets a best-effort decode with the missing bytes read as zero
    /// rather than a panic — but it is told.
    pub truncated: bool,
}

impl Disassembled {
    /// The offset of the instruction that follows this one.
    ///
    /// Guest arithmetic wraps: an instruction that ends at `0xffff` is
    /// followed by one at `0x0000` in the same segment, which is what the
    /// 8086's instruction pointer does.
    #[must_use]
    pub const fn next_ip(&self) -> u16 {
        self.ip.wrapping_add(self.len as u16)
    }

    /// The offset a relative jump or call would transfer to.
    ///
    /// `None` for every other instruction — an indirect transfer's target
    /// depends on state a static disassembler does not have.
    #[must_use]
    pub const fn branch_target(&self) -> Option<u16> {
        match (self.fields.insn.dst, self.fields.insn.src) {
            (Arg::Jb | Arg::Jv, _) => Some(self.next_ip().wrapping_add(self.fields.imm as u16)),
            _ => None,
        }
    }

    /// Whether the encoding is anything other than a documented one.
    #[must_use]
    pub const fn is_undocumented(&self) -> bool {
        !matches!(self.fields.insn.class, Class::Documented)
    }

    /// The operation, after opcode-extension resolution.
    #[must_use]
    pub const fn op(&self) -> Op {
        self.fields.insn.op
    }
}

/// Whether an operand names a register or an immediate, and therefore fixes
/// the operand size on its own.
const fn fixes_size(arg: Arg) -> bool {
    matches!(
        arg,
        Arg::Gb
            | Arg::Gv
            | Arg::Sw
            | Arg::Rb
            | Arg::Rv
            | Arg::Sr
            | Arg::Al
            | Arg::Ax
            | Arg::Cl
            | Arg::Dx
    )
}

impl fmt::Display for Disassembled {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let fields = &self.fields;
        let insn = fields.insn;

        if fields.lock {
            f.write_str("lock ")?;
        }
        match fields.rep {
            // The two repeat prefixes share an encoding with `REPE`/`REPNE`;
            // which name reads better depends on the instruction, so the
            // comparing forms get the conditional spelling.
            Some(Rep::While)
                if matches!(insn.op, Op::CMPSB | Op::CMPSW | Op::SCASB | Op::SCASW) =>
            {
                f.write_str("repe ")?;
            }
            Some(Rep::While) => f.write_str("rep ")?,
            Some(Rep::WhileNot) => f.write_str("repne ")?,
            None => {}
        }
        // A segment override with no memory operand to attach to still has to
        // be shown: it is a byte of the instruction, and on a string
        // instruction it is load-bearing.
        if let Some(sr) = fields.seg_override
            && !self.has_memory_operand()
        {
            write!(f, "{}: ", seg::name(sr))?;
        }

        f.write_str(insn.op.mnemonic())?;

        let show_size = self.needs_size_hint();
        let mut first = true;
        for arg in [insn.dst, insn.src] {
            if arg == Arg::None {
                continue;
            }
            f.write_str(if first { " " } else { ", " })?;
            first = false;
            self.write_arg(f, arg, show_size)?;
        }
        Ok(())
    }
}

impl Disassembled {
    /// Whether either operand reaches memory through a ModRM byte or a direct
    /// offset.
    fn has_memory_operand(&self) -> bool {
        let insn = self.fields.insn;
        let modrm_mem = matches!(self.fields.modrm, Some(m) if !m.is_register());
        for arg in [insn.dst, insn.src] {
            match arg {
                Arg::Eb | Arg::Ev if modrm_mem => return true,
                Arg::M | Arg::Mp => return true,
                Arg::Ob | Arg::Ov | Arg::Xb | Arg::Xv | Arg::Yb | Arg::Yv => return true,
                _ => {}
            }
        }
        false
    }

    /// Whether `byte`/`word` has to be printed in front of a memory operand.
    ///
    /// Only when no other operand says how wide the access is: `mov [bx], 1`
    /// is ambiguous, `mov [bx], al` is not.
    fn needs_size_hint(&self) -> bool {
        let insn = self.fields.insn;
        if !self.has_memory_operand() {
            return false;
        }
        // A string instruction spells its width in the mnemonic.
        if insn.op.is_string() {
            return false;
        }
        !fixes_size(insn.dst) && !fixes_size(insn.src)
    }

    fn write_arg(&self, f: &mut fmt::Formatter<'_>, arg: Arg, show_size: bool) -> fmt::Result {
        let fields = &self.fields;
        match arg {
            Arg::None => Ok(()),
            Arg::Eb | Arg::Ev => {
                let modrm = fields.modrm.unwrap_or(ModRm::new(0));
                if modrm.is_register() {
                    let names = if arg == Arg::Eb { REG8 } else { REG16 };
                    f.write_str(names[modrm.rm as usize])
                } else {
                    self.write_mem(f, modrm, arg == Arg::Eb, show_size)
                }
            }
            Arg::M | Arg::Mp => {
                let modrm = fields.modrm.unwrap_or(ModRm::new(0));
                if modrm.is_register() {
                    // `LEA reg, reg` and the register forms of `LES`/`LDS` are
                    // undefined encodings; printing the register is more use
                    // than refusing to print anything.
                    f.write_str(REG16[modrm.rm as usize])
                } else {
                    self.write_mem(f, modrm, false, false)
                }
            }
            Arg::Gb => f.write_str(REG8[fields.modrm.map_or(0, |m| m.reg) as usize]),
            Arg::Gv => f.write_str(REG16[fields.modrm.map_or(0, |m| m.reg) as usize]),
            Arg::Sw => f.write_str(seg::name(fields.modrm.map_or(0, |m| m.reg))),
            Arg::Ib => write!(f, "{:#x}", fields.imm as u8),
            Arg::Iv | Arg::Ibs => write!(f, "{:#x}", fields.imm16()),
            Arg::Jb | Arg::Jv => {
                let target = self.branch_target().unwrap_or(0);
                write!(f, "{target:#x}")
            }
            Arg::Ap => write!(f, "{:#x}:{:#x}", fields.imm_seg(), fields.imm16()),
            Arg::Ob | Arg::Ov => {
                if show_size {
                    f.write_str(if arg == Arg::Ob { "byte " } else { "word " })?;
                }
                write!(
                    f,
                    "[{}:{:#x}]",
                    seg::name(fields.segment(seg::DS)),
                    fields.imm16()
                )
            }
            Arg::Rb => f.write_str(REG8[(fields.opcode & 7) as usize]),
            Arg::Rv => f.write_str(REG16[(fields.opcode & 7) as usize]),
            Arg::Sr => f.write_str(seg::name((fields.opcode >> 3) & 3)),
            Arg::One => f.write_str("1"),
            Arg::Cl => f.write_str("cl"),
            Arg::Dx => f.write_str("dx"),
            Arg::Al => f.write_str("al"),
            Arg::Ax => f.write_str("ax"),
            // The string operands are implicit, and their width is already in
            // the mnemonic; showing the pointer is what makes an override
            // visible.
            Arg::Xb | Arg::Xv => write!(f, "[{}:si]", seg::name(fields.segment(seg::DS))),
            Arg::Yb | Arg::Yv => f.write_str("[es:di]"),
        }
    }

    fn write_mem(
        &self,
        f: &mut fmt::Formatter<'_>,
        modrm: ModRm,
        byte: bool,
        show_size: bool,
    ) -> fmt::Result {
        if show_size {
            f.write_str(if byte { "byte " } else { "word " })?;
        }
        let sr = self.fields.segment(modrm.default_segment());
        write!(f, "[{}:", seg::name(sr))?;
        let disp = self.fields.disp;
        if modrm.md == 0 && modrm.rm == 6 {
            write!(f, "{disp:#x}]")
        } else {
            f.write_str(RM_TERMS[modrm.rm as usize])?;
            if disp != 0 {
                // A displacement is signed; printing `bp-0x2` rather than
                // `bp+0xfffe` is the difference between a listing that reads
                // and one that has to be decoded again by eye.
                let signed = disp as i16;
                if signed < 0 {
                    write!(f, "-{:#x}", signed.unsigned_abs())?;
                } else {
                    write!(f, "+{signed:#x}")?;
                }
            }
            f.write_str("]")
        }
    }
}

/// Disassemble one instruction from a byte slice.
///
/// `cs:ip` is where the instruction lives, which is what makes a relative
/// jump's target printable.
#[must_use]
pub fn disassemble(cs: u16, ip: u16, bytes: &[u8]) -> Disassembled {
    let mut at = 0usize;
    let mut kept = [0u8; MAX_KEPT_BYTES];
    let fields = isa::decode_stream(&mut || {
        let b = bytes.get(at).copied();
        if let Some(b) = b
            && at < MAX_KEPT_BYTES
        {
            kept[at] = b;
        }
        at += 1;
        b
    });
    Disassembled {
        cs,
        ip,
        fields,
        bytes: kept,
        len: fields.len,
        truncated: fields.truncated,
    }
}

/// Disassemble one instruction by reading guest memory through `read`.
///
/// `read` takes a 20-bit physical address and returns `None` where nothing is
/// readable. Addresses are computed with the 8086's segment arithmetic,
/// wraparound at 1 MiB included, so a listing that runs off the top of memory
/// shows what the CPU would actually fetch.
#[must_use]
pub fn disassemble_at(cs: u16, ip: u16, mut read: impl FnMut(u32) -> Option<u8>) -> Disassembled {
    let mut offset = 0u16;
    let mut kept = [0u8; MAX_KEPT_BYTES];
    let fields = isa::decode_stream(&mut || {
        let addr = super::linear(cs, ip.wrapping_add(offset));
        let b = read(addr);
        if let Some(b) = b
            && (offset as usize) < MAX_KEPT_BYTES
        {
            kept[offset as usize] = b;
        }
        offset = offset.wrapping_add(1);
        b
    });
    Disassembled {
        cs,
        ip,
        fields,
        bytes: kept,
        len: fields.len,
        truncated: fields.truncated,
    }
}

/// Disassemble `count` consecutive instructions starting at `cs:ip`.
///
/// Stops early if an instruction cannot be read in full, because everything
/// after it would be decoded from the wrong offset.
#[must_use]
pub fn disassemble_run(
    cs: u16,
    ip: u16,
    count: usize,
    mut read: impl FnMut(u32) -> Option<u8>,
) -> Vec<Disassembled> {
    let mut out = Vec::with_capacity(count);
    let mut ip = ip;
    for _ in 0..count {
        let d = disassemble_at(cs, ip, &mut read);
        let truncated = d.truncated;
        ip = d.next_ip();
        out.push(d);
        if truncated {
            break;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;
    use alloc::string::String;

    fn text(bytes: &[u8]) -> String {
        format!("{}", disassemble(0, 0x100, bytes))
    }

    #[test]
    fn the_alu_group_prints_in_intel_order() {
        assert_eq!(text(&[0x00, 0xc1]), "add cl, al");
        assert_eq!(text(&[0x03, 0x06, 0x34, 0x12]), "add ax, [ds:0x1234]");
        assert_eq!(text(&[0x83, 0xc0, 0xfe]), "add ax, 0xfffe");
        assert_eq!(
            text(&[0x80, 0x36, 0x34, 0x12, 0x0f]),
            "xor byte [ds:0x1234], 0xf"
        );
    }

    #[test]
    fn addressing_modes_read_the_way_the_manual_writes_them() {
        assert_eq!(text(&[0x8b, 0x00]), "mov ax, [ds:bx+si]");
        assert_eq!(text(&[0x8b, 0x46, 0x00]), "mov ax, [ss:bp]");
        assert_eq!(text(&[0x8b, 0x46, 0xfe]), "mov ax, [ss:bp-0x2]");
        assert_eq!(text(&[0x8b, 0x86, 0x00, 0x01]), "mov ax, [ss:bp+0x100]");
        assert_eq!(text(&[0x8b, 0x06, 0x00, 0x01]), "mov ax, [ds:0x100]");
        assert_eq!(text(&[0x26, 0x8b, 0x07]), "mov ax, [es:bx]");
    }

    #[test]
    fn a_size_hint_appears_only_when_nothing_else_gives_the_width() {
        assert_eq!(text(&[0xff, 0x06, 0x34, 0x12]), "inc word [ds:0x1234]");
        assert_eq!(text(&[0xfe, 0x06, 0x34, 0x12]), "inc byte [ds:0x1234]");
        assert_eq!(text(&[0x8b, 0x07]), "mov ax, [ds:bx]");
        assert_eq!(text(&[0xc6, 0x07, 0x42]), "mov byte [ds:bx], 0x42");
    }

    #[test]
    fn relative_transfers_print_their_target() {
        // The displacement counts from the end of the instruction.
        let d = disassemble(0, 0x100, &[0xeb, 0xfe]);
        assert_eq!(format!("{d}"), "jmp 0x100");
        assert_eq!(d.branch_target(), Some(0x100));
        assert_eq!(text(&[0x74, 0x10]), "jz 0x112");
        assert_eq!(text(&[0xe8, 0x00, 0x01]), "call 0x203");
    }

    #[test]
    fn far_transfers_print_segment_and_offset() {
        assert_eq!(text(&[0xea, 0x5b, 0xe0, 0x00, 0xf0]), "jmpf 0xf000:0xe05b");
        assert_eq!(text(&[0xff, 0x1e, 0x00, 0x20]), "callf [ds:0x2000]");
    }

    #[test]
    fn prefixes_are_printed_before_the_mnemonic() {
        assert_eq!(text(&[0xf3, 0xa4]), "rep movsb [es:di], [ds:si]");
        assert_eq!(text(&[0xf2, 0xae]), "repne scasb al, [es:di]");
        assert_eq!(text(&[0xf3, 0xa6]), "repe cmpsb [ds:si], [es:di]");
        assert_eq!(text(&[0x26, 0xa4]), "movsb [es:di], [es:si]");
        assert_eq!(text(&[0xf0, 0x00, 0x07]), "lock add [ds:bx], al");
    }

    #[test]
    fn a_segment_override_with_nothing_to_override_is_still_shown() {
        // It is a byte of the instruction; hiding it would make the listing
        // disagree with the length.
        assert_eq!(text(&[0x2e, 0x90]), "cs: nop");
    }

    #[test]
    fn the_shift_group_names_its_count() {
        assert_eq!(text(&[0xd0, 0xe0]), "shl al, 1");
        assert_eq!(text(&[0xd3, 0xe8]), "shr ax, cl");
        assert_eq!(text(&[0xd1, 0x26, 0x00, 0x20]), "shl word [ds:0x2000], 1");
        assert_eq!(text(&[0xd0, 0xf0]), "setmo al, 1");
    }

    #[test]
    fn segment_and_accumulator_forms_name_their_registers() {
        assert_eq!(text(&[0x06]), "push es");
        assert_eq!(text(&[0x1f]), "pop ds");
        assert_eq!(text(&[0x8e, 0xd8]), "mov ds, ax");
        assert_eq!(text(&[0x8c, 0xc8]), "mov ax, cs");
        assert_eq!(text(&[0xa0, 0x34, 0x12]), "mov al, [ds:0x1234]");
        assert_eq!(text(&[0xa3, 0x34, 0x12]), "mov [ds:0x1234], ax");
        assert_eq!(text(&[0xe4, 0x60]), "in al, 0x60");
        assert_eq!(text(&[0xee]), "out dx, al");
    }

    #[test]
    fn a_run_walks_forward_by_the_decoded_length() {
        let code = [0xb8u8, 0x34, 0x12, 0x40, 0x90];
        let run = disassemble_run(0, 0, 3, |addr| code.get(addr as usize).copied());
        assert_eq!(run.len(), 3);
        assert_eq!(format!("{}", run[0]), "mov ax, 0x1234");
        assert_eq!(run[1].ip, 3);
        assert_eq!(format!("{}", run[1]), "inc ax");
        assert_eq!(format!("{}", run[2]), "nop");
    }

    #[test]
    fn undocumented_encodings_are_flagged() {
        assert!(disassemble(0, 0, &[0xd6]).is_undocumented()); // salc
        assert!(disassemble(0, 0, &[0x62, 0x00]).is_undocumented()); // alias of jb
        assert!(!disassemble(0, 0, &[0x72, 0x00]).is_undocumented()); // jb itself
    }
}
