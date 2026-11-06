//! The disassembler, generated from the same table the interpreter decodes
//! with.
//!
//! Nothing here knows an opcode: every mnemonic comes from
//! [`isa::Op::mnemonic`], every operand layout from [`isa::Fmt`], and every
//! fixed register name from [`isa::Insn::aux`]. Adding a row to
//! [`isa::TABLE`] makes it disassemble with no edit here.
//!
//! Displacements are printed **scaled** (the byte offset), and PC-relative
//! operands are resolved to absolute addresses, because that is what a
//! listing is for. FPU encodings print according to a [`Mode`] — the
//! `FPSCR.PR` and `FPSCR.SZ` the code runs under — since the same sixteen bits
//! are `fadd fr1,fr2` or `fadd dr0,dr2` depending on it (manual §10.3).

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt;

use super::exec::fpscr;
use super::isa::{self, Fmt, REG_NAMES};

/// The FPU mode that decides how `H'F***` encodings read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Mode {
    /// `FPSCR.PR`: double precision.
    pub pr: bool,
    /// `FPSCR.SZ`: pair transfers.
    pub sz: bool,
}

impl Mode {
    /// The mode an `FPSCR` value selects.
    #[must_use]
    pub const fn from_fpscr(v: u32) -> Mode {
        Mode {
            pr: v & fpscr::PR != 0,
            sz: v & fpscr::SZ != 0,
        }
    }
}

/// One disassembled instruction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Disassembled {
    /// Where it starts.
    pub addr: u64,
    /// How many bytes it occupies: always two.
    pub len: u64,
    /// The raw encoding.
    pub encoding: u16,
    /// The assembly text.
    pub text: String,
    /// Whether the instruction that follows is this one's delay slot.
    pub delay_slot: bool,
}

impl fmt::Display for Disassembled {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:08x}: {:04x}  {}", self.addr, self.encoding, self.text)
    }
}

/// A general register.
fn r(i: usize) -> &'static str {
    REG_NAMES[i & 15]
}

/// A floating-point register name for `field` under the mode: `frN`, or a
/// pair — `drN` for an even field and `xdN` for an odd one — when `pair`.
fn fpr(field: usize, pair: bool) -> String {
    if !pair {
        format!("fr{field}")
    } else if field & 1 == 0 {
        format!("dr{field}")
    } else {
        format!("xd{}", field & 14)
    }
}

/// A signed immediate, as an assembler would accept it.
fn simm(v: u32) -> String {
    let v = v as i32;
    if v.unsigned_abs() < 10 {
        format!("#{v}")
    } else if v < 0 {
        format!("#-0x{:x}", v.unsigned_abs())
    } else {
        format!("#0x{v:x}")
    }
}

/// The operand size a `.b`/`.w`/`.l` suffix names, for scaling displacements.
fn scale(mnemonic: &str) -> u32 {
    if mnemonic.ends_with(".b") {
        1
    } else if mnemonic.ends_with(".w") {
        2
    } else {
        4
    }
}

/// Format one instruction word at `pc`.
#[must_use]
pub fn format_word(word: u16, pc: u32, mode: Mode) -> String {
    let Some(insn) = isa::decode(word) else {
        return format!(".word 0x{word:04x}");
    };
    let mn = insn.op.mnemonic();
    let n = isa::n(word);
    let m = isa::m(word);
    let aux = insn.aux;
    let d4 = isa::d4(word);
    let i8 = isa::i8u(word);
    let bank = isa::bank(word);
    // The FPU mnemonics whose `.s` suffix the pair forms drop.
    let fmov = || if mode.sz { "fmov" } else { mn };
    match insn.fmt {
        Fmt::None => String::from(mn),
        Fmt::Rn => format!("{mn} {}", r(n)),
        Fmt::AtRn => format!("{mn} @{}", r(n)),
        Fmt::RmRn => format!("{mn} {}, {}", r(m), r(n)),
        Fmt::SImmRn => format!("{mn} {}, {}", simm(isa::i8s(word)), r(n)),
        Fmt::ImmR0 => format!("{mn} #0x{i8:x}, r0"),
        Fmt::SImmR0 => format!("{mn} {}, r0", simm(isa::i8s(word))),
        Fmt::ImmGbr => format!("{mn} #0x{i8:x}, @(r0, gbr)"),
        Fmt::Imm => format!("{mn} #0x{i8:x}"),
        Fmt::StoreAt => format!("{mn} {}, @{}", r(m), r(n)),
        Fmt::LoadAt => format!("{mn} @{}, {}", r(m), r(n)),
        Fmt::StorePre => format!("{mn} {}, @-{}", r(m), r(n)),
        Fmt::LoadPost => format!("{mn} @{}+, {}", r(m), r(n)),
        Fmt::StoreR0Disp => format!("{mn} r0, @({}, {})", d4 * scale(mn), r(m)),
        Fmt::LoadR0Disp => format!("{mn} @({}, {}), r0", d4 * scale(mn), r(m)),
        Fmt::StoreDisp => format!("{mn} {}, @({}, {})", r(m), d4 * 4, r(n)),
        Fmt::LoadDisp => format!("{mn} @({}, {}), {}", d4 * 4, r(m), r(n)),
        Fmt::StoreIdx => format!("{mn} {}, @(r0, {})", r(m), r(n)),
        Fmt::LoadIdx => format!("{mn} @(r0, {}), {}", r(m), r(n)),
        Fmt::StoreGbr => format!("{mn} r0, @({}, gbr)", i8 * scale(mn)),
        Fmt::LoadGbr => format!("{mn} @({}, gbr), r0", i8 * scale(mn)),
        Fmt::PcRel => {
            let ea = if scale(mn) == 2 {
                isa::pc_word(pc, word)
            } else {
                isa::pc_long(pc, word)
            };
            format!("{mn} @(0x{ea:08x}), {}", r(n))
        }
        Fmt::Mova => format!("{mn} @(0x{:08x}), r0", isa::pc_long(pc, word)),
        Fmt::Branch8 => format!("{mn} 0x{:08x}", isa::branch8(pc, word)),
        Fmt::Branch12 => format!("{mn} 0x{:08x}", isa::branch12(pc, word)),
        Fmt::R0AtRn => format!("{mn} r0, @{}", r(n)),
        Fmt::AtRmR0 => format!("{mn} @{}, r0", r(n)),
        Fmt::PostRmR0 => format!("{mn} @{}+, r0", r(n)),
        Fmt::Mac => format!("{mn} @{}+, @{}+", r(m), r(n)),
        Fmt::RmCtl => format!("{mn} {}, {aux}", r(n)),
        Fmt::CtlRn => format!("{mn} {aux}, {}", r(n)),
        Fmt::PostCtl => format!("{mn} @{}+, {aux}", r(n)),
        Fmt::CtlPre => format!("{mn} {aux}, @-{}", r(n)),
        Fmt::RmBank => format!("{mn} {}, r{bank}_bank", r(n)),
        Fmt::BankRn => format!("{mn} r{bank}_bank, {}", r(n)),
        Fmt::PostBank => format!("{mn} @{}+, r{bank}_bank", r(n)),
        Fmt::BankPre => format!("{mn} r{bank}_bank, @-{}", r(n)),
        Fmt::FRmFRn => format!("{mn} {}, {}", fpr(m, mode.pr), fpr(n, mode.pr)),
        Fmt::FRn => format!("{mn} {}", fpr(n, mode.pr)),
        Fmt::FRnS => format!("{mn} fr{n}"),
        Fmt::FMac => format!("{mn} fr0, fr{m}, fr{n}"),
        Fmt::FMov => format!("{mn} {}, {}", fpr(m, mode.sz), fpr(n, mode.sz)),
        Fmt::FLoadAt => format!("{} @{}, {}", fmov(), r(m), fpr(n, mode.sz)),
        Fmt::FLoadPost => format!("{} @{}+, {}", fmov(), r(m), fpr(n, mode.sz)),
        Fmt::FLoadIdx => format!("{} @(r0, {}), {}", fmov(), r(m), fpr(n, mode.sz)),
        Fmt::FStoreAt => format!("{} {}, @{}", fmov(), fpr(m, mode.sz), r(n)),
        Fmt::FStorePre => format!("{} {}, @-{}", fmov(), fpr(m, mode.sz), r(n)),
        Fmt::FStoreIdx => format!("{} {}, @(r0, {})", fmov(), fpr(m, mode.sz), r(n)),
        Fmt::FRmFpul => format!("{mn} {}, fpul", fpr(n, mode.pr)),
        Fmt::FpulFRn => format!("{mn} fpul, {}", fpr(n, mode.pr)),
        Fmt::DRmFpul => format!("{mn} dr{}, fpul", n & 14),
        Fmt::FpulDRn => format!("{mn} fpul, dr{}", n & 14),
        Fmt::Fipr => format!(
            "{mn} fv{}, fv{}",
            ((word >> 8) & 3) * 4,
            ((word >> 10) & 3) * 4
        ),
        Fmt::Ftrv => format!("{mn} xmtrx, fv{}", ((word >> 10) & 3) * 4),
    }
}

/// Disassemble `count` instructions starting at `pc`, reading halfwords
/// through `read`. A halfword `read` cannot supply ends the listing.
pub fn disassemble_run(
    pc: u32,
    count: usize,
    mode: Mode,
    mut read: impl FnMut(u32) -> Option<u16>,
) -> Vec<Disassembled> {
    let mut out = Vec::with_capacity(count);
    let mut at = pc & !1;
    for _ in 0..count {
        let Some(word) = read(at) else {
            break;
        };
        out.push(Disassembled {
            addr: u64::from(at),
            len: 2,
            encoding: word,
            text: format_word(word, at, mode),
            delay_slot: isa::decode(word).is_some_and(|i| i.delayed()),
        });
        at = at.wrapping_add(2);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    fn d(w: u16) -> String {
        format_word(w, 0x8c00_1000, Mode::default())
    }

    #[test]
    fn common_forms_read_as_an_assembler_writes_them() {
        assert_eq!(d(0x0009), "nop");
        assert_eq!(d(0xe1ff), "mov #-1, r1");
        assert_eq!(d(0x6123), "mov r2, r1");
        assert_eq!(d(0x2126), "mov.l r2, @-r1");
        assert_eq!(d(0x6126), "mov.l @r2+, r1");
        assert_eq!(d(0x5123), "mov.l @(12, r2), r1");
        assert_eq!(d(0x8523), "mov.w @(6, r2), r0");
        assert_eq!(d(0xc002), "mov.b r0, @(2, gbr)");
        assert_eq!(d(0x412e), "ldc r1, vbr");
        assert_eq!(d(0x0182), "stc r0_bank, r1");
        assert_eq!(d(0x41f6), "ldc.l @r1+, dbr");
        assert_eq!(d(0x4f22), "sts.l pr, @-r15");
        assert_eq!(d(0xc3ff), "trapa #0xff");
        assert_eq!(d(0x0fff), "mac.l @r15+, @r15+");
        assert_eq!(d(0xfffd), ".word 0xfffd");
    }

    #[test]
    fn pc_relative_operands_resolve() {
        assert_eq!(d(0xaffe), "bra 0x8c001000");
        assert_eq!(d(0x8901), "bt 0x8c001006");
        assert_eq!(d(0xd001), "mov.l @(0x8c001008), r0");
        assert_eq!(d(0x9001), "mov.w @(0x8c001006), r0");
    }

    #[test]
    fn fpu_encodings_follow_the_mode() {
        let single = Mode::default();
        let double = Mode { pr: true, sz: true };
        assert_eq!(format_word(0xf120, 0, single), "fadd fr2, fr1");
        assert_eq!(format_word(0xf020, 0, double), "fadd dr2, dr0");
        assert_eq!(format_word(0xf13c, 0, double), "fmov xd2, xd0");
        assert_eq!(format_word(0xf128, 0, single), "fmov.s @r2, fr1");
        assert_eq!(format_word(0xf128, 0, double), "fmov @r2, xd0");
        assert_eq!(format_word(0xf1fd, 0, single), "ftrv xmtrx, fv0");
        assert_eq!(format_word(0xf6ed, 0, single), "fipr fv8, fv4");
    }

    #[test]
    fn delayed_branches_are_marked() {
        let run = disassemble_run(0, 2, Mode::default(), |a| {
            Some(if a == 0 { 0x000b } else { 0x0009 })
        });
        assert!(run[0].delay_slot);
        assert!(!run[1].delay_slot);
        assert_eq!(run[0].to_string(), "00000000: 000b  rts");
    }
}
