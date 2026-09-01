//! The 8086/8088 instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! this file holds one declarative description, [`TABLE`] plus the group
//! tables, from which everything else is derived:
//!
//! - the interpreter's decode ([`decode`], [`resolve`]) and its operand
//!   plumbing, which comes from [`Insn::dst`] and [`Insn::src`];
//! - the disassembler ([`super::disasm`]), which formats from the same row;
//! - introspection: mnemonics, one-line summaries, and which encodings are
//!   undocumented, aliases, or outright undefined.
//!
//! # Why x86 needs more than a 256-row array
//!
//! Unlike a 6502, an x86 instruction is a *sequence*: any number of prefixes,
//! then an opcode, then optionally a ModRM byte whose `reg` field may select
//! the operation rather than a register, then a displacement, then an
//! immediate. The description is therefore two-level — [`TABLE`] for the 256
//! primary opcodes and one small array per opcode-extension group — and
//! [`resolve`] joins them once the ModRM byte is known.
//!
//! Operands are described in the notation Intel's own opcode maps use
//! ([`Arg`]): `Eb` is "byte r/m from ModRM", `Gv` is "word register from the
//! ModRM `reg` field", `Ib` is "immediate byte", and so on. Using the manual's
//! vocabulary rather than inventing one keeps the table checkable against the
//! manual line by line.
//!
//! # Why there is no cycle column
//!
//! Same reason as the 6502 core: an 8088 cycle is a bus state, and the number
//! of bus cycles an instruction spends depends on its operands — whether the
//! ModRM selects a register or memory, how many effective-address terms there
//! are, and whether the transfer is a byte or a word on an 8-bit bus. What is
//! constant per operation is the *internal* execution cost, which is
//! [`Op::clocks`]; the transfers are charged by the interpreter as it makes
//! them. See [`ea_clocks`].
//!
//! # Sources
//!
//! Intel's *iAPX 86/88, 186/188 User's Manual* (the instruction set summary,
//! the opcode map and the instruction-timing tables), the *8086 Family User's
//! Manual*, and sandpile.org's opcode tables for the encodings Intel leaves
//! out of the map. Undocumented encodings and their classification follow the
//! `SingleStepTests/8088` metadata (MIT) and were each confirmed against the
//! corpus itself; see `docs/cpu/x86.md`. No copyleft emulator was consulted.

use core::fmt;

/// Which opcode map a part decodes with.
///
/// x86 generations are close to a superset chain, but the *primary* map is not
/// quite one: the 80186 reclaimed sixteen encodings the 8086 left as aliases
/// of the conditional jumps, `0F` stopped being `POP CS` and became the
/// two-byte escape, and `C0`/`C1`/`C8`/`C9` stopped aliasing the `RET` forms.
/// Those are real differences, not extensions, so they get their own map
/// rather than being flattened into one (CLAUDE.md: "where a real difference
/// is not a superset, model it honestly").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Gen {
    /// The 8086/8088 map: no two-byte escape, sixteen jump aliases at
    /// `60`-`6F`, `0F` is `POP CS`.
    I8086,
    /// The 80186-through-80486 map, plus the two-byte `0F` page.
    I386,
}

/// The width the code segment is being decoded at.
///
/// Not the same question as [`Gen`], which is *which map*; this is which of
/// the three sets of defaults applies to the map that was chosen. It replaces
/// the `default32` flag the decoder used to take because 64-bit mode is not
/// "32-bit with a wider register": it has its own default operand size, its
/// own default address size, its own set of invalid encodings, and a prefix
/// (`REX`) that does not exist in the other two.
///
/// *Intel SDM* volume 2 §2.2.1.7, "Default 64-Bit Operand Size", and table
/// 2-4; *AMD64 Architecture Programmer's Manual* volume 3 §1.2.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Bits {
    /// A 16-bit code segment: operands and addresses default to two bytes.
    B16,
    /// A 32-bit code segment: operands and addresses default to four bytes.
    B32,
    /// A 64-bit code segment — `CS.L` set. Operands default to **four** bytes
    /// and addresses to eight, which is the pairing that surprises everyone:
    /// widening an operand needs `REX.W`, widening an address needs nothing.
    B64,
}

impl Bits {
    /// The default operand size in bytes, before any prefix.
    #[must_use]
    pub const fn operand(self) -> u8 {
        match self {
            Bits::B16 => 2,
            Bits::B32 | Bits::B64 => 4,
        }
    }

    /// The default address size in bytes, before any prefix.
    #[must_use]
    pub const fn address(self) -> u8 {
        match self {
            Bits::B16 => 2,
            Bits::B32 => 4,
            Bits::B64 => 8,
        }
    }

    /// The operand size a `66` prefix selects.
    ///
    /// In 64-bit mode `66` still means sixteen bits; it is `REX.W` that means
    /// sixty-four, and a `66` beside a `REX.W` is ignored rather than
    /// combined.
    #[must_use]
    pub const fn operand_alt(self) -> u8 {
        match self {
            Bits::B16 => 4,
            Bits::B32 | Bits::B64 => 2,
        }
    }

    /// The address size a `67` prefix selects. There is no 16-bit addressing
    /// in 64-bit mode: `67` there means thirty-two bits.
    #[must_use]
    pub const fn address_alt(self) -> u8 {
        match self {
            Bits::B16 => 4,
            Bits::B32 => 2,
            Bits::B64 => 4,
        }
    }

    /// Whether this is 64-bit mode.
    #[must_use]
    pub const fn is_64(self) -> bool {
        matches!(self, Bits::B64)
    }
}

/// How an operand is encoded, in the notation of Intel's opcode maps.
///
/// The letter is the addressing method and the suffix is the operand size:
/// `b` is a byte, `w` is always sixteen bits, and `v` is "whatever the
/// effective operand size is" — sixteen bits on an 8086 and on a 386 with no
/// `66` prefix, thirty-two with one. Keeping the manual's vocabulary means a
/// row can be checked against the manual without translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Arg {
    /// No operand in this position.
    None,
    /// ModRM r/m, byte.
    Eb,
    /// ModRM r/m, operand size.
    Ev,
    /// ModRM r/m, always sixteen bits (`LLDT`, `LTR`, `VERR`, `LAR`, `MOVZX`).
    Ew,
    /// ModRM r/m, always thirty-two bits: `MOVSXD`'s source, and nothing else.
    Ed,
    /// ModRM `reg`, byte register.
    Gb,
    /// ModRM `reg`, register at the operand size.
    Gv,
    /// ModRM `reg`, always a sixteen-bit register (`ARPL`).
    Gw,
    /// ModRM `reg`, segment register. On an 8086 only the low two bits are
    /// decoded, which is why `8C`/`8E` accept a `reg` of 4-7 and alias down;
    /// a 386 decodes all three and rejects 6 and 7.
    Sw,
    /// ModRM r/m as an *address*, never a register: `LEA`, `BOUND`, `INVLPG`.
    M,
    /// ModRM r/m as a far pointer in memory, `offset:segment`: `LES`, `LDS`,
    /// `LSS`, `LFS`, `LGS` and the indirect far `CALL`/`JMP`.
    Mp,
    /// ModRM r/m as a pseudo-descriptor in memory, `limit16:base32`:
    /// `LGDT`, `LIDT`, `SGDT`, `SIDT`.
    Ms,
    /// ModRM r/m constrained to a register, thirty-two bits: the other half of
    /// `MOV CR0,r32`. The mode field is ignored — a 386 treats `mod` as `11`
    /// whatever it holds.
    Rd,
    /// A control register named by the ModRM `reg` field.
    Cd,
    /// A debug register named by the ModRM `reg` field.
    Dd,
    /// A test register named by the ModRM `reg` field.
    Td,
    /// Immediate byte.
    Ib,
    /// Immediate word, always sixteen bits (`ENTER`, `RET imm16`).
    Iw,
    /// Immediate at the operand size — two, four or **eight** bytes.
    ///
    /// Intel's `Iv`. Only `B8`+`r` uses it, and only there does an x86 read a
    /// full 64-bit immediate; everything else that looks like it takes one
    /// takes [`Arg::Iz`] instead.
    Iv,
    /// Immediate of two or four bytes, sign-extended to the operand size.
    ///
    /// Intel's `Iz`. In 64-bit mode `add rax, imm` reads **four** immediate
    /// bytes and sign-extends them: there is no eight-byte form of any ALU
    /// immediate, and reading one would desynchronise the instruction stream
    /// (*Intel SDM* volume 2 §3.1.1.3 and Appendix A's `Iz` notation). Below
    /// 64-bit mode it is indistinguishable from [`Arg::Iv`], which is exactly
    /// why the distinction was never needed before.
    Iz,
    /// Immediate byte, sign-extended to the operand size (`83 /n`, `6A`).
    Ibs,
    /// Byte displacement relative to the end of the instruction.
    Jb,
    /// Displacement at the operand size, relative to the end of the
    /// instruction.
    Jv,
    /// Immediate far pointer, `offset:segment` — four bytes with a sixteen-bit
    /// operand size, six with a thirty-two-bit one.
    Ap,
    /// Direct memory offset, byte (`A0`/`A2`) — the offset is at the *address*
    /// size and there is no ModRM byte.
    Ob,
    /// Direct memory offset at the operand size (`A1`/`A3`).
    Ov,
    /// Byte register selected by the low three bits of the opcode.
    Rb,
    /// Register at the operand size, selected by the low three bits of the
    /// opcode.
    Rv,
    /// Segment register selected by bits 3-5 of the opcode (`PUSH ES`, the
    /// segment-override prefixes, and `0F A0` / `0F A8` for `FS` and `GS`).
    Sr,
    /// The literal 1, as the count of a single-bit shift.
    One,
    /// `CL`, as a shift count.
    Cl,
    /// `DX`, as an I/O port number. Carries no operand width of its own: the
    /// width of an `IN`/`OUT` comes from its accumulator operand.
    Dx,
    /// `AL`.
    Al,
    /// The accumulator at the operand size: `AX` or `EAX`.
    Ax,
    /// String source `DS:SI`, byte.
    Xb,
    /// String source `DS:SI`, at the operand size.
    Xv,
    /// String destination `ES:DI`, byte.
    Yb,
    /// String destination `ES:DI`, at the operand size.
    Yv,
}

impl Arg {
    /// How wide this operand is, in bytes, given an effective operand size.
    ///
    /// `None` is neither, so the answer is an [`Option`]: an instruction whose
    /// operands are all `None` (`CBW`, `HLT`) has no operand width and asking
    /// for one is a decoder bug rather than a default.
    #[must_use]
    pub const fn width_bytes(self, osz: u8) -> Option<u8> {
        match self {
            Arg::Eb
            | Arg::Gb
            | Arg::Ib
            | Arg::Ob
            | Arg::Rb
            | Arg::Al
            | Arg::Xb
            | Arg::Yb
            | Arg::Jb
            | Arg::One
            | Arg::Cl => Some(1),
            Arg::Ew | Arg::Gw | Arg::Iw | Arg::Sw | Arg::Sr => Some(2),
            Arg::Cd | Arg::Dd | Arg::Td | Arg::Rd | Arg::Ed => Some(4),
            Arg::Ev
            | Arg::Gv
            | Arg::Iv
            | Arg::Iz
            | Arg::Ibs
            | Arg::Ov
            | Arg::Rv
            | Arg::Ax
            | Arg::Xv
            | Arg::Yv
            | Arg::Jv => Some(osz),
            Arg::None | Arg::M | Arg::Mp | Arg::Ms | Arg::Ap | Arg::Dx => None,
        }
    }

    /// Whether decoding this operand requires a ModRM byte.
    #[must_use]
    pub const fn needs_modrm(self) -> bool {
        matches!(
            self,
            Arg::Eb
                | Arg::Ev
                | Arg::Ew
                | Arg::Ed
                | Arg::Gb
                | Arg::Gv
                | Arg::Gw
                | Arg::Sw
                | Arg::M
                | Arg::Mp
                | Arg::Ms
                | Arg::Rd
                | Arg::Cd
                | Arg::Dd
                | Arg::Td
        )
    }

    /// How many immediate bytes follow the displacement for this operand,
    /// given the effective operand and address sizes.
    #[must_use]
    pub const fn immediate_bytes(self, osz: u8, asz: u8) -> u8 {
        match self {
            Arg::Ib | Arg::Ibs | Arg::Jb => 1,
            Arg::Iw => 2,
            Arg::Iv => osz,
            // `Iz` and `Jz`: two bytes at a 16-bit operand size, four at
            // either wider one. A 64-bit near jump's displacement is `rel32`
            // and a 64-bit ALU immediate is `imm32`, both sign-extended.
            Arg::Iz | Arg::Jv => {
                if osz > 4 {
                    4
                } else {
                    osz
                }
            }
            // A `moffs` is an *address*, so it is the address size that
            // decides how many bytes follow — `67 A1` reads a 32-bit offset
            // even with a 16-bit operand size.
            Arg::Ob | Arg::Ov => asz,
            Arg::Ap => osz + 2,
            _ => 0,
        }
    }
}

/// How well documented an encoding is.
///
/// The 8086 has no invalid-opcode exception: every byte sequence does
/// *something*, and real software has relied on several of them. The
/// distinction is kept so a disassembler can flag an encoding and a test can
/// assert the matrix is complete. The vocabulary matches the one the
/// `SingleStepTests/8088` metadata uses, which is how each row was checked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Class {
    /// In Intel's instruction set summary.
    Documented,
    /// A second encoding of a documented instruction, produced by the opcode
    /// mask not being fully specific — `82` for `80`, `60`-`6F` for `70`-`7F`.
    Alias,
    /// Not in the manual, but well defined and useful: `SALC`, `SETMO`,
    /// `POP CS`.
    Undocumented,
    /// Not in the manual and of no obvious use, but still deterministic.
    Undefined,
    /// An instruction prefix, not an instruction.
    Prefix,
    /// A coprocessor escape (`D8`-`DF`). The 8088 computes the effective
    /// address and performs the operand read for the coprocessor's benefit,
    /// and does nothing else.
    Escape,
}

impl Class {
    /// Whether the encoding appears in Intel's instruction set summary.
    #[must_use]
    pub const fn is_documented(self) -> bool {
        matches!(self, Class::Documented)
    }
}

/// Which opcode-extension group an opcode belongs to, if any.
///
/// The 8086 packs eight operations into one opcode by using the ModRM `reg`
/// field as an extension. [`resolve`] applies the group once `reg` is known.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Grp {
    /// Not a group: the primary row is the whole story.
    None,
    /// `80`-`83`: the eight ALU operations. Operand forms come from the
    /// primary row, so only the operation varies.
    Alu,
    /// `D0`-`D3`: the eight shift and rotate operations, likewise.
    Shift,
    /// `8F`: `POP`. `reg` is not decoded at all, which is why every value is
    /// accepted and only `reg == 0` is documented.
    Pop,
    /// `FE`: byte `INC`/`DEC`.
    IncDec,
    /// `F6`/`F7`: `TEST`, `NOT`, `NEG`, and the multiply/divide family. Each
    /// row has its own operands, because `TEST` takes an immediate and the
    /// others do not.
    Unary,
    /// `FF`: the indirect control-transfer group. Each row has its own
    /// operands too — `CALLF` takes a far pointer where `CALL` takes a word.
    Misc,
    /// `C6`/`C7`: `MOV` immediate. `reg` is ignored, exactly as in [`Grp::Pop`].
    MovImm,
    /// `0F 00`: `SLDT`, `STR`, `LLDT`, `LTR`, `VERR`, `VERW`. The protection
    /// group Intel numbers 6.
    Grp6,
    /// `0F 01`: `SGDT`, `SIDT`, `LGDT`, `LIDT`, `SMSW`, `LMSW`, `INVLPG` —
    /// Intel's group 7.
    Grp7,
    /// `0F BA`: the bit-test group, `BT`/`BTS`/`BTR`/`BTC` with an immediate
    /// bit number. Intel's group 8; extensions 0-3 are undefined.
    Grp8,
}

/// Declare the operation enum, its mnemonics and its summaries in one list.
///
/// The mnemonic is the variant name lowercased, so the two cannot disagree.
/// Lower case because that is how every x86 disassembler since `as` has
/// printed them, and because `docs/cpu/x86.md`'s reference (felixcloutier)
/// indexes them that way.
macro_rules! define_ops {
    ($($name:ident = $summary:literal,)*) => {
        /// One operation, independent of how its operands are encoded.
        ///
        /// The variant name *is* the mnemonic ([`Op::mnemonic`]), so a
        /// disassembler cannot print a name the interpreter does not
        /// implement.
        // Mnemonics are conventionally written as the variant names below;
        // renaming them to satisfy the acronym lint would make every reference
        // to the manual unreadable.
        #[allow(clippy::upper_case_acronyms)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum Op {
            $(
                #[doc = $summary]
                $name,
            )*
        }

        impl Op {
            /// A one-line description, for the monitor and `rsemu describe`.
            #[must_use]
            pub const fn summary(self) -> &'static str {
                match self { $(Op::$name => $summary,)* }
            }

            /// Every operation this core implements, in declaration order.
            pub const ALL: &'static [Op] = &[$(Op::$name,)*];
        }

        /// The mnemonics, lowercased at compile time so [`Op::mnemonic`] can
        /// stay `const` without a `to_lowercase` allocation.
        const MNEMONICS: &[&str] = &[$(lower(stringify!($name)),)*];
    };
}

/// Lowercase an ASCII identifier at compile time.
///
/// `stringify!` gives the variant name in upper case; every x86 tool prints
/// mnemonics in lower case. Doing the conversion in a `const fn` keeps the
/// single-source-of-truth property without allocating.
const fn lower(name: &'static str) -> &'static str {
    // The conversion has to happen in a const context to produce a `&'static
    // str`, and const string building is not available on the MSRV, so the
    // table below is a lookup rather than a transformation. Every mnemonic
    // that appears in `define_ops!` must appear here; `mnemonics_are_all_known`
    // proves it does.
    let bytes = name.as_bytes();
    let mut i = 0;
    while i < LOWERCASE.len() {
        let cand = LOWERCASE[i].0.as_bytes();
        if cand.len() == bytes.len() {
            let mut j = 0;
            let mut same = true;
            while j < bytes.len() {
                if cand[j] != bytes[j] {
                    same = false;
                    break;
                }
                j += 1;
            }
            if same {
                return LOWERCASE[i].1;
            }
        }
        i += 1;
    }
    panic!("mnemonic missing from LOWERCASE");
}

/// Upper-case variant name to lower-case mnemonic.
const LOWERCASE: &[(&str, &str)] = &[
    ("AAA", "aaa"),
    ("AAD", "aad"),
    ("AAM", "aam"),
    ("AAS", "aas"),
    ("ADC", "adc"),
    ("ADD", "add"),
    ("AND", "and"),
    ("CALL", "call"),
    ("CALLF", "callf"),
    ("CBW", "cbw"),
    ("CLC", "clc"),
    ("CLD", "cld"),
    ("CLI", "cli"),
    ("CMC", "cmc"),
    ("CMP", "cmp"),
    ("CMPSB", "cmpsb"),
    ("CMPSW", "cmpsw"),
    ("CWD", "cwd"),
    ("DAA", "daa"),
    ("DAS", "das"),
    ("DEC", "dec"),
    ("DIV", "div"),
    ("ESC", "esc"),
    ("HLT", "hlt"),
    ("IDIV", "idiv"),
    ("IMUL", "imul"),
    ("IN", "in"),
    ("INC", "inc"),
    ("INT", "int"),
    ("INT3", "int3"),
    ("INTO", "into"),
    ("IRET", "iret"),
    ("JA", "ja"),
    ("JB", "jb"),
    ("JBE", "jbe"),
    ("JCXZ", "jcxz"),
    ("JG", "jg"),
    ("JGE", "jge"),
    ("JL", "jl"),
    ("JLE", "jle"),
    ("JMP", "jmp"),
    ("JMPF", "jmpf"),
    ("JNB", "jnb"),
    ("JNO", "jno"),
    ("JNP", "jnp"),
    ("JNS", "jns"),
    ("JNZ", "jnz"),
    ("JO", "jo"),
    ("JP", "jp"),
    ("JS", "js"),
    ("JZ", "jz"),
    ("LAHF", "lahf"),
    ("LDS", "lds"),
    ("LEA", "lea"),
    ("LES", "les"),
    ("LOCK", "lock"),
    ("LODSB", "lodsb"),
    ("LODSW", "lodsw"),
    ("LOOP", "loop"),
    ("LOOPE", "loope"),
    ("LOOPNE", "loopne"),
    ("MOV", "mov"),
    ("MOVSB", "movsb"),
    ("MOVSW", "movsw"),
    ("MUL", "mul"),
    ("NEG", "neg"),
    ("NOP", "nop"),
    ("NOT", "not"),
    ("OR", "or"),
    ("OUT", "out"),
    ("POP", "pop"),
    ("POPF", "popf"),
    ("PUSH", "push"),
    ("PUSHF", "pushf"),
    ("RCL", "rcl"),
    ("RCR", "rcr"),
    ("REP", "rep"),
    ("REPNE", "repne"),
    ("RET", "ret"),
    ("RETF", "retf"),
    ("ROL", "rol"),
    ("ROR", "ror"),
    ("SAHF", "sahf"),
    ("SALC", "salc"),
    ("SAR", "sar"),
    ("SBB", "sbb"),
    ("SCASB", "scasb"),
    ("SCASW", "scasw"),
    ("SEG", "seg"),
    ("SETMO", "setmo"),
    ("SHL", "shl"),
    ("SHR", "shr"),
    ("STC", "stc"),
    ("STD", "std"),
    ("STI", "sti"),
    ("STOSB", "stosb"),
    ("STOSW", "stosw"),
    ("SUB", "sub"),
    ("TEST", "test"),
    ("WAIT", "wait"),
    ("XCHG", "xchg"),
    ("XLAT", "xlat"),
    ("XOR", "xor"),
    // The 80186 through 80486 additions.
    ("ARPL", "arpl"),
    ("BOUND", "bound"),
    ("BSF", "bsf"),
    ("BSR", "bsr"),
    ("BSWAP", "bswap"),
    ("BT", "bt"),
    ("BTC", "btc"),
    ("BTR", "btr"),
    ("BTS", "bts"),
    ("CLTS", "clts"),
    ("CMPXCHG", "cmpxchg"),
    ("CPUID", "cpuid"),
    ("ENTER", "enter"),
    ("ICEBP", "icebp"),
    ("INSB", "insb"),
    ("INSW", "insw"),
    ("INVD", "invd"),
    ("INVLPG", "invlpg"),
    ("LAR", "lar"),
    ("LEAVE", "leave"),
    ("LFS", "lfs"),
    ("LGDT", "lgdt"),
    ("LGS", "lgs"),
    ("LIDT", "lidt"),
    ("LLDT", "lldt"),
    ("LMSW", "lmsw"),
    ("LSL", "lsl"),
    ("LSS", "lss"),
    ("LTR", "ltr"),
    ("MOVSX", "movsx"),
    ("MOVZX", "movzx"),
    ("OUTSB", "outsb"),
    ("OUTSW", "outsw"),
    ("POPA", "popa"),
    ("PUSHA", "pusha"),
    ("SETA", "seta"),
    ("SETB", "setb"),
    ("SETBE", "setbe"),
    ("SETG", "setg"),
    ("SETGE", "setge"),
    ("SETL", "setl"),
    ("SETLE", "setle"),
    ("SETNB", "setnb"),
    ("SETNO", "setno"),
    ("SETNP", "setnp"),
    ("SETNS", "setns"),
    ("SETNZ", "setnz"),
    ("SETO", "seto"),
    ("SETP", "setp"),
    ("SETS", "sets"),
    ("SETZ", "setz"),
    ("SGDT", "sgdt"),
    ("SHLD", "shld"),
    ("SHRD", "shrd"),
    ("SIDT", "sidt"),
    ("SLDT", "sldt"),
    ("SMSW", "smsw"),
    ("STR", "str"),
    ("UD", "ud"),
    ("VERR", "verr"),
    ("VERW", "verw"),
    ("WBINVD", "wbinvd"),
    ("XADD", "xadd"),
    ("CMOVA", "cmova"),
    ("CMOVB", "cmovb"),
    ("CMOVBE", "cmovbe"),
    ("CMOVG", "cmovg"),
    ("CMOVGE", "cmovge"),
    ("CMOVL", "cmovl"),
    ("CMOVLE", "cmovle"),
    ("CMOVNB", "cmovnb"),
    ("CMOVNO", "cmovno"),
    ("CMOVNP", "cmovnp"),
    ("CMOVNS", "cmovns"),
    ("CMOVNZ", "cmovnz"),
    ("CMOVO", "cmovo"),
    ("CMOVP", "cmovp"),
    ("CMOVS", "cmovs"),
    ("CMOVZ", "cmovz"),
    ("MOVSXD", "movsxd"),
    ("RDMSR", "rdmsr"),
    ("REX", "rex"),
    ("SWAPGS", "swapgs"),
    ("SYSCALL", "syscall"),
    ("SYSRET", "sysret"),
    ("WRMSR", "wrmsr"),
];

define_ops! {
    AAA = "ASCII adjust AL after addition",
    AAD = "ASCII adjust AX before division",
    AAM = "ASCII adjust AX after multiplication",
    AAS = "ASCII adjust AL after subtraction",
    ADC = "add with carry",
    ADD = "add",
    AND = "bitwise AND",
    CALL = "call a near procedure",
    CALLF = "call a far procedure",
    CBW = "sign-extend AL into AX",
    CLC = "clear the carry flag",
    CLD = "clear the direction flag",
    CLI = "clear the interrupt-enable flag",
    CMC = "complement the carry flag",
    CMP = "compare by subtracting and discarding the result",
    CMPSB = "compare a byte at DS:SI with one at ES:DI",
    CMPSW = "compare a word at DS:SI with one at ES:DI",
    CWD = "sign-extend AX into DX:AX",
    DAA = "decimal adjust AL after addition",
    DAS = "decimal adjust AL after subtraction",
    DEC = "decrement by one, leaving the carry flag alone",
    DIV = "unsigned divide",
    ESC = "coprocessor escape: read the operand and ignore it",
    HLT = "halt until an interrupt or reset",
    IDIV = "signed divide",
    IMUL = "signed multiply",
    IN = "read a byte or word from an I/O port",
    INC = "increment by one, leaving the carry flag alone",
    INT = "software interrupt",
    INT3 = "breakpoint interrupt (the one-byte encoding of INT 3)",
    INTO = "interrupt 4 if the overflow flag is set",
    IRET = "return from an interrupt, restoring the flags",
    JA = "jump if above (unsigned greater)",
    JB = "jump if below (unsigned less; carry set)",
    JBE = "jump if below or equal",
    JCXZ = "jump if CX is zero",
    JG = "jump if greater (signed)",
    JGE = "jump if greater or equal (signed)",
    JL = "jump if less (signed)",
    JLE = "jump if less or equal (signed)",
    JMP = "jump",
    JMPF = "jump to a far address",
    JNB = "jump if not below (carry clear)",
    JNO = "jump if the overflow flag is clear",
    JNP = "jump if parity odd",
    JNS = "jump if the sign flag is clear",
    JNZ = "jump if not equal (zero clear)",
    JO = "jump if the overflow flag is set",
    JP = "jump if parity even",
    JS = "jump if the sign flag is set",
    JZ = "jump if equal (zero set)",
    LAHF = "load the low flags byte into AH",
    LDS = "load a far pointer into DS and a register",
    LEA = "load the effective address, without accessing memory",
    LES = "load a far pointer into ES and a register",
    LOCK = "prefix: assert the LOCK pin for the next instruction",
    LODSB = "load a byte from DS:SI into AL",
    LODSW = "load a word from DS:SI into AX",
    LOOP = "decrement CX and jump if it is not zero",
    LOOPE = "decrement CX and jump if it is not zero and the zero flag is set",
    LOOPNE = "decrement CX and jump if it is not zero and the zero flag is clear",
    MOV = "move",
    MOVSB = "move a byte from DS:SI to ES:DI",
    MOVSW = "move a word from DS:SI to ES:DI",
    MUL = "unsigned multiply",
    NEG = "two's complement negate",
    NOP = "no operation (the XCHG AX,AX encoding)",
    NOT = "one's complement",
    OR = "bitwise OR",
    OUT = "write a byte or word to an I/O port",
    POP = "pop a word from the stack",
    POPF = "pop the flags register",
    PUSH = "push a word onto the stack",
    PUSHF = "push the flags register",
    RCL = "rotate left through the carry flag",
    RCR = "rotate right through the carry flag",
    REP = "prefix: repeat a string operation while CX is non-zero (and, for CMPS/SCAS, while equal)",
    REPNE = "prefix: repeat a string operation while CX is non-zero and not equal",
    RET = "return from a near procedure",
    RETF = "return from a far procedure",
    ROL = "rotate left",
    ROR = "rotate right",
    SAHF = "store AH into the low flags byte",
    SALC = "undocumented: set AL to 0xff if the carry flag is set, else 0",
    SAR = "arithmetic shift right, preserving the sign",
    SBB = "subtract with borrow",
    SCASB = "compare AL with the byte at ES:DI",
    SCASW = "compare AX with the word at ES:DI",
    SEG = "prefix: override the default segment",
    SETMO = "undocumented: set the operand to all ones",
    SHL = "shift left",
    SHR = "logical shift right",
    STC = "set the carry flag",
    STD = "set the direction flag",
    STI = "set the interrupt-enable flag",
    STOSB = "store AL at ES:DI",
    STOSW = "store AX at ES:DI",
    SUB = "subtract",
    TEST = "AND and set flags, discarding the result",
    WAIT = "wait for the coprocessor's TEST pin",
    XCHG = "exchange two operands",
    XLAT = "load AL from the table at DS:BX indexed by AL",
    XOR = "bitwise exclusive OR",

    // ---- 80186 --------------------------------------------------------
    BOUND = "check an array index against a pair of bounds, or raise #BR",
    ENTER = "make a stack frame, copying the enclosing frames' pointers",
    INSB = "read a byte from the port in DX to ES:DI",
    INSW = "read a word or dword from the port in DX to ES:DI",
    LEAVE = "unmake the stack frame ENTER made",
    OUTSB = "write the byte at DS:SI to the port in DX",
    OUTSW = "write the word or dword at DS:SI to the port in DX",
    POPA = "pop the eight general registers, discarding the saved SP",
    PUSHA = "push the eight general registers",

    // ---- 80286: protection --------------------------------------------
    ARPL = "raise a selector's requested privilege level to the caller's",
    CLTS = "clear the task-switched flag in CR0",
    LAR = "load a descriptor's access rights, if the selector is visible",
    LGDT = "load the global descriptor table register",
    LIDT = "load the interrupt descriptor table register",
    LLDT = "load the local descriptor table register",
    LMSW = "load the low sixteen bits of CR0 (the 286's machine status word)",
    LSL = "load a descriptor's segment limit, if the selector is visible",
    LTR = "load the task register",
    SGDT = "store the global descriptor table register",
    SIDT = "store the interrupt descriptor table register",
    SLDT = "store the local descriptor table register's selector",
    SMSW = "store the low sixteen bits of CR0",
    STR = "store the task register's selector",
    VERR = "set ZF if a segment is readable from the current privilege level",
    VERW = "set ZF if a segment is writable from the current privilege level",

    // ---- 80386 ---------------------------------------------------------
    BSF = "scan for the least significant set bit",
    BSR = "scan for the most significant set bit",
    BT = "copy one bit of the operand into the carry flag",
    BTC = "copy one bit into the carry flag and complement it",
    BTR = "copy one bit into the carry flag and clear it",
    BTS = "copy one bit into the carry flag and set it",
    ICEBP = "undocumented: interrupt 1, the in-circuit emulator breakpoint",
    LFS = "load a far pointer into FS and a register",
    LGS = "load a far pointer into GS and a register",
    LSS = "load a far pointer into SS and a register",
    MOVSX = "move with sign extension",
    MOVZX = "move with zero extension",
    SETA = "set the operand to 1 if above (unsigned greater), else 0",
    SETB = "set the operand to 1 if below (carry set), else 0",
    SETBE = "set the operand to 1 if below or equal, else 0",
    SETG = "set the operand to 1 if greater (signed), else 0",
    SETGE = "set the operand to 1 if greater or equal (signed), else 0",
    SETL = "set the operand to 1 if less (signed), else 0",
    SETLE = "set the operand to 1 if less or equal (signed), else 0",
    SETNB = "set the operand to 1 if not below (carry clear), else 0",
    SETNO = "set the operand to 1 if the overflow flag is clear, else 0",
    SETNP = "set the operand to 1 if parity odd, else 0",
    SETNS = "set the operand to 1 if the sign flag is clear, else 0",
    SETNZ = "set the operand to 1 if not equal, else 0",
    SETO = "set the operand to 1 if the overflow flag is set, else 0",
    SETP = "set the operand to 1 if parity even, else 0",
    SETS = "set the operand to 1 if the sign flag is set, else 0",
    SETZ = "set the operand to 1 if equal, else 0",
    SHLD = "shift left, filling from a second operand",
    SHRD = "shift right, filling from a second operand",
    UD = "undefined encoding: raises an invalid-opcode exception",

    // ---- 80486 ---------------------------------------------------------
    BSWAP = "reverse the byte order of a 32-bit register",
    CMPXCHG = "compare with the accumulator and exchange",
    CPUID = "report the processor's identity and features",
    INVD = "invalidate the caches without writing them back",
    INVLPG = "invalidate one page's translation-lookaside-buffer entry",
    WBINVD = "write the caches back and invalidate them",
    XADD = "exchange, then add",

    // ---- Pentium and later, still 32-bit -------------------------------
    CMOVA = "move if above (unsigned greater)",
    CMOVB = "move if below (carry set)",
    CMOVBE = "move if below or equal",
    CMOVG = "move if greater (signed)",
    CMOVGE = "move if greater or equal (signed)",
    CMOVL = "move if less (signed)",
    CMOVLE = "move if less or equal (signed)",
    CMOVNB = "move if not below (carry clear)",
    CMOVNO = "move if the overflow flag is clear",
    CMOVNP = "move if parity odd",
    CMOVNS = "move if the sign flag is clear",
    CMOVNZ = "move if not equal (zero clear)",
    CMOVO = "move if the overflow flag is set",
    CMOVP = "move if parity even",
    CMOVS = "move if the sign flag is set",
    CMOVZ = "move if equal (zero set)",
    RDMSR = "read the model-specific register ECX names into EDX:EAX",
    WRMSR = "write EDX:EAX to the model-specific register ECX names",

    // ---- x86-64 ---------------------------------------------------------
    MOVSXD = "move a 32-bit source, sign-extended to the operand size",
    REX = "prefix: extend the register fields, or widen the operand to 64 bits",
    SWAPGS = "exchange the GS base with IA32_KERNEL_GS_BASE",
    SYSCALL = "fast call to the kernel entry point in LSTAR",
    SYSRET = "fast return from SYSCALL",
}

impl Op {
    /// The assembler mnemonic, lower case.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        MNEMONICS[self as usize]
    }

    /// The condition this operation tests, numbered as the opcode map numbers
    /// them: 0 is overflow, 4 is zero, 12 is signed-less, and so on.
    ///
    /// One mapping serves the sixteen conditional jumps in both their short
    /// (`70`+cc) and near (`0F 80`+cc) forms and the sixteen `SETcc`
    /// encodings, which is the point — the condition is a property of the
    /// operation, and evaluating it belongs in exactly one place.
    #[must_use]
    pub const fn condition_code(self) -> Option<u8> {
        let cc = match self {
            Op::JO | Op::SETO | Op::CMOVO => 0,
            Op::JNO | Op::SETNO | Op::CMOVNO => 1,
            Op::JB | Op::SETB | Op::CMOVB => 2,
            Op::JNB | Op::SETNB | Op::CMOVNB => 3,
            Op::JZ | Op::SETZ | Op::CMOVZ => 4,
            Op::JNZ | Op::SETNZ | Op::CMOVNZ => 5,
            Op::JBE | Op::SETBE | Op::CMOVBE => 6,
            Op::JA | Op::SETA | Op::CMOVA => 7,
            Op::JS | Op::SETS | Op::CMOVS => 8,
            Op::JNS | Op::SETNS | Op::CMOVNS => 9,
            Op::JP | Op::SETP | Op::CMOVP => 10,
            Op::JNP | Op::SETNP | Op::CMOVNP => 11,
            Op::JL | Op::SETL | Op::CMOVL => 12,
            Op::JGE | Op::SETGE | Op::CMOVGE => 13,
            Op::JLE | Op::SETLE | Op::CMOVLE => 14,
            Op::JG | Op::SETG | Op::CMOVG => 15,
            _ => return None,
        };
        Some(cc)
    }

    /// Whether this operation is a `CMOVcc`.
    #[must_use]
    pub const fn is_cmov(self) -> bool {
        matches!(
            self,
            Op::CMOVO
                | Op::CMOVNO
                | Op::CMOVB
                | Op::CMOVNB
                | Op::CMOVZ
                | Op::CMOVNZ
                | Op::CMOVBE
                | Op::CMOVA
                | Op::CMOVS
                | Op::CMOVNS
                | Op::CMOVP
                | Op::CMOVNP
                | Op::CMOVL
                | Op::CMOVGE
                | Op::CMOVLE
                | Op::CMOVG
        )
    }

    /// Whether this operation's operand size defaults to sixty-four bits in
    /// 64-bit mode.
    ///
    /// Intel calls these `d64`: the default is eight bytes rather than four,
    /// `REX.W` is redundant on them, and a `66` prefix still narrows them to
    /// two. Everything that touches the stack implicitly is here, which is the
    /// unifying reason — `RSP` is always sixty-four bits wide in 64-bit mode,
    /// so a four-byte push would leave it misaligned by construction — along
    /// with the near branches, whose displacement stays `rel32` while the
    /// pointer they land in is full width.
    ///
    /// *Intel SDM* volume 2 §2.2.1.7 and table 2-4; the same list appears in
    /// the *AMD64 Architecture Programmer's Manual* volume 3 as "default
    /// operand size 64".
    #[must_use]
    pub const fn default_64(self) -> bool {
        matches!(
            self,
            Op::PUSH
                | Op::POP
                | Op::PUSHF
                | Op::POPF
                | Op::CALL
                | Op::RET
                | Op::JMP
                | Op::ENTER
                | Op::LEAVE
                | Op::LOOP
                | Op::LOOPE
                | Op::LOOPNE
                | Op::JCXZ
        ) || self.is_conditional_jump()
    }

    /// Whether this operation is a conditional jump, and therefore reads a
    /// flag rather than writing one.
    #[must_use]
    pub const fn is_conditional_jump(self) -> bool {
        matches!(
            self,
            Op::JO
                | Op::JNO
                | Op::JB
                | Op::JNB
                | Op::JZ
                | Op::JNZ
                | Op::JBE
                | Op::JA
                | Op::JS
                | Op::JNS
                | Op::JP
                | Op::JNP
                | Op::JL
                | Op::JGE
                | Op::JLE
                | Op::JG
        )
    }

    /// Whether this operation is a `SETcc`.
    #[must_use]
    pub const fn is_setcc(self) -> bool {
        self.condition_code().is_some() && !self.is_conditional_jump() && !self.is_cmov()
    }

    /// Whether this operation is one of the string primitives, and so may
    /// carry a `REP` prefix.
    ///
    /// The 80186's `INS` and `OUTS` join the 8086's five: they move between a
    /// port and `ES:DI` or `DS:SI` and take the same repeat prefix.
    #[must_use]
    pub const fn is_string(self) -> bool {
        matches!(
            self,
            Op::MOVSB
                | Op::MOVSW
                | Op::CMPSB
                | Op::CMPSW
                | Op::STOSB
                | Op::STOSW
                | Op::LODSB
                | Op::LODSW
                | Op::SCASB
                | Op::SCASW
                | Op::INSB
                | Op::INSW
                | Op::OUTSB
                | Op::OUTSW
        )
    }

    /// Whether a `REP` prefix on this operation stops on a flag as well as on
    /// a zero count.
    #[must_use]
    pub const fn repeat_tests_zf(self) -> bool {
        matches!(self, Op::CMPSB | Op::CMPSW | Op::SCASB | Op::SCASW)
    }

    /// Internal execution clocks, excluding every bus transfer.
    ///
    /// Intel's timing tables give a *total* that already contains the transfer
    /// cycles, which is the wrong shape for a bus-driven interpreter: it would
    /// charge a memory operand twice. What is quoted here is the manual's
    /// figure with the transfers taken back out, so the interpreter can add
    /// four clocks per bus cycle itself and arrive at the manual's number for
    /// the common cases.
    ///
    /// This is documented timing, not measured timing. The 8088 overlaps
    /// prefetching with execution and this model does not, so an instruction
    /// executed from an empty queue costs more here than on hardware. See the
    /// module docs on [`super::exec`](super) for what that means in practice.
    ///
    /// Source: iAPX 86/88 User's Manual, instruction-set timing tables.
    #[must_use]
    pub const fn clocks(self) -> u32 {
        match self {
            // The multiply and divide family dominates everything else, and
            // its figures are ranges in the manual; the low end is quoted.
            Op::MUL => 70,
            Op::IMUL => 80,
            Op::DIV => 80,
            Op::IDIV => 101,
            Op::AAM => 83,
            Op::AAD => 60,
            // Control transfer: the manual's figure less the stack transfers.
            Op::CALL => 11,
            Op::CALLF => 20,
            Op::RET => 8,
            Op::RETF => 10,
            Op::INT | Op::INT3 => 35,
            Op::INTO => 37,
            Op::IRET => 12,
            Op::JMP | Op::JMPF => 7,
            Op::LOOP | Op::LOOPE | Op::LOOPNE => 9,
            Op::JCXZ => 6,
            // String primitives, per iteration.
            Op::MOVSB | Op::MOVSW => 9,
            Op::CMPSB | Op::CMPSW => 14,
            Op::SCASB | Op::SCASW => 11,
            Op::LODSB | Op::LODSW => 8,
            Op::STOSB | Op::STOSW => 7,
            Op::XLAT => 7,
            Op::AAA | Op::AAS | Op::DAA | Op::DAS => 4,
            Op::PUSH | Op::PUSHF => 7,
            Op::POP | Op::POPF => 4,
            Op::LES | Op::LDS => 8,
            Op::LEA => 2,
            Op::IN | Op::OUT => 6,
            Op::ESC => 2,
            Op::HLT => 2,
            // The 186-and-later additions. These are the 80386 manual's
            // figures less their bus transfers, on the same principle as the
            // 8086 rows above: a 386 runs everything else here in two to nine
            // clocks, well inside the noise of a model that does not simulate
            // the pipeline.
            Op::PUSHA | Op::POPA => 18,
            Op::ENTER => 10,
            Op::LEAVE => 4,
            Op::BOUND => 10,
            Op::ARPL => 20,
            Op::LAR | Op::LSL => 15,
            Op::VERR | Op::VERW => 10,
            Op::LGDT | Op::LIDT | Op::LLDT | Op::LTR => 11,
            Op::SGDT | Op::SIDT | Op::SLDT | Op::STR | Op::SMSW => 2,
            Op::LMSW => 10,
            Op::CLTS => 5,
            Op::BSF | Op::BSR => 10,
            Op::BT => 3,
            Op::BTS | Op::BTR | Op::BTC => 6,
            Op::SHLD | Op::SHRD => 3,
            Op::MOVZX | Op::MOVSX => 3,
            Op::CPUID => 14,
            Op::INVD | Op::WBINVD => 4,
            Op::INVLPG => 12,
            Op::CMPXCHG | Op::XADD => 6,
            Op::BSWAP => 1,
            Op::INSB | Op::INSW | Op::OUTSB | Op::OUTSW => 9,
            // An undefined encoding costs nothing: it raises #UD instead of
            // executing, and the exception sequence charges its own clocks.
            Op::UD => 0,
            // Everything else is a two-to-four clock ALU or move operation;
            // the difference is inside the noise of a model that does not
            // simulate the prefetch queue's timing.
            _ => 3,
        }
    }
}

impl fmt::Display for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.mnemonic())
    }
}

/// One row of the instruction description: everything known about an encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Insn {
    /// What it does. For a group row this is the operation the primary opcode
    /// carries before [`resolve`] replaces it.
    pub op: Op,
    /// The destination operand, or [`Arg::None`].
    pub dst: Arg,
    /// The source operand, or [`Arg::None`].
    pub src: Arg,
    /// The third operand, or [`Arg::None`].
    ///
    /// Four 386-era encodings need one: `IMUL r,r/m,imm`, `SHLD`/`SHRD` and
    /// `ENTER`. Nothing on an 8086 does, which is why it defaults away.
    pub aux: Arg,
    /// Which opcode-extension group this belongs to.
    pub group: Grp,
    /// How well documented the encoding is.
    pub class: Class,
    /// What this encoding means in 64-bit mode, where that differs.
    ///
    /// The 64-bit answer lives *beside* the legacy one rather than in a second
    /// table, so a row cannot be changed in one and forgotten in the other —
    /// `ROADMAP.md` §6.1.1's "decode is gated per entry", applied to a mode
    /// rather than an extension. [`Insn::in_long`] is the only reader.
    pub long: L64,
}

/// What one encoding becomes in 64-bit mode.
///
/// Long mode did not extend the opcode map so much as **reclaim** it: the
/// sixteen `INC`/`DEC` encodings became the `REX` prefix, `ARPL` became
/// `MOVSXD`, and eighteen instructions that only make sense with 16-bit
/// segmentation or packed decimal arithmetic became invalid outright. Every
/// one of those is a difference a "just widen the registers" port gets wrong,
/// and each is named here rather than deduced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum L64 {
    /// Decodes and executes exactly as it does in the other modes.
    Same,
    /// Not an instruction in 64-bit mode: `#UD`.
    Gone,
    /// A different instruction, with its own operands.
    Alt(Op, Arg, Arg),
}

impl Insn {
    const fn new(op: Op, dst: Arg, src: Arg, group: Grp, class: Class) -> Insn {
        Insn {
            op,
            dst,
            src,
            aux: Arg::None,
            group,
            class,
            long: L64::Same,
        }
    }

    /// The same row with a third operand.
    const fn with_aux(mut self, aux: Arg) -> Insn {
        self.aux = aux;
        self
    }

    /// The same row with a 64-bit-mode override.
    const fn with_long(mut self, long: L64) -> Insn {
        self.long = long;
        self
    }

    /// This row as 64-bit mode decodes it.
    ///
    /// Applied once, by [`decode_stream_as`], before the ModRM byte is read —
    /// which matters, because whether an encoding *has* a ModRM byte can
    /// change with it (`63` gains one going from `ARPL` to `MOVSXD`; it has
    /// one either way, but `06` loses its operands entirely).
    #[must_use]
    pub const fn in_long(self) -> Insn {
        match self.long {
            L64::Same => self,
            L64::Gone => UNASSIGNED,
            L64::Alt(op, dst, src) => Insn {
                op,
                dst,
                src,
                aux: Arg::None,
                group: self.group,
                // A row that becomes `REX` becomes a *prefix*, and the class
                // is what says so — `40` is `INC EAX` in the other two modes
                // and carries no operand at all in this one.
                class: match op {
                    Op::REX => Class::Prefix,
                    _ => self.class,
                },
                long: L64::Same,
            },
        }
    }

    /// Whether decoding needs a ModRM byte.
    #[must_use]
    pub const fn needs_modrm(self) -> bool {
        !matches!(self.group, Grp::None)
            || self.dst.needs_modrm()
            || self.src.needs_modrm()
            || self.aux.needs_modrm()
    }

    /// The operand width in bytes, where the encoding fixes one, given an
    /// effective operand size.
    ///
    /// Taken from the destination first and the source second, because
    /// `MOV Ev,Ibs`-shaped rows carry the width on the destination while
    /// `PUSH Ib`-shaped ones carry it on the source.
    #[must_use]
    pub const fn width_bytes(self, osz: u8) -> Option<u8> {
        match self.dst.width_bytes(osz) {
            Some(w) => Some(w),
            None => self.src.width_bytes(osz),
        }
    }

    /// Whether this encoding is a prefix rather than an instruction.
    #[must_use]
    pub const fn is_prefix(self) -> bool {
        matches!(self.class, Class::Prefix)
    }
}

/// The row an opcode map holds where nothing is assigned.
///
/// On the 8086 map this should never survive: every one of the 256 encodings
/// does *something* on that part, and `LISTED` proves each is written out. On
/// the 386 maps it is the real answer for an unassigned encoding, which raises
/// an invalid-opcode exception.
const UNASSIGNED: Insn = Insn::new(Op::UD, Arg::None, Arg::None, Grp::None, Class::Undefined);

/// Build one opcode map from a list of rows, together with the array recording
/// which entries the list actually assigns.
///
/// Test scaffolding with a real job on the 8086 map, where an omission would
/// silently decode as a plausible-looking undefined row rather than fail.
/// Rows read `opcode mnemonic dst src [+aux] group class;` — the same order
/// the instruction-set summary prints them in.
macro_rules! opmap {
    (
        base $base:expr;
        $($opcode:literal $op:ident $dst:ident $src:ident $(+ $aux:ident)? $group:ident $class:ident
          $(=> ($($long:tt)+))? ;)*
    ) => {{
        let mut t: [Insn; 256] = $base;
        let mut listed = [false; 256];
        $(
            t[$opcode as usize] =
                Insn::new(Op::$op, Arg::$dst, Arg::$src, Grp::$group, Class::$class)
                $(.with_aux(Arg::$aux))?
                $(.with_long(long_spec!($($long)+)))?;
            listed[$opcode as usize] = true;
        )*
        (t, listed)
    }};
}

/// The `=> (…)` column of a row: what the encoding becomes in 64-bit mode.
///
/// `=> (UD)` for one long mode reclaimed, `=> (OP dst src)` for one it
/// repurposed.
macro_rules! long_spec {
    (UD) => {
        L64::Gone
    };
    ($op:ident $dst:ident $src:ident) => {
        L64::Alt(Op::$op, Arg::$dst, Arg::$src)
    };
}

/// The 8086/8088 primary map and its coverage record, built together so the
/// two cannot disagree.
const PRIMARY_8086: ([Insn; 256], [bool; 256]) = opmap! {
    base [UNASSIGNED; 256];
    0x00 ADD  Eb   Gb   None   Documented;
    0x01 ADD  Ev   Gv   None   Documented;
    0x02 ADD  Gb   Eb   None   Documented;
    0x03 ADD  Gv   Ev   None   Documented;
    0x04 ADD  Al   Ib   None   Documented;
    0x05 ADD  Ax   Iz   None   Documented;
    0x06 PUSH Sr   None None   Documented  => (UD);
    0x07 POP  Sr   None None   Documented  => (UD);
    0x08 OR   Eb   Gb   None   Documented;
    0x09 OR   Ev   Gv   None   Documented;
    0x0a OR   Gb   Eb   None   Documented;
    0x0b OR   Gv   Ev   None   Documented;
    0x0c OR   Al   Ib   None   Documented;
    0x0d OR   Ax   Iz   None   Documented;
    0x0e PUSH Sr   None None   Documented  => (UD);
    // POP CS assembles and executes; the 8086 simply does not document it,
    // and the 80186 reused the opcode for the two-byte escape.
    0x0f POP  Sr   None None   Undocumented;

    0x10 ADC  Eb   Gb   None   Documented;
    0x11 ADC  Ev   Gv   None   Documented;
    0x12 ADC  Gb   Eb   None   Documented;
    0x13 ADC  Gv   Ev   None   Documented;
    0x14 ADC  Al   Ib   None   Documented;
    0x15 ADC  Ax   Iz   None   Documented;
    0x16 PUSH Sr   None None   Documented  => (UD);
    0x17 POP  Sr   None None   Documented  => (UD);
    0x18 SBB  Eb   Gb   None   Documented;
    0x19 SBB  Ev   Gv   None   Documented;
    0x1a SBB  Gb   Eb   None   Documented;
    0x1b SBB  Gv   Ev   None   Documented;
    0x1c SBB  Al   Ib   None   Documented;
    0x1d SBB  Ax   Iz   None   Documented;
    0x1e PUSH Sr   None None   Documented  => (UD);
    0x1f POP  Sr   None None   Documented  => (UD);

    0x20 AND  Eb   Gb   None   Documented;
    0x21 AND  Ev   Gv   None   Documented;
    0x22 AND  Gb   Eb   None   Documented;
    0x23 AND  Gv   Ev   None   Documented;
    0x24 AND  Al   Ib   None   Documented;
    0x25 AND  Ax   Iz   None   Documented;
    0x26 SEG  Sr   None None   Prefix;
    0x27 DAA  None None None   Documented  => (UD);
    0x28 SUB  Eb   Gb   None   Documented;
    0x29 SUB  Ev   Gv   None   Documented;
    0x2a SUB  Gb   Eb   None   Documented;
    0x2b SUB  Gv   Ev   None   Documented;
    0x2c SUB  Al   Ib   None   Documented;
    0x2d SUB  Ax   Iz   None   Documented;
    0x2e SEG  Sr   None None   Prefix;
    0x2f DAS  None None None   Documented  => (UD);

    0x30 XOR  Eb   Gb   None   Documented;
    0x31 XOR  Ev   Gv   None   Documented;
    0x32 XOR  Gb   Eb   None   Documented;
    0x33 XOR  Gv   Ev   None   Documented;
    0x34 XOR  Al   Ib   None   Documented;
    0x35 XOR  Ax   Iz   None   Documented;
    0x36 SEG  Sr   None None   Prefix;
    0x37 AAA  None None None   Documented  => (UD);
    0x38 CMP  Eb   Gb   None   Documented;
    0x39 CMP  Ev   Gv   None   Documented;
    0x3a CMP  Gb   Eb   None   Documented;
    0x3b CMP  Gv   Ev   None   Documented;
    0x3c CMP  Al   Ib   None   Documented;
    0x3d CMP  Ax   Iz   None   Documented;
    0x3e SEG  Sr   None None   Prefix;
    0x3f AAS  None None None   Documented  => (UD);

    0x40 INC  Rv   None None   Documented  => (REX None None);
    0x41 INC  Rv   None None   Documented  => (REX None None);
    0x42 INC  Rv   None None   Documented  => (REX None None);
    0x43 INC  Rv   None None   Documented  => (REX None None);
    0x44 INC  Rv   None None   Documented  => (REX None None);
    0x45 INC  Rv   None None   Documented  => (REX None None);
    0x46 INC  Rv   None None   Documented  => (REX None None);
    0x47 INC  Rv   None None   Documented  => (REX None None);
    0x48 DEC  Rv   None None   Documented  => (REX None None);
    0x49 DEC  Rv   None None   Documented  => (REX None None);
    0x4a DEC  Rv   None None   Documented  => (REX None None);
    0x4b DEC  Rv   None None   Documented  => (REX None None);
    0x4c DEC  Rv   None None   Documented  => (REX None None);
    0x4d DEC  Rv   None None   Documented  => (REX None None);
    0x4e DEC  Rv   None None   Documented  => (REX None None);
    0x4f DEC  Rv   None None   Documented  => (REX None None);

    0x50 PUSH Rv   None None   Documented;
    0x51 PUSH Rv   None None   Documented;
    0x52 PUSH Rv   None None   Documented;
    0x53 PUSH Rv   None None   Documented;
    0x54 PUSH Rv   None None   Documented;
    0x55 PUSH Rv   None None   Documented;
    0x56 PUSH Rv   None None   Documented;
    0x57 PUSH Rv   None None   Documented;
    0x58 POP  Rv   None None   Documented;
    0x59 POP  Rv   None None   Documented;
    0x5a POP  Rv   None None   Documented;
    0x5b POP  Rv   None None   Documented;
    0x5c POP  Rv   None None   Documented;
    0x5d POP  Rv   None None   Documented;
    0x5e POP  Rv   None None   Documented;
    0x5f POP  Rv   None None   Documented;

    // 60-6F are the 8086's aliases of the conditional jumps: the opcode mask
    // that selects the microcode entry point ignores bit 4.
    0x60 JO   Jb   None None   Alias;
    0x61 JNO  Jb   None None   Alias;
    0x62 JB   Jb   None None   Alias;
    0x63 JNB  Jb   None None   Alias;
    0x64 JZ   Jb   None None   Alias;
    0x65 JNZ  Jb   None None   Alias;
    0x66 JBE  Jb   None None   Alias;
    0x67 JA   Jb   None None   Alias;
    0x68 JS   Jb   None None   Alias;
    0x69 JNS  Jb   None None   Alias;
    0x6a JP   Jb   None None   Alias;
    0x6b JNP  Jb   None None   Alias;
    0x6c JL   Jb   None None   Alias;
    0x6d JGE  Jb   None None   Alias;
    0x6e JLE  Jb   None None   Alias;
    0x6f JG   Jb   None None   Alias;

    0x70 JO   Jb   None None   Documented;
    0x71 JNO  Jb   None None   Documented;
    0x72 JB   Jb   None None   Documented;
    0x73 JNB  Jb   None None   Documented;
    0x74 JZ   Jb   None None   Documented;
    0x75 JNZ  Jb   None None   Documented;
    0x76 JBE  Jb   None None   Documented;
    0x77 JA   Jb   None None   Documented;
    0x78 JS   Jb   None None   Documented;
    0x79 JNS  Jb   None None   Documented;
    0x7a JP   Jb   None None   Documented;
    0x7b JNP  Jb   None None   Documented;
    0x7c JL   Jb   None None   Documented;
    0x7d JGE  Jb   None None   Documented;
    0x7e JLE  Jb   None None   Documented;
    0x7f JG   Jb   None None   Documented;

    0x80 ADD  Eb   Ib   Alu    Documented;
    0x81 ADD  Ev   Iz   Alu    Documented;
    // 82 is 80 again: the sign-extend bit is not decoded for byte operands.
    0x82 ADD  Eb   Ib   Alu    Alias  => (UD);
    0x83 ADD  Ev   Ibs  Alu    Documented;
    0x84 TEST Eb   Gb   None   Documented;
    0x85 TEST Ev   Gv   None   Documented;
    0x86 XCHG Eb   Gb   None   Documented;
    0x87 XCHG Ev   Gv   None   Documented;
    0x88 MOV  Eb   Gb   None   Documented;
    0x89 MOV  Ev   Gv   None   Documented;
    0x8a MOV  Gb   Eb   None   Documented;
    0x8b MOV  Gv   Ev   None   Documented;
    0x8c MOV  Ev   Sw   None   Documented;
    0x8d LEA  Gv   M    None   Documented;
    0x8e MOV  Sw   Ev   None   Documented;
    0x8f POP  Ev   None Pop    Documented;

    0x90 NOP  None None None   Documented;
    0x91 XCHG Ax   Rv   None   Documented;
    0x92 XCHG Ax   Rv   None   Documented;
    0x93 XCHG Ax   Rv   None   Documented;
    0x94 XCHG Ax   Rv   None   Documented;
    0x95 XCHG Ax   Rv   None   Documented;
    0x96 XCHG Ax   Rv   None   Documented;
    0x97 XCHG Ax   Rv   None   Documented;
    0x98 CBW  None None None   Documented;
    0x99 CWD  None None None   Documented;
    0x9a CALLF Ap  None None   Documented  => (UD);
    0x9b WAIT None None None   Documented;
    0x9c PUSHF None None None  Documented;
    0x9d POPF None None None   Documented;
    0x9e SAHF None None None   Documented;
    0x9f LAHF None None None   Documented;

    0xa0 MOV  Al   Ob   None   Documented;
    0xa1 MOV  Ax   Ov   None   Documented;
    0xa2 MOV  Ob   Al   None   Documented;
    0xa3 MOV  Ov   Ax   None   Documented;
    0xa4 MOVSB Yb  Xb   None   Documented;
    0xa5 MOVSW Yv  Xv   None   Documented;
    0xa6 CMPSB Xb  Yb   None   Documented;
    0xa7 CMPSW Xv  Yv   None   Documented;
    0xa8 TEST Al   Ib   None   Documented;
    0xa9 TEST Ax   Iz   None   Documented;
    0xaa STOSB Yb  Al   None   Documented;
    0xab STOSW Yv  Ax   None   Documented;
    0xac LODSB Al  Xb   None   Documented;
    0xad LODSW Ax  Xv   None   Documented;
    0xae SCASB Al  Yb   None   Documented;
    0xaf SCASW Ax  Yv   None   Documented;

    0xb0 MOV  Rb   Ib   None   Documented;
    0xb1 MOV  Rb   Ib   None   Documented;
    0xb2 MOV  Rb   Ib   None   Documented;
    0xb3 MOV  Rb   Ib   None   Documented;
    0xb4 MOV  Rb   Ib   None   Documented;
    0xb5 MOV  Rb   Ib   None   Documented;
    0xb6 MOV  Rb   Ib   None   Documented;
    0xb7 MOV  Rb   Ib   None   Documented;
    0xb8 MOV  Rv   Iv   None   Documented;
    0xb9 MOV  Rv   Iv   None   Documented;
    0xba MOV  Rv   Iv   None   Documented;
    0xbb MOV  Rv   Iv   None   Documented;
    0xbc MOV  Rv   Iv   None   Documented;
    0xbd MOV  Rv   Iv   None   Documented;
    0xbe MOV  Rv   Iv   None   Documented;
    0xbf MOV  Rv   Iv   None   Documented;

    // C0/C1 and C8/C9 are the near and far RET encodings again: bit 3 of the
    // opcode is not decoded. The immediate is `Iw` rather than `Iv` because
    // the byte count `RET` adds to the stack pointer is sixteen bits on every
    // part in the family, whatever the operand size is.
    0xc0 RET  Iw   None None   Alias;
    0xc1 RET  None None None   Alias;
    0xc2 RET  Iw   None None   Documented;
    0xc3 RET  None None None   Documented;
    0xc4 LES  Gv   Mp   None   Documented  => (UD);
    0xc5 LDS  Gv   Mp   None   Documented  => (UD);
    0xc6 MOV  Eb   Ib   MovImm Documented;
    0xc7 MOV  Ev   Iz   MovImm Documented;
    0xc8 RETF Iw   None None   Alias;
    0xc9 RETF None None None   Alias;
    0xca RETF Iw   None None   Documented;
    0xcb RETF None None None   Documented;
    0xcc INT3 None None None   Documented;
    0xcd INT  Ib   None None   Documented;
    0xce INTO None None None   Documented  => (UD);
    0xcf IRET None None None   Documented;

    0xd0 ROL  Eb   One  Shift  Documented;
    0xd1 ROL  Ev   One  Shift  Documented;
    0xd2 ROL  Eb   Cl   Shift  Documented;
    0xd3 ROL  Ev   Cl   Shift  Documented;
    0xd4 AAM  Ib   None None   Documented  => (UD);
    0xd5 AAD  Ib   None None   Documented  => (UD);
    0xd6 SALC None None None   Undocumented  => (UD);
    0xd7 XLAT None None None   Documented;
    0xd8 ESC  Ev   None None   Escape;
    0xd9 ESC  Ev   None None   Escape;
    0xda ESC  Ev   None None   Escape;
    0xdb ESC  Ev   None None   Escape;
    0xdc ESC  Ev   None None   Escape;
    0xdd ESC  Ev   None None   Escape;
    0xde ESC  Ev   None None   Escape;
    0xdf ESC  Ev   None None   Escape;

    0xe0 LOOPNE Jb None None   Documented;
    0xe1 LOOPE  Jb None None   Documented;
    0xe2 LOOP   Jb None None   Documented;
    0xe3 JCXZ   Jb None None   Documented;
    0xe4 IN   Al   Ib   None   Documented;
    0xe5 IN   Ax   Ib   None   Documented;
    0xe6 OUT  Ib   Al   None   Documented;
    0xe7 OUT  Ib   Ax   None   Documented;
    0xe8 CALL Jv   None None   Documented;
    0xe9 JMP  Jv   None None   Documented;
    0xea JMPF Ap   None None   Documented  => (UD);
    0xeb JMP  Jb   None None   Documented;
    0xec IN   Al   Dx   None   Documented;
    0xed IN   Ax   Dx   None   Documented;
    0xee OUT  Dx   Al   None   Documented;
    0xef OUT  Dx   Ax   None   Documented;

    0xf0 LOCK  None None None  Prefix;
    // F1 asserts LOCK exactly as F0 does; bit 0 is not decoded.
    0xf1 LOCK  None None None  Prefix;
    0xf2 REPNE None None None  Prefix;
    0xf3 REP   None None None  Prefix;
    0xf4 HLT   None None None  Documented;
    0xf5 CMC   None None None  Documented;
    0xf6 TEST  Eb   Ib   Unary Documented;
    0xf7 TEST  Ev   Iz   Unary Documented;
    0xf8 CLC   None None None  Documented;
    0xf9 STC   None None None  Documented;
    0xfa CLI   None None None  Documented;
    0xfb STI   None None None  Documented;
    0xfc CLD   None None None  Documented;
    0xfd STD   None None None  Documented;
    0xfe INC   Eb   None IncDec Documented;
    0xff INC   Ev   None Misc   Documented;
};

/// The 8086/8088 primary decode table: one row per opcode byte, and — with the
/// group tables below — the only description of that part's instruction set in
/// the crate.
pub static TABLE: [Insn; 256] = PRIMARY_8086.0;

/// Which opcodes [`TABLE`] actually assigns. All 256 of them: the 8086 has no
/// invalid-opcode exception, so every encoding does something and every one is
/// written out.
pub static LISTED: [bool; 256] = PRIMARY_8086.1;

/// The 80186-through-80486 primary map, as a delta on the 8086's.
///
/// Everything not listed here is unchanged from [`TABLE`] — which is the
/// honest shape, because that is what the parts are: one map with sixteen
/// encodings reclaimed, four aliases spent on new instructions, four prefixes
/// added, and `0F` turned into an escape.
const PRIMARY_386: ([Insn; 256], [bool; 256]) = opmap! {
    base PRIMARY_8086.0;

    // `POP CS` became the two-byte escape on the 80286. The row is a marker:
    // `decode_stream` never executes it, it switches maps on seeing the byte.
    0x0f ESC   None None None   Escape;

    // 60-6F stopped aliasing the conditional jumps on the 80186.
    0x60 PUSHA None None None   Documented  => (UD);
    0x61 POPA  None None None   Documented  => (UD);
    0x62 BOUND Gv   M    None   Documented  => (UD);
    0x63 ARPL  Ew   Gw   None   Documented  => (MOVSXD Gv Ed);
    0x64 SEG   None None None   Prefix;
    0x65 SEG   None None None   Prefix;
    0x66 SEG   None None None   Prefix;
    0x67 SEG   None None None   Prefix;
    0x68 PUSH  Iz   None None   Documented;
    0x69 IMUL  Gv   Ev  +Iz     None   Documented;
    0x6a PUSH  Ibs  None None   Documented;
    0x6b IMUL  Gv   Ev  +Ibs    None   Documented;
    0x6c INSB  Yb   Dx   None   Documented;
    0x6d INSW  Yv   Dx   None   Documented;
    0x6e OUTSB Dx   Xb   None   Documented;
    0x6f OUTSW Dx   Xv   None   Documented;

    // `MOV Sreg,r/m` reads sixteen bits whatever the operand size is, and a
    // 386 decodes all three bits of `reg` rather than two.
    0x8e MOV   Sw   Ew   None   Documented;

    // The shift group gained an immediate count, and C8/C9 the frame
    // instructions, so none of the four is an alias any more.
    0xc0 ROL   Eb   Ib   Shift  Documented;
    0xc1 ROL   Ev   Ib   Shift  Documented;
    0xc8 ENTER Iw   Ib   None   Documented;
    0xc9 LEAVE None None None   Documented;

    // `F1` is the in-circuit-emulator breakpoint, interrupt 1. Undocumented
    // by Intel, implemented by every part from the 386 on, and used by
    // debuggers precisely because it is one byte and not `CC`.
    0xf1 ICEBP None None None   Undocumented;
};

/// The 80186-through-80486 primary decode table.
pub static TABLE_386: [Insn; 256] = PRIMARY_386.0;

/// Which opcodes [`TABLE_386`] changes from the 8086's map. Not coverage —
/// the 386 map inherits the rest — but a checkable statement of the delta.
pub static CHANGED_386: [bool; 256] = PRIMARY_386.1;

/// The two-byte (`0F`) opcode map.
///
/// Everything the 80286 and 80386 added that would not fit in the primary map:
/// the protection instructions, the near conditional jumps, `SETcc`, the bit
/// instructions, the double shifts, `MOVZX`/`MOVSX`, the control- and
/// debug-register moves, and the 80486's cache and atomic additions.
/// Unassigned rows are [`Op::UD`], which is the truth: on a 386 an unassigned
/// two-byte encoding raises an invalid-opcode exception.
const SECONDARY: ([Insn; 256], [bool; 256]) = opmap! {
    base [UNASSIGNED; 256];

    0x00 SLDT  Ew   None Grp6   Documented;
    0x01 SGDT  Ms   None Grp7   Documented;
    0x02 LAR   Gv   Ev   None   Documented;
    0x03 LSL   Gv   Ev   None   Documented;
    0x06 CLTS  None None None   Documented;
    0x08 INVD  None None None   Documented;
    0x09 WBINVD None None None  Documented;

    // `SYSCALL` and `SYSRET` are 64-bit-only on an Intel part — AMD's K6
    // implemented them in legacy mode and Intel never did, so `#UD` outside
    // long mode is the behaviour every operating system is written against.
    // Modelling that as a 64-bit-mode override rather than a feature check
    // puts the difference where the encoding is (*AMD64 Architecture
    // Programmer's Manual* volume 3, `SYSCALL`; *Intel SDM* volume 2).
    0x05 UD    None None None   Undefined  => (SYSCALL None None);
    0x07 UD    None None None   Undefined  => (SYSRET None None);

    0x20 MOV   Rd   Cd   None   Documented;
    0x21 MOV   Rd   Dd   None   Documented;
    0x22 MOV   Cd   Rd   None   Documented;
    0x23 MOV   Dd   Rd   None   Documented;
    0x24 MOV   Rd   Td   None   Documented;
    0x26 MOV   Td   Rd   None   Documented;

    // The model-specific registers. Present from the Pentium, so these are
    // ordinary rows gated by `Features::msr` at execution — a 486 that decodes
    // `0F 32` must still raise `#UD`, and the check belongs where the feature
    // is known rather than in the table.
    0x30 WRMSR None None None   Documented;
    0x32 RDMSR None None None   Documented;

    // `CMOVcc`, from the Pentium Pro; gated by `Features::cmov`.
    0x40 CMOVO  Gv  Ev   None   Documented;
    0x41 CMOVNO Gv  Ev   None   Documented;
    0x42 CMOVB  Gv  Ev   None   Documented;
    0x43 CMOVNB Gv  Ev   None   Documented;
    0x44 CMOVZ  Gv  Ev   None   Documented;
    0x45 CMOVNZ Gv  Ev   None   Documented;
    0x46 CMOVBE Gv  Ev   None   Documented;
    0x47 CMOVA  Gv  Ev   None   Documented;
    0x48 CMOVS  Gv  Ev   None   Documented;
    0x49 CMOVNS Gv  Ev   None   Documented;
    0x4a CMOVP  Gv  Ev   None   Documented;
    0x4b CMOVNP Gv  Ev   None   Documented;
    0x4c CMOVL  Gv  Ev   None   Documented;
    0x4d CMOVGE Gv  Ev   None   Documented;
    0x4e CMOVLE Gv  Ev   None   Documented;
    0x4f CMOVG  Gv  Ev   None   Documented;

    0x80 JO    Jv   None None   Documented;
    0x81 JNO   Jv   None None   Documented;
    0x82 JB    Jv   None None   Documented;
    0x83 JNB   Jv   None None   Documented;
    0x84 JZ    Jv   None None   Documented;
    0x85 JNZ   Jv   None None   Documented;
    0x86 JBE   Jv   None None   Documented;
    0x87 JA    Jv   None None   Documented;
    0x88 JS    Jv   None None   Documented;
    0x89 JNS   Jv   None None   Documented;
    0x8a JP    Jv   None None   Documented;
    0x8b JNP   Jv   None None   Documented;
    0x8c JL    Jv   None None   Documented;
    0x8d JGE   Jv   None None   Documented;
    0x8e JLE   Jv   None None   Documented;
    0x8f JG    Jv   None None   Documented;

    0x90 SETO  Eb   None None   Documented;
    0x91 SETNO Eb   None None   Documented;
    0x92 SETB  Eb   None None   Documented;
    0x93 SETNB Eb   None None   Documented;
    0x94 SETZ  Eb   None None   Documented;
    0x95 SETNZ Eb   None None   Documented;
    0x96 SETBE Eb   None None   Documented;
    0x97 SETA  Eb   None None   Documented;
    0x98 SETS  Eb   None None   Documented;
    0x99 SETNS Eb   None None   Documented;
    0x9a SETP  Eb   None None   Documented;
    0x9b SETNP Eb   None None   Documented;
    0x9c SETL  Eb   None None   Documented;
    0x9d SETGE Eb   None None   Documented;
    0x9e SETLE Eb   None None   Documented;
    0x9f SETG  Eb   None None   Documented;

    0xa0 PUSH  Sr   None None   Documented;
    0xa1 POP   Sr   None None   Documented;
    0xa2 CPUID None None None   Documented;
    0xa3 BT    Ev   Gv   None   Documented;
    0xa4 SHLD  Ev   Gv  +Ib     None   Documented;
    0xa5 SHLD  Ev   Gv  +Cl     None   Documented;
    0xa8 PUSH  Sr   None None   Documented;
    0xa9 POP   Sr   None None   Documented;
    0xab BTS   Ev   Gv   None   Documented;
    0xac SHRD  Ev   Gv  +Ib     None   Documented;
    0xad SHRD  Ev   Gv  +Cl     None   Documented;
    0xaf IMUL  Gv   Ev   None   Documented;

    0xb0 CMPXCHG Eb Gb   None   Documented;
    0xb1 CMPXCHG Ev Gv   None   Documented;
    0xb2 LSS   Gv   Mp   None   Documented;
    0xb3 BTR   Ev   Gv   None   Documented;
    0xb4 LFS   Gv   Mp   None   Documented;
    0xb5 LGS   Gv   Mp   None   Documented;
    0xb6 MOVZX Gv   Eb   None   Documented;
    0xb7 MOVZX Gv   Ew   None   Documented;
    0xba BT    Ev   Ib   Grp8   Documented;
    0xbb BTC   Ev   Gv   None   Documented;
    0xbc BSF   Gv   Ev   None   Documented;
    0xbd BSR   Gv   Ev   None   Documented;
    0xbe MOVSX Gv   Eb   None   Documented;
    0xbf MOVSX Gv   Ew   None   Documented;

    0xc0 XADD  Eb   Gb   None   Documented;
    0xc1 XADD  Ev   Gv   None   Documented;
    0xc8 BSWAP Rv   None None   Documented;
    0xc9 BSWAP Rv   None None   Documented;
    0xca BSWAP Rv   None None   Documented;
    0xcb BSWAP Rv   None None   Documented;
    0xcc BSWAP Rv   None None   Documented;
    0xcd BSWAP Rv   None None   Documented;
    0xce BSWAP Rv   None None   Documented;
    0xcf BSWAP Rv   None None   Documented;
};

/// The two-byte (`0F`) decode table.
pub static TABLE_0F: [Insn; 256] = SECONDARY.0;

/// Which two-byte encodings are assigned. The rest raise `#UD`.
pub static LISTED_0F: [bool; 256] = SECONDARY.1;

/// `80`-`83`: the ModRM `reg` field picks one of the eight ALU operations.
///
/// Only the operation varies; the operand forms belong to the primary opcode,
/// which is why `80` takes `Eb,Ib` and `83` takes `Ev,Ibs`.
pub static GROUP_ALU: [Op; 8] = [
    Op::ADD,
    Op::OR,
    Op::ADC,
    Op::SBB,
    Op::AND,
    Op::SUB,
    Op::XOR,
    Op::CMP,
];

/// `D0`-`D3`: the shift and rotate group.
///
/// `reg == 6` is `SETMO`, which Intel never documented: it sets the operand to
/// all ones. With a `CL` count it is conditional on `CL != 0`, exactly like
/// the shifts beside it, which is why it needs no separate row — the count
/// operand of the primary opcode already says which form it is.
pub static GROUP_SHIFT: [Op; 8] = [
    Op::ROL,
    Op::ROR,
    Op::RCL,
    Op::RCR,
    Op::SHL,
    Op::SHR,
    Op::SETMO,
    Op::SAR,
];

/// `F6`/`F7`: `TEST`, `NOT`, `NEG` and the multiply/divide family.
///
/// Each entry carries its own operands because `TEST` takes an immediate that
/// the rest do not; `reg == 1` is `TEST` again, since bit 0 of the extension
/// is not decoded.
const fn unary_group(byte: bool) -> [Insn; 8] {
    let (e, i) = if byte {
        (Arg::Eb, Arg::Ib)
    } else {
        (Arg::Ev, Arg::Iz)
    };
    [
        Insn::new(Op::TEST, e, i, Grp::None, Class::Documented),
        Insn::new(Op::TEST, e, i, Grp::None, Class::Alias),
        Insn::new(Op::NOT, e, Arg::None, Grp::None, Class::Documented),
        Insn::new(Op::NEG, e, Arg::None, Grp::None, Class::Documented),
        Insn::new(Op::MUL, e, Arg::None, Grp::None, Class::Documented),
        Insn::new(Op::IMUL, e, Arg::None, Grp::None, Class::Documented),
        Insn::new(Op::DIV, e, Arg::None, Grp::None, Class::Documented),
        Insn::new(Op::IDIV, e, Arg::None, Grp::None, Class::Documented),
    ]
}

/// `F6`: the byte form of the unary group.
pub static GROUP_UNARY8: [Insn; 8] = unary_group(true);

/// `F7`: the word form of the unary group.
pub static GROUP_UNARY16: [Insn; 8] = unary_group(false);

/// `FF`: increment, decrement, the four indirect control transfers, and push.
///
/// `reg == 7` is undefined and behaves as `PUSH`, which is what the hardware
/// corpus shows: bit 0 of the extension is not decoded for the push entry
/// either.
pub static GROUP_MISC: [Insn; 8] = [
    Insn::new(Op::INC, Arg::Ev, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::DEC, Arg::Ev, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::CALL, Arg::Ev, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::CALLF, Arg::Mp, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::JMP, Arg::Ev, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::JMPF, Arg::Mp, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::PUSH, Arg::Ev, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::PUSH, Arg::Ev, Arg::None, Grp::None, Class::Undefined),
];

/// `FE`: the byte increment and decrement.
///
/// Extensions 2-7 are undefined. The 8086's group decode does not check the
/// operand-size bit for them, so they enter the same microcode as the `FF`
/// group and operate on a word; that is what is modelled here. It is a
/// deduction from the encoding rather than a measurement — the hardware corpus
/// deliberately omits these forms — and it is flagged [`Class::Undefined`] so
/// nothing mistakes it for a checked fact.
pub static GROUP_INCDEC: [Insn; 8] = [
    Insn::new(Op::INC, Arg::Eb, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::DEC, Arg::Eb, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::CALL, Arg::Ev, Arg::None, Grp::None, Class::Undefined),
    Insn::new(Op::CALLF, Arg::Mp, Arg::None, Grp::None, Class::Undefined),
    Insn::new(Op::JMP, Arg::Ev, Arg::None, Grp::None, Class::Undefined),
    Insn::new(Op::JMPF, Arg::Mp, Arg::None, Grp::None, Class::Undefined),
    Insn::new(Op::PUSH, Arg::Ev, Arg::None, Grp::None, Class::Undefined),
    Insn::new(Op::PUSH, Arg::Ev, Arg::None, Grp::None, Class::Undefined),
];

/// `FF` on a 386: extension 7 is not a second `PUSH` but an invalid encoding.
static GROUP_MISC_386: [Insn; 8] = {
    let mut t = GROUP_MISC;
    t[7] = UNASSIGNED;
    t
};

/// `FE` on a 386: only the byte increment and decrement are assigned.
static GROUP_INCDEC_386: [Insn; 8] = {
    let mut t = [UNASSIGNED; 8];
    t[0] = GROUP_INCDEC[0];
    t[1] = GROUP_INCDEC[1];
    t
};

/// `0F 00`: Intel's group 6, the descriptor-table and segment-check
/// instructions that do not need a memory pseudo-descriptor.
///
/// Every one of these takes an `r/m16` — a selector — whatever the operand
/// size is. Extensions 6 and 7 are unassigned.
pub static GROUP6: [Insn; 8] = [
    Insn::new(Op::SLDT, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::STR, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::LLDT, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::LTR, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::VERR, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::VERW, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    UNASSIGNED,
    UNASSIGNED,
];

/// `0F 01`: Intel's group 7 — the descriptor table registers, the machine
/// status word, and (on the 80486) `INVLPG`.
pub static GROUP7: [Insn; 8] = [
    Insn::new(Op::SGDT, Arg::Ms, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::SIDT, Arg::Ms, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::LGDT, Arg::Ms, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::LIDT, Arg::Ms, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::SMSW, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    UNASSIGNED,
    Insn::new(Op::LMSW, Arg::Ew, Arg::None, Grp::None, Class::Documented),
    Insn::new(Op::INVLPG, Arg::M, Arg::None, Grp::None, Class::Documented),
];

/// `0F BA`: Intel's group 8, the bit tests with an immediate bit number.
///
/// Extensions 0-3 are unassigned, which is why the group exists at all: the
/// bit number would otherwise have nowhere to go.
pub static GROUP8: [Insn; 8] = [
    UNASSIGNED,
    UNASSIGNED,
    UNASSIGNED,
    UNASSIGNED,
    Insn::new(Op::BT, Arg::Ev, Arg::Ib, Grp::None, Class::Documented),
    Insn::new(Op::BTS, Arg::Ev, Arg::Ib, Grp::None, Class::Documented),
    Insn::new(Op::BTR, Arg::Ev, Arg::Ib, Grp::None, Class::Documented),
    Insn::new(Op::BTC, Arg::Ev, Arg::Ib, Grp::None, Class::Documented),
];

/// Decode one primary-map opcode byte on the 8086.
///
/// Total: all 256 encodings are described, prefixes and undocumented ones
/// included, so decoding never fails. Rows whose [`Insn::group`] is not
/// [`Grp::None`] still need [`resolve`] once the ModRM byte has been read.
#[inline]
#[must_use]
pub const fn decode(opcode: u8) -> Insn {
    TABLE[opcode as usize]
}

/// Decode one primary-map opcode byte on the named generation.
#[inline]
#[must_use]
pub const fn decode_as(map: Gen, opcode: u8) -> Insn {
    match map {
        Gen::I8086 => TABLE[opcode as usize],
        Gen::I386 => TABLE_386[opcode as usize],
    }
}

/// Decode one two-byte (`0F`-prefixed) opcode byte.
#[inline]
#[must_use]
pub const fn decode_0f(opcode: u8) -> Insn {
    TABLE_0F[opcode as usize]
}

/// Apply an opcode-extension group, given the ModRM `reg` field.
///
/// A no-op for the great majority of encodings, which is why the interpreter
/// can call it unconditionally.
#[inline]
#[must_use]
pub const fn resolve(insn: Insn, reg: u8) -> Insn {
    resolve_as(Gen::I8086, insn, reg)
}

/// Apply an opcode-extension group on the named generation.
///
/// The generation matters for four groups. `8F` (`POP`) and `C6`/`C7` (`MOV`
/// immediate) do not decode the extension at all on an 8086 — every value is
/// the same instruction — while the 80386 Programmer's Reference Manual lists
/// extensions 1-7 of both as invalid. `FE` and `FF` likewise lost the
/// extensions the 8086 left to fall through the group decode.
#[inline]
#[must_use]
pub const fn resolve_as(map: Gen, insn: Insn, reg: u8) -> Insn {
    let idx = (reg & 7) as usize;
    let strict = matches!(map, Gen::I386);
    match insn.group {
        Grp::None => insn,
        Grp::Pop | Grp::MovImm => {
            if strict && idx != 0 {
                UNASSIGNED
            } else {
                insn
            }
        }
        Grp::Alu => Insn {
            op: GROUP_ALU[idx],
            ..insn
        },
        Grp::Shift => Insn {
            op: GROUP_SHIFT[idx],
            ..insn
        },
        Grp::IncDec => {
            if strict {
                GROUP_INCDEC_386[idx]
            } else {
                GROUP_INCDEC[idx]
            }
        }
        Grp::Unary => {
            if matches!(insn.dst, Arg::Eb) {
                GROUP_UNARY8[idx]
            } else {
                GROUP_UNARY16[idx]
            }
        }
        Grp::Misc => {
            if strict {
                GROUP_MISC_386[idx]
            } else {
                GROUP_MISC[idx]
            }
        }
        Grp::Grp6 => GROUP6[idx],
        Grp::Grp7 => GROUP7[idx],
        Grp::Grp8 => GROUP8[idx],
    }
}

/// How many clocks the effective-address calculation costs.
///
/// Straight from the manual's EA table. The 8086 computes the address in
/// microcode, so the cost depends on how many terms are summed, and a segment
/// override adds two clocks because it is one more microcode step.
///
/// `md` and `rm` are the ModRM fields; `override_seg` says whether a
/// segment-override prefix was present.
///
/// Source: iAPX 86/88 User's Manual, "EA calculation time".
#[must_use]
pub const fn ea_clocks(md: u8, rm: u8, override_seg: bool) -> u32 {
    let base = match (md, rm) {
        // A register operand has no address to compute.
        (3, _) => return 0,
        // The disp16 special case: no register term at all.
        (0, 6) => 6,
        // Base or index alone.
        (0, 4 | 5 | 7) => 5,
        // Base plus index: BP+DI and BX+SI are one clock cheaper than BX+DI
        // and BP+SI, an artefact of the adder's operand order.
        (0, 0 | 3) => 7,
        (0, 1 | 2) => 8,
        // Displacement plus one register.
        (1 | 2, 4..=7) => 9,
        // Displacement plus base plus index.
        (1 | 2, 0 | 3) => 11,
        (1 | 2, 1 | 2) => 12,
        _ => 5,
    };
    if override_seg { base + 2 } else { base }
}

// ---------------------------------------------------------------------------
// The instruction-stream decoder
// ---------------------------------------------------------------------------

/// Segment register numbers, as the ModRM `reg` field and bits 3-4 of a
/// `PUSH sr` opcode encode them.
pub mod seg {
    /// Extra segment.
    pub const ES: u8 = 0;
    /// Code segment.
    pub const CS: u8 = 1;
    /// Stack segment.
    pub const SS: u8 = 2;
    /// Data segment.
    pub const DS: u8 = 3;
    /// The 80386's first extra segment, with no implicit use at all.
    pub const FS: u8 = 4;
    /// The 80386's second extra segment.
    pub const GS: u8 = 5;

    /// How many segment registers the architecture has. Four on an 8086, six
    /// from the 386 on.
    pub const COUNT: usize = 6;

    /// The register's name, lower case.
    #[must_use]
    pub const fn name(sr: u8) -> &'static str {
        match sr {
            ES => "es",
            CS => "cs",
            SS => "ss",
            DS => "ds",
            FS => "fs",
            _ => "gs",
        }
    }
}

/// A decoded SIB (scale-index-base) byte.
///
/// The 80386 added it to make the addressing modes orthogonal: any register
/// as a base, any register but `ESP` as an index, and a shift of 0-3 on the
/// index. It appears only with a 32-bit address size and only when the ModRM
/// `rm` field is 4, which is exactly the encoding the 16-bit modes spent on
/// `[SI]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Sib {
    /// The index's left shift, 0 to 3.
    pub scale: u8,
    /// The index register, or 4 for "no index" — `ESP` cannot be an index,
    /// and that is the encoding spent on saying so.
    pub index: u8,
    /// The base register. A base of 5 with a ModRM mode of 0 means "no base,
    /// a 32-bit displacement instead".
    pub base: u8,
}

impl Sib {
    /// Split a raw SIB byte.
    #[inline]
    #[must_use]
    pub const fn new(byte: u8) -> Sib {
        Sib {
            scale: byte >> 6,
            index: (byte >> 3) & 7,
            base: byte & 7,
        }
    }

    /// Whether this SIB contributes an index term at all.
    #[inline]
    #[must_use]
    pub const fn has_index(self) -> bool {
        self.index != 4
    }
}

/// A decoded ModRM byte.
///
/// The 8086 has no SIB byte and no 32-bit addressing: `rm` selects one of
/// eight fixed register combinations, and `md` says how much displacement
/// follows. With a 32-bit address size the same three fields mean something
/// else, which is why the two interpretations are separate methods rather
/// than one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ModRm {
    /// The mode field, bits 6-7. `3` means the operand is a register.
    pub md: u8,
    /// The middle field, bits 3-5: a register number or an opcode extension.
    pub reg: u8,
    /// The r/m field, bits 0-2.
    pub rm: u8,
}

impl ModRm {
    /// Split a raw ModRM byte.
    #[inline]
    #[must_use]
    pub const fn new(byte: u8) -> ModRm {
        ModRm {
            md: byte >> 6,
            reg: (byte >> 3) & 7,
            rm: byte & 7,
        }
    }

    /// Whether the r/m operand is a register rather than a memory address.
    #[inline]
    #[must_use]
    pub const fn is_register(self) -> bool {
        self.md == 3
    }

    /// How many displacement bytes follow, with a **16-bit** address size.
    ///
    /// The `md == 0, rm == 6` case is the special one: there is no `[BP]` with
    /// no displacement, because that encoding was spent on the direct 16-bit
    /// address. `[BP]` is therefore always assembled as `[BP+0]` with an
    /// 8-bit displacement.
    #[must_use]
    pub const fn disp_bytes(self) -> u8 {
        match (self.md, self.rm) {
            (0, 6) => 2,
            (1, _) => 1,
            (2, _) => 2,
            _ => 0,
        }
    }

    /// The segment the address defaults to with a **16-bit** address size,
    /// before any override.
    ///
    /// Stack segment whenever `BP` is one of the address terms, data segment
    /// otherwise — the rule that makes `[BP]` reach a local variable and
    /// `[BX]` reach a global one. The direct-address encoding (`md == 0,
    /// rm == 6`) has no `BP` term despite sharing its `rm`, so it is `DS`.
    #[must_use]
    pub const fn default_segment(self) -> u8 {
        match (self.md, self.rm) {
            (0, 6) => seg::DS,
            (_, 2 | 3 | 6) => seg::SS,
            _ => seg::DS,
        }
    }

    /// How many displacement bytes follow, with a **32-bit** address size.
    ///
    /// `md == 0, rm == 5` is the direct 32-bit address, the mirror of the
    /// 16-bit `rm == 6` case; and a SIB whose base is 5 with `md == 0` says
    /// the same thing about the base register.
    #[must_use]
    pub const fn disp_bytes32(self, sib: Option<Sib>) -> u8 {
        match self.md {
            1 => 1,
            2 => 4,
            0 => match (self.rm, sib) {
                (5, _) => 4,
                (4, Some(s)) if s.base == 5 => 4,
                _ => 0,
            },
            _ => 0,
        }
    }

    /// The segment the address defaults to with a **32-bit** address size.
    ///
    /// The rule moved with the addressing modes: it is the *base* register
    /// that decides, and only `ESP` and `EBP` select `SS`. An index of `EBP`
    /// does not — `[eax+ebp*4]` is in `DS`, which surprises people who learnt
    /// the 16-bit rule.
    #[must_use]
    pub const fn default_segment32(self, sib: Option<Sib>) -> u8 {
        match (self.md, self.rm, sib) {
            // No base register at all: the two direct-address encodings.
            (0, 5, _) => seg::DS,
            (0, 4, Some(s)) if s.base == 5 => seg::DS,
            (_, 4, Some(s)) => {
                if s.base == 4 || s.base == 5 {
                    seg::SS
                } else {
                    seg::DS
                }
            }
            (_, 5, _) => seg::SS,
            _ => seg::DS,
        }
    }
}

/// Which repeat prefix an instruction carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rep {
    /// `F3`: repeat while `CX != 0`, and for `CMPS`/`SCAS` while equal.
    While,
    /// `F2`: repeat while `CX != 0` and, for `CMPS`/`SCAS`, while not equal.
    WhileNot,
}

/// Everything one pass of the decoder extracted from the instruction stream.
///
/// Produced by [`decode_stream`] and consumed by both the interpreter and the
/// disassembler, so the two cannot disagree about where an instruction ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fields {
    /// Which opcode map this was decoded from.
    pub map: Gen,
    /// The width the code segment was being decoded at.
    pub bits: Bits,
    /// The `REX` prefix byte, or zero where there was none.
    ///
    /// Zero rather than `Option` because the byte itself is never `0`: a `REX`
    /// is `4x`, so "no prefix" and "a prefix that sets nothing" stay
    /// distinguishable — and they *must*, because `40` alone still renames
    /// `AH` to `SPL` (*Intel SDM* volume 2 §2.2.1.2).
    pub rex: u8,
    /// The segment register a prefix selected, if any.
    pub seg_override: Option<u8>,
    /// The repeat prefix, if any.
    pub rep: Option<Rep>,
    /// Whether a `LOCK` prefix was seen.
    pub lock: bool,
    /// Whether the opcode came from the two-byte (`0F`) map.
    pub two_byte: bool,
    /// The effective operand size in bytes: 2, 4 or 8.
    ///
    /// The code segment's width picks the default, a `66` prefix flips it and
    /// `REX.W` overrides both, so this is already the answer rather than the
    /// prefix.
    pub opsize: u8,
    /// The effective address size in bytes: 2, 4 or 8. `67` flips it.
    pub addrsize: u8,
    /// The opcode byte, after all prefixes and after the `0F` escape.
    pub opcode: u8,
    /// The table row, already passed through [`resolve_as`].
    pub insn: Insn,
    /// The ModRM byte, where the encoding has one.
    pub modrm: Option<ModRm>,
    /// The SIB byte, where a 32-bit address form has one.
    pub sib: Option<Sib>,
    /// The displacement, sign-extended.
    pub disp: i32,
    /// The segment a memory operand defaults to, before any override — the
    /// address size and the SIB base both feed into it, so it is decided here
    /// rather than at every point of use.
    pub mem_seg: u8,
    /// Whether the memory operand is `RIP`-relative.
    ///
    /// In 64-bit mode `mod == 00` with `r/m == 101` stopped meaning "the
    /// displacement is the whole address" and started meaning "the
    /// displacement is added to the address of the *next* instruction"
    /// (*Intel SDM* volume 2 §2.2.1.6). It is the one addressing mode whose
    /// effective address depends on the instruction's own length, which is why
    /// it is a flag here rather than a term the address calculation could
    /// simply include.
    pub rip_relative: bool,
    /// The first immediate the encoding carries, or a far pointer's offset.
    pub imm: u64,
    /// The second immediate — `ENTER`'s nesting level, or a far pointer's
    /// segment. Zero when the encoding has only one.
    pub imm2: u64,
    /// Total length in bytes, prefixes included.
    pub len: u8,
    /// Whether the stream ran out before the instruction was complete.
    pub truncated: bool,
}

impl Fields {
    /// The immediate as a 16-bit value, sign-extended where the encoding says
    /// so.
    #[inline]
    #[must_use]
    pub const fn imm16(&self) -> u16 {
        self.imm as u16
    }

    /// The immediate masked to the effective operand size.
    #[inline]
    #[must_use]
    pub const fn imm_sized(&self) -> u64 {
        match self.opsize {
            2 => self.imm & 0xffff,
            4 => self.imm & 0xffff_ffff,
            _ => self.imm,
        }
    }

    /// Whether a `REX` prefix was present.
    #[inline]
    #[must_use]
    pub const fn has_rex(&self) -> bool {
        self.rex != 0
    }

    /// The `REX.W` bit: widen the operand to sixty-four bits.
    #[inline]
    #[must_use]
    pub const fn rex_w(&self) -> bool {
        self.rex & 0x8 != 0
    }

    /// The ModRM `reg` field, extended by `REX.R`.
    #[inline]
    #[must_use]
    pub const fn reg_num(&self) -> u8 {
        let base = match self.modrm {
            Some(m) => m.reg,
            None => 0,
        };
        base | ((self.rex & 0x4) << 1)
    }

    /// The ModRM `r/m` field as a *register* number, extended by `REX.B`.
    ///
    /// Only meaningful when the mode field selects a register; as a memory
    /// operand the same three bits mean a base register instead, which
    /// [`base_num`](Fields::base_num) answers.
    #[inline]
    #[must_use]
    pub const fn rm_num(&self) -> u8 {
        let base = match self.modrm {
            Some(m) => m.rm,
            None => 0,
        };
        base | ((self.rex & 0x1) << 3)
    }

    /// The SIB base register number, extended by `REX.B`.
    #[inline]
    #[must_use]
    pub const fn base_num(&self) -> u8 {
        let base = match self.sib {
            Some(s) => s.base,
            None => 0,
        };
        base | ((self.rex & 0x1) << 3)
    }

    /// The SIB index register number, extended by `REX.X`.
    ///
    /// Note that the extension applies *before* the "index 4 means no index"
    /// rule is consulted, so `R12` is a usable index while `RSP` is not —
    /// which is exactly what `REX.X` bought (*Intel SDM* volume 2 table 2-6).
    #[inline]
    #[must_use]
    pub const fn index_num(&self) -> u8 {
        let base = match self.sib {
            Some(s) => s.index,
            None => 0,
        };
        base | ((self.rex & 0x2) << 2)
    }

    /// The register the opcode's low three bits name, extended by `REX.B`.
    #[inline]
    #[must_use]
    pub const fn opcode_reg(&self) -> u8 {
        (self.opcode & 7) | ((self.rex & 0x1) << 3)
    }

    /// Whether the SIB byte contributes a scaled index.
    ///
    /// An index field of `100` means *no index* — but only without `REX.X`,
    /// which turns the same encoding into `R12`. `RSP` still cannot be an
    /// index and `R12` can, and that asymmetry is the whole content of this
    /// function.
    #[inline]
    #[must_use]
    pub const fn has_index(&self) -> bool {
        match self.sib {
            Some(s) => s.index != 4 || self.rex & 0x2 != 0,
            None => false,
        }
    }

    /// The segment half of a far-pointer immediate (`Ap`).
    #[inline]
    #[must_use]
    pub const fn imm_seg(&self) -> u16 {
        self.imm2 as u16
    }

    /// The segment this instruction's memory operand uses.
    #[inline]
    #[must_use]
    pub const fn segment(&self, default: u8) -> u8 {
        match self.seg_override {
            Some(sr) => sr,
            None => default,
        }
    }

    /// The segment this instruction's ModRM memory operand actually uses,
    /// override included.
    #[inline]
    #[must_use]
    pub const fn mem_segment(&self) -> u8 {
        self.segment(self.mem_seg)
    }

    /// Whether the r/m operand names a register rather than memory.
    ///
    /// True for the control-, debug- and test-register moves whatever their
    /// mode field holds: a 386 ignores it there.
    #[inline]
    #[must_use]
    pub const fn rm_is_register(&self) -> bool {
        if matches!(self.insn.dst, Arg::Rd) || matches!(self.insn.src, Arg::Rd) {
            return true;
        }
        match self.modrm {
            Some(m) => m.is_register(),
            None => false,
        }
    }
}

/// How many prefix bytes are decoded before the decoder gives up.
///
/// The 8086 itself has no limit — a prefix restarts the fetch loop, so a page
/// of `F3` bytes is a legal, very slow instruction. A decoder needs a bound
/// anyway, and this one is far past anything an assembler emits while staying
/// well inside the length any real program uses.
pub const MAX_PREFIXES: u8 = 15;

/// Decode one instruction from a byte stream.
///
/// `next` yields the instruction bytes in order and returns `None` at the end
/// of what is readable, which sets [`Fields::truncated`] — a monitor
/// disassembling to the end of a buffer gets a best-effort answer rather than
/// a panic, and the interpreter never sees `None` because its stream is guest
/// memory.
///
/// Prefixes accumulate rather than replace: the last segment override wins,
/// which is what the 8086 does because each prefix simply latches into the
/// same field.
pub fn decode_stream(next: &mut dyn FnMut() -> Option<u8>) -> Fields {
    decode_stream_as(Gen::I8086, Bits::B16, next)
}

/// Decode one instruction from a byte stream on the named generation and
/// code-segment width.
///
/// `bits` selects the *default* operand and address sizes, which the `66`,
/// `67` and `REX` prefixes then modify. On an 8086 it is meaningless and
/// ignored — there is one size and no prefix to change it.
///
/// See [`decode_stream`] for what `next` and [`Fields::truncated`] mean.
#[allow(clippy::too_many_lines)]
pub fn decode_stream_as(map: Gen, bits: Bits, next: &mut dyn FnMut() -> Option<u8>) -> Fields {
    // An 8086 has one width whatever it is asked for: no `D` bit, no `66`, no
    // `67`, and certainly no `REX`.
    let bits = match map {
        Gen::I8086 => Bits::B16,
        Gen::I386 => bits,
    };
    let mut f = Fields {
        map,
        bits,
        rex: 0,
        seg_override: None,
        rep: None,
        lock: false,
        two_byte: false,
        opsize: bits.operand(),
        addrsize: bits.address(),
        opcode: 0x90,
        insn: decode_as(map, 0x90),
        modrm: None,
        sib: None,
        disp: 0,
        mem_seg: seg::DS,
        rip_relative: false,
        imm: 0,
        imm2: 0,
        len: 0,
        truncated: false,
    };

    let mut take = |f: &mut Fields| -> u8 {
        match next() {
            Some(b) => {
                f.len = f.len.saturating_add(1);
                b
            }
            None => {
                f.truncated = true;
                0
            }
        }
    };

    // Prefixes. They accumulate rather than replace: each one latches into
    // its own field, so the last segment override wins and `66 66` is the
    // same as one `66` — a toggle would be wrong, because the prefix selects
    // "the non-default size" rather than inverting a bit.
    let mut prefixes = 0u8;
    let opcode = loop {
        let byte = take(&mut f);
        if f.truncated {
            return f;
        }
        let mut row = decode_as(map, byte);
        if bits.is_64() {
            row = row.in_long();
        }
        if !row.is_prefix() || prefixes >= MAX_PREFIXES {
            break byte;
        }
        prefixes += 1;
        match row.op {
            // The prefix byte itself names the register: `(byte >> 3) & 7`
            // works for `26`/`2E`/`36`/`3E` but collides for `64` and `65`,
            // whose bit 3 is part of the opcode rather than the field.
            Op::SEG => match byte {
                // In 64-bit mode `CS`, `DS`, `ES` and `SS` overrides are
                // *decoded and ignored* — those four segments have no base
                // there — while `FS` and `GS` keep working, which is why
                // thread-local storage moved to them. Recording the override
                // and letting the address path decide keeps the disassembler
                // able to print what the byte said.
                0x26 => f.seg_override = Some(seg::ES),
                0x2e => f.seg_override = Some(seg::CS),
                0x36 => f.seg_override = Some(seg::SS),
                0x3e => f.seg_override = Some(seg::DS),
                0x64 => f.seg_override = Some(seg::FS),
                0x65 => f.seg_override = Some(seg::GS),
                0x66 => f.opsize = bits.operand_alt(),
                _ => f.addrsize = bits.address_alt(),
            },
            Op::LOCK => f.lock = true,
            Op::REP => f.rep = Some(Rep::While),
            Op::REPNE => f.rep = Some(Rep::WhileNot),
            // `REX` must be the **last** prefix before the opcode: one
            // followed by another prefix is ignored entirely (*Intel SDM*
            // volume 2 §2.2.1). Clearing it here rather than checking
            // afterwards is what implements that — a later `REX` overwrites,
            // and a later non-`REX` prefix wipes.
            Op::REX => {
                f.rex = byte;
                continue;
            }
            _ => {}
        }
        f.rex = 0;
    };

    // `0F` is `POP CS` on an 8086 and the escape to the second map from the
    // 80286 on, which is why the generation has to be known before the byte
    // can be looked up at all.
    let mut insn;
    if matches!(map, Gen::I386) && opcode == 0x0f {
        f.two_byte = true;
        let second = take(&mut f);
        if f.truncated {
            return f;
        }
        f.opcode = second;
        insn = decode_0f(second);
    } else {
        f.opcode = opcode;
        insn = decode_as(map, opcode);
    }
    if bits.is_64() {
        insn = insn.in_long();
    }

    if insn.needs_modrm() {
        let byte = take(&mut f);
        let modrm = ModRm::new(byte);
        f.modrm = Some(modrm);
        insn = resolve_as(map, insn, modrm.reg);
        if bits.is_64() {
            insn = insn.in_long();
        }
        // The control-, debug- and test-register moves ignore the mode field
        // entirely: there is no memory form, so there is no displacement
        // however the two top bits are encoded.
        let register_only = matches!(insn.dst, Arg::Rd | Arg::Cd | Arg::Dd | Arg::Td)
            || matches!(insn.src, Arg::Rd);
        if register_only {
            f.mem_seg = seg::DS;
        } else if f.addrsize == 2 {
            f.mem_seg = modrm.default_segment();
            match modrm.disp_bytes() {
                1 => {
                    // An 8-bit displacement is signed, and it is added in the
                    // address size's arithmetic, so it is sign-extended here
                    // rather than at every point of use.
                    let lo = take(&mut f);
                    f.disp = i32::from(lo as i8);
                }
                2 => {
                    let lo = take(&mut f);
                    let hi = take(&mut f);
                    let value = u16::from(lo) | (u16::from(hi) << 8);
                    // Sign-extended as a 16-bit quantity: the displacement is
                    // added modulo 65536, so `0xfffe` really is minus two.
                    f.disp = i32::from(value as i16);
                }
                _ => {}
            }
        } else {
            if !modrm.is_register() && modrm.rm == 4 {
                let byte = take(&mut f);
                f.sib = Some(Sib::new(byte));
            }
            f.mem_seg = modrm.default_segment32(f.sib);
            // The one addressing mode long mode replaced rather than extended:
            // `mod == 00`, `r/m == 101` was an absolute `disp32` and is now
            // `RIP + disp32`. The `REX.B` bit does *not* enter into it — this
            // is a property of the three-bit field, not of the register it
            // would otherwise name.
            f.rip_relative = bits.is_64() && !modrm.is_register() && modrm.md == 0 && modrm.rm == 5;
            match modrm.disp_bytes32(f.sib) {
                1 => {
                    let lo = take(&mut f);
                    f.disp = i32::from(lo as i8);
                }
                4 => {
                    let mut value = 0u32;
                    for i in 0..4 {
                        let byte = take(&mut f);
                        value |= u32::from(byte) << (8 * i);
                    }
                    f.disp = value as i32;
                }
                _ => {}
            }
        }
    } else if matches!(insn.dst, Arg::Ob | Arg::Ov) || matches!(insn.src, Arg::Ob | Arg::Ov) {
        // The direct-offset moves have no ModRM byte; their address is the
        // immediate, and it is always in the data segment unless overridden.
        f.mem_seg = seg::DS;
    }
    f.insn = insn;

    // The operand size is settled here rather than beside the prefixes,
    // because the group tables can change which operation this is — `FF /6` is
    // a `PUSH`, and `PUSH` is one of the operations whose operand defaults to
    // sixty-four bits — and that is not known until the ModRM byte has been
    // read and resolved.
    if bits.is_64() {
        if f.rex_w() {
            // `REX.W` beats a `66` that came before it.
            f.opsize = 8;
        } else if insn.op.default_64() && f.opsize == 4 {
            f.opsize = 8;
        }
    }

    let osz = f.opsize;
    let asz = f.addrsize;
    let mut read = |f: &mut Fields, n: u8| -> u64 {
        let mut value = 0u64;
        for i in 0..n {
            let byte = take(f);
            value |= u64::from(byte) << (8 * u32::from(i));
        }
        value
    };
    let mut slot = 0u8;
    for arg in [insn.dst, insn.src, insn.aux] {
        if arg == Arg::Ap {
            // A far-pointer immediate is an offset at the operand size
            // followed by a two-byte selector, not one number.
            let offset = read(&mut f, osz);
            let selector = read(&mut f, 2);
            f.imm = offset;
            f.imm2 = selector;
            slot = 2;
            continue;
        }
        let n = arg.immediate_bytes(osz, asz);
        if n == 0 {
            continue;
        }
        let mut value = read(&mut f, n);
        // `83 /n`, `6A`, and every short relative branch sign-extend their
        // byte before use; doing it here keeps every consumer from repeating
        // the cast, and makes a backward jump come out as an addition.
        if matches!(arg, Arg::Ibs | Arg::Jb) {
            value = ((value as u8 as i8) as i64) as u64;
        } else if matches!(arg, Arg::Iz | Arg::Jv) {
            // `Iz` and `Jz` are sign-extended to the operand size, which is
            // only visible at an operand size wider than the immediate — that
            // is, in 64-bit mode. `and rax, -1` is `48 25 ff ff ff ff`, and a
            // zero-extending decoder makes it `and rax, 0xffffffff`.
            value = match (n, osz) {
                (2, _) => u64::from(value as u16),
                (4, 8) => ((value as u32 as i32) as i64) as u64,
                (4, _) => u64::from(value as u32),
                _ => value,
            };
        }
        if slot == 0 {
            f.imm = value;
            slot = 1;
        } else {
            f.imm2 = value;
            slot = 2;
        }
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    /// Decode from a slice, the way the disassembler does.
    fn decode_bytes(bytes: &[u8]) -> Fields {
        let mut at = 0usize;
        decode_stream(&mut || {
            let b = bytes.get(at).copied();
            at += 1;
            b
        })
    }

    #[test]
    fn every_opcode_is_described_exactly_once() {
        // The table initialiser fills with NOP; an omission would decode as
        // something harmless-looking rather than fail, which is the worst kind
        // of silence.
        let missing: Vec<usize> = (0..256).filter(|i| !LISTED[*i]).collect();
        assert!(missing.is_empty(), "opcodes not described: {missing:02x?}");
    }

    #[test]
    fn mnemonics_are_all_known() {
        // `lower` panics at compile time on an unknown name, so reaching this
        // point already proves the mapping is total; what is checked here is
        // that it is also correct and that nothing is empty.
        assert_eq!(Op::MOV.mnemonic(), "mov");
        assert_eq!(Op::CALLF.mnemonic(), "callf");
        assert_eq!(Op::SETMO.mnemonic(), "setmo");
        for op in Op::ALL {
            assert!(!op.mnemonic().is_empty(), "{op:?} has no mnemonic");
            assert!(!op.summary().is_empty(), "{op:?} has no summary");
        }
    }

    #[test]
    fn every_declared_operation_is_reachable() {
        // An operation nothing decodes to is dead code pretending to be a
        // feature. The group tables count as reachable, and so does the
        // 64-bit column — `MOVSXD`, `REX`, `SYSCALL` and `SYSRET` exist
        // *only* there, which is exactly what makes checking it worth doing.
        let reachable = |op: Op| {
            TABLE.iter().any(|i| i.op == op || i.in_long().op == op)
                || TABLE_386.iter().any(|i| i.op == op || i.in_long().op == op)
                || TABLE_0F.iter().any(|i| i.op == op || i.in_long().op == op)
                || GROUP_ALU.contains(&op)
                || GROUP_SHIFT.contains(&op)
                || GROUP_UNARY8.iter().any(|i| i.op == op)
                || GROUP_MISC.iter().any(|i| i.op == op)
                || GROUP6.iter().any(|i| i.op == op)
                || GROUP7.iter().any(|i| i.op == op)
                || GROUP8.iter().any(|i| i.op == op)
        };
        for op in Op::ALL {
            // `SWAPGS` is the one operation with no row of its own, and it is
            // not an oversight: `0F 01 F8` differs from `INVLPG` only in the
            // ModRM byte's *mode* field, which the table describes operands
            // with rather than opcodes. The interpreter picks it out there,
            // and this exception is written down so that adding a second one
            // has to be argued for rather than noticed.
            if *op == Op::SWAPGS {
                continue;
            }
            assert!(reachable(*op), "{op:?} is declared but unreachable");
        }
    }

    #[test]
    fn groups_resolve_to_the_right_operation() {
        assert_eq!(resolve(decode(0x80), 5).op, Op::SUB);
        assert_eq!(resolve(decode(0x81), 7).op, Op::CMP);
        assert_eq!(resolve(decode(0xd1), 4).op, Op::SHL);
        assert_eq!(resolve(decode(0xd3), 6).op, Op::SETMO);
        assert_eq!(resolve(decode(0xf6), 6).op, Op::DIV);
        assert_eq!(resolve(decode(0xf7), 5).op, Op::IMUL);
        assert_eq!(resolve(decode(0xff), 3).op, Op::CALLF);
        // POP and MOV-immediate ignore the extension entirely.
        assert_eq!(resolve(decode(0x8f), 5).op, Op::POP);
        assert_eq!(resolve(decode(0xc7), 3).op, Op::MOV);
    }

    #[test]
    fn the_unary_group_keeps_its_operand_width() {
        assert_eq!(resolve(decode(0xf6), 0).src, Arg::Ib);
        assert_eq!(resolve(decode(0xf7), 0).src, Arg::Iz);
        assert_eq!(resolve(decode(0xf6), 4).dst, Arg::Eb);
        assert_eq!(resolve(decode(0xf7), 4).dst, Arg::Ev);
    }

    #[test]
    fn prefixes_are_marked_as_such() {
        for op in [0x26, 0x2e, 0x36, 0x3e, 0xf0, 0xf1, 0xf2, 0xf3] {
            assert!(decode(op).is_prefix(), "{op:02x} should be a prefix");
        }
        assert!(!decode(0x90).is_prefix());
    }

    #[test]
    fn modrm_is_required_exactly_where_the_encoding_needs_one() {
        assert!(decode(0x00).needs_modrm()); // ADD Eb,Gb
        assert!(decode(0x8d).needs_modrm()); // LEA Gv,M
        assert!(decode(0xff).needs_modrm()); // group
        assert!(!decode(0x40).needs_modrm()); // INC AX
        assert!(!decode(0xb8).needs_modrm()); // MOV AX,imm16
        assert!(!decode(0xa0).needs_modrm()); // MOV AL,moffs8
    }

    #[test]
    fn operand_widths_follow_the_encoding() {
        assert_eq!(decode(0x00).width_bytes(2), Some(1));
        assert_eq!(decode(0x01).width_bytes(2), Some(2));
        assert_eq!(decode(0xa4).width_bytes(2), Some(1)); // movsb
        assert_eq!(decode(0xa5).width_bytes(2), Some(2)); // movsw
        assert_eq!(decode(0x83).width_bytes(2), Some(2)); // Ev,Ibs
        assert_eq!(decode(0xf4).width_bytes(2), None); // hlt
        // The same rows with a 32-bit operand size.
        assert_eq!(decode(0x01).width_bytes(4), Some(4));
        assert_eq!(decode(0xa5).width_bytes(4), Some(4));
        assert_eq!(decode(0x00).width_bytes(4), Some(1));
    }

    #[test]
    fn the_alias_encodings_are_the_ones_the_hardware_corpus_names() {
        let aliases: Vec<u8> = (0..=255u8)
            .filter(|o| decode(*o).class == Class::Alias)
            .collect();
        let mut want: Vec<u8> = (0x60..=0x6f).collect();
        want.extend([0x82, 0xc0, 0xc1, 0xc8, 0xc9]);
        want.sort_unstable();
        assert_eq!(aliases, want);
    }

    #[test]
    fn ea_clocks_match_the_manual() {
        assert_eq!(ea_clocks(0, 6, false), 6); // disp16
        assert_eq!(ea_clocks(0, 4, false), 5); // [SI]
        assert_eq!(ea_clocks(0, 0, false), 7); // [BX+SI]
        assert_eq!(ea_clocks(0, 1, false), 8); // [BX+DI]
        assert_eq!(ea_clocks(1, 7, false), 9); // [BX+disp8]
        assert_eq!(ea_clocks(2, 3, false), 11); // [BP+DI+disp16]
        assert_eq!(ea_clocks(2, 2, false), 12); // [BP+SI+disp16]
        assert_eq!(ea_clocks(3, 0, false), 0); // register
        assert_eq!(ea_clocks(0, 4, true), 7); // override costs two more
    }

    #[test]
    fn the_decoder_measures_instruction_length() {
        assert_eq!(decode_bytes(&[0x90]).len, 1); // nop
        assert_eq!(decode_bytes(&[0xb8, 0x34, 0x12]).len, 3); // mov ax,1234h
        assert_eq!(decode_bytes(&[0x00, 0x00]).len, 2); // add [bx+si],al
        assert_eq!(decode_bytes(&[0x00, 0x06, 0x34, 0x12]).len, 4); // add [1234h],al
        assert_eq!(decode_bytes(&[0x83, 0x46, 0x10, 0x05]).len, 4); // add [bp+10h],5
        assert_eq!(decode_bytes(&[0xea, 0x00, 0x10, 0x00, 0xf0]).len, 5); // jmpf
        // One segment override, a ModRM with a word displacement, and a word
        // immediate: the longest ordinary 8086 encoding.
        assert_eq!(
            decode_bytes(&[0x26, 0x81, 0x86, 0x34, 0x12, 0x78, 0x56]).len,
            7
        );
    }

    #[test]
    fn prefixes_accumulate_and_the_last_override_wins() {
        let f = decode_bytes(&[0xf3, 0x26, 0x2e, 0xf0, 0xa4]);
        assert_eq!(f.rep, Some(Rep::While));
        assert_eq!(f.seg_override, Some(seg::CS));
        assert!(f.lock);
        assert_eq!(f.insn.op, Op::MOVSB);
        assert_eq!(f.len, 5);
    }

    #[test]
    fn a_truncated_stream_is_reported_rather_than_guessed() {
        let f = decode_bytes(&[0xb8, 0x34]);
        assert!(f.truncated);
        assert_eq!(f.insn.op, Op::MOV);
    }

    #[test]
    fn displacements_are_sign_extended_at_decode_time() {
        // add [bp-1], al — the 8-bit displacement is signed.
        let f = decode_bytes(&[0x00, 0x46, 0xff]);
        assert_eq!(f.disp, -1);
        // 83 /0 ib likewise sign-extends its immediate to a word.
        let f = decode_bytes(&[0x83, 0xc0, 0xfe]);
        assert_eq!(f.imm16(), 0xfffe);
    }

    #[test]
    fn the_direct_address_encoding_is_not_bp_relative() {
        // md=0 rm=6 is a 16-bit direct address in DS, not [BP].
        let rm = ModRm::new(0b00_000_110);
        assert_eq!(rm.disp_bytes(), 2);
        assert_eq!(rm.default_segment(), seg::DS);
        // Every other BP-based form defaults to SS.
        assert_eq!(ModRm::new(0b01_000_110).default_segment(), seg::SS);
        assert_eq!(ModRm::new(0b00_000_010).default_segment(), seg::SS);
        assert_eq!(ModRm::new(0b00_000_011).default_segment(), seg::SS);
        assert_eq!(ModRm::new(0b00_000_000).default_segment(), seg::DS);
    }

    #[test]
    fn a_far_pointer_immediate_carries_both_halves() {
        let f = decode_bytes(&[0x9a, 0x00, 0x10, 0x00, 0xf0]);
        assert_eq!(f.insn.op, Op::CALLF);
        assert_eq!(f.imm16(), 0x1000);
        assert_eq!(f.imm_seg(), 0xf000);
    }
}
