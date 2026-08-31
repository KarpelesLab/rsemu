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

/// How an operand is encoded, in the notation of Intel's opcode maps.
///
/// The letter is the addressing method and the suffix is the operand size:
/// `b` is a byte, `v` is a word (16 bits — this core has no 32-bit operand
/// size). Keeping the manual's vocabulary means a row can be checked against
/// the manual without translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Arg {
    /// No operand in this position.
    None,
    /// ModRM r/m, byte.
    Eb,
    /// ModRM r/m, word.
    Ev,
    /// ModRM `reg`, byte register.
    Gb,
    /// ModRM `reg`, word register.
    Gv,
    /// ModRM `reg`, segment register (only the low two bits are decoded — the
    /// 8086 does not check the third, which is why `8C`/`8E` accept a `reg`
    /// of 4-7 and alias down).
    Sw,
    /// ModRM r/m as an *address*, never a register: `LEA`.
    M,
    /// ModRM r/m as a far pointer in memory, `offset:segment`: `LES`, `LDS`,
    /// and the indirect far `CALL`/`JMP`.
    Mp,
    /// Immediate byte.
    Ib,
    /// Immediate word.
    Iv,
    /// Immediate byte, sign-extended to a word (`83 /n`).
    Ibs,
    /// Byte displacement relative to the end of the instruction.
    Jb,
    /// Word displacement relative to the end of the instruction.
    Jv,
    /// Immediate far pointer, `offset:segment`, four bytes.
    Ap,
    /// Direct memory offset, byte (`A0`/`A2`) — a 16-bit offset with no ModRM.
    Ob,
    /// Direct memory offset, word (`A1`/`A3`).
    Ov,
    /// Byte register selected by the low three bits of the opcode.
    Rb,
    /// Word register selected by the low three bits of the opcode.
    Rv,
    /// Segment register selected by bits 3-4 of the opcode (`PUSH ES`, and the
    /// segment-override prefixes).
    Sr,
    /// The literal 1, as the count of a single-bit shift.
    One,
    /// `CL`, as a shift count.
    Cl,
    /// `DX`, as an I/O port number.
    Dx,
    /// `AL`.
    Al,
    /// `AX`.
    Ax,
    /// String source `DS:SI`, byte.
    Xb,
    /// String source `DS:SI`, word.
    Xv,
    /// String destination `ES:DI`, byte.
    Yb,
    /// String destination `ES:DI`, word.
    Yv,
}

impl Arg {
    /// Whether this operand is one byte wide.
    ///
    /// `None` is neither, so the answer is an [`Option`]: an instruction whose
    /// operands are all `None` (`CBW`, `HLT`) has no operand width and asking
    /// for one is a decoder bug rather than a default.
    #[must_use]
    pub const fn width_bytes(self) -> Option<u8> {
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
            Arg::Ev
            | Arg::Gv
            | Arg::Sw
            | Arg::Iv
            | Arg::Ibs
            | Arg::Ov
            | Arg::Rv
            | Arg::Sr
            | Arg::Ax
            | Arg::Xv
            | Arg::Yv
            | Arg::Jv
            | Arg::Dx => Some(2),
            Arg::None | Arg::M | Arg::Mp | Arg::Ap => None,
        }
    }

    /// Whether decoding this operand requires a ModRM byte.
    #[must_use]
    pub const fn needs_modrm(self) -> bool {
        matches!(
            self,
            Arg::Eb | Arg::Ev | Arg::Gb | Arg::Gv | Arg::Sw | Arg::M | Arg::Mp
        )
    }

    /// How many immediate bytes follow the displacement for this operand.
    #[must_use]
    pub const fn immediate_bytes(self) -> u8 {
        match self {
            Arg::Ib | Arg::Ibs | Arg::Jb => 1,
            Arg::Iv | Arg::Jv | Arg::Ob | Arg::Ov => 2,
            Arg::Ap => 4,
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
}

impl Op {
    /// The assembler mnemonic, lower case.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        MNEMONICS[self as usize]
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

    /// Whether this operation is one of the five string primitives, and so may
    /// carry a `REP` prefix.
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
        )
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
    /// Which opcode-extension group this belongs to.
    pub group: Grp,
    /// How well documented the encoding is.
    pub class: Class,
}

impl Insn {
    const fn new(op: Op, dst: Arg, src: Arg, group: Grp, class: Class) -> Insn {
        Insn {
            op,
            dst,
            src,
            group,
            class,
        }
    }

    /// Whether decoding needs a ModRM byte.
    #[must_use]
    pub const fn needs_modrm(self) -> bool {
        !matches!(self.group, Grp::None) || self.dst.needs_modrm() || self.src.needs_modrm()
    }

    /// The operand width in bytes, where the encoding fixes one.
    ///
    /// Taken from the destination first and the source second, because
    /// `MOV Ev,Ibs`-shaped rows carry the width on the destination while
    /// `PUSH Ib`-shaped ones carry it on the source.
    #[must_use]
    pub const fn width_bytes(self) -> Option<u8> {
        match self.dst.width_bytes() {
            Some(w) => Some(w),
            None => self.src.width_bytes(),
        }
    }

    /// Whether this encoding is a prefix rather than an instruction.
    #[must_use]
    pub const fn is_prefix(self) -> bool {
        matches!(self.class, Class::Prefix)
    }
}

/// Build [`TABLE`] and [`LISTED`] from one list of rows.
///
/// Every one of the 256 encodings is written out; `LISTED` records which, so a
/// test can prove the matrix is complete rather than trusting that it is.
macro_rules! primary {
    ($($opcode:literal $op:ident $dst:ident $src:ident $group:ident $class:ident;)*) => {
        /// The primary decode table: one row per opcode byte, and — with the
        /// group tables below — the only description of the instruction set in
        /// the crate.
        pub static TABLE: [Insn; 256] = {
            // The array must be initialised before it can be indexed in a
            // const block; every entry is then overwritten below, which
            // `LISTED` proves.
            let mut t =
                [Insn::new(Op::NOP, Arg::None, Arg::None, Grp::None, Class::Undefined); 256];
            $(t[$opcode as usize] =
                Insn::new(Op::$op, Arg::$dst, Arg::$src, Grp::$group, Class::$class);)*
            t
        };

        /// Which opcodes the table above actually assigns. Test scaffolding
        /// with a real job: an unassigned entry would silently decode as a
        /// plausible-looking `NOP`.
        pub static LISTED: [bool; 256] = {
            let mut t = [false; 256];
            $(t[$opcode as usize] = true;)*
            t
        };
    };
}

primary! {
    0x00 ADD  Eb   Gb   None   Documented;
    0x01 ADD  Ev   Gv   None   Documented;
    0x02 ADD  Gb   Eb   None   Documented;
    0x03 ADD  Gv   Ev   None   Documented;
    0x04 ADD  Al   Ib   None   Documented;
    0x05 ADD  Ax   Iv   None   Documented;
    0x06 PUSH Sr   None None   Documented;
    0x07 POP  Sr   None None   Documented;
    0x08 OR   Eb   Gb   None   Documented;
    0x09 OR   Ev   Gv   None   Documented;
    0x0a OR   Gb   Eb   None   Documented;
    0x0b OR   Gv   Ev   None   Documented;
    0x0c OR   Al   Ib   None   Documented;
    0x0d OR   Ax   Iv   None   Documented;
    0x0e PUSH Sr   None None   Documented;
    // POP CS assembles and executes; the 8086 simply does not document it,
    // and the 80186 reused the opcode for the two-byte escape.
    0x0f POP  Sr   None None   Undocumented;

    0x10 ADC  Eb   Gb   None   Documented;
    0x11 ADC  Ev   Gv   None   Documented;
    0x12 ADC  Gb   Eb   None   Documented;
    0x13 ADC  Gv   Ev   None   Documented;
    0x14 ADC  Al   Ib   None   Documented;
    0x15 ADC  Ax   Iv   None   Documented;
    0x16 PUSH Sr   None None   Documented;
    0x17 POP  Sr   None None   Documented;
    0x18 SBB  Eb   Gb   None   Documented;
    0x19 SBB  Ev   Gv   None   Documented;
    0x1a SBB  Gb   Eb   None   Documented;
    0x1b SBB  Gv   Ev   None   Documented;
    0x1c SBB  Al   Ib   None   Documented;
    0x1d SBB  Ax   Iv   None   Documented;
    0x1e PUSH Sr   None None   Documented;
    0x1f POP  Sr   None None   Documented;

    0x20 AND  Eb   Gb   None   Documented;
    0x21 AND  Ev   Gv   None   Documented;
    0x22 AND  Gb   Eb   None   Documented;
    0x23 AND  Gv   Ev   None   Documented;
    0x24 AND  Al   Ib   None   Documented;
    0x25 AND  Ax   Iv   None   Documented;
    0x26 SEG  Sr   None None   Prefix;
    0x27 DAA  None None None   Documented;
    0x28 SUB  Eb   Gb   None   Documented;
    0x29 SUB  Ev   Gv   None   Documented;
    0x2a SUB  Gb   Eb   None   Documented;
    0x2b SUB  Gv   Ev   None   Documented;
    0x2c SUB  Al   Ib   None   Documented;
    0x2d SUB  Ax   Iv   None   Documented;
    0x2e SEG  Sr   None None   Prefix;
    0x2f DAS  None None None   Documented;

    0x30 XOR  Eb   Gb   None   Documented;
    0x31 XOR  Ev   Gv   None   Documented;
    0x32 XOR  Gb   Eb   None   Documented;
    0x33 XOR  Gv   Ev   None   Documented;
    0x34 XOR  Al   Ib   None   Documented;
    0x35 XOR  Ax   Iv   None   Documented;
    0x36 SEG  Sr   None None   Prefix;
    0x37 AAA  None None None   Documented;
    0x38 CMP  Eb   Gb   None   Documented;
    0x39 CMP  Ev   Gv   None   Documented;
    0x3a CMP  Gb   Eb   None   Documented;
    0x3b CMP  Gv   Ev   None   Documented;
    0x3c CMP  Al   Ib   None   Documented;
    0x3d CMP  Ax   Iv   None   Documented;
    0x3e SEG  Sr   None None   Prefix;
    0x3f AAS  None None None   Documented;

    0x40 INC  Rv   None None   Documented;
    0x41 INC  Rv   None None   Documented;
    0x42 INC  Rv   None None   Documented;
    0x43 INC  Rv   None None   Documented;
    0x44 INC  Rv   None None   Documented;
    0x45 INC  Rv   None None   Documented;
    0x46 INC  Rv   None None   Documented;
    0x47 INC  Rv   None None   Documented;
    0x48 DEC  Rv   None None   Documented;
    0x49 DEC  Rv   None None   Documented;
    0x4a DEC  Rv   None None   Documented;
    0x4b DEC  Rv   None None   Documented;
    0x4c DEC  Rv   None None   Documented;
    0x4d DEC  Rv   None None   Documented;
    0x4e DEC  Rv   None None   Documented;
    0x4f DEC  Rv   None None   Documented;

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
    0x81 ADD  Ev   Iv   Alu    Documented;
    // 82 is 80 again: the sign-extend bit is not decoded for byte operands.
    0x82 ADD  Eb   Ib   Alu    Alias;
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
    0x9a CALLF Ap  None None   Documented;
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
    0xa9 TEST Ax   Iv   None   Documented;
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
    // opcode is not decoded.
    0xc0 RET  Iv   None None   Alias;
    0xc1 RET  None None None   Alias;
    0xc2 RET  Iv   None None   Documented;
    0xc3 RET  None None None   Documented;
    0xc4 LES  Gv   Mp   None   Documented;
    0xc5 LDS  Gv   Mp   None   Documented;
    0xc6 MOV  Eb   Ib   MovImm Documented;
    0xc7 MOV  Ev   Iv   MovImm Documented;
    0xc8 RETF Iv   None None   Alias;
    0xc9 RETF None None None   Alias;
    0xca RETF Iv   None None   Documented;
    0xcb RETF None None None   Documented;
    0xcc INT3 None None None   Documented;
    0xcd INT  Ib   None None   Documented;
    0xce INTO None None None   Documented;
    0xcf IRET None None None   Documented;

    0xd0 ROL  Eb   One  Shift  Documented;
    0xd1 ROL  Ev   One  Shift  Documented;
    0xd2 ROL  Eb   Cl   Shift  Documented;
    0xd3 ROL  Ev   Cl   Shift  Documented;
    0xd4 AAM  Ib   None None   Documented;
    0xd5 AAD  Ib   None None   Documented;
    0xd6 SALC None None None   Undocumented;
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
    0xea JMPF Ap   None None   Documented;
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
    0xf7 TEST  Ev   Iv   Unary Documented;
    0xf8 CLC   None None None  Documented;
    0xf9 STC   None None None  Documented;
    0xfa CLI   None None None  Documented;
    0xfb STI   None None None  Documented;
    0xfc CLD   None None None  Documented;
    0xfd STD   None None None  Documented;
    0xfe INC   Eb   None IncDec Documented;
    0xff INC   Ev   None Misc   Documented;
}

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
        (Arg::Ev, Arg::Iv)
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

/// Decode one opcode byte.
///
/// Total: all 256 encodings are described, prefixes and undocumented ones
/// included, so decoding never fails. Rows whose [`Insn::group`] is not
/// [`Grp::None`] still need [`resolve`] once the ModRM byte has been read.
#[inline]
#[must_use]
pub const fn decode(opcode: u8) -> Insn {
    TABLE[opcode as usize]
}

/// Apply an opcode-extension group, given the ModRM `reg` field.
///
/// A no-op for the great majority of encodings, which is why the interpreter
/// can call it unconditionally.
#[inline]
#[must_use]
pub const fn resolve(insn: Insn, reg: u8) -> Insn {
    let reg = (reg & 7) as usize;
    match insn.group {
        Grp::None | Grp::Pop | Grp::MovImm => insn,
        Grp::Alu => Insn {
            op: GROUP_ALU[reg],
            ..insn
        },
        Grp::Shift => Insn {
            op: GROUP_SHIFT[reg],
            ..insn
        },
        Grp::IncDec => GROUP_INCDEC[reg],
        Grp::Unary => {
            if matches!(insn.dst, Arg::Eb) {
                GROUP_UNARY8[reg]
            } else {
                GROUP_UNARY16[reg]
            }
        }
        Grp::Misc => GROUP_MISC[reg],
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

    /// The register's name, lower case.
    #[must_use]
    pub const fn name(sr: u8) -> &'static str {
        match sr & 3 {
            ES => "es",
            CS => "cs",
            SS => "ss",
            _ => "ds",
        }
    }
}

/// A decoded ModRM byte.
///
/// The 8086 has no SIB byte and no 32-bit addressing: `rm` selects one of
/// eight fixed register combinations, and `md` says how much displacement
/// follows.
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

    /// How many displacement bytes follow.
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

    /// The segment the address defaults to, before any override.
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
    /// The segment register a prefix selected, if any.
    pub seg_override: Option<u8>,
    /// The repeat prefix, if any.
    pub rep: Option<Rep>,
    /// Whether a `LOCK` prefix was seen.
    pub lock: bool,
    /// The opcode byte, after all prefixes.
    pub opcode: u8,
    /// The table row, already passed through [`resolve`].
    pub insn: Insn,
    /// The ModRM byte, where the encoding has one.
    pub modrm: Option<ModRm>,
    /// The displacement, sign-extended to 16 bits.
    pub disp: u16,
    /// The immediate, or the far pointer's `offset | segment << 16`.
    pub imm: u32,
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

    /// The segment half of a far-pointer immediate (`Ap`).
    #[inline]
    #[must_use]
    pub const fn imm_seg(&self) -> u16 {
        (self.imm >> 16) as u16
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
    let mut f = Fields {
        seg_override: None,
        rep: None,
        lock: false,
        opcode: 0x90,
        insn: decode(0x90),
        modrm: None,
        disp: 0,
        imm: 0,
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

    // Prefixes.
    let mut prefixes = 0u8;
    let opcode = loop {
        let byte = take(&mut f);
        if f.truncated {
            return f;
        }
        let row = decode(byte);
        if !row.is_prefix() || prefixes >= MAX_PREFIXES {
            break byte;
        }
        prefixes += 1;
        match row.op {
            Op::SEG => f.seg_override = Some((byte >> 3) & 3),
            Op::LOCK => f.lock = true,
            Op::REP => f.rep = Some(Rep::While),
            Op::REPNE => f.rep = Some(Rep::WhileNot),
            // `is_prefix` is true for exactly the four operations above.
            _ => {}
        }
    };

    f.opcode = opcode;
    let mut insn = decode(opcode);

    if insn.needs_modrm() {
        let byte = take(&mut f);
        let modrm = ModRm::new(byte);
        f.modrm = Some(modrm);
        insn = resolve(insn, modrm.reg);
        let n = modrm.disp_bytes();
        if n > 0 {
            let lo = take(&mut f);
            f.disp = if n == 1 {
                // An 8-bit displacement is signed, and it is added in 16-bit
                // arithmetic, so it is sign-extended here rather than at the
                // point of use.
                lo as i8 as u16
            } else {
                let hi = take(&mut f);
                u16::from(lo) | (u16::from(hi) << 8)
            };
        }
    }
    f.insn = insn;

    let imm_bytes = insn.dst.immediate_bytes() + insn.src.immediate_bytes();
    let mut imm: u32 = 0;
    for i in 0..imm_bytes {
        let byte = take(&mut f);
        imm |= u32::from(byte) << (8 * u32::from(i));
    }
    // `83 /n` and every short relative branch sign-extend their byte to a
    // word before use; doing it here keeps every consumer from repeating the
    // cast, and makes a backward jump come out as an addition.
    if insn.src == Arg::Ibs || insn.dst == Arg::Ibs || insn.dst == Arg::Jb {
        imm = u32::from(imm as u8 as i8 as u16);
    }
    f.imm = imm;
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
        // feature. The group tables count as reachable.
        let reachable = |op: Op| {
            TABLE.iter().any(|i| i.op == op)
                || GROUP_ALU.contains(&op)
                || GROUP_SHIFT.contains(&op)
                || GROUP_UNARY8.iter().any(|i| i.op == op)
                || GROUP_MISC.iter().any(|i| i.op == op)
        };
        for op in Op::ALL {
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
        assert_eq!(resolve(decode(0xf7), 0).src, Arg::Iv);
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
        assert_eq!(decode(0x00).width_bytes(), Some(1));
        assert_eq!(decode(0x01).width_bytes(), Some(2));
        assert_eq!(decode(0xa4).width_bytes(), Some(1)); // movsb
        assert_eq!(decode(0xa5).width_bytes(), Some(2)); // movsw
        assert_eq!(decode(0x83).width_bytes(), Some(2)); // Ev,Ibs
        assert_eq!(decode(0xf4).width_bytes(), None); // hlt
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
        assert_eq!(f.disp, 0xffff);
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
