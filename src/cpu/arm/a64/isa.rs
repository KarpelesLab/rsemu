//! The A64 instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift, and the disassembler is
//! not a side project: gdb and the monitor both need it (`ROADMAP.md` §6). So
//! this file holds one declarative table, [`TABLE`], and both consumers read
//! it and nothing else:
//!
//! * [`decode`], which the interpreter calls;
//! * [`super::disasm`], which knows no opcode at all — every mnemonic comes
//!   from [`Op::mnemonic`] and every operand layout from [`Fmt`].
//!
//! Adding an instruction is one line here and no edit anywhere else.
//!
//! # Two suffixes the mnemonic column does not carry
//!
//! A64 spells two things in bits that a *row* cannot: the condition on
//! `B.<cond>`, and the acquire/release ordering on the LSE atomics
//! (`LDADD`/`LDADDA`/`LDADDL`/`LDADDAL`). Writing sixteen rows for `B.cond`
//! and four for every atomic would be a table describing the encoding rather
//! than the instruction set. So the rule is stated once and applies to exactly
//! two formats: **a format may own a suffix**, and [`Fmt::suffix_kind`] says
//! which. The disassembler asks the format; nothing else in the crate spells a
//! mnemonic.
//!
//! # Why loads and stores have one row per mnemonic but few shapes
//!
//! `LDRB`, `LDRSB`, `LDRH`, `LDR` and `STR` differ only in the `size` and
//! `opc` fields of one encoding, and the mapping from those two fields to
//! *what the access does* is a rule the architecture states once. So each
//! spelling gets its own row — the table names every instruction, and `PRFM`
//! and the unallocated `size:opc` pairs are visible as present and absent —
//! while the interpreter reads the access shape out of the encoding with
//! [`ls_access`] instead of matching a hundred variants. One description, two
//! readers, still.
//!
//! # Why there is no cycle column
//!
//! The same reason as every other core here: a cycle is charged *because* an
//! access happened. Arm does not architecturally define instruction timing —
//! it is a property of a particular implementation's pipeline — so a cycle
//! table would be invention.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487),
//! chapter C4 "A64 Instruction Set Encoding" for every `(mask, bits)` pair
//! here, C6 for the operand syntax, and the `DecodeBitMasks`, `AddWithCarry`
//! and `ConditionHolds` pseudocode in the shared-pseudocode chapter. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

use core::fmt;

/// Extract bits `hi..=lo` of `word`.
///
/// Written to stay correct at `hi == 31, lo == 0`, where the obvious
/// `(1 << (hi - lo + 1)) - 1` overflows.
#[inline]
#[must_use]
pub const fn field(word: u32, hi: u32, lo: u32) -> u32 {
    (word >> lo) & (u32::MAX >> (31 - (hi - lo)))
}

/// Whether bit `n` of `word` is set.
#[inline]
#[must_use]
pub const fn bit(word: u32, n: u32) -> bool {
    word & (1 << n) != 0
}

// ---------------------------------------------------------------------------
// Condition codes
// ---------------------------------------------------------------------------

/// The four-bit condition field (DDI 0487 C1.2.4).
///
/// A `#[repr(transparent)]` newtype with `pub const` variants rather than an
/// enum: all sixteen values are meaningful, so exhaustiveness buys nothing
/// (CLAUDE.md, "Type conventions").
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Cond(pub u8);

impl Cond {
    /// Equal — `Z` set.
    pub const EQ: Cond = Cond(0x0);
    /// Not equal — `Z` clear.
    pub const NE: Cond = Cond(0x1);
    /// Carry set; unsigned higher or same.
    pub const CS: Cond = Cond(0x2);
    /// Carry clear; unsigned lower.
    pub const CC: Cond = Cond(0x3);
    /// Minus, negative.
    pub const MI: Cond = Cond(0x4);
    /// Plus, positive or zero.
    pub const PL: Cond = Cond(0x5);
    /// Overflow set.
    pub const VS: Cond = Cond(0x6);
    /// Overflow clear.
    pub const VC: Cond = Cond(0x7);
    /// Unsigned higher.
    pub const HI: Cond = Cond(0x8);
    /// Unsigned lower or same.
    pub const LS: Cond = Cond(0x9);
    /// Signed greater than or equal.
    pub const GE: Cond = Cond(0xa);
    /// Signed less than.
    pub const LT: Cond = Cond(0xb);
    /// Signed greater than.
    pub const GT: Cond = Cond(0xc);
    /// Signed less than or equal.
    pub const LE: Cond = Cond(0xd);
    /// Always.
    pub const AL: Cond = Cond(0xe);
    /// Always, in the encoding `B.cond` may not use. Spelled `NV`, and it
    /// behaves as *always* rather than as never — the name is a leftover.
    pub const NV: Cond = Cond(0xf);

    /// The assembler spelling.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self.0 & 0xf {
            0x0 => "eq",
            0x1 => "ne",
            0x2 => "cs",
            0x3 => "cc",
            0x4 => "mi",
            0x5 => "pl",
            0x6 => "vs",
            0x7 => "vc",
            0x8 => "hi",
            0x9 => "ls",
            0xa => "ge",
            0xb => "lt",
            0xc => "gt",
            0xd => "le",
            0xe => "al",
            _ => "nv",
        }
    }

    /// Whether this condition holds for the given `NZCV`.
    ///
    /// DDI 0487 `ConditionHolds`: the top three bits select the test and the
    /// bottom bit inverts it — except at `0b1111`, where inverting *always*
    /// would give *never*, and the architecture says it does not.
    #[must_use]
    pub const fn holds(self, nzcv: Nzcv) -> bool {
        let (n, z, c, v) = (nzcv.n(), nzcv.z(), nzcv.c(), nzcv.v());
        let base = match (self.0 >> 1) & 7 {
            0 => z,
            1 => c,
            2 => n,
            3 => v,
            4 => c && !z,
            5 => n == v,
            6 => n == v && !z,
            _ => true,
        };
        if self.0 & 1 == 1 && self.0 & 0xf != 0xf {
            !base
        } else {
            base
        }
    }
}

impl fmt::Display for Cond {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The four condition flags, held in the top nibble the way `NZCV` and
/// `SPSR_EL1` hold them.
///
/// A newtype rather than four `bool`s because that is how the guest sees them:
/// `MRS x0, NZCV` reads this word, `MSR NZCV, x0` writes it, and every
/// exception entry copies it into `SPSR_EL1` bits 31:28 unchanged.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct Nzcv(pub u32);

impl Nzcv {
    /// Negative.
    pub const N: u32 = 1 << 31;
    /// Zero.
    pub const Z: u32 = 1 << 30;
    /// Carry.
    pub const C: u32 = 1 << 29;
    /// Overflow.
    pub const V: u32 = 1 << 28;

    /// Build one from four booleans.
    #[must_use]
    pub const fn new(n: bool, z: bool, c: bool, v: bool) -> Nzcv {
        let mut bits = 0;
        if n {
            bits |= Nzcv::N;
        }
        if z {
            bits |= Nzcv::Z;
        }
        if c {
            bits |= Nzcv::C;
        }
        if v {
            bits |= Nzcv::V;
        }
        Nzcv(bits)
    }

    /// Build one from the low nibble, which is how `CCMP`'s immediate spells
    /// the flags it forces.
    #[must_use]
    pub const fn from_nibble(nibble: u32) -> Nzcv {
        Nzcv((nibble & 0xf) << 28)
    }

    /// Negative.
    #[must_use]
    pub const fn n(self) -> bool {
        self.0 & Nzcv::N != 0
    }

    /// Zero.
    #[must_use]
    pub const fn z(self) -> bool {
        self.0 & Nzcv::Z != 0
    }

    /// Carry.
    #[must_use]
    pub const fn c(self) -> bool {
        self.0 & Nzcv::C != 0
    }

    /// Overflow.
    #[must_use]
    pub const fn v(self) -> bool {
        self.0 & Nzcv::V != 0
    }
}

impl fmt::Display for Nzcv {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flag = |set, c| if set { c } else { '-' };
        write!(
            f,
            "{}{}{}{}",
            flag(self.n(), 'N'),
            flag(self.z(), 'Z'),
            flag(self.c(), 'C'),
            flag(self.v(), 'V')
        )
    }
}

// ---------------------------------------------------------------------------
// The extension lattice
// ---------------------------------------------------------------------------

/// Which architectural feature an encoding needs.
///
/// `ROADMAP.md` §6.1.1: Arm's versions are a lattice of independently optional
/// extensions, not a chain, so an encoding names *the feature it needs* rather
/// than a version number to compare against. An instruction whose feature the
/// configured part lacks must not decode — a guest probes for `FEAT_LSE` by
/// executing `CAS` and catching the `UNDEF`, and a core that executed it
/// anyway would be reporting a CPU it is not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Feat {
    /// Armv8.0-A: present on every A64 part there is.
    Base,
    /// `FEAT_LSE` — the large-system atomics (`CAS`, `SWP`, `LD<op>`),
    /// optional in Armv8.0 and mandatory from Armv8.1.
    Lse,
    /// `FEAT_CRC32` — the CRC-32 accelerators, optional in Armv8.0 and
    /// mandatory from Armv8.1.
    Crc32,
    /// `FEAT_FP` — scalar floating point: the SIMD&FP register file and the
    /// `F*` instructions. OPTIONAL in Armv8.0-A, and present on every part
    /// anybody ships — but optional is optional, and a guest probes for it by
    /// reading `ID_AA64PFR0_EL1.FP` and by taking the `CPACR_EL1` trap.
    Fp,
}

impl Feat {
    /// The name Arm gives the feature, as a machine file spells it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Feat::Base => "base",
            Feat::Lse => "lse",
            Feat::Crc32 => "crc32",
            Feat::Fp => "fp",
        }
    }
}

/// Which optional features one core instance has.
///
/// Total and un-`cfg`'d on purpose (`ROADMAP.md` §6.1.1): the fields exist in
/// every build, so this is one type with one shape and a downstream crate
/// naming a preset gets a construction error rather than a struct that changes
/// underneath it.
///
/// Deliberately **not** `PartialOrd`: `features >= X` is exactly the bug a
/// lattice exists to prevent, and the x86 core's `Variant` omits `Ord` for the
/// same reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Hash)]
pub struct Features {
    /// `FEAT_LSE`, the large-system atomics.
    pub lse: bool,
    /// `FEAT_CRC32`, the CRC-32 accelerators.
    pub crc32: bool,
    /// `FEAT_FP`, scalar floating point and the SIMD&FP register file.
    pub fp: bool,
}

impl Features {
    /// No optional feature at all: a bare Armv8.0-A part.
    pub const NONE: Features = Features {
        lse: false,
        crc32: false,
        fp: false,
    };

    /// Everything this core implements.
    pub const ALL: Features = Features {
        lse: true,
        crc32: true,
        fp: true,
    };

    /// Whether an encoding requiring `feat` may decode here.
    #[inline]
    #[must_use]
    pub const fn has(self, feat: Feat) -> bool {
        match feat {
            Feat::Base => true,
            Feat::Lse => self.lse,
            Feat::Crc32 => self.crc32,
            Feat::Fp => self.fp,
        }
    }
}

// ---------------------------------------------------------------------------
// Operand layouts
// ---------------------------------------------------------------------------

/// Which suffix a format spells out of the encoding rather than out of the
/// mnemonic column.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Suffix {
    /// None: the mnemonic column is the whole mnemonic.
    None,
    /// `.<cond>`, from bits 3:0 — `B.EQ`.
    Cond,
    /// `A`, `L` or `AL`, from the acquire and release bits of an LSE atomic.
    Order,
}

/// How an instruction's operands are laid out, and therefore how the
/// disassembler prints it and where the interpreter looks.
///
/// The `S` suffix on a name is the flag-setting form, and it is here rather
/// than in the mnemonic because it changes an *operand rule*: register 31
/// reads and writes the stack pointer in exactly the places DDI 0487 C1.2.5
/// lists, and `ADD`'s destination is one of them while `ADDS`'s is not.
/// Encoding that here is what stops the interpreter and the disassembler
/// disagreeing about whether `add x0, x31, #1` means `sp` or `xzr` — they ask
/// the same enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Fmt {
    /// `Rd, #label` — `ADR`/`ADRP`.
    PcRel,
    /// `Rd|SP, Rn|SP, #imm{, lsl #12}`.
    AddSubImm,
    /// `Rd, Rn|SP, #imm{, lsl #12}` — the flag-setting form, whose `Rd` is
    /// `ZR`.
    AddSubImmS,
    /// `Rd|SP, Rn, #bitmask`.
    LogImm,
    /// `Rd, Rn, #bitmask` — `ANDS`, whose `Rd` is `ZR`.
    LogImmS,
    /// `Rd, #imm16{, lsl #shift}`.
    MoveWide,
    /// `Rd, Rn, #immr, #imms`.
    Bitfield,
    /// `Rd, Rn, Rm, #lsb`.
    Extr,
    /// `#label`, from a 26-bit displacement.
    BranchImm,
    /// `#label`, from a 19-bit displacement, with a condition suffix.
    CondBranch,
    /// `Rt, #label`.
    CmpBranch,
    /// `Rt, #bit, #label`.
    TestBranch,
    /// `Rn` — `BR`, `BLR`, `RET`.
    BranchReg,
    /// No operands — `ERET`, `NOP`, `WFI`.
    NoOperands,
    /// `#imm16` — `SVC`, `BRK`.
    Exception,
    /// `<option>` — a barrier's shareability domain.
    Barrier,
    /// `<pstatefield>, #imm` — `MSR DAIFSet, #3`.
    PstateImm,
    /// `Rt, <sysreg>` — `MRS`.
    SysRead,
    /// `<sysreg>, Rt` — `MSR`, register form.
    SysWrite,
    /// `#op1, Cn, Cm, #op2, Rt` — `SYS`, and the `TLBI`/`DC`/`IC` aliases.
    SysOp,
    /// `Rt, #label` — a literal load.
    LoadLiteral,
    /// `Rt, [Rn|SP{, #pimm}]` — the scaled unsigned offset.
    LdStUImm,
    /// `Rt, [Rn|SP{, #simm}]` — `LDUR`/`STUR`.
    LdStUnscaled,
    /// `Rt, [Rn|SP], #simm` — post-indexed.
    LdStPost,
    /// `Rt, [Rn|SP, #simm]!` — pre-indexed.
    LdStPre,
    /// `Rt, [Rn|SP, Rm{, extend {#amount}}]`.
    LdStRegOff,
    /// `Rt, Rt2, [Rn|SP{, #imm}]`.
    LdStPairOff,
    /// `Rt, Rt2, [Rn|SP], #imm` — post-indexed.
    LdStPairPost,
    /// `Rt, Rt2, [Rn|SP, #imm]!` — pre-indexed.
    LdStPairPre,
    /// `Rt, [Rn|SP]` — `LDXR`, `LDAR`, `STLR`.
    LdStExclusive,
    /// `Ws, Rt, [Rn|SP]` — `STXR`, whose first operand is the status result.
    StoreExclusive,
    /// `Rs, Rt, [Rn|SP]` — an LSE atomic, with an ordering suffix.
    Atomic,
    /// `Rd|SP, Rn|SP, Rm{, extend {#amount}}`.
    AddSubExt,
    /// `Rd, Rn|SP, Rm{, extend {#amount}}` — the flag-setting form.
    AddSubExtS,
    /// `Rd, Rn, Rm{, shift #amount}`.
    ShiftedReg,
    /// `Rd, Rn, Rm` — `ADC`, `UDIV`, `SMULH`.
    ThreeReg,
    /// `Rn, Rm, #nzcv, cond`.
    CondCmpReg,
    /// `Rn, #imm5, #nzcv, cond`.
    CondCmpImm,
    /// `Rd, Rn, Rm, cond`.
    CondSel,
    /// `Rd, Rn, Rm, Ra`.
    FourReg,
    /// `Rd, Rn, Rm` where the accumulator is 32 bits wide and the destination
    /// is not — the `CRC32` shape.
    CrcReg,
    /// `Rd, Rn`.
    TwoReg,

    // -- Scalar floating point -------------------------------------------
    //
    // The SIMD&FP register file is a second register file with its own
    // widths, so these do not reuse the integer formats even where the
    // operand *count* matches: `reg()` and `vreg()` print different names,
    // and the interpreter reads different state.
    /// `<Vd>, <Vn>` at one precision — `FABS`, `FNEG`, `FSQRT`, `FRINT*`.
    FpOneSrc,
    /// `<Vd>, <Vn>` where the two differ — `FCVT`.
    FpCvt,
    /// `<Vd>, <Vn>, <Vm>` — `FADD` and its family.
    FpTwoSrc,
    /// `<Vd>, <Vn>, <Vm>, <Va>` — `FMADD` and its family.
    FpThreeSrc,
    /// `<Vn>, <Vm>` or `<Vn>, #0.0` — `FCMP`, `FCMPE`.
    FpCmp,
    /// `<Vn>, <Vm>, #nzcv, cond` — `FCCMP`, `FCCMPE`.
    FpCondCmp,
    /// `<Vd>, <Vn>, <Vm>, cond` — `FCSEL`.
    FpCondSel,
    /// `<Vd>, #imm` — `FMOV` with an eight-bit immediate.
    FpImm,
    /// A general register on one side and a SIMD&FP register on the other —
    /// `FCVTZS`, `SCVTF`, `FMOV` between the files.
    FpIntCvt,
    /// The same, with a fixed-point scale — `SCVTF <Sd>, <Wn>, #fbits`.
    FpFixCvt,
    /// `<Vt>, #label` — a SIMD&FP literal load.
    LoadFpLiteral,
    /// `<Vt>, [Rn|SP{, #pimm}]`.
    LdStFpUImm,
    /// `<Vt>, [Rn|SP{, #simm}]` — `LDUR`/`STUR` of a SIMD&FP register.
    LdStFpUnscaled,
    /// `<Vt>, [Rn|SP], #simm` — post-indexed.
    LdStFpPost,
    /// `<Vt>, [Rn|SP, #simm]!` — pre-indexed.
    LdStFpPre,
    /// `<Vt>, [Rn|SP, Rm{, extend {#amount}}]`.
    LdStFpRegOff,
    /// `<Vt>, <Vt2>, [Rn|SP{, #imm}]`.
    LdStFpPairOff,
    /// `<Vt>, <Vt2>, [Rn|SP], #imm` — post-indexed.
    LdStFpPairPost,
    /// `<Vt>, <Vt2>, [Rn|SP, #imm]!` — pre-indexed.
    LdStFpPairPre,
}

impl Fmt {
    /// Whether register 31 in the `Rd` position means the stack pointer.
    #[inline]
    #[must_use]
    pub const fn rd_is_sp(self) -> bool {
        matches!(self, Fmt::AddSubImm | Fmt::LogImm | Fmt::AddSubExt)
    }

    /// Whether register 31 in the `Rn` position means the stack pointer.
    ///
    /// DDI 0487 C1.2.5: every base register of a load or store, and the first
    /// source of `ADD`/`SUB` in the immediate and extended-register forms.
    #[inline]
    #[must_use]
    pub const fn rn_is_sp(self) -> bool {
        matches!(
            self,
            Fmt::AddSubImm
                | Fmt::AddSubImmS
                | Fmt::AddSubExt
                | Fmt::AddSubExtS
                | Fmt::LdStUImm
                | Fmt::LdStUnscaled
                | Fmt::LdStPost
                | Fmt::LdStPre
                | Fmt::LdStRegOff
                | Fmt::LdStPairOff
                | Fmt::LdStPairPost
                | Fmt::LdStPairPre
                | Fmt::LdStExclusive
                | Fmt::StoreExclusive
                | Fmt::Atomic
                | Fmt::LdStFpUImm
                | Fmt::LdStFpUnscaled
                | Fmt::LdStFpPost
                | Fmt::LdStFpPre
                | Fmt::LdStFpRegOff
                | Fmt::LdStFpPairOff
                | Fmt::LdStFpPairPost
                | Fmt::LdStFpPairPre
        )
    }

    /// Which suffix this format spells out of the encoding.
    #[inline]
    #[must_use]
    pub const fn suffix_kind(self) -> Suffix {
        match self {
            Fmt::CondBranch => Suffix::Cond,
            Fmt::Atomic => Suffix::Order,
            _ => Suffix::None,
        }
    }

    /// Whether this format is one of the load/store families the interpreter
    /// dispatches by shape rather than by operation.
    #[inline]
    #[must_use]
    pub const fn is_load_store(self) -> bool {
        matches!(
            self,
            Fmt::LdStUImm
                | Fmt::LdStUnscaled
                | Fmt::LdStPost
                | Fmt::LdStPre
                | Fmt::LdStRegOff
                | Fmt::LdStPairOff
                | Fmt::LdStPairPost
                | Fmt::LdStPairPre
        )
    }

    /// Whether this format is one of the SIMD&FP load/store families.
    ///
    /// Separate from [`Fmt::is_load_store`] rather than folded into it,
    /// because the two dispatch to different register files and share only
    /// the addressing-mode arithmetic. Folding them would put a
    /// `if is_simd { … }` inside every one of those modes.
    #[inline]
    #[must_use]
    pub const fn is_fp_load_store(self) -> bool {
        matches!(
            self,
            Fmt::LdStFpUImm
                | Fmt::LdStFpUnscaled
                | Fmt::LdStFpPost
                | Fmt::LdStFpPre
                | Fmt::LdStFpRegOff
                | Fmt::LdStFpPairOff
                | Fmt::LdStFpPairPost
                | Fmt::LdStFpPairPre
        )
    }
}

/// One row of the instruction description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// The bits of the encoding that are fixed.
    pub mask: u32,
    /// What those bits must be.
    pub bits: u32,
    /// Operand layout, and therefore the disassembly.
    pub fmt: Fmt,
    /// The architectural feature it needs.
    pub feat: Feat,
}

/// Declare the operation enum, its mnemonics, its summaries and the decode
/// table from one list of rows.
///
/// The mnemonic is a literal beside the variant because A64 mnemonics are not
/// Rust identifiers (`b.cond`, `ldrsb`) and because two rows may legitimately
/// share a spelling — `REV` is one instruction on 32-bit operands and another
/// on 64-bit ones.
macro_rules! a64 {
    ($($mask:literal $bits:literal $op:ident $mn:literal $fmt:ident $feat:ident $summary:literal;)*) => {
        /// One operation, independent of how it is encoded.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum Op {
            $(
                #[doc = $summary]
                $op,
            )*
        }

        impl Op {
            /// The assembler mnemonic, without any suffix the encoding owns
            /// (see [`Fmt::suffix_kind`]).
            #[must_use]
            pub const fn mnemonic(self) -> &'static str {
                match self { $(Op::$op => $mn,)* }
            }

            /// A one-line description, for `rsemu describe` and the monitor.
            #[must_use]
            pub const fn summary(self) -> &'static str {
                match self { $(Op::$op => $summary,)* }
            }

            /// Every operation this core implements, in encoding order.
            pub const ALL: &'static [Op] = &[$(Op::$op,)*];
        }

        /// The decode table: the only description of A64 in the crate.
        pub static TABLE: &[Insn] = &[
            $(Insn {
                op: Op::$op,
                mask: $mask,
                bits: $bits,
                fmt: Fmt::$fmt,
                feat: Feat::$feat,
            },)*
        ];
    };
}

a64! {
    // -- Data processing: PC-relative addressing ----------------------------
    0x9f000000 0x10000000 Adr  "adr"  PcRel Base "form a PC-relative address";
    0x9f000000 0x90000000 Adrp "adrp" PcRel Base "form a PC-relative address to a 4 KiB page";

    // -- Data processing: add/subtract (immediate) --------------------------
    0x7f800000 0x11000000 AddImm  "add"  AddSubImm  Base "add an immediate";
    0x7f800000 0x31000000 AddsImm "adds" AddSubImmS Base "add an immediate, setting the flags";
    0x7f800000 0x51000000 SubImm  "sub"  AddSubImm  Base "subtract an immediate";
    0x7f800000 0x71000000 SubsImm "subs" AddSubImmS Base "subtract an immediate, setting the flags";

    // -- Data processing: logical (immediate) -------------------------------
    0x7f800000 0x12000000 AndImm  "and"  LogImm  Base "bitwise AND with a bitmask immediate";
    0x7f800000 0x32000000 OrrImm  "orr"  LogImm  Base "bitwise OR with a bitmask immediate";
    0x7f800000 0x52000000 EorImm  "eor"  LogImm  Base "bitwise exclusive-OR with a bitmask immediate";
    0x7f800000 0x72000000 AndsImm "ands" LogImmS Base "bitwise AND with a bitmask immediate, setting the flags";

    // -- Data processing: move wide (immediate) -----------------------------
    0x7f800000 0x12800000 Movn "movn" MoveWide Base "move the inverse of a 16-bit immediate";
    0x7f800000 0x52800000 Movz "movz" MoveWide Base "move a 16-bit immediate, zeroing the rest";
    0x7f800000 0x72800000 Movk "movk" MoveWide Base "move a 16-bit immediate, keeping the rest";

    // -- Data processing: bitfield and extract ------------------------------
    0x7f800000 0x13000000 Sbfm "sbfm" Bitfield Base "signed bitfield move";
    0x7f800000 0x33000000 Bfm  "bfm"  Bitfield Base "bitfield move, leaving the other bits";
    0x7f800000 0x53000000 Ubfm "ubfm" Bitfield Base "unsigned bitfield move";
    0x7fa00000 0x13800000 Extr "extr" Extr     Base "extract a register pair at a bit position";

    // -- Branches -----------------------------------------------------------
    0xfc000000 0x14000000 B     "b"    BranchImm  Base "branch";
    0xfc000000 0x94000000 Bl    "bl"   BranchImm  Base "branch with link";
    0xff000010 0x54000000 Bcond "b"    CondBranch Base "branch conditionally";
    0x7f000000 0x34000000 Cbz   "cbz"  CmpBranch  Base "compare with zero and branch if equal";
    0x7f000000 0x35000000 Cbnz  "cbnz" CmpBranch  Base "compare with zero and branch if not equal";
    0x7f000000 0x36000000 Tbz   "tbz"  TestBranch Base "test a bit and branch if zero";
    0x7f000000 0x37000000 Tbnz  "tbnz" TestBranch Base "test a bit and branch if not zero";
    0xfffffc1f 0xd61f0000 Br    "br"   BranchReg  Base "branch to a register";
    0xfffffc1f 0xd63f0000 Blr   "blr"  BranchReg  Base "branch with link to a register";
    0xfffffc1f 0xd65f0000 Ret   "ret"  BranchReg  Base "return from a subroutine";
    0xffffffff 0xd69f03e0 Eret  "eret" NoOperands Base "return from an exception";

    // -- Exception generation ----------------------------------------------
    0xffe0001f 0xd4000001 Svc "svc" Exception Base "supervisor call";
    0xffe0001f 0xd4000002 Hvc "hvc" Exception Base "hypervisor call";
    0xffe0001f 0xd4000003 Smc "smc" Exception Base "secure monitor call";
    0xffe0001f 0xd4200000 Brk "brk" Exception Base "breakpoint";
    0xffe0001f 0xd4400000 Hlt "hlt" Exception Base "halt, for an external debugger";

    // -- Hints and barriers -------------------------------------------------
    0xffffffff 0xd503201f Nop   "nop"   NoOperands Base "no operation";
    0xffffffff 0xd503203f Yield "yield" NoOperands Base "hint that the thread is spinning";
    0xffffffff 0xd503205f Wfe   "wfe"   NoOperands Base "wait for an event";
    0xffffffff 0xd503207f Wfi   "wfi"   NoOperands Base "wait for an interrupt";
    0xffffffff 0xd503209f Sev   "sev"   NoOperands Base "signal an event to every core";
    0xffffffff 0xd50320bf Sevl  "sevl"  NoOperands Base "signal an event locally";
    0xfffff01f 0xd503201f Hint  "hint"  NoOperands Base "an unallocated hint, which executes as a no-op";
    0xfffff0ff 0xd503305f Clrex "clrex" Barrier    Base "clear the local exclusive monitor";
    0xfffff0ff 0xd503309f Dsb   "dsb"   Barrier    Base "data synchronization barrier";
    0xfffff0ff 0xd50330bf Dmb   "dmb"   Barrier    Base "data memory barrier";
    0xfffff0ff 0xd50330df Isb   "isb"   Barrier    Base "instruction synchronization barrier";

    // -- System register and PSTATE access ----------------------------------
    0xfffff0ff 0xd50040bf MsrSpsel   "msr"  PstateImm Base "select the stack pointer PSTATE uses";
    0xfffff0ff 0xd50340df MsrDaifset "msr"  PstateImm Base "set interrupt mask bits";
    0xfffff0ff 0xd50340ff MsrDaifclr "msr"  PstateImm Base "clear interrupt mask bits";
    0xfff00000 0xd5300000 Mrs        "mrs"  SysRead   Base "read a system register";
    0xfff00000 0xd5100000 Msr        "msr"  SysWrite  Base "write a system register";
    0xfff80000 0xd5080000 Sys        "sys"  SysOp     Base "a system operation: the TLBI, DC and IC aliases";
    0xfff80000 0xd5280000 Sysl       "sysl" SysOp     Base "a system operation with a result";

    // -- Loads and stores: literal ------------------------------------------
    0xff000000 0x18000000 LdrLitW  "ldr"   LoadLiteral Base "load a word from a PC-relative literal";
    0xff000000 0x58000000 LdrLitX  "ldr"   LoadLiteral Base "load a doubleword from a PC-relative literal";
    0xff000000 0x98000000 LdrswLit "ldrsw" LoadLiteral Base "load a sign-extended word from a PC-relative literal";
    0xff000000 0xd8000000 PrfmLit  "prfm"  LoadLiteral Base "prefetch from a PC-relative literal";

    // -- Loads and stores: register, unsigned scaled offset -----------------
    0xffc00000 0x39000000 StrbImm   "strb"  LdStUImm Base "store a byte";
    0xffc00000 0x39400000 LdrbImm   "ldrb"  LdStUImm Base "load a zero-extended byte";
    0xffc00000 0x39800000 LdrsbXImm "ldrsb" LdStUImm Base "load a byte, sign-extended to 64 bits";
    0xffc00000 0x39c00000 LdrsbWImm "ldrsb" LdStUImm Base "load a byte, sign-extended to 32 bits";
    0xffc00000 0x79000000 StrhImm   "strh"  LdStUImm Base "store a halfword";
    0xffc00000 0x79400000 LdrhImm   "ldrh"  LdStUImm Base "load a zero-extended halfword";
    0xffc00000 0x79800000 LdrshXImm "ldrsh" LdStUImm Base "load a halfword, sign-extended to 64 bits";
    0xffc00000 0x79c00000 LdrshWImm "ldrsh" LdStUImm Base "load a halfword, sign-extended to 32 bits";
    0xffc00000 0xb9000000 StrWImm   "str"   LdStUImm Base "store a word";
    0xffc00000 0xb9400000 LdrWImm   "ldr"   LdStUImm Base "load a word";
    0xffc00000 0xb9800000 LdrswImm  "ldrsw" LdStUImm Base "load a word, sign-extended to 64 bits";
    0xffc00000 0xf9000000 StrXImm   "str"   LdStUImm Base "store a doubleword";
    0xffc00000 0xf9400000 LdrXImm   "ldr"   LdStUImm Base "load a doubleword";
    0xffc00000 0xf9800000 PrfmImm   "prfm"  LdStUImm Base "prefetch memory";

    // -- Loads and stores: register, unscaled 9-bit offset ------------------
    0xffe00c00 0x38000000 Sturb   "sturb"  LdStUnscaled Base "store a byte, unscaled offset";
    0xffe00c00 0x38400000 Ldurb   "ldurb"  LdStUnscaled Base "load a zero-extended byte, unscaled offset";
    0xffe00c00 0x38800000 LdursbX "ldursb" LdStUnscaled Base "load a byte sign-extended to 64 bits, unscaled offset";
    0xffe00c00 0x38c00000 LdursbW "ldursb" LdStUnscaled Base "load a byte sign-extended to 32 bits, unscaled offset";
    0xffe00c00 0x78000000 Sturh   "sturh"  LdStUnscaled Base "store a halfword, unscaled offset";
    0xffe00c00 0x78400000 Ldurh   "ldurh"  LdStUnscaled Base "load a zero-extended halfword, unscaled offset";
    0xffe00c00 0x78800000 LdurshX "ldursh" LdStUnscaled Base "load a halfword sign-extended to 64 bits, unscaled offset";
    0xffe00c00 0x78c00000 LdurshW "ldursh" LdStUnscaled Base "load a halfword sign-extended to 32 bits, unscaled offset";
    0xffe00c00 0xb8000000 SturW   "stur"   LdStUnscaled Base "store a word, unscaled offset";
    0xffe00c00 0xb8400000 LdurW   "ldur"   LdStUnscaled Base "load a word, unscaled offset";
    0xffe00c00 0xb8800000 Ldursw  "ldursw" LdStUnscaled Base "load a word sign-extended to 64 bits, unscaled offset";
    0xffe00c00 0xf8000000 SturX   "stur"   LdStUnscaled Base "store a doubleword, unscaled offset";
    0xffe00c00 0xf8400000 LdurX   "ldur"   LdStUnscaled Base "load a doubleword, unscaled offset";
    0xffe00c00 0xf8800000 Prfum   "prfum"  LdStUnscaled Base "prefetch memory, unscaled offset";

    // -- Loads and stores: register, post-indexed ---------------------------
    0xffe00c00 0x38000400 StrbPost   "strb"  LdStPost Base "store a byte, post-indexed";
    0xffe00c00 0x38400400 LdrbPost   "ldrb"  LdStPost Base "load a zero-extended byte, post-indexed";
    0xffe00c00 0x38800400 LdrsbXPost "ldrsb" LdStPost Base "load a byte sign-extended to 64 bits, post-indexed";
    0xffe00c00 0x38c00400 LdrsbWPost "ldrsb" LdStPost Base "load a byte sign-extended to 32 bits, post-indexed";
    0xffe00c00 0x78000400 StrhPost   "strh"  LdStPost Base "store a halfword, post-indexed";
    0xffe00c00 0x78400400 LdrhPost   "ldrh"  LdStPost Base "load a zero-extended halfword, post-indexed";
    0xffe00c00 0x78800400 LdrshXPost "ldrsh" LdStPost Base "load a halfword sign-extended to 64 bits, post-indexed";
    0xffe00c00 0x78c00400 LdrshWPost "ldrsh" LdStPost Base "load a halfword sign-extended to 32 bits, post-indexed";
    0xffe00c00 0xb8000400 StrWPost   "str"   LdStPost Base "store a word, post-indexed";
    0xffe00c00 0xb8400400 LdrWPost   "ldr"   LdStPost Base "load a word, post-indexed";
    0xffe00c00 0xb8800400 LdrswPost  "ldrsw" LdStPost Base "load a word sign-extended to 64 bits, post-indexed";
    0xffe00c00 0xf8000400 StrXPost   "str"   LdStPost Base "store a doubleword, post-indexed";
    0xffe00c00 0xf8400400 LdrXPost   "ldr"   LdStPost Base "load a doubleword, post-indexed";

    // -- Loads and stores: register, pre-indexed ----------------------------
    0xffe00c00 0x38000c00 StrbPre   "strb"  LdStPre Base "store a byte, pre-indexed";
    0xffe00c00 0x38400c00 LdrbPre   "ldrb"  LdStPre Base "load a zero-extended byte, pre-indexed";
    0xffe00c00 0x38800c00 LdrsbXPre "ldrsb" LdStPre Base "load a byte sign-extended to 64 bits, pre-indexed";
    0xffe00c00 0x38c00c00 LdrsbWPre "ldrsb" LdStPre Base "load a byte sign-extended to 32 bits, pre-indexed";
    0xffe00c00 0x78000c00 StrhPre   "strh"  LdStPre Base "store a halfword, pre-indexed";
    0xffe00c00 0x78400c00 LdrhPre   "ldrh"  LdStPre Base "load a zero-extended halfword, pre-indexed";
    0xffe00c00 0x78800c00 LdrshXPre "ldrsh" LdStPre Base "load a halfword sign-extended to 64 bits, pre-indexed";
    0xffe00c00 0x78c00c00 LdrshWPre "ldrsh" LdStPre Base "load a halfword sign-extended to 32 bits, pre-indexed";
    0xffe00c00 0xb8000c00 StrWPre   "str"   LdStPre Base "store a word, pre-indexed";
    0xffe00c00 0xb8400c00 LdrWPre   "ldr"   LdStPre Base "load a word, pre-indexed";
    0xffe00c00 0xb8800c00 LdrswPre  "ldrsw" LdStPre Base "load a word sign-extended to 64 bits, pre-indexed";
    0xffe00c00 0xf8000c00 StrXPre   "str"   LdStPre Base "store a doubleword, pre-indexed";
    0xffe00c00 0xf8400c00 LdrXPre   "ldr"   LdStPre Base "load a doubleword, pre-indexed";

    // -- Loads and stores: register offset ----------------------------------
    0xffe00c00 0x38200800 StrbReg   "strb"  LdStRegOff Base "store a byte at a register offset";
    0xffe00c00 0x38600800 LdrbReg   "ldrb"  LdStRegOff Base "load a zero-extended byte at a register offset";
    0xffe00c00 0x38a00800 LdrsbXReg "ldrsb" LdStRegOff Base "load a byte sign-extended to 64 bits at a register offset";
    0xffe00c00 0x38e00800 LdrsbWReg "ldrsb" LdStRegOff Base "load a byte sign-extended to 32 bits at a register offset";
    0xffe00c00 0x78200800 StrhReg   "strh"  LdStRegOff Base "store a halfword at a register offset";
    0xffe00c00 0x78600800 LdrhReg   "ldrh"  LdStRegOff Base "load a zero-extended halfword at a register offset";
    0xffe00c00 0x78a00800 LdrshXReg "ldrsh" LdStRegOff Base "load a halfword sign-extended to 64 bits at a register offset";
    0xffe00c00 0x78e00800 LdrshWReg "ldrsh" LdStRegOff Base "load a halfword sign-extended to 32 bits at a register offset";
    0xffe00c00 0xb8200800 StrWReg   "str"   LdStRegOff Base "store a word at a register offset";
    0xffe00c00 0xb8600800 LdrWReg   "ldr"   LdStRegOff Base "load a word at a register offset";
    0xffe00c00 0xb8a00800 LdrswReg  "ldrsw" LdStRegOff Base "load a word sign-extended to 64 bits at a register offset";
    0xffe00c00 0xf8200800 StrXReg   "str"   LdStRegOff Base "store a doubleword at a register offset";
    0xffe00c00 0xf8600800 LdrXReg   "ldr"   LdStRegOff Base "load a doubleword at a register offset";
    0xffe00c00 0xf8a00800 PrfmReg   "prfm"  LdStRegOff Base "prefetch memory at a register offset";

    // -- Loads and stores: pairs --------------------------------------------
    // `LDNP`/`STNP` are the plain offset form with a *hint* attached: the
    // access is non-temporal, meaning the data is unlikely to be reused soon
    // and the caches need not keep it. There is no cache in this core, so the
    // hint has no effect and these behave exactly as `LDP`/`STP` — which is
    // architecturally correct rather than a shortcut, since the hint changes
    // no architectural state on any implementation.
    //
    // `opc == 0b01` has no row here on purpose: the signed-word `LDPSW` has no
    // non-temporal counterpart, so that encoding is unallocated and must
    // `UNDEF`. Fixing `opc` in these masks is what makes that true without a
    // check in the interpreter.
    0xffc00000 0x28000000 StnpW     "stnp"  LdStPairOff  Base "store a pair of words, non-temporal";
    0xffc00000 0x28400000 LdnpW     "ldnp"  LdStPairOff  Base "load a pair of words, non-temporal";
    0xffc00000 0xa8000000 StnpX     "stnp"  LdStPairOff  Base "store a pair of doublewords, non-temporal";
    0xffc00000 0xa8400000 LdnpX     "ldnp"  LdStPairOff  Base "load a pair of doublewords, non-temporal";
    0xffc00000 0x28800000 StpWPost  "stp"   LdStPairPost Base "store a pair of words, post-indexed";
    0xffc00000 0x28c00000 LdpWPost  "ldp"   LdStPairPost Base "load a pair of words, post-indexed";
    0xffc00000 0x29000000 StpWOff   "stp"   LdStPairOff  Base "store a pair of words";
    0xffc00000 0x29400000 LdpWOff   "ldp"   LdStPairOff  Base "load a pair of words";
    0xffc00000 0x29800000 StpWPre   "stp"   LdStPairPre  Base "store a pair of words, pre-indexed";
    0xffc00000 0x29c00000 LdpWPre   "ldp"   LdStPairPre  Base "load a pair of words, pre-indexed";
    0xffc00000 0x68c00000 LdpswPost "ldpsw" LdStPairPost Base "load a pair of sign-extended words, post-indexed";
    0xffc00000 0x69400000 LdpswOff  "ldpsw" LdStPairOff  Base "load a pair of sign-extended words";
    0xffc00000 0x69c00000 LdpswPre  "ldpsw" LdStPairPre  Base "load a pair of sign-extended words, pre-indexed";
    0xffc00000 0xa8800000 StpXPost  "stp"   LdStPairPost Base "store a pair of doublewords, post-indexed";
    0xffc00000 0xa8c00000 LdpXPost  "ldp"   LdStPairPost Base "load a pair of doublewords, post-indexed";
    0xffc00000 0xa9000000 StpXOff   "stp"   LdStPairOff  Base "store a pair of doublewords";
    0xffc00000 0xa9400000 LdpXOff   "ldp"   LdStPairOff  Base "load a pair of doublewords";
    0xffc00000 0xa9800000 StpXPre   "stp"   LdStPairPre  Base "store a pair of doublewords, pre-indexed";
    0xffc00000 0xa9c00000 LdpXPre   "ldp"   LdStPairPre  Base "load a pair of doublewords, pre-indexed";

    // -- Loads and stores: exclusives and acquire/release -------------------
    0xffe08000 0x08000000 Stxrb  "stxrb"  StoreExclusive Base "store a byte exclusively";
    0xffe08000 0x08008000 Stlxrb "stlxrb" StoreExclusive Base "store a byte exclusively, with release";
    0xffe08000 0x08400000 Ldxrb  "ldxrb"  LdStExclusive  Base "load a byte exclusively";
    0xffe08000 0x08408000 Ldaxrb "ldaxrb" LdStExclusive  Base "load a byte exclusively, with acquire";
    0xffe08000 0x08808000 Stlrb  "stlrb"  LdStExclusive  Base "store a byte with release";
    0xffe08000 0x08c08000 Ldarb  "ldarb"  LdStExclusive  Base "load a byte with acquire";
    0xffe08000 0x48000000 Stxrh  "stxrh"  StoreExclusive Base "store a halfword exclusively";
    0xffe08000 0x48008000 Stlxrh "stlxrh" StoreExclusive Base "store a halfword exclusively, with release";
    0xffe08000 0x48400000 Ldxrh  "ldxrh"  LdStExclusive  Base "load a halfword exclusively";
    0xffe08000 0x48408000 Ldaxrh "ldaxrh" LdStExclusive  Base "load a halfword exclusively, with acquire";
    0xffe08000 0x48808000 Stlrh  "stlrh"  LdStExclusive  Base "store a halfword with release";
    0xffe08000 0x48c08000 Ldarh  "ldarh"  LdStExclusive  Base "load a halfword with acquire";
    0xffe08000 0x88000000 StxrW  "stxr"   StoreExclusive Base "store a word exclusively";
    0xffe08000 0x88008000 StlxrW "stlxr"  StoreExclusive Base "store a word exclusively, with release";
    0xffe08000 0x88400000 LdxrW  "ldxr"   LdStExclusive  Base "load a word exclusively";
    0xffe08000 0x88408000 LdaxrW "ldaxr"  LdStExclusive  Base "load a word exclusively, with acquire";
    0xffe08000 0x88808000 StlrW  "stlr"   LdStExclusive  Base "store a word with release";
    0xffe08000 0x88c08000 LdarW  "ldar"   LdStExclusive  Base "load a word with acquire";
    0xffe08000 0xc8000000 StxrX  "stxr"   StoreExclusive Base "store a doubleword exclusively";
    0xffe08000 0xc8008000 StlxrX "stlxr"  StoreExclusive Base "store a doubleword exclusively, with release";
    0xffe08000 0xc8400000 LdxrX  "ldxr"   LdStExclusive  Base "load a doubleword exclusively";
    0xffe08000 0xc8408000 LdaxrX "ldaxr"  LdStExclusive  Base "load a doubleword exclusively, with acquire";
    0xffe08000 0xc8808000 StlrX  "stlr"   LdStExclusive  Base "store a doubleword with release";
    0xffe08000 0xc8c08000 LdarX  "ldar"   LdStExclusive  Base "load a doubleword with acquire";

    // -- FEAT_LSE: compare-and-swap and the atomic read-modify-writes -------
    //
    // The acquire and release bits are left free in these masks and printed by
    // the format (see `Fmt::suffix_kind`): four rows per operation would be a
    // table describing the encoding rather than the instruction set.
    0xffa07c00 0x88a07c00 CasW    "cas"    Atomic Lse "compare and swap a word";
    0xffa07c00 0xc8a07c00 CasX    "cas"    Atomic Lse "compare and swap a doubleword";
    0xff20fc00 0xb8200000 LdaddW  "ldadd"  Atomic Lse "atomic add on a word";
    0xff20fc00 0xf8200000 LdaddX  "ldadd"  Atomic Lse "atomic add on a doubleword";
    0xff20fc00 0xb8201000 LdclrW  "ldclr"  Atomic Lse "atomic bit clear on a word";
    0xff20fc00 0xf8201000 LdclrX  "ldclr"  Atomic Lse "atomic bit clear on a doubleword";
    0xff20fc00 0xb8202000 LdeorW  "ldeor"  Atomic Lse "atomic exclusive-OR on a word";
    0xff20fc00 0xf8202000 LdeorX  "ldeor"  Atomic Lse "atomic exclusive-OR on a doubleword";
    0xff20fc00 0xb8203000 LdsetW  "ldset"  Atomic Lse "atomic bit set on a word";
    0xff20fc00 0xf8203000 LdsetX  "ldset"  Atomic Lse "atomic bit set on a doubleword";
    0xff20fc00 0xb8204000 LdsmaxW "ldsmax" Atomic Lse "atomic signed maximum on a word";
    0xff20fc00 0xf8204000 LdsmaxX "ldsmax" Atomic Lse "atomic signed maximum on a doubleword";
    0xff20fc00 0xb8205000 LdsminW "ldsmin" Atomic Lse "atomic signed minimum on a word";
    0xff20fc00 0xf8205000 LdsminX "ldsmin" Atomic Lse "atomic signed minimum on a doubleword";
    0xff20fc00 0xb8206000 LdumaxW "ldumax" Atomic Lse "atomic unsigned maximum on a word";
    0xff20fc00 0xf8206000 LdumaxX "ldumax" Atomic Lse "atomic unsigned maximum on a doubleword";
    0xff20fc00 0xb8207000 LduminW "ldumin" Atomic Lse "atomic unsigned minimum on a word";
    0xff20fc00 0xf8207000 LduminX "ldumin" Atomic Lse "atomic unsigned minimum on a doubleword";
    0xff20fc00 0xb8208000 SwpW    "swp"    Atomic Lse "atomic swap of a word";
    0xff20fc00 0xf8208000 SwpX    "swp"    Atomic Lse "atomic swap of a doubleword";

    // -- Data processing (register): logical, shifted -----------------------
    0x7f200000 0x0a000000 AndShift  "and"  ShiftedReg Base "bitwise AND with a shifted register";
    0x7f200000 0x0a200000 BicShift  "bic"  ShiftedReg Base "bitwise AND with an inverted shifted register";
    0x7f200000 0x2a000000 OrrShift  "orr"  ShiftedReg Base "bitwise OR with a shifted register";
    0x7f200000 0x2a200000 OrnShift  "orn"  ShiftedReg Base "bitwise OR with an inverted shifted register";
    0x7f200000 0x4a000000 EorShift  "eor"  ShiftedReg Base "bitwise exclusive-OR with a shifted register";
    0x7f200000 0x4a200000 EonShift  "eon"  ShiftedReg Base "bitwise exclusive-OR with an inverted shifted register";
    0x7f200000 0x6a000000 AndsShift "ands" ShiftedReg Base "bitwise AND with a shifted register, setting the flags";
    0x7f200000 0x6a200000 BicsShift "bics" ShiftedReg Base "bitwise AND with an inverted shifted register, setting the flags";

    // -- Data processing (register): add/subtract ---------------------------
    0x7f200000 0x0b000000 AddShift  "add"  ShiftedReg Base "add a shifted register";
    0x7f200000 0x2b000000 AddsShift "adds" ShiftedReg Base "add a shifted register, setting the flags";
    0x7f200000 0x4b000000 SubShift  "sub"  ShiftedReg Base "subtract a shifted register";
    0x7f200000 0x6b000000 SubsShift "subs" ShiftedReg Base "subtract a shifted register, setting the flags";
    0x7fe00000 0x0b200000 AddExt    "add"  AddSubExt  Base "add an extended register";
    0x7fe00000 0x2b200000 AddsExt   "adds" AddSubExtS Base "add an extended register, setting the flags";
    0x7fe00000 0x4b200000 SubExt    "sub"  AddSubExt  Base "subtract an extended register";
    0x7fe00000 0x6b200000 SubsExt   "subs" AddSubExtS Base "subtract an extended register, setting the flags";
    0x7fe0fc00 0x1a000000 Adc       "adc"  ThreeReg   Base "add with carry";
    0x7fe0fc00 0x3a000000 Adcs      "adcs" ThreeReg   Base "add with carry, setting the flags";
    0x7fe0fc00 0x5a000000 Sbc       "sbc"  ThreeReg   Base "subtract with carry";
    0x7fe0fc00 0x7a000000 Sbcs      "sbcs" ThreeReg   Base "subtract with carry, setting the flags";

    // -- Data processing (register): conditional ----------------------------
    0x7fe00c10 0x3a400000 CcmnReg "ccmn"  CondCmpReg Base "conditionally compare negative, register";
    0x7fe00c10 0x7a400000 CcmpReg "ccmp"  CondCmpReg Base "conditionally compare, register";
    0x7fe00c10 0x3a400800 CcmnImm "ccmn"  CondCmpImm Base "conditionally compare negative, immediate";
    0x7fe00c10 0x7a400800 CcmpImm "ccmp"  CondCmpImm Base "conditionally compare, immediate";
    0x7fe00c00 0x1a800000 Csel    "csel"  CondSel    Base "conditional select";
    0x7fe00c00 0x1a800400 Csinc   "csinc" CondSel    Base "conditional select, incrementing the alternative";
    0x7fe00c00 0x5a800000 Csinv   "csinv" CondSel    Base "conditional select, inverting the alternative";
    0x7fe00c00 0x5a800400 Csneg   "csneg" CondSel    Base "conditional select, negating the alternative";

    // -- Data processing (register): two-source -----------------------------
    0x7fe0fc00 0x1ac00800 Udiv    "udiv"    ThreeReg Base  "unsigned divide";
    0x7fe0fc00 0x1ac00c00 Sdiv    "sdiv"    ThreeReg Base  "signed divide";
    0x7fe0fc00 0x1ac02000 Lslv    "lslv"    ThreeReg Base  "logical shift left by a register";
    0x7fe0fc00 0x1ac02400 Lsrv    "lsrv"    ThreeReg Base  "logical shift right by a register";
    0x7fe0fc00 0x1ac02800 Asrv    "asrv"    ThreeReg Base  "arithmetic shift right by a register";
    0x7fe0fc00 0x1ac02c00 Rorv    "rorv"    ThreeReg Base  "rotate right by a register";
    0xffe0fc00 0x1ac04000 Crc32b  "crc32b"  CrcReg   Crc32 "CRC-32 checksum over a byte";
    0xffe0fc00 0x1ac04400 Crc32h  "crc32h"  CrcReg   Crc32 "CRC-32 checksum over a halfword";
    0xffe0fc00 0x1ac04800 Crc32w  "crc32w"  CrcReg   Crc32 "CRC-32 checksum over a word";
    0xffe0fc00 0x9ac04c00 Crc32x  "crc32x"  CrcReg   Crc32 "CRC-32 checksum over a doubleword";
    0xffe0fc00 0x1ac05000 Crc32cb "crc32cb" CrcReg   Crc32 "CRC-32C checksum over a byte";
    0xffe0fc00 0x1ac05400 Crc32ch "crc32ch" CrcReg   Crc32 "CRC-32C checksum over a halfword";
    0xffe0fc00 0x1ac05800 Crc32cw "crc32cw" CrcReg   Crc32 "CRC-32C checksum over a word";
    0xffe0fc00 0x9ac05c00 Crc32cx "crc32cx" CrcReg   Crc32 "CRC-32C checksum over a doubleword";

    // -- Data processing (register): one-source -----------------------------
    0x7ffffc00 0x5ac00000 Rbit  "rbit"  TwoReg Base "reverse the bit order";
    0x7ffffc00 0x5ac00400 Rev16 "rev16" TwoReg Base "reverse the bytes in each halfword";
    0xfffffc00 0x5ac00800 RevW  "rev"   TwoReg Base "reverse the bytes of a word";
    0xfffffc00 0xdac00800 Rev32 "rev32" TwoReg Base "reverse the bytes in each word";
    0xfffffc00 0xdac00c00 RevX  "rev"   TwoReg Base "reverse the bytes of a doubleword";
    0x7ffffc00 0x5ac01000 Clz   "clz"   TwoReg Base "count leading zeroes";
    0x7ffffc00 0x5ac01400 Cls   "cls"   TwoReg Base "count leading sign bits";

    // -- Data processing (register): three-source ---------------------------
    0x7fe08000 0x1b000000 Madd   "madd"   FourReg  Base "multiply and add";
    0x7fe08000 0x1b008000 Msub   "msub"   FourReg  Base "multiply and subtract";
    0xffe08000 0x9b200000 Smaddl "smaddl" FourReg  Base "signed multiply-add of two words into a doubleword";
    0xffe08000 0x9b208000 Smsubl "smsubl" FourReg  Base "signed multiply-subtract of two words from a doubleword";
    0xffe08000 0x9ba00000 Umaddl "umaddl" FourReg  Base "unsigned multiply-add of two words into a doubleword";
    0xffe08000 0x9ba08000 Umsubl "umsubl" FourReg  Base "unsigned multiply-subtract of two words from a doubleword";
    0xffe0fc00 0x9b407c00 Smulh  "smulh"  ThreeReg Base "signed multiply, high half";
    0xffe0fc00 0x9bc07c00 Umulh  "umulh"  ThreeReg Base "unsigned multiply, high half";
    // -- Scalar floating point: data processing (1 source) -----------------
    //
    // `ptype` (bits 23:22) is deliberately *not* fixed by these masks: it
    // names the precision, and one row per precision would be three rows
    // describing the same instruction. The interpreter reads it with
    // `fp::Prec::from_ptype`, which rejects the unallocated `0b10` and — with
    // no `FEAT_FP16` here — the arithmetic uses of `0b11`.
    0xff3ffc00 0x1e204000 Fmov   "fmov"   FpOneSrc Fp "move a floating-point register";
    0xff3ffc00 0x1e20c000 Fabs   "fabs"   FpOneSrc Fp "floating-point absolute value";
    0xff3ffc00 0x1e214000 Fneg   "fneg"   FpOneSrc Fp "floating-point negate";
    0xff3ffc00 0x1e21c000 Fsqrt  "fsqrt"  FpOneSrc Fp "floating-point square root";
    0xff3e7c00 0x1e224000 Fcvt   "fcvt"   FpCvt    Fp "convert between floating-point precisions";
    0xff3ffc00 0x1e244000 Frintn "frintn" FpOneSrc Fp "round to an integral value, ties to even";
    0xff3ffc00 0x1e24c000 Frintp "frintp" FpOneSrc Fp "round to an integral value, toward +infinity";
    0xff3ffc00 0x1e254000 Frintm "frintm" FpOneSrc Fp "round to an integral value, toward -infinity";
    0xff3ffc00 0x1e25c000 Frintz "frintz" FpOneSrc Fp "round to an integral value, toward zero";
    0xff3ffc00 0x1e264000 Frinta "frinta" FpOneSrc Fp "round to an integral value, ties away from zero";
    0xff3ffc00 0x1e274000 Frintx "frintx" FpOneSrc Fp "round to an integral value, current mode, signalling inexact";
    0xff3ffc00 0x1e27c000 Frinti "frinti" FpOneSrc Fp "round to an integral value, current mode";

    // -- Scalar floating point: data processing (2 source) -----------------
    0xff20fc00 0x1e200800 Fmul   "fmul"   FpTwoSrc Fp "floating-point multiply";
    0xff20fc00 0x1e201800 Fdiv   "fdiv"   FpTwoSrc Fp "floating-point divide";
    0xff20fc00 0x1e202800 Fadd   "fadd"   FpTwoSrc Fp "floating-point add";
    0xff20fc00 0x1e203800 Fsub   "fsub"   FpTwoSrc Fp "floating-point subtract";
    0xff20fc00 0x1e204800 Fmax   "fmax"   FpTwoSrc Fp "floating-point maximum, propagating a NaN";
    0xff20fc00 0x1e205800 Fmin   "fmin"   FpTwoSrc Fp "floating-point minimum, propagating a NaN";
    0xff20fc00 0x1e206800 Fmaxnm "fmaxnm" FpTwoSrc Fp "floating-point maximum, preferring a number to a quiet NaN";
    0xff20fc00 0x1e207800 Fminnm "fminnm" FpTwoSrc Fp "floating-point minimum, preferring a number to a quiet NaN";
    0xff20fc00 0x1e208800 Fnmul  "fnmul"  FpTwoSrc Fp "floating-point multiply, negating the result";

    // -- Scalar floating point: data processing (3 source) -----------------
    0xff208000 0x1f000000 Fmadd  "fmadd"  FpThreeSrc Fp "fused multiply-add";
    0xff208000 0x1f008000 Fmsub  "fmsub"  FpThreeSrc Fp "fused multiply-subtract";
    0xff208000 0x1f200000 Fnmadd "fnmadd" FpThreeSrc Fp "fused negated multiply-add";
    0xff208000 0x1f208000 Fnmsub "fnmsub" FpThreeSrc Fp "fused negated multiply-subtract";

    // -- Scalar floating point: compare ------------------------------------
    0xff20fc1f 0x1e202000 Fcmp      "fcmp"  FpCmp Fp "floating-point compare, quiet";
    0xff20fc1f 0x1e202008 FcmpZero  "fcmp"  FpCmp Fp "floating-point compare with zero, quiet";
    0xff20fc1f 0x1e202010 Fcmpe     "fcmpe" FpCmp Fp "floating-point compare, signalling on any NaN";
    0xff20fc1f 0x1e202018 FcmpeZero "fcmpe" FpCmp Fp "floating-point compare with zero, signalling on any NaN";

    // -- Scalar floating point: conditional --------------------------------
    0xff200c10 0x1e200400 Fccmp  "fccmp"  FpCondCmp Fp "conditional floating-point compare, quiet";
    0xff200c10 0x1e200410 Fccmpe "fccmpe" FpCondCmp Fp "conditional floating-point compare, signalling on any NaN";
    0xff200c00 0x1e200c00 Fcsel  "fcsel"  FpCondSel Fp "floating-point conditional select";
    0xff201fe0 0x1e201000 FmovImm "fmov"  FpImm     Fp "move an expanded 8-bit floating-point immediate";

    // -- Conversion between floating point and a general register ----------
    //
    // `sf` (bit 31) and `ptype` are both free here for the same reason as
    // above: they name the integer and floating-point widths, and every valid
    // combination is one instruction with one behaviour. The interpreter
    // rejects the combinations DDI 0487 leaves unallocated.
    0x7f3ffc00 0x1e200000 Fcvtns   "fcvtns" FpIntCvt Fp "convert to a signed integer, ties to even";
    0x7f3ffc00 0x1e210000 Fcvtnu   "fcvtnu" FpIntCvt Fp "convert to an unsigned integer, ties to even";
    0x7f3ffc00 0x1e220000 Scvtf    "scvtf"  FpIntCvt Fp "convert a signed integer to floating point";
    0x7f3ffc00 0x1e230000 Ucvtf    "ucvtf"  FpIntCvt Fp "convert an unsigned integer to floating point";
    0x7f3ffc00 0x1e240000 Fcvtas   "fcvtas" FpIntCvt Fp "convert to a signed integer, ties away from zero";
    0x7f3ffc00 0x1e250000 Fcvtau   "fcvtau" FpIntCvt Fp "convert to an unsigned integer, ties away from zero";
    0x7f3ffc00 0x1e260000 FmovToGp "fmov"   FpIntCvt Fp "move a floating-point register to a general one";
    0x7f3ffc00 0x1e270000 FmovToFp "fmov"   FpIntCvt Fp "move a general register to a floating-point one";
    0x7f3ffc00 0x1e280000 Fcvtps   "fcvtps" FpIntCvt Fp "convert to a signed integer, toward +infinity";
    0x7f3ffc00 0x1e290000 Fcvtpu   "fcvtpu" FpIntCvt Fp "convert to an unsigned integer, toward +infinity";
    0x7f3ffc00 0x1e2e0000 FmovHiToGp "fmov" FpIntCvt Fp "move the top half of a vector register to a general one";
    0x7f3ffc00 0x1e2f0000 FmovGpToHi "fmov" FpIntCvt Fp "move a general register into the top half of a vector one";
    0x7f3ffc00 0x1e300000 Fcvtms   "fcvtms" FpIntCvt Fp "convert to a signed integer, toward -infinity";
    0x7f3ffc00 0x1e310000 Fcvtmu   "fcvtmu" FpIntCvt Fp "convert to an unsigned integer, toward -infinity";
    0x7f3ffc00 0x1e380000 Fcvtzs   "fcvtzs" FpIntCvt Fp "convert to a signed integer, toward zero";
    0x7f3ffc00 0x1e390000 Fcvtzu   "fcvtzu" FpIntCvt Fp "convert to an unsigned integer, toward zero";

    // -- Conversion with a fixed-point scale -------------------------------
    0x7f3f0000 0x1e020000 ScvtfFix  "scvtf"  FpFixCvt Fp "convert a signed fixed-point value to floating point";
    0x7f3f0000 0x1e030000 UcvtfFix  "ucvtf"  FpFixCvt Fp "convert an unsigned fixed-point value to floating point";
    0x7f3f0000 0x1e180000 FcvtzsFix "fcvtzs" FpFixCvt Fp "convert to a signed fixed-point value, toward zero";
    0x7f3f0000 0x1e190000 FcvtzuFix "fcvtzu" FpFixCvt Fp "convert to an unsigned fixed-point value, toward zero";

    // -- SIMD&FP loads and stores ------------------------------------------
    //
    // One row per direction rather than one per width: `LDR B0`, `LDR H0`,
    // `LDR S0`, `LDR D0` and `LDR Q0` are all spelled **`ldr`**, with the
    // width in the register name, so five rows would name one instruction
    // five times. Bit 22 is the load/store bit and is fixed; `size` (31:30)
    // and `opc<1>` (bit 23) together give the width and stay free.
    0x3f000000 0x1c000000 LdrLitV  "ldr"  LoadFpLiteral  Fp "load a SIMD&FP register from a PC-relative literal";
    0x3f400000 0x3d000000 StrVImm  "str"  LdStFpUImm     Fp "store a SIMD&FP register";
    0x3f400000 0x3d400000 LdrVImm  "ldr"  LdStFpUImm     Fp "load a SIMD&FP register";
    0x3f600c00 0x3c000000 SturV    "stur" LdStFpUnscaled Fp "store a SIMD&FP register, unscaled offset";
    0x3f600c00 0x3c400000 LdurV    "ldur" LdStFpUnscaled Fp "load a SIMD&FP register, unscaled offset";
    0x3f600c00 0x3c000400 StrVPost "str"  LdStFpPost     Fp "store a SIMD&FP register, post-indexed";
    0x3f600c00 0x3c400400 LdrVPost "ldr"  LdStFpPost     Fp "load a SIMD&FP register, post-indexed";
    0x3f600c00 0x3c000c00 StrVPre  "str"  LdStFpPre      Fp "store a SIMD&FP register, pre-indexed";
    0x3f600c00 0x3c400c00 LdrVPre  "ldr"  LdStFpPre      Fp "load a SIMD&FP register, pre-indexed";
    0x3f600c00 0x3c200800 StrVReg  "str"  LdStFpRegOff   Fp "store a SIMD&FP register, register offset";
    0x3f600c00 0x3c600800 LdrVReg  "ldr"  LdStFpRegOff   Fp "load a SIMD&FP register, register offset";
    0x3fc00000 0x2c000000 StnpV    "stnp" LdStFpPairOff  Fp "store a pair of SIMD&FP registers, non-temporal";
    0x3fc00000 0x2c400000 LdnpV    "ldnp" LdStFpPairOff  Fp "load a pair of SIMD&FP registers, non-temporal";
    0x3fc00000 0x2c800000 StpVPost "stp"  LdStFpPairPost Fp "store a pair of SIMD&FP registers, post-indexed";
    0x3fc00000 0x2cc00000 LdpVPost "ldp"  LdStFpPairPost Fp "load a pair of SIMD&FP registers, post-indexed";
    0x3fc00000 0x2d000000 StpVOff  "stp"  LdStFpPairOff  Fp "store a pair of SIMD&FP registers";
    0x3fc00000 0x2d400000 LdpVOff  "ldp"  LdStFpPairOff  Fp "load a pair of SIMD&FP registers";
    0x3fc00000 0x2d800000 StpVPre  "stp"  LdStFpPairPre  Fp "store a pair of SIMD&FP registers, pre-indexed";
    0x3fc00000 0x2dc00000 LdpVPre  "ldp"  LdStFpPairPre  Fp "load a pair of SIMD&FP registers, pre-indexed";
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

/// How many rows may share one top-level bucket.
///
/// A compile-time bound: [`build_index`] panics during const evaluation if a
/// bucket overflows it, which is a build break rather than a silently
/// truncated decode table. `the_busiest_decode_bucket_has_headroom` reports
/// the margin so that break never arrives as a surprise — the scalar
/// floating-point rows took the busiest bucket from 46 to 90, which is why
/// this is no longer 96.
const BUCKET_CAP: usize = 160;

/// One bucket of [`INDEX`]: the rows whose fixed bits are compatible with one
/// value of the top-level `op0` field.
#[derive(Debug, Clone, Copy)]
pub struct Bucket {
    /// How many entries of `rows` are used.
    len: u16,
    /// Indices into [`TABLE`], in table order.
    rows: [u16; BUCKET_CAP],
}

/// The rows to scan for each value of the top-level `op0` field, bits 28:25.
///
/// DDI 0487 C4.1 classifies every A64 encoding on those four bits, so they are
/// the natural first cut. A row whose mask leaves one of them free — `B`,
/// whose 26-bit displacement reaches bit 25 — appears in every bucket it is
/// compatible with, so the index cannot hide an instruction the table
/// declares. Built by a `const fn` from `TABLE` itself, so it is a derived
/// cache in the strict sense: it cannot disagree with what it indexes, and
/// adding a row needs no second edit.
pub static INDEX: [Bucket; 16] = build_index(TABLE);

/// Compute [`INDEX`] at compile time.
const fn build_index(table: &[Insn]) -> [Bucket; 16] {
    let mut index = [Bucket {
        len: 0,
        rows: [0; BUCKET_CAP],
    }; 16];
    let mut op0 = 0usize;
    while op0 < 16 {
        let probe = (op0 as u32) << 25;
        let mut i = 0usize;
        while i < table.len() {
            let row = &table[i];
            // Compatible when every bit of 28:25 the row fixes agrees.
            if (probe ^ row.bits) & row.mask & 0x1e00_0000 == 0 {
                let n = index[op0].len as usize;
                assert!(n < BUCKET_CAP, "raise BUCKET_CAP: a decode bucket is full");
                index[op0].rows[n] = i as u16;
                index[op0].len += 1;
            }
            i += 1;
        }
        op0 += 1;
    }
    index
}

/// Decode a 32-bit A64 instruction word.
///
/// `None` is UNDEFINED, which covers both an encoding nothing allocates and
/// one whose feature `features` lacks — a guest cannot tell the two apart, and
/// neither may we: `ROADMAP.md` §6.1.1 makes probing by `UNDEF` the way a
/// guest discovers what it is running on.
#[must_use]
pub fn decode(word: u32, features: Features) -> Option<&'static Insn> {
    let bucket = &INDEX[field(word, 28, 25) as usize];
    let mut i = 0usize;
    while i < bucket.len as usize {
        let insn = &TABLE[bucket.rows[i] as usize];
        if word & insn.mask == insn.bits && features.has(insn.feat) {
            return Some(insn);
        }
        i += 1;
    }
    None
}

// ---------------------------------------------------------------------------
// Field extraction
// ---------------------------------------------------------------------------

/// The destination register field, bits 4:0. Also `Rt` on a load or store.
#[inline]
#[must_use]
pub const fn rd(word: u32) -> u32 {
    word & 31
}

/// The first source register field, bits 9:5. Also the base register of a
/// load or store.
#[inline]
#[must_use]
pub const fn rn(word: u32) -> u32 {
    (word >> 5) & 31
}

/// The second source register field, bits 20:16. Also `Rs` on an exclusive or
/// an atomic.
#[inline]
#[must_use]
pub const fn rm(word: u32) -> u32 {
    (word >> 16) & 31
}

/// The accumulator register field, bits 14:10. Also `Rt2` on a pair.
#[inline]
#[must_use]
pub const fn ra(word: u32) -> u32 {
    (word >> 10) & 31
}

/// The `sf` bit: set for 64-bit operands, clear for 32-bit.
#[inline]
#[must_use]
pub const fn sf(word: u32) -> bool {
    word & (1 << 31) != 0
}

/// The operand width in bits the `sf` bit selects.
#[inline]
#[must_use]
pub const fn datasize(word: u32) -> u32 {
    if sf(word) { 64 } else { 32 }
}

/// The condition field of `CSEL`, `CCMP` and relatives, bits 15:12.
#[inline]
#[must_use]
pub const fn cond_hi(word: u32) -> Cond {
    Cond(field(word, 15, 12) as u8)
}

/// The condition field of `B.cond`, bits 3:0.
#[inline]
#[must_use]
pub const fn cond_lo(word: u32) -> Cond {
    Cond((word & 0xf) as u8)
}

/// Sign-extend the low `bits` of `value`.
#[inline]
#[must_use]
pub const fn sext(value: u64, bits: u32) -> i64 {
    let shift = 64 - bits;
    ((value << shift) as i64) >> shift
}

/// The 26-bit branch displacement, in bytes.
#[inline]
#[must_use]
pub const fn imm26(word: u32) -> i64 {
    sext((word & 0x03ff_ffff) as u64, 26) * 4
}

/// The 19-bit branch or literal displacement, in bytes.
#[inline]
#[must_use]
pub const fn imm19(word: u32) -> i64 {
    sext(field(word, 23, 5) as u64, 19) * 4
}

/// The 14-bit `TBZ`/`TBNZ` displacement, in bytes.
#[inline]
#[must_use]
pub const fn imm14(word: u32) -> i64 {
    sext(field(word, 18, 5) as u64, 14) * 4
}

/// The 16-bit immediate of `MOVZ` and the exception-generating instructions.
#[inline]
#[must_use]
pub const fn imm16(word: u32) -> u32 {
    field(word, 20, 5)
}

/// The 12-bit unsigned immediate of the add/subtract and unsigned-offset
/// load/store encodings.
#[inline]
#[must_use]
pub const fn imm12(word: u32) -> u32 {
    field(word, 21, 10)
}

/// The 9-bit signed offset of the unscaled and indexed load/store encodings.
#[inline]
#[must_use]
pub const fn imm9(word: u32) -> i64 {
    sext(field(word, 20, 12) as u64, 9)
}

/// The 7-bit signed offset of a load/store pair, in units of the access size.
#[inline]
#[must_use]
pub const fn imm7(word: u32) -> i64 {
    sext(field(word, 21, 15) as u64, 7)
}

/// The `immr` field of the bitfield and logical-immediate encodings.
#[inline]
#[must_use]
pub const fn immr(word: u32) -> u32 {
    field(word, 21, 16)
}

/// The `imms` field of the bitfield and logical-immediate encodings.
#[inline]
#[must_use]
pub const fn imms(word: u32) -> u32 {
    field(word, 15, 10)
}

/// The `N` bit of the bitfield, logical-immediate and extract encodings.
#[inline]
#[must_use]
pub const fn n_bit(word: u32) -> u32 {
    (word >> 22) & 1
}

/// The two-bit shift selector of the shifted-register encodings.
#[inline]
#[must_use]
pub const fn shift_type(word: u32) -> u32 {
    field(word, 23, 22)
}

/// The shift amount of the shifted-register encodings.
#[inline]
#[must_use]
pub const fn shift_amount(word: u32) -> u32 {
    field(word, 15, 10)
}

/// The `option` field of the extended-register and register-offset encodings.
#[inline]
#[must_use]
pub const fn extend_option(word: u32) -> u32 {
    field(word, 15, 13)
}

/// The `size` field of a load or store, bits 31:30: the base-2 logarithm of
/// the access width in bytes.
#[inline]
#[must_use]
pub const fn ls_size(word: u32) -> u32 {
    word >> 30
}

/// The `opc` field of a load or store, bits 23:22.
#[inline]
#[must_use]
pub const fn ls_opc(word: u32) -> u32 {
    field(word, 23, 22)
}

/// What a load or store encoding's `size` and `opc` fields say the access
/// does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LsAccess {
    /// Store `bytes` bytes from the low end of the register.
    Store {
        /// The access width in bytes.
        bytes: u64,
    },
    /// Load `bytes` bytes, zero-extended.
    Load {
        /// The access width in bytes.
        bytes: u64,
        /// Whether the destination register is 64 bits wide.
        wide: bool,
    },
    /// Load `bytes` bytes, sign-extended.
    LoadSigned {
        /// The access width in bytes.
        bytes: u64,
        /// Whether the sign extension goes to 64 bits rather than 32.
        wide: bool,
    },
    /// A prefetch: architecturally a hint, and this core makes no access at
    /// all rather than pretending to have a cache.
    Prefetch,
}

/// Decode a load or store's access shape from its `size` and `opc` fields.
///
/// DDI 0487 states this mapping once for the whole "Loads and stores" group,
/// which is why it lives here as a rule rather than in a hundred table rows:
/// the rows name the instructions, and this decides what they do.
///
/// `None` is an unallocated pair — `size == 0b10, opc == 0b11` would be
/// "load a word sign-extended to 32 bits", which is a no-op spelling the
/// architecture does not allocate.
#[inline]
#[must_use]
pub const fn ls_access(size: u32, opc: u32) -> Option<LsAccess> {
    let bytes = 1u64 << size;
    match opc {
        0 => Some(LsAccess::Store { bytes }),
        1 => Some(LsAccess::Load {
            bytes,
            wide: size == 3,
        }),
        2 => {
            if size == 3 {
                Some(LsAccess::Prefetch)
            } else {
                Some(LsAccess::LoadSigned { bytes, wide: true })
            }
        }
        _ => {
            if size >= 2 {
                None
            } else {
                Some(LsAccess::LoadSigned { bytes, wide: false })
            }
        }
    }
}

/// The `ptype` field of a scalar floating-point encoding, bits 23:22.
///
/// It names the precision, and `0b10` is unallocated on every encoding that
/// carries it except the `FMOV` forms that reach the top half of a vector
/// register — which is why decoding it is [`super::fp::Prec::from_ptype`]'s
/// job and not this function's.
#[inline]
#[must_use]
pub const fn ptype(word: u32) -> u32 {
    field(word, 23, 22)
}

/// The `rmode` field of a conversion between floating point and a general
/// register, bits 20:19.
#[inline]
#[must_use]
pub const fn cvt_rmode(word: u32) -> u32 {
    field(word, 20, 19)
}

/// The `opcode` field of a conversion between floating point and a general
/// register, bits 18:16.
#[inline]
#[must_use]
pub const fn cvt_opcode(word: u32) -> u32 {
    field(word, 18, 16)
}

/// The number of fraction bits a fixed-point conversion uses.
///
/// The encoding holds `64 - fbits` in bits 15:10, so a scale of one is spelled
/// `0b111111`. Subtracting rather than reading it directly is the whole
/// content of the field.
#[inline]
#[must_use]
pub const fn fbits(word: u32) -> u32 {
    64 - field(word, 15, 10)
}

/// The eight-bit immediate `FMOV` expands, bits 20:13.
#[inline]
#[must_use]
pub const fn fp_imm8(word: u32) -> u32 {
    field(word, 20, 13)
}

/// The base-2 logarithm of a SIMD&FP load or store's access width in bytes.
///
/// The width is spelled across two fields that are not adjacent: `size` in
/// bits 31:30 and `opc<1>` in bit 23. Together they give `B`, `H`, `S`, `D`
/// and — with `opc<1>` set and `size` zero — `Q`. Every other combination
/// with `opc<1>` set is unallocated, which is the `None`.
#[inline]
#[must_use]
pub const fn fp_ls_scale(word: u32) -> Option<u32> {
    let size = ls_size(word);
    if bit(word, 23) {
        if size == 0 { Some(4) } else { None }
    } else {
        Some(size)
    }
}

/// The base-2 logarithm, in bytes, of the width the SIMD&FP `opc` field names.
///
/// Bits 31:30, shared by the pair encodings and the literal load; unlike the
/// single-register form it is a width on its own, with no second field to
/// combine with: `00` is `S`, `01` is `D`, `10` is `Q`, and `11` is
/// unallocated.
#[inline]
#[must_use]
pub const fn fp_opc_scale(word: u32) -> Option<u32> {
    match field(word, 31, 30) {
        0b00 => Some(2),
        0b01 => Some(3),
        0b10 => Some(4),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Shared pseudocode
// ---------------------------------------------------------------------------

/// `Ones(n)` as a 64-bit value, correct at `n == 64`.
#[inline]
#[must_use]
pub const fn ones(n: u32) -> u64 {
    if n >= 64 { u64::MAX } else { (1u64 << n) - 1 }
}

/// Rotate `value` right by `amount` within `esize` bits.
#[inline]
const fn ror_within(value: u64, amount: u32, esize: u32) -> u64 {
    let amount = amount % esize;
    if amount == 0 {
        value & ones(esize)
    } else {
        ((value >> amount) | (value << (esize - amount))) & ones(esize)
    }
}

/// Replicate an `esize`-bit element up to `width` bits.
#[inline]
const fn replicate(elem: u64, esize: u32, width: u32) -> u64 {
    let mut out = 0u64;
    let mut pos = 0u32;
    while pos < width {
        out |= elem << pos;
        pos += esize;
    }
    out & ones(width)
}

/// The architecture's `DecodeBitMasks`, returning `(wmask, tmask)`.
///
/// DDI 0487 shared pseudocode: this one function generates the
/// logical-immediate constants *and* the bitfield masks of
/// `SBFM`/`BFM`/`UBFM`, which is why it is one function here too.
///
/// `None` is the `UNDEFINED` the pseudocode raises — `len < 1`, an element
/// wider than the operand (an `N` bit set on a 32-bit operation), or the
/// all-ones `imms` that a logical immediate may not use.
///
/// `immediate` is the pseudocode's argument of the same name: `true` for a
/// logical immediate and `false` for a bitfield move, and it selects that last
/// check.
#[must_use]
pub const fn decode_bit_masks(
    n: u32,
    imms: u32,
    immr: u32,
    immediate: bool,
    width: u32,
) -> Option<(u64, u64)> {
    // HighestSetBit(N:NOT(imms)) over seven bits.
    let combined = ((n & 1) << 6) | ((!imms) & 0x3f);
    if combined == 0 {
        return None;
    }
    let len = 31 - combined.leading_zeros();
    if len < 1 {
        return None;
    }
    let esize = 1u32 << len;
    if esize > width {
        return None;
    }
    let levels = (1u32 << len) - 1;
    if immediate && (imms & levels) == levels {
        return None;
    }
    let s = imms & levels;
    let r = immr & levels;
    let diff = s.wrapping_sub(r) & levels;
    let welem = ones(s + 1);
    let telem = ones(diff + 1);
    let wmask = replicate(ror_within(welem, r, esize), esize, width);
    let tmask = replicate(telem, esize, width);
    Some((wmask, tmask))
}

/// Add with carry, returning the result and the flags it sets.
///
/// DDI 0487 `AddWithCarry`. `C` is the carry *out* of the unsigned addition
/// and `V` the signed overflow, and getting either backwards is the classic
/// flag bug — so the carry is recovered from an overflow check rather than
/// inferred from the result's bits.
///
/// `SUB` is this with the second operand inverted and a carry in of one, which
/// is why there is no separate subtract: `SUBS x0, x1, x2` sets `C` when there
/// was **no** borrow, and deriving that from a bespoke subtract is where the
/// bug usually lives.
#[must_use]
pub const fn add_with_carry(x: u64, y: u64, carry_in: bool, width: u32) -> (u64, Nzcv) {
    let mask = ones(width);
    let x = x & mask;
    let y = y & mask;
    let cin = carry_in as u64;
    // Wrapping is the definition, not an accident: guest arithmetic wraps
    // (CLAUDE.md, "Arithmetic").
    let sum = x.wrapping_add(y).wrapping_add(cin) & mask;
    let carry = if width == 64 {
        let (partial, c1) = x.overflowing_add(y);
        let (_, c2) = partial.overflowing_add(cin);
        c1 || c2
    } else {
        // Safe without a wider type: both operands are below 2^32 here.
        (x + y + cin) >> width != 0
    };
    let sign = 1u64 << (width - 1);
    let overflow = ((x ^ sum) & (y ^ sum) & sign) != 0;
    let flags = Nzcv::new(sum & sign != 0, sum == 0, carry, overflow);
    (sum, flags)
}

/// The named shift kinds of the shifted-register operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShiftKind {
    /// Logical shift left.
    Lsl,
    /// Logical shift right.
    Lsr,
    /// Arithmetic shift right.
    Asr,
    /// Rotate right. Not available on the add/subtract encodings.
    Ror,
}

impl ShiftKind {
    /// Decode the two-bit selector.
    #[must_use]
    pub const fn from_bits(bits: u32) -> ShiftKind {
        match bits & 3 {
            0 => ShiftKind::Lsl,
            1 => ShiftKind::Lsr,
            2 => ShiftKind::Asr,
            _ => ShiftKind::Ror,
        }
    }

    /// The assembler spelling.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            ShiftKind::Lsl => "lsl",
            ShiftKind::Lsr => "lsr",
            ShiftKind::Asr => "asr",
            ShiftKind::Ror => "ror",
        }
    }

    /// Apply the shift to a `width`-bit value.
    #[must_use]
    pub const fn apply(self, value: u64, amount: u32, width: u32) -> u64 {
        let mask = ones(width);
        let value = value & mask;
        let amount = amount % width;
        match self {
            ShiftKind::Lsl => (value << amount) & mask,
            ShiftKind::Lsr => value >> amount,
            ShiftKind::Asr => {
                let signed = sext(value, width);
                ((signed >> amount) as u64) & mask
            }
            ShiftKind::Ror => ror_within(value, amount, width),
        }
    }
}

/// The assembler spelling of an `option` field (DDI 0487 C1.2.3).
#[must_use]
pub const fn extend_name(option: u32) -> &'static str {
    match option & 7 {
        0 => "uxtb",
        1 => "uxth",
        2 => "uxtw",
        3 => "uxtx",
        4 => "sxtb",
        5 => "sxth",
        6 => "sxtw",
        _ => "sxtx",
    }
}

/// Extend `value` according to an `option` field, then shift it left by
/// `amount`.
///
/// The low two bits of `option` give the source width — byte, halfword, word,
/// doubleword — and bit 2 says whether the extension is signed.
#[must_use]
pub const fn extend_reg(value: u64, option: u32, amount: u32) -> u64 {
    let bits = match option & 3 {
        0 => 8,
        1 => 16,
        2 => 32,
        _ => 64,
    };
    let narrowed = value & ones(bits);
    let extended = if option & 4 != 0 {
        sext(narrowed, bits) as u64
    } else {
        narrowed
    };
    extended.wrapping_shl(amount)
}

/// The named shareability domains of a barrier's `CRm` field.
#[must_use]
pub const fn barrier_domain(crm: u32) -> &'static str {
    match crm & 0xf {
        0b0001 => "oshld",
        0b0010 => "oshst",
        0b0011 => "osh",
        0b0101 => "nshld",
        0b0110 => "nshst",
        0b0111 => "nsh",
        0b1001 => "ishld",
        0b1010 => "ishst",
        0b1011 => "ish",
        0b1101 => "ld",
        0b1110 => "st",
        _ => "sy",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The busiest index bucket must leave room.
    ///
    /// [`build_index`] already panics during const evaluation if a bucket
    /// overflows, so an overflow is a build break rather than a silently
    /// truncated table. But a build break at the moment somebody adds a row is
    /// a poor place to discover there was no margin, so this reports the
    /// margin while there still is one.
    #[test]
    fn the_busiest_decode_bucket_has_headroom() {
        let busiest = INDEX.iter().map(|b| b.len as usize).max().unwrap_or(0);
        println!("busiest bucket: {busiest} of {BUCKET_CAP}");
        assert!(
            busiest + 16 <= BUCKET_CAP,
            "the busiest decode bucket holds {busiest} of {BUCKET_CAP} rows: \
             raise BUCKET_CAP before adding more"
        );
    }

    /// Every row must be reachable: the index buckets on bits 28:25, and a row
    /// that no bucket accepted would be an instruction the table declares and
    /// `decode` can never return.
    #[test]
    fn every_row_is_indexed() {
        for (i, row) in TABLE.iter().enumerate() {
            let found = INDEX
                .iter()
                .any(|b| b.rows[..b.len as usize].contains(&(i as u16)));
            assert!(found, "row {i} ({:?}) is in no bucket", row.op);
        }
    }

    /// A row whose `bits` set a bit its `mask` leaves free can never match.
    #[test]
    fn bits_are_covered_by_masks() {
        for row in TABLE {
            assert_eq!(
                row.bits & !row.mask,
                0,
                "{:?} fixes a bit outside its mask",
                row.op
            );
        }
    }

    /// Two rows that accept the same word would make decoding depend on table
    /// order in a way the table does not show. The generic `HINT` row is the
    /// one deliberate exception: `NOP` and its named relatives are more
    /// specific spellings of encodings it also accepts, and they precede it.
    #[test]
    fn no_unintended_overlap() {
        for (i, a) in TABLE.iter().enumerate() {
            for b in &TABLE[i + 1..] {
                let common = a.mask & b.mask;
                if a.bits & common == b.bits & common {
                    assert!(
                        a.op == Op::Hint || b.op == Op::Hint,
                        "{:?} and {:?} accept the same encodings",
                        a.op,
                        b.op
                    );
                }
            }
        }
    }

    #[test]
    fn decode_bit_masks_matches_the_manual() {
        // N=1, immr=0, imms=0: a single set bit.
        assert_eq!(decode_bit_masks(1, 0, 0, true, 64).unwrap().0, 1);
        // N=1, imms=0b111110: 63 set bits.
        assert_eq!(
            decode_bit_masks(1, 0b111110, 0, true, 64).unwrap().0,
            0x7fff_ffff_ffff_ffff
        );
        // N=0, imms=0b110000, immr=0 on 32 bits: `NOT(imms)` has its highest
        // set bit at 3, so the element is 8 bits wide with one bit set, and it
        // replicates to 0x01010101.
        assert_eq!(
            decode_bit_masks(0, 0b110000, 0, true, 32).unwrap().0,
            0x0101_0101
        );
        // The element width comes from `NOT(imms)`, not from `imms`: with
        // `imms == 0` the element is the whole 32 bits and the mask is one bit.
        assert_eq!(decode_bit_masks(0, 0, 0, true, 32).unwrap().0, 1);
        // The all-ones element is UNDEFINED for a logical immediate.
        assert!(decode_bit_masks(1, 0b111111, 0, true, 64).is_none());
        // N=1 on a 32-bit operation asks for a 64-bit element.
        assert!(decode_bit_masks(1, 0, 0, true, 32).is_none());
    }

    #[test]
    fn add_with_carry_sets_c_and_v_independently() {
        // 32-bit: 0x7fffffff + 1 overflows signed but does not carry.
        let (r, f) = add_with_carry(0x7fff_ffff, 1, false, 32);
        assert_eq!(r, 0x8000_0000);
        assert!(f.v() && !f.c() && f.n() && !f.z());
        // 0xffffffff + 1 carries but does not overflow signed.
        let (r, f) = add_with_carry(0xffff_ffff, 1, false, 32);
        assert_eq!(r, 0);
        assert!(f.c() && !f.v() && f.z());
        // 64-bit carry out, the case the widened-sum shortcut cannot take.
        let (r, f) = add_with_carry(u64::MAX, 1, false, 64);
        assert_eq!(r, 0);
        assert!(f.c() && !f.v() && f.z());
        // `SUBS x0, x5, x5`: zero, and carry set because there was no borrow.
        let (r, f) = add_with_carry(5, !5, true, 64);
        assert_eq!(r, 0);
        assert!(f.c() && f.z());
    }

    #[test]
    fn conditions_follow_the_pseudocode() {
        let z = Nzcv::new(false, true, false, false);
        assert!(Cond::EQ.holds(z));
        assert!(!Cond::NE.holds(z));
        // `NV` is *always*, not never.
        assert!(Cond::NV.holds(z));
        assert!(Cond::AL.holds(z));
        let neg = Nzcv::new(true, false, false, false);
        assert!(Cond::LT.holds(neg));
        assert!(!Cond::GE.holds(neg));
    }

    #[test]
    fn features_gate_decoding() {
        // `cas x0, x1, [x2]` needs FEAT_LSE.
        let word = 0xc8a0_7c41;
        assert!(decode(word, Features::NONE).is_none());
        assert_eq!(decode(word, Features::ALL).unwrap().op, Op::CasX);
        // `crc32b w0, w1, w2` needs FEAT_CRC32.
        let word = 0x1ac2_4020;
        assert!(decode(word, Features::NONE).is_none());
        assert_eq!(decode(word, Features::ALL).unwrap().op, Op::Crc32b);
    }

    #[test]
    fn known_encodings_decode() {
        let cases: &[(u32, Op)] = &[
            (0xd503201f, Op::Nop),
            (0xd65f03c0, Op::Ret),
            (0x14000000, Op::B),
            (0x54000000, Op::Bcond),
            (0x910003fd, Op::AddImm),   // mov x29, sp
            (0xf9400000, Op::LdrXImm),  // ldr x0, [x0]
            (0xa9bf7bfd, Op::StpXPre),  // stp x29, x30, [sp, #-16]!
            (0xa8c17bfd, Op::LdpXPost), // ldp x29, x30, [sp], #16
            (0xd2800000, Op::Movz),     // movz x0, #0
            (0xd4000001, Op::Svc),      // svc #0
            (0x8b000000, Op::AddShift), // add x0, x0, x0
            (0x9ac00800, Op::Udiv),     // udiv x0, x0, x0
            (0xd5181000, Op::Msr),      // msr sctlr_el1, x0
            (0xd5381000, Op::Mrs),      // mrs x0, sctlr_el1
            (0xd508871f, Op::Sys),      // tlbi vmalle1
            (0xd69f03e0, Op::Eret),
            (0x9ac02000, Op::Lslv),
            (0x5ac00000, Op::Rbit),
            (0xdac00c00, Op::RevX),
            // The scalar floating-point rows, every word below assembled by
            // `llvm-mc -triple=aarch64` rather than derived from the masks
            // here — which is the only way this test says anything the table
            // does not already say about itself.
            (0x1e204020, Op::Fmov),     // fmov s0, s1
            (0x1e20c0a4, Op::Fabs),     // fabs s4, s5
            (0x1e6140e6, Op::Fneg),     // fneg d6, d7
            (0x1e21c128, Op::Fsqrt),    // fsqrt s8, s9
            (0x1e22c020, Op::Fcvt),     // fcvt d0, s1
            (0x1e23c020, Op::Fcvt),     // fcvt h0, s1
            (0x1e244020, Op::Frintn),   // frintn s0, s1
            (0x1e674020, Op::Frintx),   // frintx d0, d1
            (0x1e210820, Op::Fmul),     // fmul s0, s1, s2
            (0x1e622820, Op::Fadd),     // fadd d0, d1, d2
            (0x1e628820, Op::Fnmul),    // fnmul d0, d1, d2
            (0x1f020c20, Op::Fmadd),    // fmadd s0, s1, s2, s3
            (0x1f628c20, Op::Fnmsub),   // fnmsub d0, d1, d2, d3
            (0x1e212000, Op::Fcmp),     // fcmp s0, s1
            (0x1e602008, Op::FcmpZero), // fcmp d0, #0.0
            (0x1e212010, Op::Fcmpe),    // fcmpe s0, s1
            (0x1e210403, Op::Fccmp),    // fccmp s0, s1, #3, eq
            (0x1e22cc20, Op::Fcsel),    // fcsel s0, s1, s2, gt
            (0x1e2e1000, Op::FmovImm),  // fmov s0, #1.0
            (0x1e200020, Op::Fcvtns),   // fcvtns w0, s1
            (0x1e220020, Op::Scvtf),    // scvtf s0, w1
            (0x1e260020, Op::FmovToGp), // fmov w0, s1
            (0x1e270020, Op::FmovToFp), // fmov s0, w1
            // The pair that decides `rmode == 01` is a rounding direction and
            // not a register half. Keying on `rmode` alone made every `FCVTP`
            // UNDEFINED, which is what the llvm-mc cross-check found.
            (0x1e280020, Op::Fcvtps),     // fcvtps w0, s1
            (0x9eae0020, Op::FmovHiToGp), // fmov x0, v1.d[1]
            (0x9eaf0020, Op::FmovGpToHi), // fmov v0.d[1], x1
            (0x1e380020, Op::Fcvtzs),     // fcvtzs w0, s1
            (0x1e02fc20, Op::ScvtfFix),   // scvtf s0, w1, #1
            (0x1e188020, Op::FcvtzsFix),  // fcvtzs w0, s1, #32
            (0x1c000000, Op::LdrLitV),    // ldr s0, .
            (0x3d800420, Op::StrVImm),    // str q0, [x1, #16]
            (0x3dc00420, Op::LdrVImm),    // ldr q0, [x1, #16]
            (0xbc1fc020, Op::SturV),      // stur s0, [x1, #-4]
            (0xfc408420, Op::LdrVPost),   // ldr d0, [x1], #8
            (0x3c810c20, Op::StrVPre),    // str q0, [x1, #16]!
            (0xfc227820, Op::StrVReg),    // str d0, [x1, x2, lsl #3]
            (0x2c810440, Op::StpVPost),   // stp s0, s1, [x2], #8
            (0xad010440, Op::StpVOff),    // stp q0, q1, [x2, #32]
            (0xadc10440, Op::LdpVPre),    // ldp q0, q1, [x2, #32]!
        ];
        for (word, op) in cases {
            let insn =
                decode(*word, Features::ALL).unwrap_or_else(|| panic!("{word:08x} did not decode"));
            assert_eq!(insn.op, *op, "{word:08x}");
        }
    }
}
