//! The SH-4A instruction set, described **once**.
//!
//! CLAUDE.md forbids writing an instruction table twice — once for decode and
//! once for disassembly — because the two then drift. [`TABLE`] is the only
//! description of the instruction set in this core: [`decode`] reads it, the
//! interpreter matches on the [`Op`] it yields, and [`disasm`](super::disasm)
//! prints from its [`Fmt`] column.
//!
//! # Decoding
//!
//! Every SH instruction is sixteen bits, and every row is a (mask, match)
//! pair transcribed from the *Instruction Code* column of the *SH-4A Software
//! Manual* (REJ09B0003-0150, section 3.3, tables 3.4–3.13). The manual's
//! opcode maps are organised by the top nibble and then, within it, mostly by
//! the bottom nibble or the bottom byte — so [`INDEX`] buckets the rows by
//! *(top nibble, bottom nibble)* at compile time, and [`decode`] scans a
//! bucket of at most a couple of dozen rows. The index is built by a `const
//! fn` from `TABLE` itself, so it cannot fall out of step with it.
//!
//! # The flags column
//!
//! Section 5.6.2 of the manual sorts instructions into classes that decide
//! exceptions *before* execution, and those classes are properties of the
//! encoding — so they are columns of the row rather than `match` arms in the
//! interpreter:
//!
//! * [`PRIV`] — privileged; decoded in user mode it raises a general (or slot)
//!   illegal instruction exception. `LDC`/`STC` naming `GBR` are not.
//! * [`FPU`] — an FPU instruction: every `H'F***` code plus `LDS`/`STS` of
//!   `FPUL` and `FPSCR`. With `SR.FD` set it raises an FPU-disable exception.
//! * [`DELAYED`] — a delayed branch (`JMP`, `JSR`, `BRA`, `BRAF`, `BSR`,
//!   `BSRF`, `RTS`, `RTE`, `BT/S`, `BF/S`).
//! * [`NOSLOT`] — not delayed, but still a *slot illegal instruction* in a
//!   delay slot: the other instructions that modify the PC (`BT`, `BF`,
//!   `TRAPA`, `LDC Rm,SR`, `LDC.L @Rm+,SR`, `ICBI`, `PREFI`) and the
//!   PC-relative `MOV` and `MOVA`.
//!
//! # FPU encodings are shared between modes
//!
//! `FPSCR.PR` and `FPSCR.SZ` change what an `H'F***` encoding *means* —
//! `FADD FRm,FRn` and `FADD DRm,DRn` are the same sixteen bits — so a row
//! names the operation and the interpreter consults the mode, exactly as the
//! manual's `PR`/`SZ` columns in section 10.3 do. The disassembler takes the
//! mode as a parameter for the same reason.
//!
//! # Sources
//!
//! *SH-4A Software Manual*, Renesas REJ09B0003-0150, Rev. 1.50 (2004):
//! section 3.3 for every encoding, section 5.6.2 for the illegal-instruction
//! classes, and section 10 for the per-instruction notes cited at the rows
//! where they decide something. No emulator, assembler or disassembler source
//! of any licence was consulted.

/// No special class.
pub const NONE: u8 = 0;
/// Privileged: illegal in user mode (manual §5.6.2, *General Illegal
/// Instruction Exception*).
pub const PRIV: u8 = 1 << 0;
/// An FPU instruction, gated by `SR.FD` (manual §5.6.2, *General FPU Disable
/// Exception*).
pub const FPU: u8 = 1 << 1;
/// A delayed branch: the next instruction is its delay slot.
pub const DELAYED: u8 = 1 << 2;
/// Illegal in a delay slot without itself being delayed (manual §5.6.2,
/// *Slot Illegal Instruction Exception*).
pub const NOSLOT: u8 = 1 << 3;

/// How an instruction's operands are laid out, and therefore how the
/// disassembler prints it.
///
/// `n` is bits 11–8 of the encoding and `m` bits 7–4, as in the manual's
/// `nnnn`/`mmmm` notation; a row that names its source register in bits 11–8
/// (`LDC Rm,SR` is `0100mmmm00001110`) uses a form that says so.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Fmt {
    /// No operands.
    None,
    /// `rn` — one register in bits 11–8.
    Rn,
    /// `@rn` — a register-indirect operand in bits 11–8.
    AtRn,
    /// `rm, rn`.
    RmRn,
    /// `#imm, rn` — sign-extended eight-bit immediate.
    SImmRn,
    /// `#imm, r0` — zero-extended immediate.
    ImmR0,
    /// `#imm, r0` — sign-extended immediate (`CMP/EQ`).
    SImmR0,
    /// `#imm, @(r0, gbr)`.
    ImmGbr,
    /// `#imm` — `TRAPA`.
    Imm,
    /// `rm, @rn`.
    StoreAt,
    /// `@rm, rn`.
    LoadAt,
    /// `rm, @-rn`.
    StorePre,
    /// `@rm+, rn`.
    LoadPost,
    /// `r0, @(disp, rn)` — `MOV.B`/`MOV.W` with a four-bit displacement; the
    /// base register is in bits 7–4.
    StoreR0Disp,
    /// `@(disp, rm), r0` — the base register is in bits 7–4.
    LoadR0Disp,
    /// `rm, @(disp, rn)` — `MOV.L`.
    StoreDisp,
    /// `@(disp, rm), rn` — `MOV.L`.
    LoadDisp,
    /// `rm, @(r0, rn)`.
    StoreIdx,
    /// `@(r0, rm), rn`.
    LoadIdx,
    /// `r0, @(disp, gbr)`.
    StoreGbr,
    /// `@(disp, gbr), r0`.
    LoadGbr,
    /// `@(disp, pc), rn` — a PC-relative load.
    PcRel,
    /// `@(disp, pc), r0` — `MOVA`.
    Mova,
    /// An eight-bit PC-relative branch target.
    Branch8,
    /// A twelve-bit PC-relative branch target.
    Branch12,
    /// `r0, @rn`.
    R0AtRn,
    /// `@rm, r0` — the source register is in bits 11–8.
    AtRmR0,
    /// `@rm+, r0` — the source register is in bits 11–8.
    PostRmR0,
    /// `@rm+, @rn+` — `MAC`.
    Mac,
    /// `rm, <aux>` — `LDC`/`LDS`; the source is in bits 11–8.
    RmCtl,
    /// `<aux>, rn` — `STC`/`STS`.
    CtlRn,
    /// `@rm+, <aux>` — `LDC.L`/`LDS.L`; the source is in bits 11–8.
    PostCtl,
    /// `<aux>, @-rn` — `STC.L`/`STS.L`.
    CtlPre,
    /// `rm, rn_bank` — the bank register is in bits 6–4, the source in 11–8.
    RmBank,
    /// `rm_bank, rn` — the bank register is in bits 6–4.
    BankRn,
    /// `@rm+, rn_bank`.
    PostBank,
    /// `rm_bank, @-rn`.
    BankPre,
    /// `frm, frn` — `drm, drn` when `FPSCR.PR` is set.
    FRmFRn,
    /// `frn` — `drn` when `FPSCR.PR` is set.
    FRn,
    /// `frn` — single precision only.
    FRnS,
    /// `fr0, frm, frn` — `FMAC`.
    FMac,
    /// `frm, frn` moving data: `drm`/`xdm` pairs when `FPSCR.SZ` is set.
    FMov,
    /// `@rm, frn` (pair when `SZ`).
    FLoadAt,
    /// `@rm+, frn` (pair when `SZ`).
    FLoadPost,
    /// `@(r0, rm), frn` (pair when `SZ`).
    FLoadIdx,
    /// `frm, @rn` (pair when `SZ`).
    FStoreAt,
    /// `frm, @-rn` (pair when `SZ`).
    FStorePre,
    /// `frm, @(r0, rn)` (pair when `SZ`).
    FStoreIdx,
    /// `frm, fpul` — the source is in bits 11–8; `drm` under `PR` for
    /// `FTRC`.
    FRmFpul,
    /// `fpul, frn` — `drn` under `PR` for `FLOAT`.
    FpulFRn,
    /// `drm, fpul` — `FCNVDS`.
    DRmFpul,
    /// `fpul, drn` — `FCNVSD`, `FSCA`.
    FpulDRn,
    /// `fvm, fvn` — `FIPR`; `fvn` in bits 11–10, `fvm` in 9–8.
    Fipr,
    /// `xmtrx, fvn` — `FTRV`.
    Ftrv,
}

/// One row of the instruction description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Insn {
    /// What it does.
    pub op: Op,
    /// The bits of the encoding that are fixed.
    pub mask: u16,
    /// What those bits must be.
    pub bits: u16,
    /// Operand layout, and therefore the disassembly.
    pub fmt: Fmt,
    /// The fixed register an `LDC`/`STC`/`LDS`/`STS` names, for the
    /// disassembler; empty otherwise.
    pub aux: &'static str,
    /// The exception classes of section 5.6.2: [`PRIV`], [`FPU`],
    /// [`DELAYED`], [`NOSLOT`].
    pub flags: u8,
}

impl Insn {
    /// Whether this is a privileged instruction.
    #[must_use]
    #[inline]
    pub const fn privileged(&self) -> bool {
        self.flags & PRIV != 0
    }

    /// Whether this is an FPU instruction (gated by `SR.FD`).
    #[must_use]
    #[inline]
    pub const fn fpu(&self) -> bool {
        self.flags & FPU != 0
    }

    /// Whether this is a delayed branch.
    #[must_use]
    #[inline]
    pub const fn delayed(&self) -> bool {
        self.flags & DELAYED != 0
    }

    /// Whether this raises a slot illegal instruction exception when it is
    /// decoded in a delay slot.
    #[must_use]
    #[inline]
    pub const fn slot_illegal(&self) -> bool {
        self.flags & (DELAYED | NOSLOT) != 0
    }
}

/// Declare the operation enum, its mnemonics, its summaries and the decode
/// table from one list of rows.
macro_rules! isa {
    ($($mask:literal $bits:literal $op:ident $mn:literal $fmt:ident $aux:literal ($flags:expr) $summary:literal;)*) => {
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
            /// The assembler mnemonic.
            #[must_use]
            pub const fn mnemonic(self) -> &'static str {
                match self { $(Op::$op => $mn,)* }
            }

            /// A one-line description, for `rsemu describe` and the monitor.
            #[must_use]
            pub const fn summary(self) -> &'static str {
                match self { $(Op::$op => $summary,)* }
            }

            /// Every operation this core implements, in table order.
            pub const ALL: &'static [Op] = &[$(Op::$op,)*];
        }

        /// The decode table: the only description of the instruction set in
        /// this core.
        pub static TABLE: &[Insn] = &[
            $(Insn {
                op: Op::$op,
                mask: $mask,
                bits: $bits,
                fmt: Fmt::$fmt,
                aux: $aux,
                flags: $flags,
            },)*
        ];
    };
}

isa! {
    // -- 0000 -------------------------------------------------------------
    0xf0ff 0x0002 StcSr      "stc"      CtlRn     "sr"     (PRIV)          "store SR into a register";
    0xf0ff 0x0012 StcGbr     "stc"      CtlRn     "gbr"    (NONE)          "store GBR into a register";
    0xf0ff 0x0022 StcVbr     "stc"      CtlRn     "vbr"    (PRIV)          "store VBR into a register";
    0xf0ff 0x0032 StcSsr     "stc"      CtlRn     "ssr"    (PRIV)          "store SSR into a register";
    0xf0ff 0x0042 StcSpc     "stc"      CtlRn     "spc"    (PRIV)          "store SPC into a register";
    0xf0ff 0x003a StcSgr     "stc"      CtlRn     "sgr"    (PRIV)          "store SGR into a register";
    0xf0ff 0x00fa StcDbr     "stc"      CtlRn     "dbr"    (PRIV)          "store DBR into a register";
    0xf08f 0x0082 StcBank    "stc"      BankRn    ""       (PRIV)          "store a register of the other bank";
    0xf0ff 0x0003 Bsrf       "bsrf"     Rn        ""       (DELAYED)       "branch to subroutine, PC-relative by a register, delayed";
    0xf0ff 0x0023 Braf       "braf"     Rn        ""       (DELAYED)       "branch PC-relative by a register, delayed";
    0xf0ff 0x0083 Pref       "pref"     AtRn      ""       (NONE)          "prefetch an operand-cache block, or flush a store queue";
    0xf0ff 0x0093 Ocbi       "ocbi"     AtRn      ""       (NONE)          "invalidate an operand-cache block";
    0xf0ff 0x00a3 Ocbp       "ocbp"     AtRn      ""       (NONE)          "write back and invalidate an operand-cache block";
    0xf0ff 0x00b3 Ocbwb      "ocbwb"    AtRn      ""       (NONE)          "write back an operand-cache block";
    0xf0ff 0x00c3 Movcal     "movca.l"  R0AtRn    ""       (NONE)          "store R0 allocating a cache block without fetching it";
    0xf0ff 0x00d3 Prefi      "prefi"    AtRn      ""       (NOSLOT)        "prefetch an instruction-cache block";
    0xf0ff 0x00e3 Icbi       "icbi"     AtRn      ""       (NOSLOT)        "invalidate an instruction-cache block";
    0xf0ff 0x0063 Movli      "movli.l"  AtRmR0    ""       (NONE)          "load linked: set LDST and load a longword into R0";
    0xf0ff 0x0073 Movco      "movco.l"  R0AtRn    ""       (NONE)          "store conditional: store R0 if LDST is still set";
    0xffff 0x00ab Synco      "synco"    None      ""       (NONE)          "complete preceding data accesses";
    0xf00f 0x0004 MovbS0     "mov.b"    StoreIdx  ""       (NONE)          "store a byte at R0 + Rn";
    0xf00f 0x0005 MovwS0     "mov.w"    StoreIdx  ""       (NONE)          "store a word at R0 + Rn";
    0xf00f 0x0006 MovlS0     "mov.l"    StoreIdx  ""       (NONE)          "store a longword at R0 + Rn";
    0xf00f 0x0007 Mull       "mul.l"    RmRn      ""       (NONE)          "multiply 32 x 32 into MACL";
    0xffff 0x0008 Clrt       "clrt"     None      ""       (NONE)          "clear T";
    0xffff 0x0018 Sett       "sett"     None      ""       (NONE)          "set T";
    0xffff 0x0028 Clrmac     "clrmac"   None      ""       (NONE)          "clear MACH and MACL";
    0xffff 0x0038 Ldtlb      "ldtlb"    None      ""       (PRIV)          "load PTEH/PTEL into the UTLB entry MMUCR.URC names";
    0xffff 0x0048 Clrs       "clrs"     None      ""       (NONE)          "clear S";
    0xffff 0x0058 Sets       "sets"     None      ""       (NONE)          "set S";
    0xffff 0x0009 Nop        "nop"      None      ""       (NONE)          "no operation";
    0xffff 0x0019 Div0u      "div0u"    None      ""       (NONE)          "set up an unsigned division";
    0xf0ff 0x0029 Movt       "movt"     Rn        ""       (NONE)          "copy T into a register";
    0xffff 0x000b Rts        "rts"      None      ""       (DELAYED)       "return from subroutine, delayed";
    0xffff 0x001b Sleep      "sleep"    None      ""       (PRIV)          "enter the power-down state";
    0xffff 0x002b Rte        "rte"      None      ""       (PRIV | DELAYED) "return from exception, delayed";
    0xf0ff 0x000a StsMach    "sts"      CtlRn     "mach"   (NONE)          "store MACH into a register";
    0xf0ff 0x001a StsMacl    "sts"      CtlRn     "macl"   (NONE)          "store MACL into a register";
    0xf0ff 0x002a StsPr      "sts"      CtlRn     "pr"     (NONE)          "store PR into a register";
    0xf0ff 0x005a StsFpul    "sts"      CtlRn     "fpul"   (FPU)           "store FPUL into a register";
    0xf0ff 0x006a StsFpscr   "sts"      CtlRn     "fpscr"  (FPU)           "store FPSCR into a register";
    0xf00f 0x000c MovbL0     "mov.b"    LoadIdx   ""       (NONE)          "load a sign-extended byte from R0 + Rm";
    0xf00f 0x000d MovwL0     "mov.w"    LoadIdx   ""       (NONE)          "load a sign-extended word from R0 + Rm";
    0xf00f 0x000e MovlL0     "mov.l"    LoadIdx   ""       (NONE)          "load a longword from R0 + Rm";
    0xf00f 0x000f Macl       "mac.l"    Mac       ""       (NONE)          "multiply two longwords from memory and accumulate";

    // -- 0001 -------------------------------------------------------------
    0xf000 0x1000 MovlS4     "mov.l"    StoreDisp ""       (NONE)          "store a longword at Rn + disp*4";

    // -- 0010 -------------------------------------------------------------
    0xf00f 0x2000 MovbS      "mov.b"    StoreAt   ""       (NONE)          "store a byte at Rn";
    0xf00f 0x2001 MovwS      "mov.w"    StoreAt   ""       (NONE)          "store a word at Rn";
    0xf00f 0x2002 MovlS      "mov.l"    StoreAt   ""       (NONE)          "store a longword at Rn";
    0xf00f 0x2004 MovbM      "mov.b"    StorePre  ""       (NONE)          "pre-decrement Rn and store a byte";
    0xf00f 0x2005 MovwM      "mov.w"    StorePre  ""       (NONE)          "pre-decrement Rn and store a word";
    0xf00f 0x2006 MovlM      "mov.l"    StorePre  ""       (NONE)          "pre-decrement Rn and store a longword";
    0xf00f 0x2007 Div0s      "div0s"    RmRn      ""       (NONE)          "set up a signed division";
    0xf00f 0x2008 Tst        "tst"      RmRn      ""       (NONE)          "set T if Rn & Rm is zero";
    0xf00f 0x2009 And        "and"      RmRn      ""       (NONE)          "bitwise AND";
    0xf00f 0x200a Xor        "xor"      RmRn      ""       (NONE)          "bitwise exclusive-OR";
    0xf00f 0x200b Or         "or"       RmRn      ""       (NONE)          "bitwise OR";
    0xf00f 0x200c CmpStr     "cmp/str"  RmRn      ""       (NONE)          "set T if any byte of Rn equals the same byte of Rm";
    0xf00f 0x200d Xtrct      "xtrct"    RmRn      ""       (NONE)          "extract the middle 32 bits of Rm:Rn";
    0xf00f 0x200e Muluw      "mulu.w"   RmRn      ""       (NONE)          "multiply unsigned words into MACL";
    0xf00f 0x200f Mulsw      "muls.w"   RmRn      ""       (NONE)          "multiply signed words into MACL";

    // -- 0011 -------------------------------------------------------------
    0xf00f 0x3000 CmpEq      "cmp/eq"   RmRn      ""       (NONE)          "set T if Rn = Rm";
    0xf00f 0x3002 CmpHs      "cmp/hs"   RmRn      ""       (NONE)          "set T if Rn >= Rm, unsigned";
    0xf00f 0x3003 CmpGe      "cmp/ge"   RmRn      ""       (NONE)          "set T if Rn >= Rm, signed";
    0xf00f 0x3004 Div1       "div1"     RmRn      ""       (NONE)          "one step of a division";
    0xf00f 0x3005 Dmulu      "dmulu.l"  RmRn      ""       (NONE)          "multiply unsigned 32 x 32 into MACH:MACL";
    0xf00f 0x3006 CmpHi      "cmp/hi"   RmRn      ""       (NONE)          "set T if Rn > Rm, unsigned";
    0xf00f 0x3007 CmpGt      "cmp/gt"   RmRn      ""       (NONE)          "set T if Rn > Rm, signed";
    0xf00f 0x3008 Sub        "sub"      RmRn      ""       (NONE)          "subtract";
    0xf00f 0x300a Subc       "subc"     RmRn      ""       (NONE)          "subtract with borrow in and out of T";
    0xf00f 0x300b Subv       "subv"     RmRn      ""       (NONE)          "subtract, setting T on signed underflow";
    0xf00f 0x300c Add        "add"      RmRn      ""       (NONE)          "add";
    0xf00f 0x300d Dmuls      "dmuls.l"  RmRn      ""       (NONE)          "multiply signed 32 x 32 into MACH:MACL";
    0xf00f 0x300e Addc       "addc"     RmRn      ""       (NONE)          "add with carry in and out of T";
    0xf00f 0x300f Addv       "addv"     RmRn      ""       (NONE)          "add, setting T on signed overflow";

    // -- 0100 -------------------------------------------------------------
    0xf0ff 0x4000 Shll       "shll"     Rn        ""       (NONE)          "shift left one bit into T";
    0xf0ff 0x4001 Shlr       "shlr"     Rn        ""       (NONE)          "shift right logically one bit into T";
    0xf0ff 0x4002 StslMach   "sts.l"    CtlPre    "mach"   (NONE)          "push MACH";
    0xf0ff 0x4003 StclSr     "stc.l"    CtlPre    "sr"     (PRIV)          "push SR";
    0xf0ff 0x4004 Rotl       "rotl"     Rn        ""       (NONE)          "rotate left, the MSB into T";
    0xf0ff 0x4005 Rotr       "rotr"     Rn        ""       (NONE)          "rotate right, the LSB into T";
    0xf0ff 0x4006 LdslMach   "lds.l"    PostCtl   "mach"   (NONE)          "pop MACH";
    0xf0ff 0x4007 LdclSr     "ldc.l"    PostCtl   "sr"     (PRIV | NOSLOT) "pop SR";
    0xf0ff 0x4008 Shll2      "shll2"    Rn        ""       (NONE)          "shift left two bits";
    0xf0ff 0x4009 Shlr2      "shlr2"    Rn        ""       (NONE)          "shift right logically two bits";
    0xf0ff 0x400a LdsMach    "lds"      RmCtl     "mach"   (NONE)          "load MACH from a register";
    0xf0ff 0x400b Jsr        "jsr"      AtRn      ""       (DELAYED)       "jump to subroutine, delayed";
    0xf0ff 0x400e LdcSr      "ldc"      RmCtl     "sr"     (PRIV | NOSLOT) "load SR from a register";
    0xf0ff 0x4010 Dt         "dt"       Rn        ""       (NONE)          "decrement and set T if zero";
    0xf0ff 0x4011 CmpPz      "cmp/pz"   Rn        ""       (NONE)          "set T if Rn >= 0";
    0xf0ff 0x4012 StslMacl   "sts.l"    CtlPre    "macl"   (NONE)          "push MACL";
    0xf0ff 0x4013 StclGbr    "stc.l"    CtlPre    "gbr"    (NONE)          "push GBR";
    0xf0ff 0x4015 CmpPl      "cmp/pl"   Rn        ""       (NONE)          "set T if Rn > 0";
    0xf0ff 0x4016 LdslMacl   "lds.l"    PostCtl   "macl"   (NONE)          "pop MACL";
    0xf0ff 0x4017 LdclGbr    "ldc.l"    PostCtl   "gbr"    (NONE)          "pop GBR";
    0xf0ff 0x4018 Shll8      "shll8"    Rn        ""       (NONE)          "shift left eight bits";
    0xf0ff 0x4019 Shlr8      "shlr8"    Rn        ""       (NONE)          "shift right logically eight bits";
    0xf0ff 0x401a LdsMacl    "lds"      RmCtl     "macl"   (NONE)          "load MACL from a register";
    0xf0ff 0x401b Tas        "tas.b"    AtRn      ""       (NONE)          "test a byte and set its MSB, indivisibly";
    0xf0ff 0x401e LdcGbr     "ldc"      RmCtl     "gbr"    (NONE)          "load GBR from a register";
    0xf0ff 0x4020 Shal       "shal"     Rn        ""       (NONE)          "shift left arithmetically one bit into T";
    0xf0ff 0x4021 Shar       "shar"     Rn        ""       (NONE)          "shift right arithmetically one bit into T";
    0xf0ff 0x4022 StslPr     "sts.l"    CtlPre    "pr"     (NONE)          "push PR";
    0xf0ff 0x4023 StclVbr    "stc.l"    CtlPre    "vbr"    (PRIV)          "push VBR";
    0xf0ff 0x4024 Rotcl      "rotcl"    Rn        ""       (NONE)          "rotate left through T";
    0xf0ff 0x4025 Rotcr      "rotcr"    Rn        ""       (NONE)          "rotate right through T";
    0xf0ff 0x4026 LdslPr     "lds.l"    PostCtl   "pr"     (NONE)          "pop PR";
    0xf0ff 0x4027 LdclVbr    "ldc.l"    PostCtl   "vbr"    (PRIV)          "pop VBR";
    0xf0ff 0x4028 Shll16     "shll16"   Rn        ""       (NONE)          "shift left sixteen bits";
    0xf0ff 0x4029 Shlr16     "shlr16"   Rn        ""       (NONE)          "shift right logically sixteen bits";
    0xf0ff 0x402a LdsPr      "lds"      RmCtl     "pr"     (NONE)          "load PR from a register";
    0xf0ff 0x402b Jmp        "jmp"      AtRn      ""       (DELAYED)       "jump, delayed";
    0xf0ff 0x402e LdcVbr     "ldc"      RmCtl     "vbr"    (PRIV)          "load VBR from a register";
    0xf0ff 0x4032 StclSgr    "stc.l"    CtlPre    "sgr"    (PRIV)          "push SGR";
    0xf0ff 0x4033 StclSsr    "stc.l"    CtlPre    "ssr"    (PRIV)          "push SSR";
    0xf0ff 0x4036 LdclSgr    "ldc.l"    PostCtl   "sgr"    (PRIV)          "pop SGR";
    0xf0ff 0x4037 LdclSsr    "ldc.l"    PostCtl   "ssr"    (PRIV)          "pop SSR";
    0xf0ff 0x403a LdcSgr     "ldc"      RmCtl     "sgr"    (PRIV)          "load SGR from a register";
    0xf0ff 0x403e LdcSsr     "ldc"      RmCtl     "ssr"    (PRIV)          "load SSR from a register";
    0xf0ff 0x4043 StclSpc    "stc.l"    CtlPre    "spc"    (PRIV)          "push SPC";
    0xf0ff 0x4047 LdclSpc    "ldc.l"    PostCtl   "spc"    (PRIV)          "pop SPC";
    0xf0ff 0x404e LdcSpc     "ldc"      RmCtl     "spc"    (PRIV)          "load SPC from a register";
    0xf0ff 0x4052 StslFpul   "sts.l"    CtlPre    "fpul"   (FPU)           "push FPUL";
    0xf0ff 0x4056 LdslFpul   "lds.l"    PostCtl   "fpul"   (FPU)           "pop FPUL";
    0xf0ff 0x405a LdsFpul    "lds"      RmCtl     "fpul"   (FPU)           "load FPUL from a register";
    0xf0ff 0x4062 StslFpscr  "sts.l"    CtlPre    "fpscr"  (FPU)           "push FPSCR";
    0xf0ff 0x4066 LdslFpscr  "lds.l"    PostCtl   "fpscr"  (FPU)           "pop FPSCR";
    0xf0ff 0x406a LdsFpscr   "lds"      RmCtl     "fpscr"  (FPU)           "load FPSCR from a register";
    0xf0ff 0x40a9 Movua      "movua.l"  AtRmR0    ""       (NONE)          "load an unaligned longword into R0";
    0xf0ff 0x40e9 MovuaP     "movua.l"  PostRmR0  ""       (NONE)          "load an unaligned longword into R0 and post-increment";
    0xf0ff 0x40f2 StclDbr    "stc.l"    CtlPre    "dbr"    (PRIV)          "push DBR";
    0xf0ff 0x40f6 LdclDbr    "ldc.l"    PostCtl   "dbr"    (PRIV)          "pop DBR";
    0xf0ff 0x40fa LdcDbr     "ldc"      RmCtl     "dbr"    (PRIV)          "load DBR from a register";
    0xf08f 0x4083 StclBank   "stc.l"    BankPre   ""       (PRIV)          "push a register of the other bank";
    0xf08f 0x4087 LdclBank   "ldc.l"    PostBank  ""       (PRIV)          "pop a register of the other bank";
    0xf08f 0x408e LdcBank    "ldc"      RmBank    ""       (PRIV)          "load a register of the other bank";
    0xf00f 0x400c Shad       "shad"     RmRn      ""       (NONE)          "shift arithmetically by a signed register amount";
    0xf00f 0x400d Shld       "shld"     RmRn      ""       (NONE)          "shift logically by a signed register amount";
    0xf00f 0x400f Macw       "mac.w"    Mac       ""       (NONE)          "multiply two words from memory and accumulate";

    // -- 0101 -------------------------------------------------------------
    0xf000 0x5000 MovlL4     "mov.l"    LoadDisp  ""       (NONE)          "load a longword from Rm + disp*4";

    // -- 0110 -------------------------------------------------------------
    0xf00f 0x6000 MovbL      "mov.b"    LoadAt    ""       (NONE)          "load a sign-extended byte";
    0xf00f 0x6001 MovwL      "mov.w"    LoadAt    ""       (NONE)          "load a sign-extended word";
    0xf00f 0x6002 MovlL      "mov.l"    LoadAt    ""       (NONE)          "load a longword";
    0xf00f 0x6003 Mov        "mov"      RmRn      ""       (NONE)          "copy a register";
    0xf00f 0x6004 MovbP      "mov.b"    LoadPost  ""       (NONE)          "load a sign-extended byte and post-increment";
    0xf00f 0x6005 MovwP      "mov.w"    LoadPost  ""       (NONE)          "load a sign-extended word and post-increment";
    0xf00f 0x6006 MovlP      "mov.l"    LoadPost  ""       (NONE)          "load a longword and post-increment";
    0xf00f 0x6007 Not        "not"      RmRn      ""       (NONE)          "bitwise complement";
    0xf00f 0x6008 Swapb      "swap.b"   RmRn      ""       (NONE)          "swap the two low bytes";
    0xf00f 0x6009 Swapw      "swap.w"   RmRn      ""       (NONE)          "swap the two halves";
    0xf00f 0x600a Negc       "negc"     RmRn      ""       (NONE)          "negate with borrow in and out of T";
    0xf00f 0x600b Neg        "neg"      RmRn      ""       (NONE)          "negate";
    0xf00f 0x600c Extub      "extu.b"   RmRn      ""       (NONE)          "zero-extend a byte";
    0xf00f 0x600d Extuw      "extu.w"   RmRn      ""       (NONE)          "zero-extend a word";
    0xf00f 0x600e Extsb      "exts.b"   RmRn      ""       (NONE)          "sign-extend a byte";
    0xf00f 0x600f Extsw      "exts.w"   RmRn      ""       (NONE)          "sign-extend a word";

    // -- 0111 -------------------------------------------------------------
    0xf000 0x7000 AddI       "add"      SImmRn    ""       (NONE)          "add a sign-extended immediate";

    // -- 1000 -------------------------------------------------------------
    0xff00 0x8000 MovbS4     "mov.b"    StoreR0Disp ""     (NONE)          "store R0's low byte at Rn + disp";
    0xff00 0x8100 MovwS4     "mov.w"    StoreR0Disp ""     (NONE)          "store R0's low word at Rn + disp*2";
    0xff00 0x8400 MovbL4     "mov.b"    LoadR0Disp ""      (NONE)          "load a sign-extended byte from Rm + disp into R0";
    0xff00 0x8500 MovwL4     "mov.w"    LoadR0Disp ""      (NONE)          "load a sign-extended word from Rm + disp*2 into R0";
    0xff00 0x8800 CmpEqI     "cmp/eq"   SImmR0    ""       (NONE)          "set T if R0 equals a sign-extended immediate";
    0xff00 0x8900 Bt         "bt"       Branch8   ""       (NOSLOT)        "branch if T";
    0xff00 0x8b00 Bf         "bf"       Branch8   ""       (NOSLOT)        "branch if not T";
    0xff00 0x8d00 Bts        "bt/s"     Branch8   ""       (DELAYED)       "branch if T, delayed";
    0xff00 0x8f00 Bfs        "bf/s"     Branch8   ""       (DELAYED)       "branch if not T, delayed";

    // -- 1001 -------------------------------------------------------------
    0xf000 0x9000 MovwPc     "mov.w"    PcRel     ""       (NOSLOT)        "load a sign-extended PC-relative word";

    // -- 1010, 1011 -------------------------------------------------------
    0xf000 0xa000 Bra        "bra"      Branch12  ""       (DELAYED)       "branch, delayed";
    0xf000 0xb000 Bsr        "bsr"      Branch12  ""       (DELAYED)       "branch to subroutine, delayed";

    // -- 1100 -------------------------------------------------------------
    0xff00 0xc000 MovbSg     "mov.b"    StoreGbr  ""       (NONE)          "store R0's low byte at GBR + disp";
    0xff00 0xc100 MovwSg     "mov.w"    StoreGbr  ""       (NONE)          "store R0's low word at GBR + disp*2";
    0xff00 0xc200 MovlSg     "mov.l"    StoreGbr  ""       (NONE)          "store R0 at GBR + disp*4";
    0xff00 0xc300 Trapa      "trapa"    Imm       ""       (NOSLOT)        "unconditional trap";
    0xff00 0xc400 MovbLg     "mov.b"    LoadGbr   ""       (NONE)          "load a sign-extended byte from GBR + disp";
    0xff00 0xc500 MovwLg     "mov.w"    LoadGbr   ""       (NONE)          "load a sign-extended word from GBR + disp*2";
    0xff00 0xc600 MovlLg     "mov.l"    LoadGbr   ""       (NONE)          "load a longword from GBR + disp*4";
    0xff00 0xc700 Mova       "mova"     Mova      ""       (NOSLOT)        "compute a PC-relative longword address into R0";
    0xff00 0xc800 TstI       "tst"      ImmR0     ""       (NONE)          "set T if R0 & imm is zero";
    0xff00 0xc900 AndI       "and"      ImmR0     ""       (NONE)          "AND R0 with a zero-extended immediate";
    0xff00 0xca00 XorI       "xor"      ImmR0     ""       (NONE)          "exclusive-OR R0 with a zero-extended immediate";
    0xff00 0xcb00 OrI        "or"       ImmR0     ""       (NONE)          "OR R0 with a zero-extended immediate";
    0xff00 0xcc00 TstB       "tst.b"    ImmGbr    ""       (NONE)          "set T if the byte at R0 + GBR & imm is zero";
    0xff00 0xcd00 AndB       "and.b"    ImmGbr    ""       (NONE)          "AND the byte at R0 + GBR with an immediate";
    0xff00 0xce00 XorB       "xor.b"    ImmGbr    ""       (NONE)          "exclusive-OR the byte at R0 + GBR with an immediate";
    0xff00 0xcf00 OrB        "or.b"     ImmGbr    ""       (NONE)          "OR the byte at R0 + GBR with an immediate";

    // -- 1101, 1110 -------------------------------------------------------
    0xf000 0xd000 MovlPc     "mov.l"    PcRel     ""       (NOSLOT)        "load a PC-relative longword";
    0xf000 0xe000 MovI       "mov"      SImmRn    ""       (NONE)          "load a sign-extended immediate";

    // -- 1111: the FPU ------------------------------------------------------
    //
    // The PR/SZ-specific forms (`FADD DRm,DRn`, `FMOV XDm,@Rn`, ...) share
    // these encodings; section 10.3 gives each instruction's behaviour per
    // mode, and the interpreter follows it.
    0xf00f 0xf000 Fadd       "fadd"     FRmFRn    ""       (FPU)           "floating-point add";
    0xf00f 0xf001 Fsub       "fsub"     FRmFRn    ""       (FPU)           "floating-point subtract";
    0xf00f 0xf002 Fmul       "fmul"     FRmFRn    ""       (FPU)           "floating-point multiply";
    0xf00f 0xf003 Fdiv       "fdiv"     FRmFRn    ""       (FPU)           "floating-point divide";
    0xf00f 0xf004 FcmpEq     "fcmp/eq"  FRmFRn    ""       (FPU)           "set T if equal";
    0xf00f 0xf005 FcmpGt     "fcmp/gt"  FRmFRn    ""       (FPU)           "set T if FRn > FRm";
    0xf00f 0xf006 FmovLIdx   "fmov.s"   FLoadIdx  ""       (FPU)           "load from R0 + Rm";
    0xf00f 0xf007 FmovSIdx   "fmov.s"   FStoreIdx ""       (FPU)           "store at R0 + Rn";
    0xf00f 0xf008 FmovL      "fmov.s"   FLoadAt   ""       (FPU)           "load from Rm";
    0xf00f 0xf009 FmovLP     "fmov.s"   FLoadPost ""       (FPU)           "load from Rm and post-increment";
    0xf00f 0xf00a FmovS      "fmov.s"   FStoreAt  ""       (FPU)           "store at Rn";
    0xf00f 0xf00b FmovSM     "fmov.s"   FStorePre ""       (FPU)           "pre-decrement Rn and store";
    0xf00f 0xf00c Fmov       "fmov"     FMov      ""       (FPU)           "copy a floating-point register or pair";
    0xf00f 0xf00e Fmac       "fmac"     FMac      ""       (FPU)           "fused FR0 * FRm + FRn";
    0xf0ff 0xf00d Fsts       "fsts"     FpulFRn   ""       (FPU)           "copy FPUL into a floating-point register";
    0xf0ff 0xf01d Flds       "flds"     FRmFpul   ""       (FPU)           "copy a floating-point register into FPUL";
    0xf0ff 0xf02d Float      "float"    FpulFRn   ""       (FPU)           "convert the integer in FPUL to floating point";
    0xf0ff 0xf03d Ftrc       "ftrc"     FRmFpul   ""       (FPU)           "truncate to an integer in FPUL";
    0xf0ff 0xf04d Fneg       "fneg"     FRn       ""       (FPU)           "flip the sign";
    0xf0ff 0xf05d Fabs       "fabs"     FRn       ""       (FPU)           "clear the sign";
    0xf0ff 0xf06d Fsqrt      "fsqrt"    FRn       ""       (FPU)           "square root";
    0xf0ff 0xf07d Fsrra      "fsrra"    FRnS      ""       (FPU)           "approximate reciprocal square root";
    0xf0ff 0xf08d Fldi0      "fldi0"    FRnS      ""       (FPU)           "load 0.0";
    0xf0ff 0xf09d Fldi1      "fldi1"    FRnS      ""       (FPU)           "load 1.0";
    0xf1ff 0xf0ad Fcnvsd     "fcnvsd"   FpulDRn   ""       (FPU)           "convert the single in FPUL to double";
    0xf1ff 0xf0bd Fcnvds     "fcnvds"   DRmFpul   ""       (FPU)           "convert a double to single into FPUL";
    0xf0ff 0xf0ed Fipr       "fipr"     Fipr      ""       (FPU)           "four-element inner product";
    0xf1ff 0xf0fd Fsca       "fsca"     FpulDRn   ""       (FPU)           "approximate sine and cosine of the FPUL angle";
    0xf3ff 0xf1fd Ftrv       "ftrv"     Ftrv      ""       (FPU)           "multiply a vector by XMTRX";
    0xffff 0xf3fd Fschg      "fschg"    None      ""       (FPU)           "flip FPSCR.SZ";
    0xffff 0xf7fd Fpchg      "fpchg"    None      ""       (FPU)           "flip FPSCR.PR";
    0xffff 0xfbfd Frchg      "frchg"    None      ""       (FPU)           "flip FPSCR.FR";
}

/// How many rows a bucket of [`INDEX`] can hold. A `const` assertion in
/// [`build_index`] keeps the table honest about it.
const BUCKET: usize = 24;

/// The rows that can match an encoding with a given *(top nibble, bottom
/// nibble)*, as indices into [`TABLE`] and a count.
///
/// Built from `TABLE` by a `const fn`, so it is a derived cache in the strict
/// sense — it cannot disagree with the table it indexes, and adding a row
/// needs no second edit.
pub static INDEX: [([u16; BUCKET], u8); 256] = build_index(TABLE);

/// Compute [`INDEX`] at compile time.
const fn build_index(table: &[Insn]) -> [([u16; BUCKET], u8); 256] {
    let mut index = [([0u16; BUCKET], 0u8); 256];
    let mut key = 0usize;
    while key < 256 {
        // The two nibbles the key fixes, placed where they sit in a word.
        let probe = (((key >> 4) as u16) << 12) | (key & 0xf) as u16;
        let mut i = 0usize;
        while i < table.len() {
            let row = &table[i];
            let fixed = row.mask & 0xf00f;
            if probe & fixed == row.bits & fixed {
                let n = index[key].1 as usize;
                assert!(n < BUCKET, "an INDEX bucket overflowed; raise BUCKET");
                index[key].0[n] = i as u16;
                index[key].1 += 1;
            }
            i += 1;
        }
        key += 1;
    }
    index
}

/// Decode an instruction word, or `None` if nothing in the table matches.
///
/// The first matching row wins; the table has no overlapping rows (its own
/// test proves it), so the order inside a bucket is only a matter of speed.
#[must_use]
#[inline]
pub fn decode(word: u16) -> Option<&'static Insn> {
    let key = (((word >> 12) << 4) | (word & 0xf)) as usize;
    let (rows, count) = &INDEX[key];
    for &i in &rows[..*count as usize] {
        let insn = &TABLE[i as usize];
        if word & insn.mask == insn.bits {
            return Some(insn);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Field extraction
// ---------------------------------------------------------------------------

/// The `nnnn` field, bits 11–8.
#[inline]
#[must_use]
pub const fn n(word: u16) -> usize {
    ((word >> 8) & 0xf) as usize
}

/// The `mmmm` field, bits 7–4.
#[inline]
#[must_use]
pub const fn m(word: u16) -> usize {
    ((word >> 4) & 0xf) as usize
}

/// The bank-register field `nnn`/`mmm`, bits 6–4.
#[inline]
#[must_use]
pub const fn bank(word: u16) -> usize {
    ((word >> 4) & 0x7) as usize
}

/// The four-bit displacement, bits 3–0.
#[inline]
#[must_use]
pub const fn d4(word: u16) -> u32 {
    (word & 0xf) as u32
}

/// The eight-bit immediate or displacement, zero-extended.
#[inline]
#[must_use]
pub const fn i8u(word: u16) -> u32 {
    (word & 0xff) as u32
}

/// The eight-bit immediate, sign-extended.
#[inline]
#[must_use]
pub const fn i8s(word: u16) -> u32 {
    (word & 0xff) as u8 as i8 as i32 as u32
}

/// The target of an eight-bit PC-relative branch at `pc`: `PC + 4 + disp*2`,
/// `disp` sign-extended (manual §3.2, table 3.2).
#[inline]
#[must_use]
pub const fn branch8(pc: u32, word: u16) -> u32 {
    pc.wrapping_add(4).wrapping_add(i8s(word) << 1)
}

/// The target of a twelve-bit PC-relative branch at `pc`.
#[inline]
#[must_use]
pub const fn branch12(pc: u32, word: u16) -> u32 {
    let disp = (((word & 0xfff) as i32) << 20) >> 19;
    pc.wrapping_add(4).wrapping_add(disp as u32)
}

/// The effective address of `MOV.W @(disp,PC),Rn`: `PC + 4 + disp*2`.
#[inline]
#[must_use]
pub const fn pc_word(pc: u32, word: u16) -> u32 {
    pc.wrapping_add(4).wrapping_add(i8u(word) << 1)
}

/// The effective address of `MOV.L @(disp,PC),Rn` and `MOVA`:
/// `(PC & ~3) + 4 + disp*4`.
#[inline]
#[must_use]
pub const fn pc_long(pc: u32, word: u16) -> u32 {
    (pc & !3).wrapping_add(4).wrapping_add(i8u(word) << 2)
}

/// The general registers' names.
pub const REG_NAMES: [&str; 16] = [
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14",
    "r15",
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_row_has_its_bits_inside_its_mask() {
        for row in TABLE {
            assert_eq!(row.bits & !row.mask, 0, "{:?}", row.op);
        }
    }

    #[test]
    fn no_two_rows_overlap() {
        // Two rows overlap when some word matches both: their fixed bits agree
        // wherever both masks fix them. The manual's map has no such pair, so
        // decode order is irrelevant — and a typo that makes one would show
        // up here rather than as a silently shadowed instruction.
        for (i, a) in TABLE.iter().enumerate() {
            for b in &TABLE[i + 1..] {
                let both = a.mask & b.mask;
                assert!(
                    a.bits & both != b.bits & both,
                    "{:?} and {:?} overlap",
                    a.op,
                    b.op
                );
            }
        }
    }

    #[test]
    fn decode_agrees_with_a_linear_scan_for_every_word() {
        for word in 0..=u16::MAX {
            let linear = TABLE.iter().find(|r| word & r.mask == r.bits);
            assert_eq!(
                decode(word).map(|r| r.op),
                linear.map(|r| r.op),
                "{word:04x}"
            );
        }
    }

    #[test]
    fn the_architecturally_undefined_code_is_undefined() {
        // H'FFFD is the one code the manual guarantees to be illegal (§5.6.2).
        assert!(decode(0xfffd).is_none());
    }

    #[test]
    fn a_sample_of_encodings_decodes_to_the_manuals_operations() {
        assert_eq!(decode(0x0009).unwrap().op, Op::Nop);
        assert_eq!(decode(0x6123).unwrap().op, Op::Mov);
        assert_eq!(decode(0xe1ff).unwrap().op, Op::MovI);
        assert_eq!(decode(0x412e).unwrap().op, Op::LdcVbr);
        assert_eq!(decode(0x41de).unwrap().op, Op::LdcBank);
        assert_eq!(decode(0x0182).unwrap().op, Op::StcBank);
        assert_eq!(decode(0xf1fd).unwrap().op, Op::Ftrv);
        assert_eq!(decode(0xf2fd).unwrap().op, Op::Fsca);
        assert_eq!(decode(0xf3fd).unwrap().op, Op::Fschg);
        assert_eq!(decode(0x002b).unwrap().op, Op::Rte);
        assert!(decode(0x3001).is_none());
    }

    #[test]
    fn branch_targets_are_relative_to_pc_plus_four() {
        assert_eq!(branch8(0x1000, 0x89fe), 0x1000);
        assert_eq!(branch12(0x1000, 0xaffe), 0x1000);
        assert_eq!(branch12(0x1000, 0xa7ff), 0x1000 + 4 + 0xffe);
        assert_eq!(pc_long(0x1002, 0xd001), 0x1008);
    }
}
