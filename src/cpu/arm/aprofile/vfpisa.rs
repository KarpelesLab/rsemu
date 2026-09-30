//! The VFPv2/VFPv3 A32 encodings, described once.
//!
//! Same rule as [`super::isa`] (CLAUDE.md, "CPU cores"): one description
//! serves the interpreter and the disassembler. [`decode`] turns a
//! coprocessor-10/11 word into a [`VfpInsn`], the executor runs that value,
//! and [`VfpInsn::display`] prints it. Nothing re-reads the raw encoding.
//!
//! # Where these live in the A32 map
//!
//! Inside the coprocessor space, with `coproc == 0b101x` (DDI 0406C A5.6,
//! "Coprocessor instructions, and Supervisor Call"): bit 8 is `sz`, single or
//! double precision, which is why the architecture hands VFP two coprocessor
//! numbers rather than one.
//!
//! | bits 27:24 | bit 4 | group (DDI 0406C) |
//! | --- | --- | --- |
//! | `1110` | 0 | data processing (A7.5) |
//! | `1110` | 1 | 8, 16 and 32-bit transfers to and from core registers (A7.8) |
//! | `110x` | — | extension register load/store (A7.6), and the 64-bit transfers (A7.9) |
//!
//! The encodings are the same bits in T32 below the top nibble — `1110 110x`
//! becomes `111T 110x` — which is why [`decode`] ignores the condition field:
//! a Thumb-2 front end can hand it the same word with the halfwords swapped.
//!
//! # Register numbering
//!
//! A single-precision register is `Vx:X` (the four-bit field, then the extra
//! bit); a double is `X:Vx`. Every [`VfpInsn`] carries register numbers with
//! that already done — `0..=31` either way — plus the precision, so nothing
//! downstream has to remember which half of the number lives where.
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition* (ARM DDI
//! 0406C): A5.6, A7.5–A7.9 for the tables, and A8.8's alphabetical entries for
//! each instruction's fields and UNDEFINED/UNPREDICTABLE conditions. Checked
//! against `arm-none-eabi-as -mfpu=vfpv3` and its `objdump`, run as black
//! boxes. No emulator source of any licence was consulted (`ROADMAP.md` §1).

use core::fmt;

use super::isa::{Cond, RegName, bit, field};

/// A three-operand data-processing operation (DDI 0406C A7.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DataOp {
    /// `VMLA`: `d = d + n × m`, rounded twice.
    Mla,
    /// `VMLS`: `d = d − n × m`.
    Mls,
    /// `VNMLA`: `d = −d − n × m`.
    Nmla,
    /// `VNMLS`: `d = −d + n × m`.
    Nmls,
    /// `VMUL`.
    Mul,
    /// `VNMUL`: `d = −(n × m)`.
    Nmul,
    /// `VADD`.
    Add,
    /// `VSUB`.
    Sub,
    /// `VDIV`.
    Div,
}

impl DataOp {
    /// The mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            DataOp::Mla => "VMLA",
            DataOp::Mls => "VMLS",
            DataOp::Nmla => "VNMLA",
            DataOp::Nmls => "VNMLS",
            DataOp::Mul => "VMUL",
            DataOp::Nmul => "VNMUL",
            DataOp::Add => "VADD",
            DataOp::Sub => "VSUB",
            DataOp::Div => "VDIV",
        }
    }
}

/// A two-operand data-processing operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum UnaryOp {
    /// `VMOV` register to register.
    Mov,
    /// `VABS`.
    Abs,
    /// `VNEG`.
    Neg,
    /// `VSQRT`.
    Sqrt,
}

impl UnaryOp {
    /// The mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            UnaryOp::Mov => "VMOV",
            UnaryOp::Abs => "VABS",
            UnaryOp::Neg => "VNEG",
            UnaryOp::Sqrt => "VSQRT",
        }
    }
}

/// One decoded VFP instruction.
///
/// `dp` is the `sz` bit: double precision. Register numbers are assembled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VfpInsn {
    /// `d = n op m`, or an accumulate into `d`.
    Data {
        /// Which operation.
        op: DataOp,
        /// Double precision.
        dp: bool,
        /// Destination (and accumulator).
        d: u8,
        /// First source.
        n: u8,
        /// Second source.
        m: u8,
    },
    /// `d = op m`.
    Unary {
        /// Which operation.
        op: UnaryOp,
        /// Double precision.
        dp: bool,
        /// Destination.
        d: u8,
        /// Source.
        m: u8,
    },
    /// `VMOV Sd/Dd, #imm` (VFPv3). `imm8` is the encoded byte; the value is
    /// [`super::vfp::expand_imm`] of it.
    MovImm {
        /// Double precision.
        dp: bool,
        /// Destination.
        d: u8,
        /// The encoded eight-bit constant.
        imm8: u8,
    },
    /// `VCMP`/`VCMPE`, against `m` or against `+0.0`.
    Cmp {
        /// Double precision.
        dp: bool,
        /// Left operand.
        d: u8,
        /// Right operand; ignored when `with_zero`.
        m: u8,
        /// The `#0.0` form.
        with_zero: bool,
        /// `VCMPE`: a quiet NaN raises invalid too.
        signal_all: bool,
    },
    /// `VCVT.F64.F32 Dd, Sm` or `VCVT.F32.F64 Sd, Dm`.
    CvtPrec {
        /// Single to double rather than double to single.
        to_double: bool,
        /// Destination, at the destination's precision.
        d: u8,
        /// Source, at the source's precision.
        m: u8,
    },
    /// `VCVT{R}.S32/U32.F32/F64 Sd, Sm/Dm`: float to integer.
    CvtToInt {
        /// The source is a double.
        dp: bool,
        /// Destination single.
        d: u8,
        /// Source.
        m: u8,
        /// Signed rather than unsigned.
        signed: bool,
        /// Round toward zero (`VCVT`) rather than in the `FPSCR` mode
        /// (`VCVTR`).
        round_zero: bool,
    },
    /// `VCVT.F32/F64.S32/U32 Sd/Dd, Sm`: integer to float.
    CvtFromInt {
        /// The destination is a double.
        dp: bool,
        /// Destination.
        d: u8,
        /// Source single.
        m: u8,
        /// Signed rather than unsigned.
        signed: bool,
    },
    /// `VCVT` between float and fixed point, in place in `d` (VFPv3).
    CvtFixed {
        /// Double precision.
        dp: bool,
        /// Source and destination.
        d: u8,
        /// Float to fixed rather than fixed to float.
        to_fixed: bool,
        /// Unsigned rather than signed.
        unsigned: bool,
        /// Container width: 16 or 32.
        size: u8,
        /// Fraction bits, `0..=size`.
        fbits: u8,
    },
    /// `VCVTB`/`VCVTT` between a single and a half in one halfword of a
    /// single (VFPv3 half-precision extension).
    CvtHalf {
        /// Destination single.
        d: u8,
        /// Source single.
        m: u8,
        /// Single to half rather than half to single.
        to_half: bool,
        /// The top halfword (`T`) rather than the bottom (`B`).
        top: bool,
    },
    /// `VMOV Sn, Rt` / `VMOV Rt, Sn`.
    MovCore {
        /// Core to floating point.
        to_fp: bool,
        /// Core register.
        rt: u8,
        /// Single register.
        n: u8,
    },
    /// `VMOV` between two core registers and either one double or two
    /// consecutive singles (DDI 0406C A7.9).
    MovCore2 {
        /// Core to floating point.
        to_fp: bool,
        /// A double rather than two singles.
        dp: bool,
        /// The core register holding the low word / `Sm`.
        rt: u8,
        /// The core register holding the high word / `Sm+1`.
        rt2: u8,
        /// `Dm`, or the first of the two singles.
        m: u8,
    },
    /// `VMOV.32 Dd[x], Rt` / `VMOV.32 Rt, Dn[x]`: a word of a double. The 8-
    /// and 16-bit forms are Advanced SIMD and do not decode here.
    MovScalar {
        /// Core to floating point.
        to_fp: bool,
        /// Core register.
        rt: u8,
        /// The double.
        d: u8,
        /// Which word: 0 low, 1 high.
        index: u8,
    },
    /// `VMRS`/`VMSR`.
    Sys {
        /// System register to core (`VMRS`).
        to_core: bool,
        /// Core register; fifteen with `FPSCR` is `APSR_nzcv`.
        rt: u8,
        /// The four-bit `reg` field (see [`super::vfp::sysreg`]).
        reg: u8,
    },
    /// `VLDR`/`VSTR`.
    Mem {
        /// Load rather than store.
        load: bool,
        /// Double precision.
        dp: bool,
        /// The register.
        d: u8,
        /// Base register; fifteen is the literal form.
        rn: u8,
        /// Byte offset, already scaled.
        imm: u32,
        /// Add the offset rather than subtract it.
        add: bool,
    },
    /// `VLDM`/`VSTM` (and `VPUSH`/`VPOP`, and the deprecated
    /// `FLDMX`/`FSTMX`).
    Multi {
        /// Load rather than store.
        load: bool,
        /// Double registers.
        dp: bool,
        /// First register.
        d: u8,
        /// Base register.
        rn: u8,
        /// Number of registers.
        count: u8,
        /// Increment after rather than decrement before.
        add: bool,
        /// Write the updated base back.
        writeback: bool,
        /// The `imm8` field, which is the number of *words* the base moves by
        /// — one more than twice `count` for `FLDMX`/`FSTMX`.
        words: u8,
    },
}

impl VfpInsn {
    /// Whether this is a data-processing instruction — the group the
    /// Cortex-A9 makes UNDEFINED when `FPSCR.Len` or `Stride` is non-zero.
    #[must_use]
    pub const fn is_data_processing(self) -> bool {
        matches!(
            self,
            VfpInsn::Data { .. }
                | VfpInsn::Unary { .. }
                | VfpInsn::MovImm { .. }
                | VfpInsn::Cmp { .. }
                | VfpInsn::CvtPrec { .. }
                | VfpInsn::CvtToInt { .. }
                | VfpInsn::CvtFromInt { .. }
                | VfpInsn::CvtFixed { .. }
                | VfpInsn::CvtHalf { .. }
        )
    }

    /// Whether the instruction first appeared in VFPv3: the immediate `VMOV`,
    /// the fixed-point `VCVT`, and the half-precision conversions. A VFPv2
    /// part must take them as UNDEFINED.
    #[must_use]
    pub const fn needs_v3(self) -> bool {
        matches!(
            self,
            VfpInsn::MovImm { .. } | VfpInsn::CvtFixed { .. } | VfpInsn::CvtHalf { .. }
        )
    }

    /// The highest double register the instruction names, if it names any —
    /// what a D16 part checks against sixteen.
    #[must_use]
    pub const fn max_double(self) -> Option<u8> {
        const fn max(a: u8, b: u8) -> u8 {
            if a > b { a } else { b }
        }
        match self {
            VfpInsn::Data {
                dp: true, d, n, m, ..
            } => Some(max(d, max(n, m))),
            VfpInsn::Unary { dp: true, d, m, .. } | VfpInsn::Cmp { dp: true, d, m, .. } => {
                Some(max(d, m))
            }
            VfpInsn::MovImm { dp: true, d, .. }
            | VfpInsn::CvtFixed { dp: true, d, .. }
            | VfpInsn::CvtFromInt { dp: true, d, .. }
            | VfpInsn::Mem { dp: true, d, .. }
            | VfpInsn::MovScalar { d, .. } => Some(d),
            VfpInsn::CvtToInt { dp: true, m, .. } | VfpInsn::MovCore2 { dp: true, m, .. } => {
                Some(m)
            }
            VfpInsn::CvtPrec { to_double, d, m } => Some(if to_double { d } else { m }),
            VfpInsn::Multi {
                dp: true, d, count, ..
            } => Some(d + count - 1),
            _ => None,
        }
    }

    /// Print this instruction with its condition, in UAL.
    #[must_use]
    pub const fn display(self, cond: Cond) -> Display {
        Display { insn: self, cond }
    }
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

/// Whether a word is in the VFP part of the coprocessor space at all:
/// `coproc == 0b101x`, in the `110x` or `1110` groups. Does not look at the
/// condition.
#[must_use]
#[inline]
pub const fn is_vfp_space(w: u32) -> bool {
    field(w, 11, 9) == 0b101 && (field(w, 27, 25) == 0b110 || field(w, 27, 24) == 0b1110)
}

/// Decode a VFP word, or `None` when it is in the VFP space but not an
/// instruction a VFPv3 part without Advanced SIMD has — which the executor
/// turns into an Undefined Instruction exception. The condition field is
/// ignored; see the module docs.
#[must_use]
pub fn decode(w: u32) -> Option<VfpInsn> {
    if !is_vfp_space(w) {
        return None;
    }
    if field(w, 27, 24) == 0b1110 {
        if bit(w, 4) {
            decode_transfer(w)
        } else {
            decode_data(w)
        }
    } else {
        decode_load_store(w)
    }
}

/// The destination register at a precision: `Vd:D` or `D:Vd`.
const fn reg_d(w: u32, dp: bool) -> u8 {
    reg(field(w, 15, 12), (w >> 22) & 1, dp)
}

/// The first operand register: `Vn:N` or `N:Vn`.
const fn reg_n(w: u32, dp: bool) -> u8 {
    reg(field(w, 19, 16), (w >> 7) & 1, dp)
}

/// The second operand register: `Vm:M` or `M:Vm`.
const fn reg_m(w: u32, dp: bool) -> u8 {
    reg(field(w, 3, 0), (w >> 5) & 1, dp)
}

const fn reg(four: u32, one: u32, dp: bool) -> u8 {
    if dp {
        ((one << 4) | four) as u8
    } else {
        ((four << 1) | one) as u8
    }
}

/// A7.5's table: `opc1` is bits 23:20 with bit 22 (`D`) masked out.
fn decode_data(w: u32) -> Option<VfpInsn> {
    let dp = bit(w, 8);
    let (d, n, m) = (reg_d(w, dp), reg_n(w, dp), reg_m(w, dp));
    let opc1 = field(w, 23, 20) & 0b1011;
    let op6 = bit(w, 6);
    let op = match (opc1, op6) {
        (0b0000, false) => DataOp::Mla,
        (0b0000, true) => DataOp::Mls,
        (0b0001, false) => DataOp::Nmls,
        (0b0001, true) => DataOp::Nmla,
        (0b0010, false) => DataOp::Mul,
        (0b0010, true) => DataOp::Nmul,
        (0b0011, false) => DataOp::Add,
        (0b0011, true) => DataOp::Sub,
        (0b1000, false) => DataOp::Div,
        (0b1011, false) => {
            // `VMOV (immediate)`: bits 7:5 must be zero, the two nibbles of
            // the constant are 19:16 and 3:0.
            if field(w, 7, 4) != 0 {
                return None;
            }
            let imm8 = ((field(w, 19, 16) << 4) | field(w, 3, 0)) as u8;
            return Some(VfpInsn::MovImm { dp, d, imm8 });
        }
        (0b1011, true) => return decode_other(w, dp),
        // `1x01` and `1x10` are VFPv4's fused multiply-adds; `1000` with
        // bit 6 set is unallocated.
        _ => return None,
    };
    Some(VfpInsn::Data { op, dp, d, n, m })
}

/// Table A7-17: `opc1 == 1x11`, `opc3 == x1`, selected by `opc2` (19:16).
fn decode_other(w: u32, dp: bool) -> Option<VfpInsn> {
    let (d, m) = (reg_d(w, dp), reg_m(w, dp));
    let opc2 = field(w, 19, 16);
    let op7 = bit(w, 7);
    let unary = |op| Some(VfpInsn::Unary { op, dp, d, m });
    match opc2 {
        0b0000 if !op7 => unary(UnaryOp::Mov),
        0b0000 => unary(UnaryOp::Abs),
        0b0001 if !op7 => unary(UnaryOp::Neg),
        0b0001 => unary(UnaryOp::Sqrt),
        // `VCVTB`/`VCVTT`: single precision only; bit 16 is the direction,
        // bit 7 the halfword.
        0b0010 | 0b0011 if !dp => Some(VfpInsn::CvtHalf {
            d: reg_d(w, false),
            m: reg_m(w, false),
            to_half: bit(w, 16),
            top: op7,
        }),
        0b0100 => Some(VfpInsn::Cmp {
            dp,
            d,
            m,
            with_zero: false,
            signal_all: op7,
        }),
        // The `#0.0` form requires `Vm:M` to be zero.
        0b0101 if field(w, 5, 5) == 0 && field(w, 3, 0) == 0 => Some(VfpInsn::Cmp {
            dp,
            d,
            m: 0,
            with_zero: true,
            signal_all: op7,
        }),
        // `VCVT` between double and single: `sz` names the *source*.
        0b0111 if op7 => Some(VfpInsn::CvtPrec {
            to_double: !dp,
            d: reg_d(w, !dp),
            m: reg_m(w, dp),
        }),
        // Integer to float: the source is always a single; bit 7 is signed.
        0b1000 => Some(VfpInsn::CvtFromInt {
            dp,
            d,
            m: reg_m(w, false),
            signed: op7,
        }),
        // Float to integer: the destination is always a single; bit 16 is
        // signed, bit 7 set is round-toward-zero.
        0b1100 | 0b1101 => Some(VfpInsn::CvtToInt {
            dp,
            d: reg_d(w, false),
            m,
            signed: opc2 & 1 != 0,
            round_zero: op7,
        }),
        // `1 op 1 U`: fixed point. `op` (bit 18) is the direction, `U` (bit
        // 16) the signedness, `sx` (bit 7) the container width, and the
        // fraction bits are `size - imm4:i`.
        0b1010 | 0b1011 | 0b1110 | 0b1111 => {
            let size: u32 = if op7 { 32 } else { 16 };
            let imm5 = (field(w, 3, 0) << 1) | field(w, 5, 5);
            // `frac_bits < 0` is UNPREDICTABLE (A8.8, `VCVT` fixed-point); refused.
            if imm5 > size {
                return None;
            }
            Some(VfpInsn::CvtFixed {
                dp,
                d,
                to_fixed: bit(w, 18),
                unsigned: bit(w, 16),
                size: size as u8,
                fbits: (size - imm5) as u8,
            })
        }
        _ => None,
    }
}

/// A7.8: 8, 16 and 32-bit transfers between core and extension registers.
fn decode_transfer(w: u32) -> Option<VfpInsn> {
    let rt = field(w, 15, 12) as u8;
    let load = bit(w, 20);
    let c = bit(w, 8);
    let a = field(w, 23, 21);
    // Bits 3:0 are always zero in this group.
    if field(w, 3, 0) != 0 {
        return None;
    }
    match (c, a) {
        // `VMOV Sn, Rt` / `VMOV Rt, Sn`: bits 6:5 zero.
        (false, 0b000) if field(w, 6, 5) == 0 => Some(VfpInsn::MovCore {
            to_fp: !load,
            rt,
            n: reg_n(w, false),
        }),
        // `VMSR` / `VMRS`: bits 7:5 zero.
        (false, 0b111) if field(w, 7, 5) == 0 => Some(VfpInsn::Sys {
            to_core: load,
            rt,
            reg: field(w, 19, 16) as u8,
        }),
        // `VMOV.32 Dd[x], Rt`: `opc1 == 0x` (bits 22 clear, 21 the index),
        // `opc2 == 00`. Every other size is Advanced SIMD, and so is `VDUP`
        // (bit 23 set on the store side).
        (true, _) if !bit(w, 23) && !bit(w, 22) && field(w, 6, 5) == 0 => {
            Some(VfpInsn::MovScalar {
                to_fp: !load,
                rt,
                d: reg_n(w, true),
                index: u8::from(bit(w, 21)),
            })
        }
        _ => None,
    }
}

/// A7.6 (extension register load/store) and A7.9 (64-bit transfers), which
/// share the `110x` window.
fn decode_load_store(w: u32) -> Option<VfpInsn> {
    let dp = bit(w, 8);
    let p = bit(w, 24);
    let u = bit(w, 23);
    let wb = bit(w, 21);
    let load = bit(w, 20);
    let rn = field(w, 19, 16) as u8;
    let imm8 = field(w, 7, 0);

    // `P:U:W == 000` with bit 22 set is the 64-bit transfer group; with it
    // clear it is unallocated.
    if !p && !u && !wb {
        if !bit(w, 22) || field(w, 7, 6) != 0 || !bit(w, 4) {
            return None;
        }
        return Some(VfpInsn::MovCore2 {
            to_fp: !load,
            dp,
            rt: field(w, 15, 12) as u8,
            rt2: field(w, 19, 16) as u8,
            m: reg_m(w, dp),
        });
    }
    let d = reg_d(w, dp);
    if p && !wb {
        return Some(VfpInsn::Mem {
            load,
            dp,
            d,
            rn,
            imm: imm8 * 4,
            add: u,
        });
    }
    // The multiples: increment-after with or without writeback, or
    // decrement-before with it. `P == U` with writeback is unallocated.
    let add = match (p, u) {
        (false, true) => true,
        (true, false) => false,
        _ => return None,
    };
    let count = if dp { imm8 / 2 } else { imm8 };
    // No registers, more than sixteen doubles, or a list running past the
    // end of the file is UNPREDICTABLE (A8.8, `VLDM`/`VSTM`); refused.
    if count == 0 || (dp && count > 16) || u32::from(d) + count > 32 {
        return None;
    }
    // `FLDMX`/`FSTMX` (odd `imm8` on the double form) exist in the
    // increment-after and decrement-before-with-writeback forms like the
    // rest; a single-precision odd count is just an odd count.
    Some(VfpInsn::Multi {
        load,
        dp,
        d,
        rn,
        count: count as u8,
        add,
        writeback: wb,
        words: imm8 as u8,
    })
}

// ---------------------------------------------------------------------------
// Disassembly — the same description, printed
// ---------------------------------------------------------------------------

/// A register at a precision.
struct R(bool, u8);

impl fmt::Display for R {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}{}", if self.0 { 'd' } else { 's' }, self.1)
    }
}

/// `.F32` or `.F64`.
const fn ty(dp: bool) -> &'static str {
    if dp { ".F64" } else { ".F32" }
}

/// The name `VMRS`/`VMSR` prints for a `reg` field.
const fn sysreg_name(reg: u8) -> &'static str {
    match reg {
        0b0000 => "fpsid",
        0b0001 => "fpscr",
        0b0110 => "mvfr1",
        0b0111 => "mvfr0",
        0b1000 => "fpexc",
        0b1001 => "fpinst",
        0b1010 => "fpinst2",
        _ => "<reserved>",
    }
}

/// The decimal value of a `VFPExpandImm` constant, printed with integers only.
///
/// Every such constant is `±(16 + frac) × 2^(e − 4)` with `e` in `−3..=4`,
/// so the value is `n / 2^k` with `k` in `0..=7` and has an exact decimal
/// expansion of at most `k` digits — no host float needed to print it.
struct Imm(u8);

impl fmt::Display for Imm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let imm8 = u32::from(self.0);
        let n = 16 + (imm8 & 0xf);
        let cd = (imm8 >> 4) & 3;
        // Unbiased exponent: 1 + cd when b is clear, cd − 3 when it is set.
        let e = if imm8 & 0x40 == 0 {
            1 + cd as i32
        } else {
            cd as i32 - 3
        };
        let k = (4 - e) as u32; // value = n / 2^k, 0 <= k <= 7
        let sign = if imm8 & 0x80 != 0 { "-" } else { "" };
        let whole = n >> k;
        let rem = n & ((1 << k) - 1);
        // rem / 2^k == rem × 5^k / 10^k: exactly k decimal digits.
        let digits = rem * 5u32.pow(k);
        if k == 0 || rem == 0 {
            return write!(f, "#{sign}{whole}.0");
        }
        let text = alloc::format!("{digits:0width$}", width = k as usize);
        write!(f, "#{sign}{whole}.{}", text.trim_end_matches('0'))
    }
}

/// A register list, `{d0-d15}` or `{s3}`.
struct List(bool, u8, u8);

impl fmt::Display for List {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let List(dp, first, count) = *self;
        if count == 1 {
            write!(f, "{{{}}}", R(dp, first))
        } else {
            write!(f, "{{{}-{}}}", R(dp, first), R(dp, first + count - 1))
        }
    }
}

/// A [`VfpInsn`] with its condition, printable.
#[derive(Debug, Clone, Copy)]
pub struct Display {
    insn: VfpInsn,
    cond: Cond,
}

impl fmt::Display for Display {
    #[allow(clippy::too_many_lines)] // One arm per form; splitting hides the map.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let c = self.cond;
        match self.insn {
            VfpInsn::Data { op, dp, d, n, m } => write!(
                f,
                "{}{c}{} {}, {}, {}",
                op.mnemonic(),
                ty(dp),
                R(dp, d),
                R(dp, n),
                R(dp, m)
            ),
            VfpInsn::Unary { op, dp, d, m } => {
                write!(
                    f,
                    "{}{c}{} {}, {}",
                    op.mnemonic(),
                    ty(dp),
                    R(dp, d),
                    R(dp, m)
                )
            }
            VfpInsn::MovImm { dp, d, imm8 } => {
                write!(f, "VMOV{c}{} {}, {}", ty(dp), R(dp, d), Imm(imm8))
            }
            VfpInsn::Cmp {
                dp,
                d,
                m,
                with_zero,
                signal_all,
            } => {
                let e = if signal_all { "E" } else { "" };
                if with_zero {
                    write!(f, "VCMP{e}{c}{} {}, #0.0", ty(dp), R(dp, d))
                } else {
                    write!(f, "VCMP{e}{c}{} {}, {}", ty(dp), R(dp, d), R(dp, m))
                }
            }
            VfpInsn::CvtPrec { to_double, d, m } => {
                if to_double {
                    write!(f, "VCVT{c}.F64.F32 {}, {}", R(true, d), R(false, m))
                } else {
                    write!(f, "VCVT{c}.F32.F64 {}, {}", R(false, d), R(true, m))
                }
            }
            VfpInsn::CvtToInt {
                dp,
                d,
                m,
                signed,
                round_zero,
            } => {
                let r = if round_zero { "" } else { "R" };
                let int = if signed { ".S32" } else { ".U32" };
                write!(f, "VCVT{r}{c}{int}{} {}, {}", ty(dp), R(false, d), R(dp, m))
            }
            VfpInsn::CvtFromInt { dp, d, m, signed } => {
                let int = if signed { ".S32" } else { ".U32" };
                write!(f, "VCVT{c}{}{int} {}, {}", ty(dp), R(dp, d), R(false, m))
            }
            VfpInsn::CvtFixed {
                dp,
                d,
                to_fixed,
                unsigned,
                size,
                fbits,
            } => {
                let fx = alloc::format!(".{}{size}", if unsigned { 'U' } else { 'S' });
                if to_fixed {
                    write!(
                        f,
                        "VCVT{c}{fx}{} {}, {}, #{fbits}",
                        ty(dp),
                        R(dp, d),
                        R(dp, d)
                    )
                } else {
                    write!(
                        f,
                        "VCVT{c}{}{fx} {}, {}, #{fbits}",
                        ty(dp),
                        R(dp, d),
                        R(dp, d)
                    )
                }
            }
            VfpInsn::CvtHalf { d, m, to_half, top } => {
                let t = if top { "T" } else { "B" };
                let types = if to_half { ".F16.F32" } else { ".F32.F16" };
                write!(f, "VCVT{t}{c}{types} {}, {}", R(false, d), R(false, m))
            }
            VfpInsn::MovCore { to_fp, rt, n } => {
                if to_fp {
                    write!(f, "VMOV{c} {}, {}", R(false, n), RegName(rt))
                } else {
                    write!(f, "VMOV{c} {}, {}", RegName(rt), R(false, n))
                }
            }
            VfpInsn::MovCore2 {
                to_fp,
                dp,
                rt,
                rt2,
                m,
            } => {
                let (rt, rt2) = (RegName(rt), RegName(rt2));
                match (to_fp, dp) {
                    (true, true) => write!(f, "VMOV{c} {}, {rt}, {rt2}", R(true, m)),
                    (false, true) => write!(f, "VMOV{c} {rt}, {rt2}, {}", R(true, m)),
                    (true, false) => write!(
                        f,
                        "VMOV{c} {}, {}, {rt}, {rt2}",
                        R(false, m),
                        R(false, m + 1)
                    ),
                    (false, false) => write!(
                        f,
                        "VMOV{c} {rt}, {rt2}, {}, {}",
                        R(false, m),
                        R(false, m + 1)
                    ),
                }
            }
            VfpInsn::MovScalar {
                to_fp,
                rt,
                d,
                index,
            } => {
                if to_fp {
                    write!(f, "VMOV{c}.32 {}[{index}], {}", R(true, d), RegName(rt))
                } else {
                    write!(f, "VMOV{c}.32 {}, {}[{index}]", RegName(rt), R(true, d))
                }
            }
            VfpInsn::Sys { to_core, rt, reg } => {
                let name = sysreg_name(reg);
                if !to_core {
                    write!(f, "VMSR{c} {name}, {}", RegName(rt))
                } else if rt == 15 && reg == 1 {
                    write!(f, "VMRS{c} APSR_nzcv, {name}")
                } else {
                    write!(f, "VMRS{c} {}, {name}", RegName(rt))
                }
            }
            VfpInsn::Mem {
                load,
                dp,
                d,
                rn,
                imm,
                add,
            } => {
                let op = if load { "VLDR" } else { "VSTR" };
                let sign = if add { "" } else { "-" };
                if imm == 0 && add {
                    write!(f, "{op}{c} {}, [{}]", R(dp, d), RegName(rn))
                } else {
                    write!(f, "{op}{c} {}, [{}, #{sign}{imm}]", R(dp, d), RegName(rn))
                }
            }
            VfpInsn::Multi {
                load,
                dp,
                d,
                rn,
                count,
                add,
                writeback,
                words,
            } => {
                let list = List(dp, d, count);
                let fmx = dp && words & 1 != 0;
                // `VPUSH`/`VPOP` is the `SP` spelling of the same encoding
                // and is what a listing should say when it sees one.
                if rn == 13 && writeback && !fmx {
                    if load && add {
                        return write!(f, "VPOP{c} {list}");
                    }
                    if !load && !add {
                        return write!(f, "VPUSH{c} {list}");
                    }
                }
                let bang = if writeback { "!" } else { "" };
                let mode = if add { "IA" } else { "DB" };
                if fmx {
                    let op = if load { "FLDM" } else { "FSTM" };
                    write!(f, "{op}{mode}X{c} {}{bang}, {list}", RegName(rn))
                } else {
                    let op = if load { "VLDM" } else { "VSTM" };
                    write!(f, "{op}{mode}{c} {}{bang}, {list}", RegName(rn))
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    /// Each word was produced by `arm-none-eabi-as -mfpu=vfpv3 -mcpu=cortex-a9`
    /// and the text is its `objdump -d` (lower case, tabs as spaces). The
    /// immediate `VMOV` is printed as a value here where objdump prints the
    /// raw byte, so those rows carry the value instead.
    const LISTING: &[(u32, &str)] = &[
        (0xee31_0b02, "vadd.f64 d0, d1, d2"),
        (0xee30_0a81, "vadd.f32 s0, s1, s2"),
        (0xee71_0bef, "vsub.f64 d16, d17, d31"),
        (0xee62_1aa3, "vmul.f32 s3, s5, s7"),
        (0xee24_3b45, "vnmul.f64 d3, d4, d5"),
        (0xee81_0b02, "vdiv.f64 d0, d1, d2"),
        (0xee00_0a81, "vmla.f32 s0, s1, s2"),
        (0xee01_0b42, "vmls.f64 d0, d1, d2"),
        (0xee10_0ac1, "vnmla.f32 s0, s1, s2"),
        (0xee11_0b02, "vnmls.f64 d0, d1, d2"),
        (0xeeb0_1bc2, "vabs.f64 d1, d2"),
        (0xeef1_0a41, "vneg.f32 s1, s2"),
        (0xeeb1_1be1, "vsqrt.f64 d1, d17"),
        (0xeeb0_1b42, "vmov.f64 d1, d2"),
        (0xeef0_0a41, "vmov.f32 s1, s2"),
        (0xeeb7_0a00, "vmov.f32 s0, #1.0"),
        (0xeeb8_0b04, "vmov.f64 d0, #-2.5"),
        (0xeef4_1b00, "vmov.f64 d17, #0.125"),
        (0xeeb4_0b41, "vcmp.f64 d0, d1"),
        (0xeeb4_0ae0, "vcmpe.f32 s0, s1"),
        (0xeeb5_0b40, "vcmp.f64 d0, #0.0"),
        (0xeef5_1ac0, "vcmpe.f32 s3, #0.0"),
        (0xeeb7_0ae0, "vcvt.f64.f32 d0, s1"),
        (0xeef7_0be1, "vcvt.f32.f64 s1, d17"),
        (0xeebd_0bc1, "vcvt.s32.f64 s0, d1"),
        (0xeebd_0b41, "vcvtr.s32.f64 s0, d1"),
        (0xeebc_0ae0, "vcvt.u32.f32 s0, s1"),
        (0xeebc_0a60, "vcvtr.u32.f32 s0, s1"),
        (0xeeb8_0be0, "vcvt.f64.s32 d0, s1"),
        (0xeeb8_0a60, "vcvt.f32.u32 s0, s1"),
        (0xeebe_0b46, "vcvt.s16.f64 d0, d0, #4"),
        (0xeebf_0ac0, "vcvt.u32.f32 s0, s0, #32"),
        (0xeeba_3bef, "vcvt.f64.s32 d3, d3, #1"),
        (0xeebb_1a40, "vcvt.f32.u16 s2, s2, #16"),
        (0xeeb2_0a60, "vcvtb.f32.f16 s0, s1"),
        (0xeeb3_0ae0, "vcvtt.f16.f32 s0, s1"),
        (0xed90_0b02, "vldr d0, [r0, #8]"),
        (0xed51_0a01, "vldr s1, [r1, #-4]"),
        (0xedcd_1b00, "vstr d17, [sp]"),
        (0xed9f_0b04, "vldr d0, [pc, #16]"),
        (0xecb0_0b20, "vldmia r0!, {d0-d15}"),
        (0xec90_0a04, "vldmia r0, {s0-s3}"),
        (0xed60_0b20, "vstmdb r0!, {d16-d31}"),
        (0xed2d_8b10, "vpush {d8-d15}"),
        (0xecbd_8a10, "vpop {s16-s31}"),
        (0xecb0_0b21, "fldmiax r0!, {d0-d15}"),
        (0xeca0_0b21, "fstmiax r0!, {d0-d15}"),
        (0xee00_1a10, "vmov s0, r1"),
        (0xee1f_2a90, "vmov r2, s31"),
        (0xec42_1b10, "vmov d0, r1, r2"),
        (0xec52_1b3f, "vmov r1, r2, d31"),
        (0xec42_1a10, "vmov s0, s1, r1, r2"),
        (0xec52_1a1f, "vmov r1, r2, s30, s31"),
        (0xee21_3b90, "vmov.32 d17[1], r3"),
        (0xee11_3b90, "vmov.32 r3, d17[0]"),
        (0xeef1_0a10, "vmrs r0, fpscr"),
        (0xeef1_fa10, "vmrs apsr_nzcv, fpscr"),
        (0xeee1_1a10, "vmsr fpscr, r1"),
        (0xeef8_0a10, "vmrs r0, fpexc"),
        (0xeee8_0a10, "vmsr fpexc, r0"),
        (0xeef0_1a10, "vmrs r1, fpsid"),
        (0xeef7_2a10, "vmrs r2, mvfr0"),
        (0xeef6_2a10, "vmrs r2, mvfr1"),
        (0x0e31_0b02, "vaddeq.f64 d0, d1, d2"),
        (0xece0_0b20, "vstmia r0!, {d16-d31}"),
    ];

    #[test]
    fn the_listing_matches_binutils() {
        for &(word, text) in LISTING {
            let insn = decode(word).unwrap_or_else(|| panic!("{word:08x} did not decode"));
            let cond = Cond(field(word, 31, 28) as u8);
            let got = insn.display(cond).to_string().to_lowercase();
            assert_eq!(got, text, "{word:08x}");
        }
    }

    #[test]
    fn the_fused_and_simd_forms_do_not_decode() {
        // vfma.f64 d0, d1, d2 (VFPv4).
        assert_eq!(decode(0xeea1_0b02), None);
        // vmov.8 d0[1], r1 (Advanced SIMD scalar).
        assert_eq!(decode(0xee40_1b30), None);
        // vdup.32 d0, r1.
        assert_eq!(decode(0xee80_1b10), None);
        // Not the VFP space at all: an MCR to p15.
        assert_eq!(decode(0xee01_0f10), None);
        // vldmia with zero registers.
        assert_eq!(decode(0xecb0_0b00), None);
    }

    #[test]
    fn every_constant_prints_exactly() {
        // Every one of the 256 constants, checked against the bits it expands
        // to: the printed decimal parses back to the same value.
        for imm8 in 0..=255u8 {
            let text = Imm(imm8).to_string();
            let parsed: f64 = text[1..].parse().expect("a decimal");
            let bits = super::super::vfp::expand_imm(imm8, true);
            assert_eq!(parsed.to_bits(), bits, "imm8 {imm8:#04x} printed {text}");
        }
    }
}
