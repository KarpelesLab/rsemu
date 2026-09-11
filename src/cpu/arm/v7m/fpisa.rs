//! The FPv4-SP / FPv5-SP encodings, described once.
//!
//! Same rule as [`super::isa`], for the same reason (CLAUDE.md, "CPU cores"):
//! one description serves the interpreter and the disassembler, so the two
//! cannot drift. [`decode`] turns a thirty-two-bit T32 coprocessor encoding
//! into an [`FpInsn`], and [`FpInsn`]'s [`fmt::Display`] is the disassembler
//! for the same value. Nothing re-reads the raw encoding.
//!
//! # Where these live in the T32 map
//!
//! Three windows of the coprocessor space, all of which A5.3's tables already
//! route to "Coprocessor, Advanced SIMD, and Floating-point instructions":
//!
//! | First halfword | Group |
//! | --- | --- |
//! | `0xEE__` | data processing, and the 8/16/32-bit core-register transfers |
//! | `0xEC__`, `0xED__` | extension-register load/store, and the 64-bit transfers |
//! | `0xFE__` | the FPv5 additions, which are unconditional |
//!
//! The `coproc` field (bits 11:8 of the second halfword) is `0b1010` for
//! single precision and `0b1011` for double. This core is single-precision
//! only, so `0b1011` decodes to nothing here and the caller sees an ordinary
//! coprocessor encoding — which the interpreter then turns into UNDEFINED when
//! an FPU is present, because a double-precision instruction on an FPv4-SP
//! part is exactly that.
//!
//! # Sources
//!
//! *ARMv7-M Architecture Reference Manual*, ARM DDI 0403E: A6.4 (the
//! floating-point encoding tables), A7.5 (the data-processing sub-table) and
//! A7.7's alphabetical list for each instruction's field layout. The FPv5
//! group (`VSEL`, `VMAXNM`/`VMINNM`, `VCVTA/N/P/M`, `VRINT*`) is the Armv8-M /
//! FPv5 addition, encoded in the unconditional `0b1111 1110` window. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

use core::fmt;

use super::fp::S_NAMES;
use super::isa::{Cond, RegName, bit, field};

// ---------------------------------------------------------------------------
// The decoded form
// ---------------------------------------------------------------------------

/// A two-operand floating-point data-processing operation, `Sd = Sn op Sm`.
///
/// The four multiply-accumulates in the middle are **chained**, not fused:
/// DDI 0403E writes `VMLA` as `FPAdd(Sd, FPMul(Sn, Sm))`, two roundings, and
/// only the `VF*` four go through `FPMulAdd`. Collapsing them would change the
/// last bit of a great many results.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DataOp {
    /// `VMLA`: `Sd = Sd + Sn × Sm`, rounded twice.
    Mla,
    /// `VMLS`: `Sd = Sd − Sn × Sm`, rounded twice.
    Mls,
    /// `VNMLA`: `Sd = −Sd − Sn × Sm`.
    Nmla,
    /// `VNMLS`: `Sd = −Sd + Sn × Sm`.
    Nmls,
    /// `VMUL`.
    Mul,
    /// `VNMUL`: `Sd = −(Sn × Sm)`.
    Nmul,
    /// `VADD`.
    Add,
    /// `VSUB`.
    Sub,
    /// `VDIV`.
    Div,
    /// `VFMA`: `Sd = Sd + Sn × Sm`, fused, rounded once.
    Fma,
    /// `VFMS`: `Sd = Sd − Sn × Sm`, fused.
    Fms,
    /// `VFNMA`: `Sd = −Sd − Sn × Sm`, fused.
    Fnma,
    /// `VFNMS`: `Sd = −Sd + Sn × Sm`, fused.
    Fnms,
    /// `VMAXNM` (FPv5).
    Maxnm,
    /// `VMINNM` (FPv5).
    Minnm,
}

impl DataOp {
    /// The mnemonic, without the `.F32`.
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
            DataOp::Fma => "VFMA",
            DataOp::Fms => "VFMS",
            DataOp::Fnma => "VFNMA",
            DataOp::Fnms => "VFNMS",
            DataOp::Maxnm => "VMAXNM",
            DataOp::Minnm => "VMINNM",
        }
    }

    /// Whether the operation is one of the FPv5 additions, which an FPv4-SP
    /// part must reject as UNDEFINED.
    #[must_use]
    pub const fn is_v5(self) -> bool {
        matches!(self, DataOp::Maxnm | DataOp::Minnm)
    }
}

/// A one-operand data-processing operation, `Sd = op Sm`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum UnaryOp {
    /// `VMOV Sd, Sm`.
    Mov,
    /// `VABS`.
    Abs,
    /// `VNEG`.
    Neg,
    /// `VSQRT`.
    Sqrt,
    /// `VRINTR` — round to integral in the `FPSCR` mode, inexact not raised
    /// (FPv5).
    RintR,
    /// `VRINTZ` — round toward zero (FPv5).
    RintZ,
    /// `VRINTX` — the `FPSCR` mode, and the one form that *does* raise
    /// inexact (FPv5).
    RintX,
}

impl UnaryOp {
    /// The mnemonic, without the `.F32`.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            UnaryOp::Mov => "VMOV",
            UnaryOp::Abs => "VABS",
            UnaryOp::Neg => "VNEG",
            UnaryOp::Sqrt => "VSQRT",
            UnaryOp::RintR => "VRINTR",
            UnaryOp::RintZ => "VRINTZ",
            UnaryOp::RintX => "VRINTX",
        }
    }

    /// Whether the operation is one of the FPv5 additions.
    #[must_use]
    pub const fn is_v5(self) -> bool {
        matches!(self, UnaryOp::RintR | UnaryOp::RintZ | UnaryOp::RintX)
    }
}

/// The explicit rounding mode the FPv5 `VRINTA/N/P/M` and `VCVTA/N/P/M`
/// carry in their `RM` field, which overrides `FPSCR.RMode` for that one
/// instruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RoundMode {
    /// `A` — ties away from zero.
    Away,
    /// `N` — ties to even.
    Nearest,
    /// `P` — toward `+∞`.
    Plus,
    /// `M` — toward `−∞`.
    Minus,
}

impl RoundMode {
    /// Decode the two-bit `RM` field: `00` A, `01` N, `10` P, `11` M.
    #[must_use]
    pub const fn from_rm(rm: u32) -> RoundMode {
        match rm & 3 {
            0b00 => RoundMode::Away,
            0b01 => RoundMode::Nearest,
            0b10 => RoundMode::Plus,
            _ => RoundMode::Minus,
        }
    }

    /// The letter the mnemonic ends with.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            RoundMode::Away => "A",
            RoundMode::Nearest => "N",
            RoundMode::Plus => "P",
            RoundMode::Minus => "M",
        }
    }

    /// The [`crate::float::Round`] it names.
    #[must_use]
    pub const fn round(self) -> crate::float::Round {
        match self {
            RoundMode::Away => crate::float::Round::TiesAway,
            RoundMode::Nearest => crate::float::Round::TiesEven,
            RoundMode::Plus => crate::float::Round::TowardPositive,
            RoundMode::Minus => crate::float::Round::TowardNegative,
        }
    }
}

/// One decoded floating-point instruction.
///
/// Every register number is a **single-precision** number, `0..=31`: the
/// encodings spell `Vd:D` and this is that already assembled, so nothing
/// downstream has to remember which half of the number lives where.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FpInsn {
    /// `Sd = Sn op Sm`, or an accumulate into `Sd`.
    Data {
        /// Which operation.
        op: DataOp,
        /// Destination and, for the accumulates, one source.
        d: u8,
        /// First multiplicand or addend.
        n: u8,
        /// Second source.
        m: u8,
    },
    /// `Sd = op Sm`.
    Unary {
        /// Which operation.
        op: UnaryOp,
        /// Destination.
        d: u8,
        /// Source.
        m: u8,
    },
    /// `VMOV Sd, #imm` — the already-expanded `VFPExpandImm` value.
    MovImm {
        /// Destination.
        d: u8,
        /// The single-precision bit pattern.
        imm: u32,
    },
    /// `VCMP` / `VCMPE`, against `Sm` or against `+0.0`.
    Cmp {
        /// The left operand.
        d: u8,
        /// The right operand, ignored when `with_zero`.
        m: u8,
        /// The `VCMP Sd, #0.0` form.
        with_zero: bool,
        /// `VCMPE`: a quiet NaN raises invalid too.
        signal_all: bool,
    },
    /// `VCVT`/`VCVTR` between a single and a 32-bit integer, both in `S`
    /// registers.
    CvtInt {
        /// Destination.
        d: u8,
        /// Source.
        m: u8,
        /// Float to integer rather than integer to float.
        to_int: bool,
        /// Signed rather than unsigned.
        signed: bool,
        /// Round toward zero regardless of `FPSCR.RMode` — `VCVT` rather than
        /// `VCVTR`. Only meaningful when `to_int`.
        round_zero: bool,
    },
    /// `VCVTA/N/P/M.S32.F32` and friends (FPv5): float to integer at a
    /// rounding mode the instruction names.
    CvtMode {
        /// Which mode.
        mode: RoundMode,
        /// Destination.
        d: u8,
        /// Source.
        m: u8,
        /// Signed rather than unsigned.
        signed: bool,
    },
    /// `VRINTA/N/P/M` (FPv5): round to integral at a named mode, staying in
    /// the floating-point format.
    RintMode {
        /// Which mode.
        mode: RoundMode,
        /// Destination.
        d: u8,
        /// Source.
        m: u8,
    },
    /// `VCVT` between a single and a fixed-point value, both in `Sd`.
    CvtFixed {
        /// Source and destination.
        d: u8,
        /// Float to fixed rather than fixed to float.
        to_fixed: bool,
        /// Unsigned rather than signed.
        unsigned: bool,
        /// A 32-bit container rather than a 16-bit one.
        wide: bool,
        /// How many fraction bits.
        frac: u8,
    },
    /// `VCVTB`/`VCVTT` between a single and a half in half of `Sd`/`Sm`.
    CvtHalf {
        /// Destination.
        d: u8,
        /// Source.
        m: u8,
        /// Single to half rather than half to single.
        to_half: bool,
        /// The top halfword rather than the bottom.
        top: bool,
    },
    /// `VSEL<cc>.F32 Sd, Sn, Sm` (FPv5): `Sd = cond ? Sn : Sm`, from the
    /// *integer* condition flags in `APSR`.
    Sel {
        /// The condition the `cc` field names.
        cond: Cond,
        /// Destination.
        d: u8,
        /// Taken when the condition passes.
        n: u8,
        /// Taken when it does not.
        m: u8,
    },
    /// `VMOV` between a core register and a single-precision register.
    MovCore {
        /// Core to floating-point rather than the other way.
        to_fp: bool,
        /// The core register.
        rt: u8,
        /// The single-precision register.
        n: u8,
    },
    /// `VMOV` between two core registers and two consecutive singles.
    MovCore2 {
        /// Core to floating-point rather than the other way.
        to_fp: bool,
        /// The core register paired with `Sm`.
        rt: u8,
        /// The core register paired with `Sm+1`.
        rt2: u8,
        /// The first single-precision register.
        m: u8,
    },
    /// `VMRS`/`VMSR`.
    Sys {
        /// Floating-point to core rather than core to floating-point.
        to_core: bool,
        /// The core register; fifteen with `FPSCR` is `APSR_nzcv`.
        rt: u8,
        /// The four-bit `reg` field: `0` `FPSID`, `1` `FPSCR`, `5` `MVFR2`,
        /// `6` `MVFR1`, `7` `MVFR0`.
        reg: u8,
    },
    /// `VLDR`/`VSTR`.
    Mem {
        /// A load rather than a store.
        load: bool,
        /// The floating-point register.
        d: u8,
        /// The base register; fifteen is the literal form.
        rn: u8,
        /// The byte offset, already multiplied by four.
        imm: u32,
        /// Add the offset rather than subtract it.
        add: bool,
    },
    /// `VLDM`/`VSTM`, and their `VPOP`/`VPUSH` spellings.
    Multi {
        /// A load rather than a store.
        load: bool,
        /// The first floating-point register.
        d: u8,
        /// The base register.
        rn: u8,
        /// How many registers, which is the `imm8` field.
        count: u8,
        /// Increment rather than decrement.
        add: bool,
        /// Write the updated base back.
        writeback: bool,
    },
}

impl FpInsn {
    /// Whether this encoding needs the FPv5 instruction group.
    ///
    /// An FPv4-SP part must take an UNDEFINED UsageFault on one of these
    /// rather than executing it: that is how firmware probes for FPv5
    /// (`ROADMAP.md` §6.1.1).
    #[must_use]
    pub const fn needs_v5(self) -> bool {
        match self {
            FpInsn::Data { op, .. } => op.is_v5(),
            FpInsn::Unary { op, .. } => op.is_v5(),
            FpInsn::CvtMode { .. } | FpInsn::RintMode { .. } | FpInsn::Sel { .. } => true,
            _ => false,
        }
    }
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------

/// The `coproc` value that means single precision. `0b1011` is double, which
/// this core does not have.
const CP_SINGLE: u32 = 0b1010;

/// Decode a thirty-two-bit coprocessor encoding as a floating-point
/// instruction, or `None` if it is not one this core implements.
///
/// `None` is not "undefined": the caller still has an [`super::isa::Insn::Coproc`]
/// and decides between `UFSR.NOCP` and `UFSR.UNDEFINSTR` from the
/// configuration, which is a runtime fact this function does not have.
#[must_use]
pub fn decode(hw1: u16, hw2: u16) -> Option<FpInsn> {
    let w = (u32::from(hw1) << 16) | u32::from(hw2);
    if field(w, 11, 8) != CP_SINGLE {
        return None;
    }
    match field(w, 31, 24) {
        0xee => {
            if bit(w, 4) {
                decode_transfer(w)
            } else {
                decode_data(w)
            }
        }
        0xec | 0xed => decode_load_store(w),
        0xfe => decode_v5(w),
        _ => None,
    }
}

/// `Sd`, assembled from `Vd:D`.
const fn rd(w: u32) -> u8 {
    ((field(w, 15, 12) << 1) | ((w >> 22) & 1)) as u8
}

/// `Sn`, assembled from `Vn:N`.
const fn rn(w: u32) -> u8 {
    ((field(w, 19, 16) << 1) | ((w >> 7) & 1)) as u8
}

/// `Sm`, assembled from `Vm:M`.
const fn rm(w: u32) -> u8 {
    ((field(w, 3, 0) << 1) | ((w >> 5) & 1)) as u8
}

/// A7.5's floating-point data-processing table.
///
/// `opc1` is bits 23:20 with bit 22 masked out, because bit 22 is the `D` half
/// of the destination register and never selects an operation — the manual
/// writes those rows as `0x00`, `1x11` and so on for exactly that reason.
fn decode_data(w: u32) -> Option<FpInsn> {
    let (d, n, m) = (rd(w), rn(w), rm(w));
    let opc1 = field(w, 23, 20) & 0b1011;
    let opc3_low = bit(w, 6);
    let op = match (opc1, opc3_low) {
        (0b0000, false) => DataOp::Mla,
        (0b0000, true) => DataOp::Mls,
        (0b0001, false) => DataOp::Nmls,
        (0b0001, true) => DataOp::Nmla,
        (0b0010, false) => DataOp::Mul,
        (0b0010, true) => DataOp::Nmul,
        (0b0011, false) => DataOp::Add,
        (0b0011, true) => DataOp::Sub,
        (0b1000, false) => DataOp::Div,
        (0b1001, false) => DataOp::Fnms,
        (0b1001, true) => DataOp::Fnma,
        (0b1010, false) => DataOp::Fma,
        (0b1010, true) => DataOp::Fms,
        (0b1011, false) => {
            // `VMOV (immediate)`: the four-bit halves are not adjacent.
            let imm8 = ((field(w, 19, 16) << 4) | field(w, 3, 0)) as u8;
            return Some(FpInsn::MovImm {
                d,
                imm: super::fp::expand_imm(imm8),
            });
        }
        (0b1011, true) => return decode_data_other(w),
        _ => return None,
    };
    Some(FpInsn::Data { op, d, n, m })
}

/// The `opc1 == 1x11`, `opc3<0> == 1` sub-table, selected by `opc2`.
fn decode_data_other(w: u32) -> Option<FpInsn> {
    let (d, m) = (rd(w), rm(w));
    let opc2 = field(w, 19, 16);
    let opc3_high = bit(w, 7);
    let unary = |op| Some(FpInsn::Unary { op, d, m });
    match opc2 {
        0b0000 if !opc3_high => unary(UnaryOp::Mov),
        0b0000 => unary(UnaryOp::Abs),
        0b0001 if !opc3_high => unary(UnaryOp::Neg),
        0b0001 => unary(UnaryOp::Sqrt),
        // `VCVTB`/`VCVTT`: bit 16 picks the direction, bit 7 the halfword.
        0b0010 | 0b0011 => Some(FpInsn::CvtHalf {
            d,
            m,
            to_half: bit(w, 16),
            top: opc3_high,
        }),
        0b0100 | 0b0101 => Some(FpInsn::Cmp {
            d,
            m,
            with_zero: opc2 == 0b0101,
            signal_all: opc3_high,
        }),
        0b0110 if !opc3_high => unary(UnaryOp::RintR),
        0b0110 => unary(UnaryOp::RintZ),
        0b0111 if !opc3_high => unary(UnaryOp::RintX),
        // `opc2 == 0b0111`, `opc3 == 0b11` is `VCVT` between single and
        // double, which needs the double-precision registers this part does
        // not have.
        0b0111 => None,
        0b1000 => Some(FpInsn::CvtInt {
            d,
            m,
            to_int: false,
            signed: opc3_high,
            round_zero: false,
        }),
        0b1100 | 0b1101 => Some(FpInsn::CvtInt {
            d,
            m,
            to_int: true,
            signed: opc2 == 0b1101,
            round_zero: opc3_high,
        }),
        // `1 op 1 U`: fixed point, with `op` in bit 18 and `U` in bit 16.
        0b1010 | 0b1011 | 0b1110 | 0b1111 => {
            let wide = opc3_high;
            let size = if wide { 32 } else { 16 };
            let imm5 = (field(w, 3, 0) << 1) | u32::from(bit(w, 5));
            let frac = size - imm5;
            // `frac_bits` below zero is UNPREDICTABLE; refusing it is the only
            // answer that cannot silently shift by a negative amount.
            if frac > size {
                return None;
            }
            Some(FpInsn::CvtFixed {
                d,
                to_fixed: bit(w, 18),
                unsigned: bit(w, 16),
                wide,
                frac: frac as u8,
            })
        }
        _ => None,
    }
}

/// The 8/16/32-bit core-register transfers (`bit 4 == 1` in the `0xEE`
/// window).
fn decode_transfer(w: u32) -> Option<FpInsn> {
    let rt = field(w, 15, 12) as u8;
    match field(w, 23, 21) {
        // `VMOV Sn, Rt` / `VMOV Rt, Sn`.
        0b000 if field(w, 6, 5) == 0 && field(w, 3, 0) == 0 => Some(FpInsn::MovCore {
            to_fp: !bit(w, 20),
            rt,
            n: rn(w),
        }),
        // `VMSR` / `VMRS`.
        0b111 => Some(FpInsn::Sys {
            to_core: bit(w, 20),
            rt,
            reg: field(w, 19, 16) as u8,
        }),
        // `VMOV` between a core register and a scalar is Advanced SIMD, which
        // no M-profile part has.
        _ => None,
    }
}

/// Extension-register load/store, and the 64-bit transfers that share the
/// `0b110` window (DDI 0403E A6.4 and A7.6).
fn decode_load_store(w: u32) -> Option<FpInsn> {
    let p = bit(w, 24);
    let u = bit(w, 23);
    let wb = bit(w, 21);
    let load = bit(w, 20);
    let base = field(w, 19, 16) as u8;
    let d = rd(w);
    let imm8 = field(w, 7, 0);

    // `VMOV` two core registers and two singles: `P:U:D:W == 0b0010`, which is
    // the one row of this window that is not a load or a store.
    if !p && !u && bit(w, 22) && !wb {
        if field(w, 7, 6) != 0 || !bit(w, 4) {
            return None;
        }
        return Some(FpInsn::MovCore2 {
            to_fp: !load,
            rt: field(w, 15, 12) as u8,
            rt2: field(w, 19, 16) as u8,
            m: rm(w),
        });
    }

    if p && !wb {
        // `VLDR`/`VSTR`, with the immediate already scaled.
        return Some(FpInsn::Mem {
            load,
            d,
            rn: base,
            imm: imm8 * 4,
            add: u,
        });
    }
    // Everything else is a multiple transfer; `imm8` is the register count.
    // Zero registers, or a list that runs past `S31`, is UNPREDICTABLE.
    let count = imm8;
    if count == 0 || u32::from(d) + count > 32 {
        return None;
    }
    match (p, u, wb) {
        // Increment after, with or without writeback.
        (false, true, _) => Some(FpInsn::Multi {
            load,
            d,
            rn: base,
            count: count as u8,
            add: true,
            writeback: wb,
        }),
        // Decrement before, always with writeback. `VPUSH`/`VPOP` is this
        // with `Rn == SP`.
        (true, false, true) => Some(FpInsn::Multi {
            load,
            d,
            rn: base,
            count: count as u8,
            add: false,
            writeback: true,
        }),
        _ => None,
    }
}

/// The FPv5 group in the unconditional `0b1111 1110` window.
fn decode_v5(w: u32) -> Option<FpInsn> {
    let (d, n, m) = (rd(w), rn(w), rm(w));
    if !bit(w, 23) {
        // `VSEL<cc>`: `cond = cc<1> : cc<0> : (cc<1> EOR cc<0>) : 0`, which
        // spells the only four conditions the instruction has — EQ, VS, GE
        // and GT.
        if bit(w, 6) || bit(w, 4) {
            return None;
        }
        let cc = field(w, 21, 20);
        let (hi, lo) = (cc >> 1, cc & 1);
        let cond = (hi << 3) | (lo << 2) | ((hi ^ lo) << 1);
        return Some(FpInsn::Sel {
            cond: Cond(cond as u8),
            d,
            n,
            m,
        });
    }
    match field(w, 21, 20) {
        // `VMAXNM` / `VMINNM`, selected by bit 6.
        0b00 if !bit(w, 4) => Some(FpInsn::Data {
            op: if bit(w, 6) {
                DataOp::Minnm
            } else {
                DataOp::Maxnm
            },
            d,
            n,
            m,
        }),
        0b11 => match field(w, 19, 18) {
            // `VRINTA/N/P/M`, with `opc3 == 0b01`.
            0b10 if field(w, 7, 6) == 0b01 => Some(FpInsn::RintMode {
                mode: RoundMode::from_rm(field(w, 17, 16)),
                d,
                m,
            }),
            // `VCVTA/N/P/M`, with bit 7 choosing signed or unsigned.
            0b11 if bit(w, 6) => Some(FpInsn::CvtMode {
                mode: RoundMode::from_rm(field(w, 17, 16)),
                d,
                m,
                signed: bit(w, 7),
            }),
            _ => None,
        },
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Disassembly — the same description, printed
// ---------------------------------------------------------------------------

/// An `S` register, by number.
struct S(u8);

impl fmt::Display for S {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(S_NAMES[(self.0 & 31) as usize])
    }
}

/// The name `VMRS`/`VMSR` prints for its `reg` field.
fn sysreg(reg: u8) -> &'static str {
    match reg {
        0 => "FPSID",
        1 => "FPSCR",
        5 => "MVFR2",
        6 => "MVFR1",
        7 => "MVFR0",
        8 => "FPEXC",
        _ => "<reserved>",
    }
}

impl fmt::Display for FpInsn {
    #[allow(clippy::too_many_lines)] // One arm per form; splitting hides the map.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            FpInsn::Data { op, d, n, m } => {
                write!(f, "{}.F32 {}, {}, {}", op.mnemonic(), S(d), S(n), S(m))
            }
            FpInsn::Unary { op, d, m } => {
                write!(f, "{}.F32 {}, {}", op.mnemonic(), S(d), S(m))
            }
            FpInsn::MovImm { d, imm } => write!(f, "VMOV.F32 {}, #0x{imm:08x}", S(d)),
            FpInsn::Cmp {
                d,
                m,
                with_zero,
                signal_all,
            } => {
                let e = if signal_all { "E" } else { "" };
                if with_zero {
                    write!(f, "VCMP{e}.F32 {}, #0.0", S(d))
                } else {
                    write!(f, "VCMP{e}.F32 {}, {}", S(d), S(m))
                }
            }
            FpInsn::CvtInt {
                d,
                m,
                to_int,
                signed,
                round_zero,
            } => {
                let int = if signed { "S32" } else { "U32" };
                if to_int {
                    let r = if round_zero { "" } else { "R" };
                    write!(f, "VCVT{r}.{int}.F32 {}, {}", S(d), S(m))
                } else {
                    write!(f, "VCVT.F32.{int} {}, {}", S(d), S(m))
                }
            }
            FpInsn::CvtMode { mode, d, m, signed } => write!(
                f,
                "VCVT{}.{}.F32 {}, {}",
                mode.suffix(),
                if signed { "S32" } else { "U32" },
                S(d),
                S(m)
            ),
            FpInsn::RintMode { mode, d, m } => {
                write!(f, "VRINT{}.F32 {}, {}", mode.suffix(), S(d), S(m))
            }
            FpInsn::CvtFixed {
                d,
                to_fixed,
                unsigned,
                wide,
                frac,
            } => {
                let fx = match (unsigned, wide) {
                    (false, false) => "S16",
                    (false, true) => "S32",
                    (true, false) => "U16",
                    (true, true) => "U32",
                };
                if to_fixed {
                    write!(f, "VCVT.{fx}.F32 {}, {}, #{frac}", S(d), S(d))
                } else {
                    write!(f, "VCVT.F32.{fx} {}, {}, #{frac}", S(d), S(d))
                }
            }
            FpInsn::CvtHalf { d, m, to_half, top } => {
                let t = if top { "T" } else { "B" };
                if to_half {
                    write!(f, "VCVT{t}.F16.F32 {}, {}", S(d), S(m))
                } else {
                    write!(f, "VCVT{t}.F32.F16 {}, {}", S(d), S(m))
                }
            }
            FpInsn::Sel { cond, d, n, m } => {
                write!(f, "VSEL{cond}.F32 {}, {}, {}", S(d), S(n), S(m))
            }
            FpInsn::MovCore { to_fp, rt, n } => {
                if to_fp {
                    write!(f, "VMOV {}, {}", S(n), RegName(rt))
                } else {
                    write!(f, "VMOV {}, {}", RegName(rt), S(n))
                }
            }
            FpInsn::MovCore2 { to_fp, rt, rt2, m } => {
                if to_fp {
                    write!(
                        f,
                        "VMOV {}, {}, {}, {}",
                        S(m),
                        S(m + 1),
                        RegName(rt),
                        RegName(rt2)
                    )
                } else {
                    write!(
                        f,
                        "VMOV {}, {}, {}, {}",
                        RegName(rt),
                        RegName(rt2),
                        S(m),
                        S(m + 1)
                    )
                }
            }
            FpInsn::Sys { to_core, rt, reg } => {
                if to_core {
                    let dest = if rt == 15 && reg == 1 {
                        "APSR_nzcv"
                    } else {
                        return write!(f, "VMRS {}, {}", RegName(rt), sysreg(reg));
                    };
                    write!(f, "VMRS {dest}, {}", sysreg(reg))
                } else {
                    write!(f, "VMSR {}, {}", sysreg(reg), RegName(rt))
                }
            }
            FpInsn::Mem {
                load,
                d,
                rn,
                imm,
                add,
            } => {
                let op = if load { "VLDR" } else { "VSTR" };
                let sign = if add { "" } else { "-" };
                if imm == 0 && add {
                    write!(f, "{op} {}, [{}]", S(d), RegName(rn))
                } else {
                    write!(f, "{op} {}, [{}, #{sign}{imm}]", S(d), RegName(rn))
                }
            }
            FpInsn::Multi {
                load,
                d,
                rn,
                count,
                add,
                writeback,
            } => {
                // `VPUSH`/`VPOP` is the `SP` spelling of the same encoding,
                // and is what a listing should say when it sees one.
                if rn == 13 && writeback {
                    if load && add {
                        return write!(f, "VPOP {{{}-{}}}", S(d), S(d + count - 1));
                    }
                    if !load && !add {
                        return write!(f, "VPUSH {{{}-{}}}", S(d), S(d + count - 1));
                    }
                }
                let op = if load { "VLDM" } else { "VSTM" };
                let mode = if add { "IA" } else { "DB" };
                let bang = if writeback { "!" } else { "" };
                write!(
                    f,
                    "{op}{mode} {}{bang}, {{{}-{}}}",
                    RegName(rn),
                    S(d),
                    S(d + count - 1)
                )
            }
        }
    }
}
