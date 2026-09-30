//! The VFP register file, its system registers, and the ARMv7 rules that wrap
//! [`crate::float`].
//!
//! The arithmetic is not here. Every value this module computes comes out of
//! [`crate::float`], the crate's one software IEEE-754 implementation, because
//! `ROADMAP.md` §9.1 makes guest floating point bit-reproducible across hosts.
//! What *is* here is the paperwork that turns IEEE-754 into a VFP:
//!
//! 1. **The register file.** Thirty-two 64-bit registers `D0`–`D31`, of which
//!    the first sixteen are also the thirty-two singles `S0`–`S31`: `S(2n)` is
//!    the low half of `Dn` and `S(2n+1)` the high half (DDI 0406C,
//!    "Advanced SIMD and Floating-point Extension registers"). Held as
//!    doubles, so the aliasing is a shift and a mask rather than a second copy
//!    that has to be kept in step.
//! 2. **`FPSCR` as a [`float::Env`]** — which NaN rule `DN` selects, which
//!    direction `RMode` names, what `FZ` flushes.
//! 3. **The rules that are ARMv7's pseudocode rather than IEEE's**: the
//!    four-way `FPCompare`, the chained (twice-rounded) multiply-accumulates,
//!    `FPRound`'s flush-to-zero raising only `UFC`, and the alternative
//!    half-precision format.
//!
//! # The system registers, as a Cortex-A9 reports them
//!
//! * `FPSID` is `0x41033094`: implementer `0x41` (ARM), hardware
//!   implementation, subarchitecture `0x03` (the VFPv3 common architecture,
//!   version 2), part `0x30`, variant `0x9` (Cortex-A9), revision `0x4`
//!   (*Cortex-A9 Floating-Point Unit TRM*, ARM DDI 0408, "Floating-point
//!   System ID Register"). The revision nibble follows the silicon; `4` is
//!   the r2/r3 value.
//! * `MVFR0` is `0x10110222` (DDI 0408, "Media and VFP Feature Register 0"):
//!   all rounding modes, no short vectors, hardware square root and divide,
//!   no exception trapping, double and single precision at VFPv3 level, and
//!   thirty-two 64-bit registers.
//! * `MVFR1` is `0x01000011`, **not** the A9's `0x01111111`. The part has
//!   NEON; this core does not implement it. DDI 0406C's `MVFR1` description gives the
//!   fields: `[3:0]` FtZ = 1 (denormals are handled, not only flushed),
//!   `[7:4]` D_NaN = 1 (NaN payloads propagate), `[27:24]` VFP HPFP = 1
//!   (`VCVTB`/`VCVTT`), and the four Advanced SIMD fields — load/store,
//!   integer, single-precision, half-precision — zero. A preset that told
//!   the guest it had NEON would invite instructions that then trap.
//! * `FPEXC`: only `EN` (bit 30) is writable. `EX` (bit 31) and the rest are
//!   RAZ/WI: the Cortex-A9 FPU handles every case in hardware and never
//!   enters the exceptional state that needs support code (DDI 0408,
//!   "Floating-Point Exception Register").
//!
//! # FPSCR bits, and what is not implemented
//!
//! `N Z C V`, `AHP`, `DN`, `FZ`, `RMode`, `Stride`, `Len` and the six
//! cumulative flags are writable. `QC` is RAZ/WI because it belongs to
//! Advanced SIMD. The six trap-enable bits (`IDE`, `IXE`, `UFE`, `OFE`, `DZE`,
//! `IOE`) are RAZ/WI, as `MVFR0.FP_Exception_Trapping == 0` says.
//!
//! `Len` and `Stride` are writable because the Cortex-A9 FPU says what happens
//! when they are non-zero: it does not implement short vectors, and a
//! data-processing instruction executed with either field non-zero is
//! UNDEFINED (DDI 0408, "VFP short vectors"). The executor enforces that.
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition* (ARM DDI
//! 0406C): A2 (the extension register file; A2.7 the floating-point data types,
//! `FPSCR`, flush-to-zero, default NaN, the half-precision formats and the
//! shared pseudocode `FPUnpack`, `FPRound`, `FPAdd`, `FPMul`, `FPCompare`,
//! `FPToFixed`, `FixedToFP`, `FPHalfToSingle`, `FPSingleToHalf`,
//! `VFPExpandImm`), the `FPEXC`, `FPSID`, `MVFR0` and `MVFR1` descriptions
//! among the system control registers in B4.1; A7 and A8 for the encodings
//! and per-instruction pseudocode; *Cortex-A9
//! Floating-Point Unit Technical Reference Manual* (ARM DDI 0408) for the
//! identification values. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use crate::core::error::Result;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::float::{self, B16, B32, B64, Category, Env, Flags, Round};

use super::arch::Vfp;

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

/// How many 64-bit registers a D32 part has.
pub const D_COUNT: usize = 32;

/// The floating-point register file and the writable system registers.
///
/// `Copy` because it lives in the core's execution state, which is copied out
/// whole for a snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VfpRegs {
    /// `D0`–`D31`. Architecturally UNKNOWN at reset, zero here.
    pub d: [u64; D_COUNT],
    /// `FPSCR`.
    pub fpscr: u32,
    /// `FPEXC`. Only `EN` is ever set.
    pub fpexc: u32,
}

impl VfpRegs {
    /// The power-on state: every register zero, the unit disabled.
    #[must_use]
    pub const fn new() -> VfpRegs {
        VfpRegs {
            d: [0; D_COUNT],
            fpscr: 0,
            fpexc: 0,
        }
    }

    /// Read `Dn`.
    #[inline]
    #[must_use]
    pub const fn d(&self, n: u8) -> u64 {
        self.d[(n & 31) as usize]
    }

    /// Write `Dn`.
    #[inline]
    pub const fn set_d(&mut self, n: u8, value: u64) {
        self.d[(n & 31) as usize] = value;
    }

    /// Read `Sn`: the low half of `D(n/2)` for an even `n`, the high half for
    /// an odd one.
    #[inline]
    #[must_use]
    pub const fn s(&self, n: u8) -> u32 {
        let n = n & 31;
        (self.d[(n >> 1) as usize] >> ((n & 1) * 32)) as u32
    }

    /// Write `Sn`, leaving the other half of the double it lives in alone.
    #[inline]
    pub const fn set_s(&mut self, n: u8, value: u32) {
        let n = n & 31;
        let shift = (n & 1) * 32;
        let slot = &mut self.d[(n >> 1) as usize];
        *slot = (*slot & !(0xffff_ffffu64 << shift)) | ((value as u64) << shift);
    }

    /// Read a register at a precision: `Dn` when `dp`, `Sn` otherwise, as a
    /// bit pattern in a `u64` the way [`crate::float`] takes one.
    #[inline]
    #[must_use]
    pub const fn get(&self, dp: bool, n: u8) -> u64 {
        if dp { self.d(n) } else { self.s(n) as u64 }
    }

    /// Write a register at a precision.
    #[inline]
    pub const fn set(&mut self, dp: bool, n: u8, value: u64) {
        if dp {
            self.set_d(n, value);
        } else {
            self.set_s(n, value as u32);
        }
    }

    /// Whether the unit is enabled (`FPEXC.EN`).
    #[inline]
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.fpexc & fpexc::EN != 0
    }

    /// The environment the current `FPSCR` describes.
    #[inline]
    #[must_use]
    pub fn env(&self) -> Env {
        env(self.fpscr)
    }

    /// Fold a set of exceptions into `FPSCR`'s cumulative bits.
    #[inline]
    pub fn accumulate(&mut self, flags: Flags) {
        self.fpscr |= flags.to_fpsr() & fpscr::CUMULATIVE;
    }

    /// What a reset does to the control registers: the unit comes up
    /// disabled, and `FPSCR` — UNKNOWN in the architecture — reads zero. The
    /// data registers are left alone; a warm reset does not clear them on
    /// hardware either.
    pub const fn reset_control(&mut self) {
        self.fpscr = 0;
        self.fpexc = 0;
    }

    /// Append the register file to a snapshot chunk.
    ///
    /// # Errors
    ///
    /// If the writer refuses.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        for value in self.d {
            w.write_u64(value)?;
        }
        w.write_u32(self.fpscr)?;
        w.write_u32(self.fpexc)?;
        Ok(())
    }

    /// Read what [`save`](VfpRegs::save) wrote.
    ///
    /// # Errors
    ///
    /// If the chunk is short.
    pub fn load(r: &mut ChunkReader<'_>) -> Result<VfpRegs> {
        let mut regs = VfpRegs::new();
        for value in &mut regs.d {
            *value = r.read_u64()?;
        }
        // Masked on the way in, so a hand-edited snapshot cannot set a bit a
        // `VMSR` could not.
        regs.fpscr = r.read_u32()? & fpscr::WRITABLE;
        regs.fpexc = r.read_u32()? & fpexc::WRITABLE;
        Ok(regs)
    }
}

impl Default for VfpRegs {
    fn default() -> VfpRegs {
        VfpRegs::new()
    }
}

// ---------------------------------------------------------------------------
// The system registers
// ---------------------------------------------------------------------------

/// `FPSID` for a Cortex-A9 FPU (DDI 0408). See the module docs for the fields.
pub const FPSID_CORTEX_A9: u32 = 0x4103_3094;

/// `MVFR0` for VFPv3-D32 as a Cortex-A9 has it (DDI 0408).
pub const MVFR0_VFPV3_D32: u32 = 0x1011_0222;

/// `MVFR1` for a VFPv3 part with the half-precision extension and **no**
/// Advanced SIMD — see the module docs for why this is not the A9's value.
pub const MVFR1_VFP_ONLY: u32 = 0x0100_0011;

/// `FPSID` for a VFP unit.
///
/// VFPv3 reports the Cortex-A9's value, the only VFPv3 part this crate has a
/// preset for. VFPv2 reports the VFP11 layout (ARM1136JF-S/ARM1176JZF-S TRMs:
/// implementer `0x41`, subarchitecture `0x01`, part `0x20`, variant `0xB`);
/// its revision nibble, like the A9's, follows the silicon.
#[must_use]
pub const fn fpsid(unit: Vfp) -> u32 {
    if unit.version >= 3 {
        FPSID_CORTEX_A9
    } else {
        0x4101_20b4
    }
}

/// `MVFR0` for a VFP unit, from its fields (DDI 0406C, the `MVFR0` description): the register
/// count, single and double precision at the unit's level, hardware divide
/// and square root, all four rounding modes. Short vectors and exception
/// trapping read zero, because this core implements neither — a VFPv2 part
/// that has them in silicon still has them reported absent here, which is
/// the honest answer for what the guest will find.
#[must_use]
pub const fn mvfr0(unit: Vfp) -> u32 {
    if unit.version >= 3 && unit.d32 {
        return MVFR0_VFPV3_D32;
    }
    let regs = if unit.d32 { 2 } else { 1 };
    let level = if unit.version >= 3 { 2 } else { 1 };
    (1 << 28) | (1 << 20) | (1 << 16) | (level << 8) | (level << 4) | regs
}

/// `MVFR1` for a VFP unit: denormals and NaN propagation handled, the
/// half-precision conversions on VFPv3, and no Advanced SIMD field set.
#[must_use]
pub const fn mvfr1(unit: Vfp) -> u32 {
    if unit.version >= 3 {
        MVFR1_VFP_ONLY
    } else {
        0x0000_0011
    }
}

/// The `reg` field `VMRS`/`VMSR` name the system registers by (DDI 0406C
/// A8.8, `VMRS` and `VMSR`).
pub mod sysreg {
    /// `FPSID`.
    pub const FPSID: u8 = 0b0000;
    /// `FPSCR`.
    pub const FPSCR: u8 = 0b0001;
    /// `MVFR1`.
    pub const MVFR1: u8 = 0b0110;
    /// `MVFR0`.
    pub const MVFR0: u8 = 0b0111;
    /// `FPEXC`.
    pub const FPEXC: u8 = 0b1000;
}

/// The `FPEXC` bits (DDI 0406C, the `FPEXC` description).
pub mod fpexc {
    /// The unit is in the exceptional state. RAZ on a Cortex-A9.
    pub const EX: u32 = 1 << 31;
    /// The unit is enabled.
    pub const EN: u32 = 1 << 30;
    /// Every bit a `VMSR FPEXC` may set.
    pub const WRITABLE: u32 = EN;
}

/// The `FPSCR` bits (DDI 0406C, "Floating-point Status and Control
/// Register, FPSCR").
pub mod fpscr {
    /// Negative condition flag, written by `VCMP`.
    pub const N: u32 = 1 << 31;
    /// Zero condition flag.
    pub const Z: u32 = 1 << 30;
    /// Carry condition flag.
    pub const C: u32 = 1 << 29;
    /// Overflow condition flag.
    pub const V: u32 = 1 << 28;
    /// Alternative half-precision format for `VCVTB`/`VCVTT`.
    pub const AHP: u32 = 1 << 26;
    /// Default NaN mode.
    pub const DN: u32 = 1 << 25;
    /// Flush-to-zero mode.
    pub const FZ: u32 = 1 << 24;
    /// The low bit of the rounding-mode field.
    pub const RMODE_SHIFT: u32 = 22;
    /// The rounding-mode field, in place.
    pub const RMODE: u32 = 3 << RMODE_SHIFT;
    /// The short-vector stride field, in place.
    pub const STRIDE: u32 = 3 << 20;
    /// The short-vector length field, in place.
    pub const LEN: u32 = 7 << 16;
    /// Input denormal, cumulative.
    pub const IDC: u32 = 1 << 7;
    /// Inexact, cumulative.
    pub const IXC: u32 = 1 << 4;
    /// Underflow, cumulative.
    pub const UFC: u32 = 1 << 3;
    /// Overflow, cumulative.
    pub const OFC: u32 = 1 << 2;
    /// Divide by zero, cumulative.
    pub const DZC: u32 = 1 << 1;
    /// Invalid operation, cumulative.
    pub const IOC: u32 = 1 << 0;

    /// The four condition flags together.
    pub const FLAGS: u32 = N | Z | C | V;
    /// The six cumulative exception bits together.
    pub const CUMULATIVE: u32 = IDC | IXC | UFC | OFC | DZC | IOC;
    /// Every bit a `VMSR` may set. `QC` and the trap enables are absent; the
    /// module docs say why.
    pub const WRITABLE: u32 = FLAGS | AHP | DN | FZ | RMODE | STRIDE | LEN | CUMULATIVE;
}

/// The rounding direction `FPSCR.RMode` selects: `00` nearest, `01` toward
/// `+∞`, `10` toward `−∞`, `11` toward zero (DDI 0406C, the `FPSCR`
/// description).
#[must_use]
pub const fn rounding(fpscr: u32) -> Round {
    match (fpscr >> fpscr::RMODE_SHIFT) & 3 {
        0b00 => Round::TiesEven,
        0b01 => Round::TowardPositive,
        0b10 => Round::TowardNegative,
        _ => Round::TowardZero,
    }
}

/// The floating-point environment `FPSCR` describes.
#[must_use]
pub fn env(fpscr: u32) -> Env {
    let base = if fpscr & fpscr::DN != 0 {
        Env::ARM_DEFAULT_NAN
    } else {
        Env::ARM
    };
    base.round(rounding(fpscr)).flush(fpscr & fpscr::FZ != 0)
}

// ---------------------------------------------------------------------------
// The arithmetic, all of it through `crate::float`
// ---------------------------------------------------------------------------

/// Dispatch one `float` operation on the precision.
macro_rules! by_prec {
    ($dp:expr, $f:ident, $($arg:expr),*) => {
        if $dp {
            float::$f::<B64>($($arg),*)
        } else {
            float::$f::<B32>($($arg),*)
        }
    };
}

/// ARMv7's `FPRound` under flush-to-zero sets **only** `UFC` for a result it
/// flushed: "if FPSCR.FZ == '1' && exponent < minimum_exp then result =
/// FPZero(sign); FPSCR.UFC = '1'" (DDI 0406C, the shared floating-point pseudocode).
/// [`crate::float`]'s flush reports underflow *and* inexact, which is x86's
/// `MXCSR.FTZ` rule, so the inexact is taken back here. Under `FZ` every tiny
/// result is flushed, so an underflow flag always means a flush and the
/// inexact beside it is always the flush's.
#[inline]
fn fz_fixup(flags: Flags, env: Env) -> Flags {
    if env.flush_outputs && flags.contains(Flags::UNDERFLOW) {
        Flags(flags.0 & !Flags::INEXACT.0)
    } else {
        flags
    }
}

/// Apply [`fz_fixup`] to an operation's result.
#[inline]
fn fixed((v, f): (u64, Flags), env: Env) -> (u64, Flags) {
    (v, fz_fixup(f, env))
}

/// The sign bit at a precision.
#[inline]
const fn sign_bit(dp: bool) -> u64 {
    if dp { 1 << 63 } else { 1 << 31 }
}

/// `FPNeg`: flip the sign bit and nothing else — no NaN is quietened and no
/// exception is raised (DDI 0406C's `FPNeg`).
#[inline]
#[must_use]
pub const fn neg(dp: bool, a: u64) -> u64 {
    a ^ sign_bit(dp)
}

/// `FPAbs`: clear the sign bit, with the same caveat as [`neg`].
#[inline]
#[must_use]
pub const fn abs(dp: bool, a: u64) -> u64 {
    a & !sign_bit(dp)
}

/// Classify a value at a precision.
#[inline]
#[must_use]
pub fn classify(dp: bool, a: u64) -> Category {
    if dp {
        float::classify::<B64>(a)
    } else {
        float::classify::<B32>(a)
    }
}

/// `FPAdd`.
#[must_use]
pub fn add(dp: bool, a: u64, b: u64, env: Env) -> (u64, Flags) {
    fixed(by_prec!(dp, add, a, b, env), env)
}

/// `FPSub`.
#[must_use]
pub fn sub(dp: bool, a: u64, b: u64, env: Env) -> (u64, Flags) {
    fixed(by_prec!(dp, sub, a, b, env), env)
}

/// `FPMul`.
#[must_use]
pub fn mul(dp: bool, a: u64, b: u64, env: Env) -> (u64, Flags) {
    fixed(by_prec!(dp, mul, a, b, env), env)
}

/// `FPDiv`.
#[must_use]
pub fn div(dp: bool, a: u64, b: u64, env: Env) -> (u64, Flags) {
    fixed(by_prec!(dp, div, a, b, env), env)
}

/// `FPSqrt`.
#[must_use]
pub fn sqrt(dp: bool, a: u64, env: Env) -> (u64, Flags) {
    fixed(by_prec!(dp, sqrt, a, env), env)
}

/// The four chained multiply-accumulates, as the VFPv3 pseudocode writes them
/// (DDI 0406C A8.8, `VMLA`/`VMLS (floating-point)` and
/// `VNMLA`/`VNMLS`/`VNMUL`):
///
/// ```text
/// product = FPMul(n, m)
/// VMLA:  d = FPAdd(d, product)
/// VMLS:  d = FPAdd(d, FPNeg(product))
/// VNMLA: d = FPAdd(FPNeg(d), FPNeg(product))
/// VNMLS: d = FPAdd(FPNeg(d), product)
/// ```
///
/// **Two roundings**, not one: these are not the VFPv4 fused forms, and
/// routing them through `fma` would change the last bit of a great many
/// results. The negations are `FPNeg`, so they flip a NaN's sign too — which
/// is visible in the payload a NaN operand propagates as.
#[must_use]
pub fn mul_acc(
    dp: bool,
    acc: u64,
    n: u64,
    m: u64,
    negate_product: bool,
    negate_acc: bool,
    env: Env,
) -> (u64, Flags) {
    let (product, f1) = mul(dp, n, m, env);
    let product = if negate_product {
        neg(dp, product)
    } else {
        product
    };
    let acc = if negate_acc { neg(dp, acc) } else { acc };
    let (out, f2) = add(dp, acc, product, env);
    (out, f1 | f2)
}

/// Flush a subnormal operand the way `FPUnpack` does under `FZ`, reporting
/// the input-denormal exception, for the operations that read an operand
/// without going through an arithmetic routine that would do it.
fn flush_input(dp: bool, v: u64, env: Env, flags: &mut Flags) -> u64 {
    let subnormal = matches!(
        classify(dp, v),
        Category::NegativeSubnormal | Category::PositiveSubnormal
    );
    if subnormal && env.subnormal_inputs.flushes() {
        if env.subnormal_inputs.reports() {
            *flags |= Flags::DENORMAL;
        }
        v & sign_bit(dp)
    } else {
        v
    }
}

/// `FPCompare`: the four `FPSCR` condition bits a comparison writes.
///
/// Unordered is `0b0011` — `C` and `V` both — and equal is `0b0110`, which is
/// why `VMRS APSR_nzcv` followed by an ordinary condition works at all
/// (DDI 0406C A8.8, the table in `VCMP`/`VCMPE`). `signal_all` is `VCMPE`, which raises
/// invalid for a quiet NaN too. The operands are unpacked through `FPSCR`, so
/// under `FZ` a subnormal compares equal to zero and sets `IDC`.
#[must_use]
pub fn compare(dp: bool, a: u64, b: u64, signal_all: bool, env: Env) -> (u32, Flags) {
    use core::cmp::Ordering;
    let mut flags = Flags::NONE;
    let a = flush_input(dp, a, env, &mut flags);
    let b = flush_input(dp, b, env, &mut flags);
    let ord = if dp {
        float::compare::<B64>(a, b)
    } else {
        float::compare::<B32>(a, b)
    };
    let nzcv = match ord {
        Some(Ordering::Equal) => fpscr::Z | fpscr::C,
        Some(Ordering::Less) => fpscr::N,
        Some(Ordering::Greater) => fpscr::C,
        None => {
            let snan = classify(dp, a) == Category::SignalingNan
                || classify(dp, b) == Category::SignalingNan;
            if snan || signal_all {
                flags |= Flags::INVALID;
            }
            fpscr::C | fpscr::V
        }
    };
    (nzcv, flags)
}

/// `VCVT` between single and double precision (`FPSingleToDouble`,
/// `FPDoubleToSingle`). Both ends honour `FZ`, since both are 32- or 64-bit.
#[must_use]
pub fn convert_precision(to_double: bool, value: u64, env: Env) -> (u64, Flags) {
    if to_double {
        fixed(float::convert::<B32, B64>(value, env), env)
    } else {
        fixed(float::convert::<B64, B32>(value, env), env)
    }
}

/// Float to a 32-bit integer (`VCVT`/`VCVTR`, `FPToFixed` with no fraction
/// bits). Out of range saturates and a NaN gives zero, both raising only
/// invalid; `Env::ARM` carries that rule.
#[must_use]
pub fn to_int(dp: bool, value: u64, signed: bool, env: Env) -> (u32, Flags) {
    to_fixed(dp, value, 32, 0, !signed, env)
}

/// Float to fixed point in a `size`-bit container, extended to the register
/// width by the caller (`FPToFixed`).
#[must_use]
pub fn to_fixed(
    dp: bool,
    value: u64,
    size: u32,
    fbits: u32,
    unsigned: bool,
    env: Env,
) -> (u32, Flags) {
    if unsigned {
        let (v, f) = by_prec!(dp, to_unsigned_fixed, value, size, fbits, env);
        (v as u32, f)
    } else {
        let (v, f) = by_prec!(dp, to_signed_fixed, value, size, fbits, env);
        (v as u32, f)
    }
}

/// A 32-bit integer to float (`VCVT.F32.S32` and friends, `FixedToFP` with no
/// fraction bits), in the `FPSCR` rounding mode.
#[must_use]
pub fn from_int(dp: bool, value: u32, signed: bool, env: Env) -> (u64, Flags) {
    from_fixed(dp, value, 32, 0, !signed, env)
}

/// Fixed point in the low `size` bits of `value` to float (`FixedToFP`),
/// rounded once.
#[must_use]
pub fn from_fixed(
    dp: bool,
    value: u32,
    size: u32,
    fbits: u32,
    unsigned: bool,
    env: Env,
) -> (u64, Flags) {
    if unsigned {
        fixed(
            by_prec!(dp, from_unsigned_fixed, u64::from(value), size, fbits, env),
            env,
        )
    } else {
        fixed(
            by_prec!(
                dp,
                from_signed_fixed,
                i64::from(value as i32),
                size,
                fbits,
                env
            ),
            env,
        )
    }
}

/// `FPHalfToSingle`: a half in either format to a single.
///
/// Exact — every half, in either format, is a normal or zero single — so no
/// rounding and no flush: `FZ` is defined on the 32- and 64-bit formats and a
/// half operand is neither. Under `AHP` there are no infinities and no NaNs:
/// the all-ones exponent is one more binade of ordinary numbers
/// (DDI 0406C A2.7, the half-precision floating-point formats).
#[must_use]
pub fn half_to_single(half: u16, ahp: bool, env: Env) -> (u32, Flags) {
    let exp = (half >> 10) & 0x1f;
    if ahp && exp == 0x1f {
        // 1.frac × 2^(31 - 15): rebias the exponent into the single format.
        let sign = u32::from(half >> 15) << 31;
        let frac = u32::from(half & 0x3ff) << 13;
        return (sign | ((31 - 15 + 127) << 23) | frac, Flags::NONE);
    }
    let (v, f) = float::convert::<B16, B32>(u64::from(half), env.flush(false));
    (v as u32, f)
}

/// `FPSingleToHalf`: a single to a half in either format.
///
/// The operand is a single and is unpacked through `FPSCR`, so `FZ` flushes a
/// subnormal one (with `IDC`); the half result is never flushed. Under `AHP`
/// a NaN becomes `+0` and an infinity the largest magnitude, both raising
/// invalid, and a finite value too large for the format saturates with
/// invalid rather than overflowing (DDI 0406C A2.7 and `FPSingleToHalf`).
#[must_use]
pub fn single_to_half(value: u32, ahp: bool, env: Env) -> (u16, Flags) {
    let mut flags = Flags::NONE;
    let value = flush_input(false, u64::from(value), env, &mut flags) as u32;
    let env = env.flush(false);
    let sign = (value >> 16) as u16 & 0x8000;
    if !ahp {
        let (v, f) = float::convert::<B32, B16>(u64::from(value), env);
        return (v as u16, f | flags);
    }
    match float::classify::<B32>(u64::from(value)) {
        Category::QuietNan | Category::SignalingNan => (0, flags | Flags::INVALID),
        Category::NegativeInfinity | Category::PositiveInfinity => {
            (sign | 0x7fff, flags | Flags::INVALID)
        }
        _ => {
            let magnitude = value & 0x7fff_ffff;
            // Below 2^15 the two formats are the same encoding: the extra
            // binade AHP has is exponent field 31, which holds [2^16, 2^17).
            if magnitude < 0x4700_0000 {
                let (v, f) = float::convert::<B32, B16>(u64::from(value), env);
                return (v as u16, f | flags);
            }
            // At or above 2^15, halving is exact and moves the value onto the
            // IEEE format's grid one binade down; rounding there and adding
            // one to the exponent field is rounding on AHP's grid.
            let halved = value - (1 << 23);
            let (v, f) = float::convert::<B32, B16>(u64::from(halved), env);
            // Overflow in the IEEE format one binade down is overflow in AHP:
            // `FPRound` then delivers the largest magnitude and raises invalid
            // — not overflow, not inexact — in every rounding mode, including
            // toward zero, where IEEE would have stopped at its own maximum.
            if f.contains(Flags::OVERFLOW) {
                return (sign | 0x7fff, flags | Flags::INVALID);
            }
            let v = v as u16;
            (v + (1 << 10), f | flags)
        }
    }
}

/// `VFPExpandImm`: the eight-bit immediate `VMOV` carries, expanded to a
/// single or a double (DDI 0406C A7, "Operation of modified immediate
/// constants, Floating-point").
///
/// The byte is a sign, a three-bit exponent offset and a four-bit fraction:
/// `imm8<7> : NOT(imm8<6>) : Replicate(imm8<6>, E-3) : imm8<5:0> : Zeros`.
#[must_use]
pub const fn expand_imm(imm8: u8, dp: bool) -> u64 {
    let imm8 = imm8 as u64;
    let (exp_bits, frac_bits) = if dp { (11u32, 52u32) } else { (8, 23) };
    let sign = (imm8 >> 7) & 1;
    let b = (imm8 >> 6) & 1;
    let mut exp = b ^ 1;
    let mut i = 0;
    while i < exp_bits - 3 {
        exp = (exp << 1) | b;
        i += 1;
    }
    exp = (exp << 2) | ((imm8 >> 4) & 3);
    let frac = (imm8 & 0xf) << (frac_bits - 4);
    (sign << (exp_bits + frac_bits)) | (exp << frac_bits) | frac
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Host floats, only to write readable expectations; the implementation
    /// never sees one.
    fn d(v: f64) -> u64 {
        v.to_bits()
    }

    fn s(v: f32) -> u64 {
        u64::from(v.to_bits())
    }

    /// `ROADMAP.md` §9.1, one level up from `src/float`'s own check: the
    /// files that carry this core's guest floating point name no host float
    /// type outside comments and tests. The disassembler is included because
    /// printing an immediate is exactly where a host `f64` would creep in.
    #[test]
    fn no_host_float_on_the_guest_path() {
        let sources = [
            ("vfp.rs", include_str!("vfp.rs")),
            ("vfpisa.rs", include_str!("vfpisa.rs")),
            ("exec/vfp.rs", include_str!("exec/vfp.rs")),
            ("exec.rs", include_str!("exec.rs")),
        ];
        for (name, src) in sources {
            let body = src.split("#[cfg(test)]").next().unwrap_or(src);
            for (n, line) in body.lines().enumerate() {
                let code = line.find("//").map_or(line, |i| &line[..i]);
                for needle in ["f16", "f32", "f64", "f128", "sqrtf", "libm"] {
                    assert!(
                        !code.contains(needle),
                        "{name}:{}: `{needle}` on the guest path — guest \
                         floating point must go through `crate::float`",
                        n + 1
                    );
                }
            }
        }
    }

    #[test]
    fn singles_alias_the_low_sixteen_doubles() {
        let mut r = VfpRegs::new();
        r.set_d(1, 0x1122_3344_5566_7788);
        assert_eq!(r.s(2), 0x5566_7788);
        assert_eq!(r.s(3), 0x1122_3344);
        r.set_s(3, 0xdead_beef);
        assert_eq!(r.d(1), 0xdead_beef_5566_7788);
        r.set_s(31, 1);
        assert_eq!(r.d(15), 1 << 32);
        // D16 and up have no single-precision names.
        r.set_d(16, u64::MAX);
        assert_eq!(r.s(0), 0);
    }

    #[test]
    fn the_rounding_field_is_arms_order() {
        assert_eq!(rounding(0), Round::TiesEven);
        assert_eq!(rounding(1 << 22), Round::TowardPositive);
        assert_eq!(rounding(2 << 22), Round::TowardNegative);
        assert_eq!(rounding(3 << 22), Round::TowardZero);
    }

    #[test]
    fn expand_imm_matches_the_assembler() {
        // vmov.f32 s0, #1.0 is imm8 0x70; vmov.f64 d0, #-2.5 is 0x84;
        // #0.125 is 0x40 (arm-none-eabi-as -mfpu=vfpv3, read back).
        assert_eq!(expand_imm(0x70, false), s(1.0));
        assert_eq!(expand_imm(0x70, true), d(1.0));
        assert_eq!(expand_imm(0x84, true), d(-2.5));
        assert_eq!(expand_imm(0x40, false), s(0.125));
        assert_eq!(expand_imm(0x00, false), s(2.0));
        assert_eq!(expand_imm(0x3f, true), d(31.0));
        assert_eq!(expand_imm(0x7f, true), d(1.9375));
        assert_eq!(expand_imm(0x4e, false), s(0.234_375));
    }

    #[test]
    fn nan_propagation_and_default_nan() {
        let e = env(0);
        let snan = 0x7f80_0001u64;
        let qnan = 0xffc0_0002u64;
        // A signaling NaN wins over a quiet one in either position, quieted.
        assert_eq!(add(false, qnan, snan, e), (0x7fc0_0001, Flags::INVALID));
        assert_eq!(add(false, qnan, s(1.0), e), (qnan, Flags::NONE));
        // DN replaces the payload with the positive default NaN.
        let dn = env(fpscr::DN);
        assert_eq!(add(false, qnan, s(1.0), dn), (0x7fc0_0000, Flags::NONE));
        assert_eq!(
            sub(true, d(f64::INFINITY), d(f64::INFINITY), e),
            (0x7ff8_0000_0000_0000, Flags::INVALID)
        );
    }

    #[test]
    fn rounding_modes_and_signed_zero() {
        let one = s(1.0);
        let tiny = s(f32::MIN_POSITIVE);
        let up = env(1 << 22);
        let down = env(2 << 22);
        assert_eq!(add(false, one, tiny, env(0)).0, one);
        assert_eq!(add(false, one, tiny, up).0, one + 1);
        assert_eq!(sub(false, one, tiny, down).0, s(0.999_999_94));
        // x - x is +0 in every mode but toward −∞, where it is −0.
        assert_eq!(sub(true, d(2.0), d(2.0), env(0)).0, d(0.0));
        assert_eq!(sub(true, d(2.0), d(2.0), down).0, d(-0.0));
    }

    #[test]
    fn flush_to_zero_sets_only_ufc_for_a_flushed_result() {
        let fz = env(fpscr::FZ);
        let (v, f) = mul(false, s(f32::MIN_POSITIVE), s(0.5), fz);
        assert_eq!(v, 0);
        assert_eq!(f, Flags::UNDERFLOW);
        // A subnormal operand is flushed and reported as IDC.
        let (v, f) = add(false, 1, s(1.0), fz);
        assert_eq!((v, f), (s(1.0), Flags::DENORMAL));
        // Without FZ the same product is a subnormal, inexact-free.
        assert_eq!(
            mul(false, s(f32::MIN_POSITIVE), s(0.5), env(0)).1,
            Flags::NONE
        );
    }

    #[test]
    fn chained_multiply_accumulate_rounds_twice() {
        // (1 + 2^-23)^2 = 1 + 2^-22 + 2^-46: the product rounds away the
        // 2^-46 term, then subtracting 1 leaves exactly 2^-22. A fused
        // operation would keep the 2^-46.
        let x = s(1.0) + 1;
        let (v, _) = mul_acc(false, s(-1.0), x, x, false, false, env(0));
        assert_eq!(v, s(2.0f32.powi(-22)));
        // VNMLA: -d - n*m.
        let (v, _) = mul_acc(true, d(1.0), d(2.0), d(3.0), true, true, env(0));
        assert_eq!(v, d(-7.0));
        // VNMLS: -d + n*m.
        let (v, _) = mul_acc(true, d(1.0), d(2.0), d(3.0), false, true, env(0));
        assert_eq!(v, d(5.0));
    }

    #[test]
    fn compare_writes_arms_four_way_flags() {
        let e = env(0);
        assert_eq!(
            compare(true, d(1.0), d(1.0), false, e).0,
            fpscr::Z | fpscr::C
        );
        assert_eq!(compare(true, d(1.0), d(2.0), false, e).0, fpscr::N);
        assert_eq!(compare(true, d(2.0), d(1.0), false, e).0, fpscr::C);
        let q = 0x7ff8_0000_0000_0000;
        assert_eq!(
            compare(true, q, d(1.0), false, e),
            (fpscr::C | fpscr::V, Flags::NONE)
        );
        assert_eq!(
            compare(true, q, d(1.0), true, e),
            (fpscr::C | fpscr::V, Flags::INVALID)
        );
        // -0 == +0.
        assert_eq!(compare(false, s(-0.0), 0, false, e).0, fpscr::Z | fpscr::C);
    }

    #[test]
    fn integer_conversion_saturates_and_nan_is_zero() {
        let rz = env(0).round(Round::TowardZero);
        assert_eq!(
            to_int(true, d(1e10), true, rz),
            (0x7fff_ffff, Flags::INVALID)
        );
        assert_eq!(
            to_int(true, d(-1e10), true, rz),
            (0x8000_0000, Flags::INVALID)
        );
        assert_eq!(to_int(false, s(-1.0), false, rz), (0, Flags::INVALID));
        assert_eq!(
            to_int(true, 0x7ff8_0000_0000_0000, true, rz),
            (0, Flags::INVALID)
        );
        assert_eq!(
            to_int(true, d(-2.7), true, rz),
            ((-2i32) as u32, Flags::INEXACT)
        );
        assert_eq!(to_int(true, d(-2.5), true, env(0)).0, (-2i32) as u32);
        assert_eq!(
            from_int(true, 0xffff_ffff, true, env(0)),
            (d(-1.0), Flags::NONE)
        );
        assert_eq!(
            from_int(true, 0xffff_ffff, false, env(0)),
            (d(4_294_967_295.0), Flags::NONE)
        );
    }

    #[test]
    fn half_precision_both_formats() {
        let e = env(0);
        assert_eq!(half_to_single(0x3c00, false, e), (0x3f80_0000, Flags::NONE));
        assert_eq!(half_to_single(0x7c00, false, e).0, 0x7f80_0000);
        // Under AHP 0x7c00 is 2^16, not infinity.
        assert_eq!(half_to_single(0x7c00, true, e).0, s(65536.0) as u32);
        assert_eq!(half_to_single(0xffff, true, e).0, s(-131_008.0) as u32);
        assert_eq!(
            single_to_half(s(1.0) as u32, false, e),
            (0x3c00, Flags::NONE)
        );
        assert_eq!(
            single_to_half(s(65536.0) as u32, true, e),
            (0x7c00, Flags::NONE)
        );
        assert_eq!(
            single_to_half(s(65536.0) as u32, false, e),
            (0x7c00, Flags::OVERFLOW | Flags::INEXACT)
        );
        assert_eq!(
            single_to_half(s(1e9) as u32, true, e),
            (0x7fff, Flags::INVALID)
        );
        assert_eq!(single_to_half(0x7fc0_0000, true, e), (0, Flags::INVALID));
        assert_eq!(
            single_to_half(0xff80_0000, true, e),
            (0xffff, Flags::INVALID)
        );
        // 2^15 + 2^4 is a tie on AHP's 2^5 grid in that binade, and rounds to
        // even.
        assert_eq!(single_to_half(s(32784.0) as u32, true, e).0, 0x7800);
        // Largest AHP value.
        assert_eq!(single_to_half(s(131_008.0) as u32, true, e).0, 0x7fff);
        // Toward zero still saturates with invalid alone.
        let rz = env(3 << 22);
        assert_eq!(
            single_to_half(s(1e9) as u32, true, rz),
            (0x7fff, Flags::INVALID)
        );
        assert_eq!(
            single_to_half(s(131_071.0) as u32, true, rz),
            (0x7fff, Flags::INEXACT)
        );
    }
}
