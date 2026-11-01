//! The floating-point unit.
//!
//! Arithmetic goes through the crate's software IEEE-754 ([`crate::float`]),
//! never the host FPU, so results are bit-identical on every host
//! (`ROADMAP.md` §9.1). What this file adds is the SH-4A's paperwork around
//! that arithmetic, all of it from section 10.3 of the manual, whose C
//! descriptions are explicit about every special case:
//!
//! * **NaNs are the other way up.** On the SH-4A a NaN whose fraction MSB is
//!   *set* is the **signaling** one (§6.2.2, figure 6.3, table 6.2), the
//!   reverse of IEEE 754-2008. So NaN operands never reach the soft-float:
//!   [`class`] decides them here, an sNaN raises invalid and a qNaN gives the
//!   one qNaN the FPU ever produces, `H'7FBFFFFF` (or `H'7FF7FFFF_FFFFFFFF`)
//!   (§6.2.2). A NaN the soft-float *generates* from an invalid operation is
//!   replaced by the same value.
//! * **Denormals.** With `FPSCR.DN = 1` a denormal operand is a zero of its
//!   sign and a denormal result is flushed to one, raising inexact and
//!   underflow (`check_single_exception`, §10.3). With `DN = 0` a denormal
//!   operand to an arithmetic instruction raises the **FPU error** `E`, which
//!   cannot be masked and always traps (§6.5.2).
//! * **Two rounding modes**: `RM = 00` nearest, `01` toward zero (§6.4).
//! * **Exception traps.** A cause bit whose enable is set traps (`EXPEVT =
//!   H'120`), leaving the destination unchanged. Beyond that, the manual's C
//!   (`check_single_exception`) traps an operation on ordinary operands
//!   whenever *any* of the `O`, `U`, `I` enables is set, "regardless of
//!   whether or not an exception has occurred" — the hardware decides
//!   pessimistically before it knows. That rule is followed as written.
//! * **`FIPR`, `FTRV`, `FSCA`, `FSRRA` are approximations** whose exact
//!   results "differ between processor cores" (§6.6.1). They are computed
//!   here deterministically — products exact in double precision and summed
//!   there, sine and cosine by a fixed polynomial in software double
//!   precision — within the manual's stated error bounds, and always raise
//!   inexact as the manual requires.

use crate::core::value::{Endian, Width};
use crate::float::{self, B32, B64, Env, Flags, IntOverflow, Round};

use super::exec::{Exec, Trap, fpscr};
use super::isa::{self, Op};

/// The one quiet NaN the FPU produces, single precision (§6.2.2).
pub const QNAN32: u32 = 0x7fbf_ffff;
/// The one quiet NaN the FPU produces, double precision.
pub const QNAN64: u64 = 0x7ff7_ffff_ffff_ffff;

/// `FPSCR` cause/flag bits for each event (the `SET_*` constants of §10.3).
mod cause {
    /// FPU error: cause only, no flag, no enable.
    pub(super) const E: u32 = 0x0002_0000;
    /// Invalid operation.
    pub(super) const V: u32 = 0x0001_0040;
    /// Division by zero.
    pub(super) const Z: u32 = 0x0000_8020;
    /// Overflow.
    pub(super) const O: u32 = 0x0000_4010;
    /// Underflow.
    pub(super) const U: u32 = 0x0000_2008;
    /// Inexact.
    pub(super) const I: u32 = 0x0000_1004;
    /// The `V` enable.
    pub(super) const EN_V: u32 = 0x0000_0800;
    /// The `Z` enable.
    pub(super) const EN_Z: u32 = 0x0000_0400;
    /// The `O`, `U` and `I` enables.
    pub(super) const EN_OUI: u32 = 0x0000_0380;
    /// The `I` enable.
    pub(super) const EN_I: u32 = 0x0000_0080;
}

/// What an operand is, after `FPSCR.DN` has had its say (`data_type_of`,
/// §10.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Class {
    /// A zero (or a denormal under `DN = 1`), with its sign.
    Zero(bool),
    /// A denormal under `DN = 0`.
    Denorm,
    /// A finite, normal number.
    Norm,
    /// An infinity, with its sign.
    Inf(bool),
    /// A quiet NaN — fraction MSB *clear* on this FPU.
    QNan,
    /// A signaling NaN — fraction MSB *set*.
    SNan,
}

/// Classify a value of the given precision.
#[must_use]
pub fn class(bits: u64, double: bool, dn: bool) -> Class {
    let (sign, exp, frac, emax, qbit) = if double {
        (
            bits >> 63 != 0,
            (bits >> 52) & 0x7ff,
            bits & 0x000f_ffff_ffff_ffff,
            0x7ff,
            1u64 << 51,
        )
    } else {
        (
            bits >> 31 & 1 != 0,
            (bits >> 23) & 0xff,
            bits & 0x007f_ffff,
            0xff,
            1u64 << 22,
        )
    };
    if exp == 0 {
        if frac == 0 || dn {
            Class::Zero(sign)
        } else {
            Class::Denorm
        }
    } else if exp == emax {
        if frac == 0 {
            Class::Inf(sign)
        } else if frac & qbit != 0 {
            Class::SNan
        } else {
            Class::QNan
        }
    } else {
        Class::Norm
    }
}

/// A zero of the given sign and precision.
#[inline]
const fn zero(sign: bool, double: bool) -> u64 {
    if double {
        (sign as u64) << 63
    } else {
        (sign as u64) << 31
    }
}

/// The value the FPU computes with: a `DN`-flushed denormal becomes a zero.
#[inline]
fn operand(bits: u64, c: Class, double: bool) -> u64 {
    match c {
        Class::Zero(s) => zero(s, double),
        _ => bits,
    }
}

/// The precision's quiet NaN.
#[inline]
const fn qnan(double: bool) -> u64 {
    if double { QNAN64 } else { QNAN32 as u64 }
}

/// The binary operations that share the classification preamble.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bin {
    Add,
    Sub,
    Mul,
    Div,
}

impl Exec<'_> {
    // -----------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------

    /// The bank `FRn` names (`FPSCR.FR`).
    #[inline]
    fn fbank(&self) -> usize {
        usize::from(self.st.fpscr & fpscr::FR != 0)
    }

    /// `FRn`.
    #[inline]
    pub(super) fn fr(&self, i: usize) -> u32 {
        self.st.fpr[self.fbank()][i & 15]
    }

    /// Write `FRn`.
    #[inline]
    pub(super) fn set_fr(&mut self, i: usize, v: u32) {
        let b = self.fbank();
        self.st.fpr[b][i & 15] = v;
    }

    /// `XFn`, the other bank.
    #[inline]
    fn xf(&self, i: usize) -> u32 {
        self.st.fpr[self.fbank() ^ 1][i & 15]
    }

    /// `DRn`: `FRn` is the high half, `FRn+1` the low (§2.2.3).
    #[inline]
    pub(super) fn dr(&self, i: usize) -> u64 {
        let i = i & 14;
        (u64::from(self.fr(i)) << 32) | u64::from(self.fr(i + 1))
    }

    /// Write `DRn`.
    #[inline]
    pub(super) fn set_dr(&mut self, i: usize, v: u64) {
        let i = i & 14;
        self.set_fr(i, (v >> 32) as u32);
        self.set_fr(i + 1, v as u32);
    }

    /// The register a pair transfer names: `DRn` for an even field, `XDn-1`
    /// for an odd one (`1111nnn1....` is `XDn` in table 3.13).
    fn pair(&self, field: usize) -> (u32, u32) {
        let i = field & 14;
        if field & 1 == 0 {
            (self.fr(i), self.fr(i + 1))
        } else {
            (self.xf(i), self.xf(i + 1))
        }
    }

    /// Write a pair transfer's register.
    fn set_pair(&mut self, field: usize, hi: u32, lo: u32) {
        let i = field & 14;
        let b = if field & 1 == 0 {
            self.fbank()
        } else {
            self.fbank() ^ 1
        };
        self.st.fpr[b][i] = hi;
        self.st.fpr[b][i + 1] = lo;
    }

    /// `FPSCR.PR`.
    #[inline]
    fn pr(&self) -> bool {
        self.st.fpscr & fpscr::PR != 0
    }

    /// `FPSCR.SZ`.
    #[inline]
    fn sz(&self) -> bool {
        self.st.fpscr & fpscr::SZ != 0
    }

    /// `FPSCR.DN`.
    #[inline]
    fn dn(&self) -> bool {
        self.st.fpscr & fpscr::DN != 0
    }

    /// The soft-float environment `FPSCR.RM` selects. Only `RM[0]` matters:
    /// the manual's C reads `FPSCR_RM` as `FPSCR & 1` (§10.3).
    fn env(&self) -> Env {
        let round = if self.st.fpscr & 1 != 0 {
            Round::TowardZero
        } else {
            Round::TiesEven
        };
        Env {
            int_overflow: IntOverflow::SaturateNanMax,
            ..Env::RISCV
        }
        .round(round)
    }

    // -----------------------------------------------------------------
    // Exception bookkeeping
    // -----------------------------------------------------------------

    /// Clear the cause field, as every arithmetic instruction does first.
    #[inline]
    fn clear_cause(&mut self) {
        self.st.fpscr &= !fpscr::CAUSE;
    }

    /// Record events in the cause and flag fields.
    #[inline]
    fn raise(&mut self, bits: u32) {
        self.st.fpscr |= bits;
    }

    /// The FPU exception (`EXPEVT = H'120`), a re-execution type exception
    /// through the general vector (table 5.3).
    fn fpu_trap(&self) -> Trap {
        Trap {
            code: super::exec::exc::FPU,
            vector: super::exec::vector::GENERAL,
            tea: None,
            kind: super::exec::Kind::Reexec,
        }
    }

    /// `invalid(n)`: set `V`, then trap if enabled or produce the qNaN.
    fn invalid(&mut self) -> Result<u64, Trap> {
        self.raise(cause::V);
        if self.st.fpscr & cause::EN_V != 0 {
            Err(self.fpu_trap())
        } else {
            Ok(qnan(self.pr()))
        }
    }

    /// `set_E()`: the FPU error always traps.
    fn error(&mut self) -> Trap {
        self.raise(cause::E);
        self.fpu_trap()
    }

    /// Fold soft-float flags into `FPSCR` for an operation that went through
    /// the arithmetic, apply `DN` to a denormal result, and decide whether to
    /// trap. `normal` says the operands were ordinary numbers, which is when
    /// the manual's `check_*_exception` runs and its pessimistic O/U/I rule
    /// applies.
    fn settle(
        &mut self,
        mut r: u64,
        flags: Flags,
        double: bool,
        normal: bool,
    ) -> Result<u64, Trap> {
        if flags.contains(Flags::INVALID) {
            return self.invalid();
        }
        if flags.contains(Flags::DIV_BY_ZERO) {
            self.raise(cause::Z);
            if self.st.fpscr & cause::EN_Z != 0 {
                return Err(self.fpu_trap());
            }
            return Ok(r);
        }
        let mut bits = 0;
        if flags.contains(Flags::OVERFLOW) {
            bits |= cause::O | cause::I;
        }
        if flags.contains(Flags::INEXACT) {
            bits |= cause::I;
        }
        if flags.contains(Flags::UNDERFLOW) {
            bits |= cause::U;
        }
        // `check_*_exception`: a tiny non-zero result under DN = 1 is zeroed
        // and is inexact; then a tiny result that is inexact underflows.
        if let Class::Denorm = class(r, double, false)
            && self.dn()
        {
            r = zero(r >> (if double { 63 } else { 31 }) & 1 != 0, double);
            bits |= cause::I | cause::U;
        }
        self.raise(bits);
        if normal && self.st.fpscr & cause::EN_OUI != 0 {
            return Err(self.fpu_trap());
        }
        Ok(r)
    }

    // -----------------------------------------------------------------
    // Dispatch
    // -----------------------------------------------------------------

    /// Execute an `H'F***` instruction.
    pub(super) fn fpu(&mut self, op: Op, w: u16) -> Result<(), Trap> {
        let n = isa::n(w);
        let m = isa::m(w);
        match op {
            Op::Fadd => self.binary(Bin::Add, n, m),
            Op::Fsub => self.binary(Bin::Sub, n, m),
            Op::Fmul => self.binary(Bin::Mul, n, m),
            Op::Fdiv => self.binary(Bin::Div, n, m),
            Op::FcmpEq | Op::FcmpGt => self.fcmp(op == Op::FcmpGt, n, m),
            Op::Fmac => self.fmac(n, m),
            Op::Fsqrt => self.fsqrt(n),
            Op::Fneg | Op::Fabs => {
                // §10.3.1/§10.3.16: a bit operation on FRn — the high word of
                // DRn under PR — that touches neither cause nor flag.
                let v = self.fr(n);
                let v = if op == Op::Fneg {
                    v ^ 0x8000_0000
                } else {
                    v & 0x7fff_ffff
                };
                let i = if self.pr() { n & 14 } else { n };
                self.set_fr(i, v);
                Ok(())
            }
            Op::Fldi0 | Op::Fldi1 => {
                if self.pr() {
                    return Err(self.undefined());
                }
                self.set_fr(n, if op == Op::Fldi0 { 0 } else { 0x3f80_0000 });
                Ok(())
            }
            Op::Fsts => {
                self.set_fr(n, self.st.fpul);
                Ok(())
            }
            Op::Flds => {
                self.st.fpul = self.fr(n);
                Ok(())
            }
            Op::Float => self.float(n),
            Op::Ftrc => self.ftrc(n),
            Op::Fcnvsd => self.fcnvsd(n),
            Op::Fcnvds => self.fcnvds(n),
            Op::Fipr => self.fipr_insn(w),
            Op::Ftrv => self.ftrv(w),
            Op::Fsca => self.fsca(n),
            Op::Fsrra => self.fsrra(n),
            Op::Fschg | Op::Frchg => {
                // §10.3.18, §10.3.20: undefined with PR = 1.
                if self.pr() {
                    return Err(self.undefined());
                }
                self.st.fpscr ^= if op == Op::Fschg {
                    fpscr::SZ
                } else {
                    fpscr::FR
                };
                Ok(())
            }
            Op::Fpchg => {
                self.st.fpscr ^= fpscr::PR;
                Ok(())
            }
            Op::Fmov => {
                if self.sz() {
                    let (hi, lo) = self.pair(m);
                    self.set_pair(n, hi, lo);
                } else {
                    self.set_fr(n, self.fr(m));
                }
                Ok(())
            }
            Op::FmovL | Op::FmovLP | Op::FmovLIdx => {
                let base = self.r(m);
                let a = if op == Op::FmovLIdx {
                    base.wrapping_add(self.r(0))
                } else {
                    base
                };
                let size = if self.sz() {
                    let (hi, lo) = self.load_pair(a)?;
                    self.set_pair(n, hi, lo);
                    8
                } else {
                    let v = self.read(a, Width::U32)?;
                    self.set_fr(n, v);
                    4
                };
                if op == Op::FmovLP {
                    self.set_r(m, base.wrapping_add(size));
                }
                Ok(())
            }
            Op::FmovS | Op::FmovSIdx => {
                let a = if op == Op::FmovSIdx {
                    self.r(n).wrapping_add(self.r(0))
                } else {
                    self.r(n)
                };
                if self.sz() {
                    let (hi, lo) = self.pair(m);
                    self.store_pair(a, hi, lo)
                } else {
                    self.write(a, Width::U32, self.fr(m))
                }
            }
            Op::FmovSM => {
                if self.sz() {
                    let a = self.r(n).wrapping_sub(8);
                    let (hi, lo) = self.pair(m);
                    self.store_pair(a, hi, lo)?;
                    self.set_r(n, a);
                    Ok(())
                } else {
                    let v = self.fr(m);
                    self.push(n, Width::U32, v)
                }
            }
            _ => Err(self.undefined()),
        }
    }

    // -----------------------------------------------------------------
    // Pair transfers
    // -----------------------------------------------------------------

    /// Where the two halves of a pair live in memory (figure 2.5): the high
    /// register at the lower address, except little-endian with `PR = 1`,
    /// where the pair is one little-endian 64-bit double.
    fn pair_order(&self, a: u32) -> (u32, u32) {
        if self.cfg.endian == Endian::Little && self.pr() {
            (a.wrapping_add(4), a)
        } else {
            (a, a.wrapping_add(4))
        }
    }

    /// Load a pair. A quadword access must be eight-byte aligned (§5.6.2).
    fn load_pair(&mut self, a: u32) -> Result<(u32, u32), Trap> {
        if a & 7 != 0 {
            return Err(Trap {
                code: super::exec::exc::ADDR_READ,
                vector: super::exec::vector::GENERAL,
                tea: Some(a),
                kind: super::exec::Kind::Reexec,
            });
        }
        let (ah, al) = self.pair_order(a);
        let hi = self.read(ah, Width::U32)?;
        let lo = self.read(al, Width::U32)?;
        Ok((hi, lo))
    }

    /// Store a pair; both halves are checked before either is written.
    fn store_pair(&mut self, a: u32, hi: u32, lo: u32) -> Result<(), Trap> {
        if a & 7 != 0 {
            return Err(Trap {
                code: super::exec::exc::ADDR_WRITE,
                vector: super::exec::vector::GENERAL,
                tea: Some(a),
                kind: super::exec::Kind::Reexec,
            });
        }
        let (ah, al) = self.pair_order(a);
        let th = self.probe(ah, Width::U32, super::exec::Access::Write)?;
        let tl = self.probe(al, Width::U32, super::exec::Access::Write)?;
        self.access_write(th, Width::U32, hi);
        self.access_write(tl, Width::U32, lo);
        Ok(())
    }

    // -----------------------------------------------------------------
    // Arithmetic
    // -----------------------------------------------------------------

    /// Read the operand register `i` at the current precision.
    fn val(&self, i: usize) -> u64 {
        if self.pr() {
            self.dr(i)
        } else {
            u64::from(self.fr(i))
        }
    }

    /// Write the destination register `i` at the current precision.
    fn set_val(&mut self, i: usize, v: u64) {
        if self.pr() {
            self.set_dr(i, v);
        } else {
            self.set_fr(i, v as u32);
        }
    }

    /// `FADD`, `FSUB`, `FMUL`, `FDIV` (§10.3.2, §10.3.6, §10.3.15, §10.3.24).
    fn binary(&mut self, op: Bin, n: usize, m: usize) -> Result<(), Trap> {
        self.clear_cause();
        let d = self.pr();
        let dn = self.dn();
        let (a, b) = (self.val(n), self.val(m));
        let (ca, cb) = (class(a, d, dn), class(b, d, dn));
        let r = if ca == Class::SNan || cb == Class::SNan {
            self.invalid()?
        } else if ca == Class::QNan || cb == Class::QNan {
            qnan(d)
        } else if ca == Class::Denorm || cb == Class::Denorm {
            return Err(self.error());
        } else {
            let (x, y) = (operand(a, ca, d), operand(b, cb, d));
            let env = self.env();
            let (r, f) = match (op, d) {
                (Bin::Add, false) => float::add::<B32>(x, y, env),
                (Bin::Add, true) => float::add::<B64>(x, y, env),
                (Bin::Sub, false) => float::sub::<B32>(x, y, env),
                (Bin::Sub, true) => float::sub::<B64>(x, y, env),
                (Bin::Mul, false) => float::mul::<B32>(x, y, env),
                (Bin::Mul, true) => float::mul::<B64>(x, y, env),
                (Bin::Div, false) => float::div::<B32>(x, y, env),
                (Bin::Div, true) => float::div::<B64>(x, y, env),
            };
            let normal = ca == Class::Norm && cb == Class::Norm;
            self.settle(r, f, d, normal)?
        };
        self.set_val(n, r);
        Ok(())
    }

    /// `FCMP/EQ` and `FCMP/GT` (§10.3.3): an sNaN is invalid for both, a qNaN
    /// is invalid for `GT` only and simply unequal for `EQ`.
    fn fcmp(&mut self, gt: bool, n: usize, m: usize) -> Result<(), Trap> {
        self.clear_cause();
        let d = self.pr();
        let dn = self.dn();
        let (a, b) = (self.val(n), self.val(m));
        let (ca, cb) = (class(a, d, dn), class(b, d, dn));
        let snan = ca == Class::SNan || cb == Class::SNan;
        let qnan = ca == Class::QNan || cb == Class::QNan;
        if snan || (gt && qnan) {
            self.raise(cause::V);
            if self.st.fpscr & cause::EN_V != 0 {
                return Err(self.fpu_trap());
            }
            self.st.set_t(false);
            return Ok(());
        }
        if qnan {
            self.st.set_t(false);
            return Ok(());
        }
        let (x, y) = (operand(a, ca, d), operand(b, cb, d));
        let ord = if d {
            float::compare::<B64>(x, y)
        } else {
            float::compare::<B32>(x, y)
        };
        let t = match ord {
            Some(core::cmp::Ordering::Equal) => !gt,
            Some(core::cmp::Ordering::Greater) => gt,
            _ => false,
        };
        self.st.set_t(t);
        Ok(())
    }

    /// `FMAC FR0,FRm,FRn` (§10.3.12): one rounding.
    fn fmac(&mut self, n: usize, m: usize) -> Result<(), Trap> {
        self.clear_cause();
        if self.pr() {
            return Err(self.undefined());
        }
        let dn = self.dn();
        let (a, b, c) = (
            u64::from(self.fr(0)),
            u64::from(self.fr(m)),
            u64::from(self.fr(n)),
        );
        let (ca, cb, cc) = (
            class(a, false, dn),
            class(b, false, dn),
            class(c, false, dn),
        );
        let all = [ca, cb, cc];
        let r = if all.contains(&Class::SNan) {
            self.invalid()?
        } else if ca == Class::QNan || cb == Class::QNan {
            QNAN32 as u64
        } else if ca == Class::Denorm || cb == Class::Denorm {
            return Err(self.error());
        } else if matches!(
            (ca, cb),
            (Class::Zero(_), Class::Inf(_)) | (Class::Inf(_), Class::Zero(_))
        ) {
            // A zero times an infinity is invalid even with a qNaN addend.
            self.invalid()?
        } else if cc == Class::QNan {
            QNAN32 as u64
        } else if cc == Class::Denorm {
            return Err(self.error());
        } else {
            let (x, y, z) = (
                operand(a, ca, false),
                operand(b, cb, false),
                operand(c, cc, false),
            );
            let (r, f) = float::fma::<B32>(x, y, z, self.env());
            let normal = ca == Class::Norm
                && cb == Class::Norm
                && matches!(cc, Class::Norm | Class::Zero(_));
            self.settle(r, f, false, normal)?
        };
        self.set_fr(n, r as u32);
        Ok(())
    }

    /// `FSQRT` (§10.3.21). Only the `I` enable traps on an ordinary operand.
    fn fsqrt(&mut self, n: usize) -> Result<(), Trap> {
        self.clear_cause();
        let d = self.pr();
        let a = self.val(n);
        let r = match class(a, d, self.dn()) {
            Class::Norm if a >> (if d { 63 } else { 31 }) & 1 == 0 => {
                let (r, f) = if d {
                    float::sqrt::<B64>(a, self.env())
                } else {
                    float::sqrt::<B32>(a, self.env())
                };
                if f.contains(Flags::INEXACT) {
                    self.raise(cause::I);
                }
                if self.st.fpscr & cause::EN_I != 0 {
                    return Err(self.fpu_trap());
                }
                r
            }
            Class::Norm | Class::Inf(true) | Class::SNan => self.invalid()?,
            Class::Denorm if a >> (if d { 63 } else { 31 }) & 1 != 0 => self.invalid()?,
            Class::Denorm => return Err(self.error()),
            Class::Zero(s) => zero(s, d),
            Class::Inf(false) => a,
            Class::QNan => qnan(d),
        };
        self.set_val(n, r);
        Ok(())
    }

    /// `FLOAT FPUL,FRn` (§10.3.11): only the single-precision form can be
    /// inexact, and `I`'s enable traps it regardless.
    fn float(&mut self, n: usize) -> Result<(), Trap> {
        self.clear_cause();
        let v = i64::from(self.st.fpul as i32);
        if self.pr() {
            let (r, _) = float::from_signed::<B64>(v, 32, self.env());
            self.set_dr(n, r);
        } else {
            let (r, f) = float::from_signed::<B32>(v, 32, self.env());
            if f.contains(Flags::INEXACT) {
                self.raise(cause::I);
            }
            if self.st.fpscr & cause::EN_I != 0 {
                return Err(self.fpu_trap());
            }
            self.set_fr(n, r as u32);
        }
        Ok(())
    }

    /// `FTRC FRm,FPUL` (§10.3.25): always truncates; out-of-range values
    /// (and NaNs, by the ranges the manual gives) are invalid and saturate.
    fn ftrc(&mut self, m: usize) -> Result<(), Trap> {
        self.clear_cause();
        let d = self.pr();
        let a = self.val(m);
        // The manual's own range test, on the raw bits: positive values above
        // 1.fffffe*2^30 (or 2^31 in double) and every positive NaN are "+MAX"
        // ... except that a positive NaN is classed `NINF` and gives -MAX.
        let (neg, pos_nan, pos_over, neg_over) = if d {
            let mag = a & 0x7fff_ffff_ffff_ffff;
            (
                a >> 63 != 0,
                mag > 0x7ff0_0000_0000_0000,
                a >= 0x41e0_0000_0000_0000,
                mag >= 0x41e0_0000_0020_0000,
            )
        } else {
            let a = a as u32;
            let mag = a & 0x7fff_ffff;
            (
                a >> 31 != 0,
                a > 0x7f80_0000,
                a > 0x4eff_ffff,
                mag > 0x4f00_0000,
            )
        };
        let saturate = if !neg {
            if pos_nan {
                Some(0x8000_0000)
            } else if pos_over {
                Some(0x7fff_ffff)
            } else {
                None
            }
        } else if neg_over {
            Some(0x8000_0000)
        } else {
            None
        };
        if let Some(v) = saturate {
            self.raise(cause::V);
            if self.st.fpscr & cause::EN_V != 0 {
                return Err(self.fpu_trap());
            }
            self.st.fpul = v;
            return Ok(());
        }
        let env = self.env().round(Round::TowardZero);
        let x = operand(a, class(a, d, self.dn()), d);
        let (r, _) = if d {
            float::to_signed::<B64>(x, 32, env)
        } else {
            float::to_signed::<B32>(x, 32, env)
        };
        self.st.fpul = r as u32;
        Ok(())
    }

    /// `FCNVSD FPUL,DRn` (§10.3.5): exact, PR = 1 only.
    fn fcnvsd(&mut self, n: usize) -> Result<(), Trap> {
        self.clear_cause();
        if !self.pr() {
            return Err(self.undefined());
        }
        let a = u64::from(self.st.fpul);
        let r = match class(a, false, self.dn()) {
            Class::SNan => self.invalid()?,
            Class::QNan => QNAN64,
            Class::Denorm => return Err(self.error()),
            c => float::convert::<B32, B64>(operand(a, c, false), self.env()).0,
        };
        self.set_dr(n, r);
        Ok(())
    }

    /// `FCNVDS DRm,FPUL` (§10.3.4): rounds, PR = 1 only. A NaN input gives
    /// the single-precision qNaN.
    fn fcnvds(&mut self, m: usize) -> Result<(), Trap> {
        self.clear_cause();
        if !self.pr() {
            return Err(self.undefined());
        }
        let a = self.dr(m);
        let r = match class(a, true, self.dn()) {
            Class::SNan => {
                self.raise(cause::V);
                if self.st.fpscr & cause::EN_V != 0 {
                    return Err(self.fpu_trap());
                }
                QNAN32 as u64
            }
            Class::QNan => QNAN32 as u64,
            Class::Denorm => return Err(self.error()),
            Class::Inf(s) => {
                if s {
                    0xff80_0000
                } else {
                    0x7f80_0000
                }
            }
            c @ (Class::Norm | Class::Zero(_)) => {
                let (r, f) = float::convert::<B64, B32>(operand(a, c, true), self.env());
                self.settle(r, f, false, true)?
            }
        };
        self.st.fpul = r as u32;
        Ok(())
    }

    /// The inner product of `FVm` and `FVn` the way `fipr()` in §10.3
    /// defines its special cases: an sNaN anywhere, or a zero times an
    /// infinity, is invalid; then a qNaN anywhere is a qNaN; then infinities
    /// of both signs are invalid and of one sign are that infinity.
    /// Otherwise the four products — exact in double precision — are summed
    /// there and rounded once to single, always inexact (§6.6.1).
    fn inner(&mut self, a: [u32; 4], b: [u32; 4]) -> Result<u32, Trap> {
        let dn = self.dn();
        let ca = a.map(|v| class(u64::from(v), false, dn));
        let cb = b.map(|v| class(u64::from(v), false, dn));
        let snan = ca.contains(&Class::SNan) || cb.contains(&Class::SNan);
        let zero_inf = (0..4).any(|i| {
            matches!(
                (ca[i], cb[i]),
                (Class::Zero(_), Class::Inf(_)) | (Class::Inf(_), Class::Zero(_))
            )
        });
        if snan || zero_inf {
            return Ok(self.invalid()? as u32);
        }
        if ca.contains(&Class::QNan) || cb.contains(&Class::QNan) {
            return Ok(QNAN32);
        }
        let mut pos_inf = false;
        let mut neg_inf = false;
        for i in 0..4 {
            if matches!(ca[i], Class::Inf(_)) || matches!(cb[i], Class::Inf(_)) {
                if (a[i] ^ b[i]) >> 31 != 0 {
                    neg_inf = true;
                } else {
                    pos_inf = true;
                }
            }
        }
        if pos_inf && neg_inf {
            return Ok(self.invalid()? as u32);
        }
        if pos_inf || neg_inf {
            return Ok(if neg_inf { 0xff80_0000 } else { 0x7f80_0000 });
        }
        let env = self.env();
        let wide =
            |v: u32, c: Class| float::convert::<B32, B64>(operand(u64::from(v), c, false), env).0;
        let mut sum = 0u64;
        let mut flags = Flags::NONE;
        for i in 0..4 {
            let (p, f) = float::mul::<B64>(wide(a[i], ca[i]), wide(b[i], cb[i]), env);
            let (s, g) = float::add::<B64>(sum, p, env);
            sum = s;
            flags |= f | g;
        }
        let (r, f) = float::convert::<B64, B32>(sum, env);
        // "the inexact exception (I) bit in both the FPU exception cause field
        // and flag field are always set to 1" (§6.6.1).
        let r = self.settle(r, f | flags | Flags::INEXACT, false, true)?;
        Ok(r as u32)
    }

    /// `FIPR FVm,FVn` (§10.3.7): the result goes to `FR[n+3]`.
    fn fipr_insn(&mut self, w: u16) -> Result<(), Trap> {
        self.clear_cause();
        if self.pr() {
            return Err(self.undefined());
        }
        let vn = (((w >> 10) & 3) * 4) as usize;
        let vm = (((w >> 8) & 3) * 4) as usize;
        let a = [
            self.fr(vm),
            self.fr(vm + 1),
            self.fr(vm + 2),
            self.fr(vm + 3),
        ];
        let b = [
            self.fr(vn),
            self.fr(vn + 1),
            self.fr(vn + 2),
            self.fr(vn + 3),
        ];
        let r = self.inner(a, b)?;
        self.set_fr(vn + 3, r);
        Ok(())
    }

    /// `FTRV XMTRX,FVn` (§10.3.26): four inner products of the rows of
    /// `XMTRX` with `FVn`, with every enable masked while they run and the
    /// trap decided once at the end — "when FPSCR.enable.V/O/U/I is set, an
    /// FPU exception trap is generated regardless".
    fn ftrv(&mut self, w: u16) -> Result<(), Trap> {
        self.clear_cause();
        if self.pr() {
            return Err(self.undefined());
        }
        let vn = (((w >> 10) & 3) * 4) as usize;
        let v = [
            self.fr(vn),
            self.fr(vn + 1),
            self.fr(vn + 2),
            self.fr(vn + 3),
        ];
        let saved = self.st.fpscr;
        self.st.fpscr &= !(cause::EN_V | cause::EN_OUI);
        let mut out = [0u32; 4];
        for (i, slot) in out.iter_mut().enumerate() {
            let row = [self.xf(i), self.xf(i + 4), self.xf(i + 8), self.xf(i + 12)];
            *slot = self.inner(row, v)?;
        }
        let events = self.st.fpscr & (fpscr::CAUSE | fpscr::FLAG);
        self.st.fpscr = saved | events;
        if saved & (cause::EN_V | cause::EN_OUI) != 0 {
            return Err(self.fpu_trap());
        }
        for (i, r) in out.iter().enumerate() {
            self.set_fr(vn + i, *r);
        }
        Ok(())
    }

    /// `FSCA FPUL,DRn` (§10.3.19): the sine and cosine of `FPUL[15:0]/65536`
    /// of a turn, into `FRn` and `FRn+1`, within ±2^-21, always inexact.
    fn fsca(&mut self, n: usize) -> Result<(), Trap> {
        self.clear_cause();
        if self.pr() {
            return Err(self.undefined());
        }
        self.raise(cause::I);
        if self.st.fpscr & cause::EN_I != 0 {
            return Err(self.fpu_trap());
        }
        let (s, c) = sin_cos_turn(self.st.fpul & 0xffff);
        let i = n & 14;
        self.set_fr(i, s);
        self.set_fr(i + 1, c);
        Ok(())
    }

    /// `FSRRA FRn` (§10.3.22): `1/sqrt(FRn)` within ±2^-21, inexact for a
    /// normal input.
    fn fsrra(&mut self, n: usize) -> Result<(), Trap> {
        self.clear_cause();
        if self.pr() {
            return Err(self.undefined());
        }
        let a = u64::from(self.fr(n));
        let r = match class(a, false, self.dn()) {
            Class::Norm if a >> 31 == 0 => {
                self.raise(cause::I);
                if self.st.fpscr & cause::EN_I != 0 {
                    return Err(self.fpu_trap());
                }
                let env = self.env();
                let x = float::convert::<B32, B64>(a, env).0;
                let (root, _) = float::sqrt::<B64>(x, env);
                let (inv, _) = float::div::<B64>(0x3ff0_0000_0000_0000, root, env);
                float::convert::<B64, B32>(inv, env).0
            }
            Class::Norm | Class::Inf(true) | Class::SNan => self.invalid()?,
            Class::Denorm if a >> 31 != 0 => self.invalid()?,
            Class::Denorm => return Err(self.error()),
            Class::Zero(s) => {
                self.raise(cause::Z);
                if self.st.fpscr & cause::EN_Z != 0 {
                    return Err(self.fpu_trap());
                }
                if s { 0xff80_0000 } else { 0x7f80_0000 }
            }
            Class::Inf(false) => 0,
            Class::QNan => QNAN32 as u64,
        };
        self.set_fr(n, r as u32);
        Ok(())
    }
}

/// Sine and cosine of `k/65536` of a full turn, as single-precision bits.
///
/// Computed in software double precision so it is the same on every host:
/// the angle is folded into the first octant by the turn's symmetries —
/// exact, because `k` is an integer — and a fixed Taylor polynomial in
/// `x ≤ π/4` runs to the `x^15` term, whose truncation error (below 10^-14)
/// is far inside the manual's ±2^-21 (§10.3.19).
#[must_use]
pub fn sin_cos_turn(k: u32) -> (u32, u32) {
    let k = k & 0xffff;
    let quadrant = k >> 14;
    let r = k & 0x3fff;
    // Within the quadrant, reduce to the first octant: sin(π/2 - x) = cos x.
    let (j, swap) = if r > 0x2000 {
        (0x4000 - r, true)
    } else {
        (r, false)
    };
    let env = Env::RISCV;
    // x = j * 2π / 65536 = j * (π / 32768).
    const PI_OVER_32768: u64 = 0x3f19_21fb_5444_2d18; // π / 2^15
    let x = float::mul::<B64>(
        float::from_unsigned::<B64>(u64::from(j), 32, env).0,
        PI_OVER_32768,
        env,
    )
    .0;
    let (s, c) = taylor_sin_cos(x, env);
    let (s, c) = if swap { (c, s) } else { (s, c) };
    let neg = |v: u64| v ^ (1 << 63);
    // Rotate by the quadrant: (sin, cos) of θ + q·π/2.
    let (s, c) = match quadrant {
        0 => (s, c),
        1 => (c, neg(s)),
        2 => (neg(s), neg(c)),
        _ => (neg(c), s),
    };
    let to32 = |v: u64| float::convert::<B64, B32>(v, env).0 as u32;
    (to32(s), to32(c))
}

/// sin and cos of `x` in `[0, π/4]` by their Taylor series, in software
/// double precision.
fn taylor_sin_cos(x: u64, env: Env) -> (u64, u64) {
    let mul = |a, b| float::mul::<B64>(a, b, env).0;
    let add = |a, b| float::add::<B64>(a, b, env).0;
    let x2 = mul(x, x);
    // Horner from the top term down: the coefficients are ±1/n!, rounded to
    // double once, written as bits so nothing depends on a host's parsing.
    const SIN: [u64; 8] = [
        0x3ff0_0000_0000_0000, // 1
        0xbfc5_5555_5555_5555, // -1/3!
        0x3f81_1111_1111_1111, // 1/5!
        0xbf2a_01a0_1a01_a01a, // -1/7!
        0x3ec7_1de3_a556_c734, // 1/9!
        0xbe5a_e645_67f5_44e4, // -1/11!
        0x3de6_1246_13a8_6d09, // 1/13!
        0xbd6a_e7f3_e733_b81f, // -1/15!
    ];
    const COS: [u64; 8] = [
        0x3ff0_0000_0000_0000, // 1
        0xbfe0_0000_0000_0000, // -1/2!
        0x3fa5_5555_5555_5555, // 1/4!
        0xbf56_c16c_16c1_6c17, // -1/6!
        0x3efa_01a0_1a01_a01a, // 1/8!
        0xbe92_7e4f_b778_9f5c, // -1/10!
        0x3e21_eed8_eff8_d898, // 1/12!
        0xbda9_3974_a8c0_7c9d, // -1/14!
    ];
    let mut s = SIN[7];
    let mut c = COS[7];
    for i in (0..7).rev() {
        s = add(mul(s, x2), SIN[i]);
        c = add(mul(c, x2), COS[i]);
    }
    (mul(s, x), c)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A host double, for comparison in tests only — the implementation
    /// never touches one.
    fn f(bits: u32) -> f64 {
        f64::from(f32::from_bits(bits))
    }

    #[test]
    fn the_nan_convention_is_the_sh4s() {
        // Fraction MSB set is *signaling* on this FPU (table 6.2).
        assert_eq!(class(0x7fc0_0000, false, false), Class::SNan);
        assert_eq!(class(0x7fbf_ffff, false, false), Class::QNan);
        assert_eq!(class(0x7ff8_0000_0000_0000, true, false), Class::SNan);
        assert_eq!(class(0x0000_0001, false, false), Class::Denorm);
        assert_eq!(class(0x8000_0001, false, true), Class::Zero(true));
    }

    #[test]
    fn fsca_is_within_the_manuals_error_bound_everywhere() {
        let bound = 2f64.powi(-21);
        for k in (0..65536u32).step_by(7) {
            let (s, c) = sin_cos_turn(k);
            let angle = 2.0 * core::f64::consts::PI * f64::from(k) / 65536.0;
            assert!((f(s) - angle.sin()).abs() <= bound, "sin at {k}");
            assert!((f(c) - angle.cos()).abs() <= bound, "cos at {k}");
        }
        // The exact points are exact.
        assert_eq!(sin_cos_turn(0), (0, 0x3f80_0000));
        assert_eq!(sin_cos_turn(0x4000).0, 0x3f80_0000);
    }
}
