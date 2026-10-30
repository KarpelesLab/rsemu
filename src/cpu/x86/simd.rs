//! Executing the extensions past SSE2: SSE3, SSSE3, SSE4.1 and SSE4.2.
//!
//! Every one of these is an SSE instruction in everything but its `CPUID`
//! bit — the same sixteen `XMM` registers, the same `MXCSR`, the same `CR0`
//! and `CR4` gates — so the dispatcher in `fpexec` applies those gates and
//! hands the operation here. What this file adds is the per-extension
//! feature check, which is a property of the *row* ([`Insn::extension`]), and
//! the instructions themselves.
//!
//! # Where the arithmetic comes from
//!
//! The floating-point lanes go through [`crate::float`] with `MXCSR` mapped
//! onto its [`Env`](crate::float::Env), exactly as SSE's own do — no host
//! float is involved — and an unmasked exception leaves the destination
//! untouched, through the same [`Exec::sse_raise`]. The lane kernels are free
//! functions, so the tests at the bottom can check them without a machine.
//!
//! # Sources
//!
//! *Intel SDM* volume 2, each instruction's reference page and "Operation"
//! section, and appendix A's opcode maps. No copyleft emulator was consulted
//! (`CLAUDE.md`, provenance).

use crate::float::{B32, B64, Env, Flags, InputSubnormal, Round, binary};

use super::exec::{Ex, Exec, Fault, VEC_UD};
use super::flags;
use super::isa::{Arg, Extension, Fields, Insn, ModRm, Op};

impl Exec<'_> {
    /// Whether this instance has the extension a row belongs to.
    ///
    /// `true` for a row that belongs to none: those are gated elsewhere, by
    /// the SSE and MMX dispatchers' own checks.
    pub(super) fn has_extension(&self, insn: Insn) -> bool {
        let features = self.cfg.features;
        match insn.extension() {
            None => true,
            Some(Extension::Sse3) => features.sse3,
            Some(Extension::Ssse3) => features.ssse3,
            Some(Extension::Sse41) => features.sse41,
            Some(Extension::Sse42) => features.sse42,
            Some(Extension::Popcnt) => features.popcnt,
            Some(Extension::Lzcnt) => features.lzcnt,
            Some(Extension::Movbe) => features.movbe,
            Some(Extension::Avx) => features.avx,
        }
    }

    /// `POPCNT`: the number of set bits in the source.
    ///
    /// Every arithmetic flag is cleared except `ZF`, which says the source
    /// was zero (*Intel SDM* volume 2, `POPCNT`, "Flags Affected").
    pub(super) fn popcnt(&mut self, f: &Fields, size: u8) -> Ex<()> {
        if !self.has_extension(f.insn) {
            return Err(Fault::bare(VEC_UD));
        }
        let value = self.read_arg(f, f.insn.src, size)?;
        self.write_arg(f, f.insn.dst, size, u64::from(value.count_ones()))?;
        for bit in [flags::OF, flags::SF, flags::AF, flags::CF, flags::PF] {
            self.set_flag(bit, false);
        }
        self.set_flag(flags::ZF, value == 0);
        Ok(())
    }

    /// `LZCNT`: the number of zero bits above the most significant set one.
    ///
    /// A part without the feature ignores the `F3` and executes `BSR`
    /// (*Intel SDM* volume 2, `LZCNT`: "on processors that do not support
    /// LZCNT, the instruction byte encoding is executed as BSR"), so the
    /// fallback is [`Exec::bit_scan`] rather than `#UD`. `TZCNT` and `BSF`
    /// are the same arrangement one opcode down.
    ///
    /// `CF` says the source was zero — the result is then the operand width
    /// — and `ZF` that the result is zero, the top bit set. `OF`, `SF`, `AF`
    /// and `PF` are undefined and are left as they were, as `BSF`/`BSR` leave
    /// theirs in this core: deterministic, and the cheapest answer.
    pub(super) fn lzcnt(&mut self, f: &Fields, size: u8) -> Ex<()> {
        if !self.has_extension(f.insn) {
            return self.bit_scan(f, size, true);
        }
        let value = self.read_arg(f, f.insn.src, size)?;
        // The source is masked to the operand size, so the count from 64
        // overstates by the bits above the operand.
        let count = value.leading_zeros() - (64 - u32::from(size) * 8);
        self.write_arg(f, f.insn.dst, size, u64::from(count))?;
        self.set_flag(flags::CF, value == 0);
        self.set_flag(flags::ZF, count == 0);
        Ok(())
    }

    /// `MOVBE`: a load or a store with the operand's bytes reversed.
    ///
    /// 16, 32 or 64 bits by the operand size; a 16-bit load leaves the rest
    /// of the register alone, like any 16-bit write. No flags. The decoder
    /// has already made the register form unassigned, and the check here is
    /// the same rule stated where it is enforced (*Intel SDM* volume 2,
    /// `MOVBE`).
    pub(super) fn movbe(&mut self, f: &Fields, size: u8) -> Ex<()> {
        if !self.has_extension(f.insn) || f.modrm.is_some_and(|m| m.is_register()) {
            return Err(Fault::bare(VEC_UD));
        }
        let value = self.read_arg(f, f.insn.src, size)?;
        let swapped = match size {
            2 => u64::from((value as u16).swap_bytes()),
            4 => u64::from((value as u32).swap_bytes()),
            _ => value.swap_bytes(),
        };
        self.write_arg(f, f.insn.dst, size, swapped)
    }

    /// `CRC32`: accumulate a CRC-32C over the source into the destination.
    ///
    /// A general-purpose instruction, so the only gate is its `CPUID` bit.
    /// The source is a byte (`F0`) or the operand size (`F1`: a word with
    /// `66`, a doubleword, or a quadword with `REX.W`); the destination is
    /// the low thirty-two bits of `Gy`, and the result is written back at
    /// `Gy`'s width — so the `REX.W` forms clear the upper half of the
    /// 64-bit register (*Intel SDM* volume 2, `CRC32`). Flags are untouched.
    pub(super) fn crc32(&mut self, f: &Fields) -> Ex<()> {
        if !self.has_extension(f.insn) {
            return Err(Fault::bare(VEC_UD));
        }
        let bytes = if f.insn.src == Arg::Eb { 1 } else { f.opsize };
        let data = self.read_arg(f, f.insn.src, bytes)?;
        let crc = self.read_arg(f, Arg::Gy, 4)? as u32;
        let out = crc32c(crc, data, bytes);
        let width = if f.rex_w() { 8 } else { 4 };
        self.write_arg(f, Arg::Gy, width, u64::from(out))
    }

    /// Execute one SSE3-or-later instruction on `XMM` registers, its gates
    /// already passed.
    pub(super) fn simd_extension(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        let reg = f.reg_num();
        let register_form = f.modrm.is_some_and(ModRm::is_register);
        match op {
            Op::ADDSUBPS | Op::HADDPS | Op::HSUBPS | Op::ADDSUBPD | Op::HADDPD | Op::HSUBPD => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let dst = self.state.sse.get(reg);
                let (out, flags) = horizontal(op, dst, src, self.state.sse.env());
                // An unmasked exception in any lane leaves the whole
                // destination as it was, which is what the `?` does.
                self.sse_raise(flags)?;
                self.state.sse.set(reg, out);
                Ok(())
            }
            Op::MOVDDUP | Op::MOVSLDUP | Op::MOVSHDUP => {
                // A register source contributes its low double; a memory
                // source of `MOVDDUP` is that double alone.
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                self.state.sse.set(reg, duplicate(op, src));
                Ok(())
            }
            Op::LDDQU => {
                // Memory only: the instruction exists to load data that
                // straddles a cache line, and a register has no such thing.
                if register_form {
                    return Err(Fault::bare(VEC_UD));
                }
                let src = self.sse_read_rm(f, Arg::Wx, false)?;
                self.state.sse.set(reg, src);
                Ok(())
            }
            _ if op.extension() == Some(Extension::Ssse3) => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let dst = self.state.sse.get(reg);
                self.state
                    .sse
                    .set(reg, ssse3(op, dst, src, 16, f.imm as u8));
                Ok(())
            }
            _ if f.insn.extension() == Some(Extension::Sse41) => self.sse41(f),
            Op::PCMPGTQ => {
                let src = self.sse_read_rm(f, Arg::Wx, false)?;
                let dst = self.state.sse.get(reg);
                self.state.sse.set(reg, pcmpgtq(dst, src));
                Ok(())
            }
            Op::PCMPESTRI | Op::PCMPESTRM | Op::PCMPISTRI | Op::PCMPISTRM => self.pcmpstr(f),
            // The table and this match are written together; a row that
            // reaches here unmatched is a decoder bug, and `#UD` is the
            // architecture's answer to an instruction a part does not have.
            _ => Err(Fault::bare(VEC_UD)),
        }
    }

    /// SSE4.1.
    fn sse41(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        let reg = f.reg_num();
        let register_form = f.modrm.is_some_and(ModRm::is_register);
        let imm = f.imm as u8;
        match op {
            // -- Between an `XMM` lane and a general register or memory ----
            Op::PEXTRB | Op::PEXTRW => {
                let v = bytes(self.state.sse.get(reg));
                let value = if op == Op::PEXTRB {
                    u64::from(v[usize::from(imm & 15)])
                } else {
                    let i = 2 * usize::from(imm & 7);
                    u64::from(u16::from_le_bytes([v[i], v[i + 1]]))
                };
                self.write_narrow(f, if op == Op::PEXTRB { 1 } else { 2 }, value)
            }
            Op::PEXTRD => {
                let v = self.state.sse.get(reg);
                if f.rex_w() {
                    self.write_arg(f, Arg::Ey, 8, v[usize::from(imm & 1)])
                } else {
                    let value = u64::from(dwords(v)[usize::from(imm & 3)] as u32);
                    self.write_arg(f, Arg::Ey, 4, value)
                }
            }
            // `REX.W` changes nothing here: the single is four bytes, and a
            // 64-bit destination register is zero-extended by the 32-bit
            // write (*Intel SDM* volume 2, `EXTRACTPS`).
            Op::EXTRACTPS => {
                let v = self.state.sse.get(reg);
                let value = u64::from(dwords(v)[usize::from(imm & 3)] as u32);
                self.write_arg(f, Arg::Ed, 4, value)
            }
            Op::PINSRB => {
                let value = self.read_narrow(f, 1)? as u8;
                let mut v = bytes(self.xmm_src1(f));
                v[usize::from(imm & 15)] = value;
                self.state.sse.set(reg, of_bytes(v));
                Ok(())
            }
            Op::PINSRD => {
                let mut v = self.xmm_src1(f);
                if f.rex_w() {
                    v[usize::from(imm & 1)] = self.read_arg(f, Arg::Ey, 8)?;
                } else {
                    let value = self.read_arg(f, Arg::Ey, 4)? as i32;
                    let mut d = dwords(v);
                    d[usize::from(imm & 3)] = value;
                    v = of_dwords(d);
                }
                self.state.sse.set(reg, v);
                Ok(())
            }
            // The source lane is `imm[7:6]` of a register, or the four bytes
            // of memory, which ignore those bits; it lands in lane
            // `imm[5:4]`, and then `imm[3:0]` zeroes lanes — the inserted one
            // included, if its bit says so (*Intel SDM* volume 2,
            // `INSERTPS`).
            Op::INSERTPS => {
                let src = self.sse_read_rm(f, Arg::Wd, false)?;
                let value = if register_form {
                    dwords(src)[usize::from(imm >> 6)]
                } else {
                    src[0] as i32
                };
                let mut d = dwords(self.xmm_src1(f));
                d[usize::from((imm >> 4) & 3)] = value;
                for (i, lane) in d.iter_mut().enumerate() {
                    if imm & (1 << i) != 0 {
                        *lane = 0;
                    }
                }
                self.state.sse.set(reg, of_dwords(d));
                Ok(())
            }

            // -- Loads -----------------------------------------------------
            // A non-temporal *load* is an ordinary aligned one here, for the
            // reason `66 0F E7` gives in `isa` about the store; memory only,
            // and misalignment is `#GP(0)` (*Intel SDM* volume 2,
            // `MOVNTDQA`).
            Op::MOVNTDQA => {
                if register_form {
                    return Err(Fault::bare(VEC_UD));
                }
                let v = self.sse_read_rm(f, Arg::Wx, true)?;
                self.state.sse.set(reg, v);
                Ok(())
            }

            // -- Flags -----------------------------------------------------
            // `ZF` if `src AND dst` is zero, `CF` if `src AND NOT dst` is,
            // and the other four arithmetic flags cleared (*Intel SDM*
            // volume 2, `PTEST`).
            Op::PTEST => {
                let src = self.sse_read_rm(f, Arg::Wx, false)?;
                let dst = self.state.sse.get(reg);
                self.set_test_flags(&dst, &src, u64::MAX);
                Ok(())
            }

            // -- Floating point --------------------------------------------
            Op::ROUNDPS | Op::ROUNDPD | Op::ROUNDSS | Op::ROUNDSD => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let dst = self.state.sse.get(reg);
                let (out, flags) = round(op, dst, src, imm, self.state.sse.env());
                self.sse_raise(flags)?;
                self.state.sse.set(reg, out);
                Ok(())
            }
            Op::DPPS | Op::DPPD => {
                let src = self.sse_read_rm(f, Arg::Wx, false)?;
                let dst = self.state.sse.get(reg);
                let (out, flags) = dot(op, dst, src, imm, self.state.sse.env());
                self.sse_raise(flags)?;
                self.state.sse.set(reg, out);
                Ok(())
            }

            // -- Everything else is exact lane arithmetic -----------------
            _ => {
                let src = self.sse_read_rm(f, f.insn.src, false)?;
                let dst = self.state.sse.get(reg);
                let mask = self.state.sse.get(0);
                self.state.sse.set(reg, sse41(op, dst, src, mask, imm));
                Ok(())
            }
        }
    }

    /// The flags `PTEST`, `VPTEST`, `VTESTPS` and `VTESTPD` set: `ZF` if
    /// `src AND dst` is zero and `CF` if `src AND NOT dst` is, over every
    /// quadword of the two operands, each first masked by `bits` — all of
    /// them for `PTEST`, the sign bits of the lanes for `VTESTPS`/`PD`. The
    /// other four arithmetic flags are cleared (*Intel SDM* volume 2,
    /// `PTEST` and `VTESTPD/VTESTPS`).
    pub(super) fn set_test_flags(&mut self, dst: &[u64], src: &[u64], bits: u64) {
        let mut and = 0;
        let mut andn = 0;
        for (d, s) in dst.iter().zip(src) {
            and |= d & s & bits;
            andn |= !d & s & bits;
        }
        self.set_flag(flags::ZF, and == 0);
        self.set_flag(flags::CF, andn == 0);
        for bit in [flags::AF, flags::OF, flags::PF, flags::SF] {
            self.set_flag(bit, false);
        }
    }

    /// Read an Intel `Rd/Mb` or `Rd/Mw` operand: the low `bytes` of a whole
    /// general register, or exactly `bytes` of memory.
    fn read_narrow(&mut self, f: &Fields, bytes: u8) -> Ex<u64> {
        let mask = (1u64 << (8 * u32::from(bytes))) - 1;
        if f.modrm.is_some_and(ModRm::is_register) {
            return Ok(self.state.regs.read(f.rm_num(), 4, f.has_rex()) & mask);
        }
        let (sr, off) = self.ea();
        self.read_mem(sr, off, bytes)
    }

    /// Write one: a register gets the value zero-extended through a 32-bit
    /// write — which clears the upper half of a 64-bit register, as the
    /// architecture says it must — and memory gets exactly `bytes`.
    fn write_narrow(&mut self, f: &Fields, bytes: u8, value: u64) -> Ex<()> {
        if f.modrm.is_some_and(ModRm::is_register) {
            self.state
                .regs
                .write(f.rm_num(), 4, f.has_rex(), value & 0xffff_ffff);
            return Ok(());
        }
        let (sr, off) = self.ea();
        self.write_mem(sr, off, bytes, value)
    }

    /// The four SSE4.2 string compares.
    ///
    /// The first operand (`reg`) and the second (`r/m`) are each a string of
    /// sixteen bytes or eight words. The *explicit*-length forms take the
    /// lengths from `EAX` and `EDX` — `RAX` and `RDX` with `REX.W` — as
    /// absolute values saturated to the element count; the *implicit* ones
    /// end each string at its first zero element. [`pcmpstr`] does the
    /// comparison; this reads the operands, writes `ECX` (the `I` forms, a
    /// 32-bit write) or `XMM0` (the `M` forms), and sets the flags.
    ///
    /// The flags are the instruction's own and not an arithmetic result's:
    /// `CF` says the result is non-zero, `ZF` that the second string is
    /// shorter than a register and `SF` that the first is, `OF` is bit 0 of
    /// the result, and `AF` and `PF` are cleared (*Intel SDM* volume 2, each
    /// instruction's "Flags Affected"; the index and mask outputs are §4.1.5,
    /// "Output Selection").
    fn pcmpstr(&mut self, f: &Fields) -> Ex<()> {
        let op = f.insn.op;
        let imm = f.imm as u8;
        let src = self.sse_read_rm(f, Arg::Wx, false)?;
        let dst = self.state.sse.get(f.reg_num());
        let words = imm & 1 != 0;
        let n = if words { 8 } else { 16 };
        let (la, lb) = if matches!(op, Op::PCMPESTRI | Op::PCMPESTRM) {
            let regs = &self.state.regs;
            let length = |v: u64| {
                let magnitude = if f.rex_w() {
                    (v as i64).unsigned_abs()
                } else {
                    u64::from((v as i32).unsigned_abs())
                };
                magnitude.min(n as u64) as usize
            };
            (length(regs.rax), length(regs.rdx))
        } else {
            (implicit_length(dst, words), implicit_length(src, words))
        };
        let result = pcmpstr(dst, src, la, lb, imm);
        if matches!(op, Op::PCMPESTRI | Op::PCMPISTRI) {
            let index = if result == 0 {
                n as u32
            } else if imm & 0x40 != 0 {
                15 - result.leading_zeros()
            } else {
                result.trailing_zeros()
            };
            self.state.regs.set_dword(1, index);
        } else {
            let mask = if imm & 0x40 == 0 {
                [u64::from(result), 0]
            } else {
                expand_mask(result, words)
            };
            self.state.sse.set(0, mask);
        }
        self.set_flag(flags::CF, result != 0);
        self.set_flag(flags::ZF, lb < n);
        self.set_flag(flags::SF, la < n);
        self.set_flag(flags::OF, result & 1 != 0);
        self.set_flag(flags::AF, false);
        self.set_flag(flags::PF, false);
        Ok(())
    }
}

/// `MOVDDUP`, `MOVSLDUP` and `MOVSHDUP` on one 128-bit lane: the low
/// double twice, or the even or the odd singles each twice.
pub(super) fn duplicate(op: Op, src: [u64; 2]) -> [u64; 2] {
    match op {
        Op::MOVDDUP => [src[0], src[0]],
        _ => {
            let odd = op == Op::MOVSHDUP;
            let pick = |q: u64| {
                let lane = if odd { q >> 32 } else { q & 0xffff_ffff };
                lane | (lane << 32)
            };
            [pick(src[0]), pick(src[1])]
        }
    }
}

/// `PCMPGTQ` on one 128-bit lane: signed quadwords, greater than.
pub(super) fn pcmpgtq(dst: [u64; 2], src: [u64; 2]) -> [u64; 2] {
    let gt = |i: usize| {
        if (dst[i] as i64) > (src[i] as i64) {
            u64::MAX
        } else {
            0
        }
    };
    [gt(0), gt(1)]
}

/// The CRC-32C (Castagnoli) table, one entry per byte value, for the
/// bit-reflected polynomial `0x82F6_3B78`.
///
/// That is `0x1EDC_6F41`, the polynomial *Intel SDM* volume 2 names for
/// `CRC32`, with its bits reversed — the instruction's pseudo-code reflects
/// the operands, divides, and reflects the remainder back, which is the
/// least-significant-bit-first form a table indexes directly. Computed here
/// from the polynomial rather than written out.
const CRC32C: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0x82f6_3b78
            } else {
                c >> 1
            };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
};

/// Fold the low `bytes` of `data`, least significant first, into `crc`.
///
/// No initial or final inversion: the instruction leaves both to software,
/// which is why the standard check value takes a `!0` in and a `!` out.
pub(super) fn crc32c(mut crc: u32, data: u64, bytes: u8) -> u32 {
    for i in 0..u32::from(bytes) {
        let byte = (data >> (8 * i)) as u8;
        crc = (crc >> 8) ^ CRC32C[usize::from((crc as u8) ^ byte)];
    }
    crc
}

/// The number of elements before the first zero one: an implicit length.
fn implicit_length(v: [u64; 2], words: bool) -> usize {
    if words {
        let w = self::words(v);
        w.iter().position(|&x| x == 0).unwrap_or(8)
    } else {
        bytes(v).iter().position(|&x| x == 0).unwrap_or(16)
    }
}

/// A result bit per element, widened to an all-ones element per set bit.
fn expand_mask(result: u16, words: bool) -> [u64; 2] {
    if words {
        of_words(core::array::from_fn(|i| -i16::from((result >> i) & 1 != 0)))
    } else {
        of_bytes(core::array::from_fn(|i| {
            0u8.wrapping_sub(u8::from((result >> i) & 1 != 0))
        }))
    }
}

/// The comparison behind the SSE4.2 string instructions: `IntRes2`, one bit
/// per element of the second operand.
///
/// `a` is the first operand (the register) with `la` valid elements and `b`
/// the second (register or memory) with `lb`. The immediate, *Intel SDM*
/// volume 2 §4.1 ("Imm8 Control Byte Operation for PCMPESTRI / PCMPESTRM /
/// PCMPISTRI / PCMPISTRM"):
///
/// - `imm[1:0]`, the **element**: unsigned bytes, unsigned words, signed
///   bytes, signed words.
/// - `imm[3:2]`, the **aggregation** (§4.1.3, "Aggregation Operation"),
///   producing `IntRes1`:
///   - *equal any* — bit `j` if `b[j]` equals any element of `a`;
///   - *ranges* — bit `j` if `a[i] <= b[j] <= a[i+1]` for some even `i`;
///   - *equal each* — bit `i` if `a[i] == b[i]`;
///   - *equal ordered* — bit `j` if `a` occurs in `b` starting at `j`, which
///     is a substring search: the needle is the first operand.
/// - `imm[5:4]`, the **polarity** (§4.1.4, "Polarity"), producing `IntRes2`: as is,
///   negated, as is again ("masked +"), or negated only where `b` is valid
///   ("masked −").
///
/// What an invalid element — past the end of its string — does is the
/// override table of §4.1.6 ("Valid/Invalid Override of Comparisons"), and
/// it is what makes the four aggregations
/// useful on strings rather than registers: a comparison involving one is
/// forced *false* for equal any and ranges; for equal each it is *true* when
/// both are invalid (two strings that have both ended agree) and false when
/// one is; for equal ordered it is *true* whenever the needle's element is
/// invalid (the needle has ended, so it matched) and false when only the
/// haystack's is. Equal ordered also compares only the needle elements that
/// still fall inside the register (`i < n - j`), so a needle that runs off
/// the end of the haystack register is a *partial* match at the tail — the
/// architecture's way of letting a loop resume at the next block.
pub(super) fn pcmpstr(a: [u64; 2], b: [u64; 2], la: usize, lb: usize, imm: u8) -> u16 {
    let words = imm & 1 != 0;
    let signed = imm & 2 != 0;
    let n = if words { 8 } else { 16 };
    let element = |v: [u64; 2], i: usize| -> i32 {
        match (words, signed) {
            (true, true) => i32::from(self::words(v)[i]),
            (true, false) => i32::from(self::words(v)[i] as u16),
            (false, true) => i32::from(bytes(v)[i] as i8),
            (false, false) => i32::from(bytes(v)[i]),
        }
    };
    let ea: [i32; 16] = core::array::from_fn(|i| if i < n { element(a, i) } else { 0 });
    let eb: [i32; 16] = core::array::from_fn(|i| if i < n { element(b, i) } else { 0 });
    let mut res1 = 0u16;
    for j in 0..n {
        let hit = match (imm >> 2) & 3 {
            // Equal any.
            0 => j < lb && (0..la).any(|i| ea[i] == eb[j]),
            // Ranges: the pairs `a[0..1]`, `a[2..3]`, …, each wholly valid.
            1 => j < lb && (0..la / 2).any(|p| ea[2 * p] <= eb[j] && eb[j] <= ea[2 * p + 1]),
            // Equal each.
            2 => match (j < la, j < lb) {
                (true, true) => ea[j] == eb[j],
                (false, false) => true,
                _ => false,
            },
            // Equal ordered.
            _ => (0..n - j).all(|i| {
                if i >= la {
                    true
                } else if j + i >= lb {
                    false
                } else {
                    ea[i] == eb[j + i]
                }
            }),
        };
        if hit {
            res1 |= 1 << j;
        }
    }
    let all = if words { 0xff } else { 0xffff };
    let valid_b = ((1u32 << lb) - 1) as u16;
    match (imm >> 4) & 3 {
        1 => !res1 & all,
        3 => res1 ^ valid_b,
        _ => res1,
    }
}

/// The rounding direction a `ROUND*` immediate selects.
///
/// `imm[2]` set means `MXCSR.RC`; clear, `imm[1:0]` names one directly —
/// nearest, down, up, toward zero, the same order as `RC` itself. `imm[3]`
/// is the precision-exception suppression bit and is the caller's business.
/// *Intel SDM* volume 2, `ROUNDPD`: the immediate's layout and the rounding
/// modes table on that page, which is the `RC` encoding of volume 1 §4.8.4.
fn round_mode(imm: u8, mxcsr_round: Round) -> Round {
    if imm & 4 != 0 {
        return mxcsr_round;
    }
    match imm & 3 {
        0 => Round::TiesEven,
        1 => Round::TowardNegative,
        2 => Round::TowardPositive,
        _ => Round::TowardZero,
    }
}

/// `ROUNDPS`, `ROUNDPD`, `ROUNDSS` and `ROUNDSD`: each lane to an integral
/// value in its own format.
///
/// The exceptions are narrower than the arithmetic's: invalid only for a
/// signalling NaN, precision only when `imm[3]` is clear, and **no**
/// denormal-operand exception — but `MXCSR.DAZ` still turns a denormal source
/// into a zero first. The scalar forms replace the low lane and keep the rest
/// of the destination.
pub(super) fn round(op: Op, dst: [u64; 2], src: [u64; 2], imm: u8, env: Env) -> ([u64; 2], Flags) {
    let env = Env {
        round: round_mode(imm, env.round),
        subnormal_inputs: if env.subnormal_inputs.flushes() {
            InputSubnormal::Flushed
        } else {
            InputSubnormal::Exact
        },
        ..env
    };
    let inexact = imm & 8 == 0;
    let mut flags = Flags::NONE;
    match op {
        Op::ROUNDPS | Op::ROUNDSS => {
            let s = singles(src);
            let mut out = singles(dst);
            let lanes = if op == Op::ROUNDSS { 1 } else { 4 };
            for i in 0..lanes {
                let (v, fl) = binary::round_to_integral::<B32>(s[i], env, inexact);
                flags |= fl;
                out[i] = v;
            }
            (of_singles(out), flags)
        }
        _ => {
            let mut out = dst;
            let lanes = if op == Op::ROUNDSD { 1 } else { 2 };
            for i in 0..lanes {
                let (v, fl) = binary::round_to_integral::<B64>(src[i], env, inexact);
                flags |= fl;
                out[i] = v;
            }
            (out, flags)
        }
    }
}

/// `DPPS` and `DPPD`: a dot product under two masks.
///
/// The high nibble of the immediate (`imm[5:4]` for doubles) picks which
/// lane products take part — an unselected one is `+0.0` and its multiply is
/// never performed, so it raises nothing — and the low nibble picks which
/// result lanes receive the sum; the rest are `+0.0`. The sum is formed in
/// the order the manual gives, `(p0 + p1) + (p2 + p3)`, each step rounded
/// under `MXCSR`, and the flags are every step's together (*Intel SDM*
/// volume 2, `DPPS`, the `DP_primitive` pseudo-code).
pub(super) fn dot(op: Op, dst: [u64; 2], src: [u64; 2], imm: u8, env: Env) -> ([u64; 2], Flags) {
    let mut flags = Flags::NONE;
    let mut mul = |a: u64, b: u64, on: bool, single: bool| {
        if !on {
            return 0;
        }
        let (v, fl) = if single {
            binary::mul::<B32>(a, b, env)
        } else {
            binary::mul::<B64>(a, b, env)
        };
        flags |= fl;
        v
    };
    if op == Op::DPPS {
        let (d, s) = (singles(dst), singles(src));
        let mut p = [0u64; 4];
        for i in 0..4 {
            p[i] = mul(d[i], s[i], imm & (0x10 << i) != 0, true);
        }
        let (a, fa) = binary::add::<B32>(p[0], p[1], env);
        let (b, fb) = binary::add::<B32>(p[2], p[3], env);
        let (sum, fs) = binary::add::<B32>(a, b, env);
        flags |= fa | fb | fs;
        let mut out = [0u64; 4];
        for (i, lane) in out.iter_mut().enumerate() {
            if imm & (1 << i) != 0 {
                *lane = sum;
            }
        }
        (of_singles(out), flags)
    } else {
        let p0 = mul(dst[0], src[0], imm & 0x10 != 0, false);
        let p1 = mul(dst[1], src[1], imm & 0x20 != 0, false);
        let (sum, fs) = binary::add::<B64>(p0, p1, env);
        flags |= fs;
        let out = [
            if imm & 1 != 0 { sum } else { 0 },
            if imm & 2 != 0 { sum } else { 0 },
        ];
        (out, flags)
    }
}

/// Widen the low lanes of `src` by sign or zero extension: `from` bytes each
/// into `to` bytes each, as many as fit in sixteen bytes.
fn widen(src: u64, from: u32, to: u32, signed: bool) -> [u64; 2] {
    let mut out = [0u8; 16];
    let lanes = 16 / to;
    for i in 0..lanes {
        let raw = (src >> (8 * from * i)) & ((1u64 << (8 * from)) - 1);
        let value = if signed {
            let shift = 64 - 8 * from;
            (((raw << shift) as i64) >> shift) as u64
        } else {
            raw
        };
        let at = (i * to) as usize;
        out[at..at + to as usize].copy_from_slice(&value.to_le_bytes()[..to as usize]);
    }
    of_bytes(out)
}

/// The rest of SSE4.1: exact integer arithmetic and lane selection, no
/// `MXCSR` and no exception. `mask` is `XMM0`, which only the variable
/// blends read.
#[allow(clippy::too_many_lines)]
pub(super) fn sse41(op: Op, dst: [u64; 2], src: [u64; 2], mask: [u64; 2], imm: u8) -> [u64; 2] {
    let (d, s) = (bytes(dst), bytes(src));
    let (dw, sw) = (words(dst), words(src));
    let (dd, sd) = (dwords(dst), dwords(src));
    match op {
        // -- Blends: the source where the selector says, else the
        //    destination ---------------------------------------------------
        Op::PBLENDVB => {
            let m = bytes(mask);
            of_bytes(core::array::from_fn(|i| {
                if m[i] & 0x80 != 0 { s[i] } else { d[i] }
            }))
        }
        Op::BLENDVPS => {
            let m = dwords(mask);
            of_dwords(core::array::from_fn(
                |i| if m[i] < 0 { sd[i] } else { dd[i] },
            ))
        }
        Op::BLENDVPD => {
            core::array::from_fn(|i| if (mask[i] as i64) < 0 { src[i] } else { dst[i] })
        }
        Op::BLENDPS => of_dwords(core::array::from_fn(|i| {
            if imm & (1 << i) != 0 { sd[i] } else { dd[i] }
        })),
        Op::BLENDPD => core::array::from_fn(|i| if imm & (1 << i) != 0 { src[i] } else { dst[i] }),
        Op::PBLENDW => of_words(core::array::from_fn(|i| {
            if imm & (1 << i) != 0 { sw[i] } else { dw[i] }
        })),

        // -- Minimum and maximum, at the widths SSE2 left out ------------
        Op::PMINSB | Op::PMAXSB => of_bytes(core::array::from_fn(|i| {
            let (a, b) = (d[i] as i8, s[i] as i8);
            (if op == Op::PMINSB { a.min(b) } else { a.max(b) }) as u8
        })),
        Op::PMINUW | Op::PMAXUW => of_words(core::array::from_fn(|i| {
            let (a, b) = (dw[i] as u16, sw[i] as u16);
            (if op == Op::PMINUW { a.min(b) } else { a.max(b) }) as i16
        })),
        Op::PMINSD | Op::PMAXSD => of_dwords(core::array::from_fn(|i| {
            if op == Op::PMINSD {
                dd[i].min(sd[i])
            } else {
                dd[i].max(sd[i])
            }
        })),
        Op::PMINUD | Op::PMAXUD => of_dwords(core::array::from_fn(|i| {
            let (a, b) = (dd[i] as u32, sd[i] as u32);
            (if op == Op::PMINUD { a.min(b) } else { a.max(b) }) as i32
        })),

        // -- Multiplies and compares ---------------------------------------
        // The even doublewords, signed, into full quadwords.
        Op::PMULDQ => [
            (i64::from(dd[0]) * i64::from(sd[0])) as u64,
            (i64::from(dd[2]) * i64::from(sd[2])) as u64,
        ],
        Op::PMULLD => of_dwords(core::array::from_fn(|i| dd[i].wrapping_mul(sd[i]))),
        Op::PCMPEQQ => core::array::from_fn(|i| if dst[i] == src[i] { u64::MAX } else { 0 }),
        // Signed doublewords to unsigned words, destination low.
        Op::PACKUSDW => of_words(core::array::from_fn(|i| {
            let v = if i < 4 { dd[i] } else { sd[i - 4] };
            v.clamp(0, 0xffff) as u16 as i16
        })),

        // -- The two that reduce ----------------------------------------
        // The smallest unsigned word in bits 15:0, its index in bits 18:16
        // — the lowest index on a tie — and zero above (*Intel SDM*
        // volume 2, `PHMINPOSUW`).
        Op::PHMINPOSUW => {
            let mut best = 0usize;
            for i in 1..8 {
                if (sw[i] as u16) < (sw[best] as u16) {
                    best = i;
                }
            }
            [u64::from(sw[best] as u16) | ((best as u64) << 16), 0]
        }
        // Eight sums of four absolute byte differences. The destination
        // block starts at byte `4 * imm[2]` and slides one byte per result;
        // the source block is the fixed four bytes at `4 * imm[1:0]`
        // (*Intel SDM* volume 2, `MPSADBW`).
        Op::MPSADBW => {
            let a = 4 * usize::from((imm >> 2) & 1);
            let b = 4 * usize::from(imm & 3);
            of_words(core::array::from_fn(|i| {
                (0..4)
                    .map(|j| u16::from(d[a + i + j].abs_diff(s[b + j])))
                    .sum::<u16>() as i16
            }))
        }

        // -- The widening moves --------------------------------------------
        Op::PMOVSXBW => widen(src[0], 1, 2, true),
        Op::PMOVSXBD => widen(src[0], 1, 4, true),
        Op::PMOVSXBQ => widen(src[0], 1, 8, true),
        Op::PMOVSXWD => widen(src[0], 2, 4, true),
        Op::PMOVSXWQ => widen(src[0], 2, 8, true),
        Op::PMOVSXDQ => widen(src[0], 4, 8, true),
        Op::PMOVZXBW => widen(src[0], 1, 2, false),
        Op::PMOVZXBD => widen(src[0], 1, 4, false),
        Op::PMOVZXBQ => widen(src[0], 1, 8, false),
        Op::PMOVZXWD => widen(src[0], 2, 4, false),
        Op::PMOVZXWQ => widen(src[0], 2, 8, false),
        Op::PMOVZXDQ => widen(src[0], 4, 8, false),
        _ => dst,
    }
}

/// The four singles of a register, low lane first.
fn singles(v: [u64; 2]) -> [u64; 4] {
    [
        v[0] & 0xffff_ffff,
        v[0] >> 32,
        v[1] & 0xffff_ffff,
        v[1] >> 32,
    ]
}

/// Four singles back into a register.
fn of_singles(l: [u64; 4]) -> [u64; 2] {
    [
        (l[0] & 0xffff_ffff) | (l[1] << 32),
        (l[2] & 0xffff_ffff) | (l[3] << 32),
    ]
}

/// SSE3's horizontal and alternating arithmetic.
///
/// Each result lane is one `float` operation rounded under `MXCSR`, and the
/// flags of every lane are gathered before any is written: the *Intel SDM*
/// volume 2 pages for `HADDPS` and its siblings give the operation lane by
/// lane, and their exception lists are the union of the lanes'.
///
/// - `ADDSUB`: even lanes subtract the source from the destination, odd lanes
///   add.
/// - `HADD`/`HSUB`: the low half of the result is the destination's adjacent
///   pairs, the high half the source's — `d0 + d1`, `d2 + d3`, `s0 + s1`,
///   `s2 + s3` — and the subtraction is always the even lane minus the odd.
pub(super) fn horizontal(op: Op, dst: [u64; 2], src: [u64; 2], env: Env) -> ([u64; 2], Flags) {
    let mut flags = Flags::NONE;
    match op {
        Op::ADDSUBPS | Op::HADDPS | Op::HSUBPS => {
            let (d, s) = (singles(dst), singles(src));
            let mut out = [0u64; 4];
            for (i, lane) in out.iter_mut().enumerate() {
                let (a, b, add) = match op {
                    Op::ADDSUBPS => (d[i], s[i], i % 2 == 1),
                    _ => {
                        // Lanes 0 and 1 pair up the destination, 2 and 3
                        // the source.
                        let from = if i < 2 { d } else { s };
                        let k = 2 * (i % 2);
                        (from[k], from[k + 1], op == Op::HADDPS)
                    }
                };
                let (v, fl) = if add {
                    binary::add::<B32>(a, b, env)
                } else {
                    binary::sub::<B32>(a, b, env)
                };
                flags |= fl;
                *lane = v;
            }
            (of_singles(out), flags)
        }
        _ => {
            let (pairs, add) = match op {
                Op::ADDSUBPD => ([(dst[0], src[0]), (dst[1], src[1])], [false, true]),
                Op::HADDPD => ([(dst[0], dst[1]), (src[0], src[1])], [true, true]),
                _ => ([(dst[0], dst[1]), (src[0], src[1])], [false, false]),
            };
            let mut out = [0u64; 2];
            for i in 0..2 {
                let (a, b) = pairs[i];
                let (v, fl) = if add[i] {
                    binary::add::<B64>(a, b, env)
                } else {
                    binary::sub::<B64>(a, b, env)
                };
                flags |= fl;
                out[i] = v;
            }
            (out, flags)
        }
    }
}

/// A register's bytes, low first.
fn bytes(v: [u64; 2]) -> [u8; 16] {
    let mut out = [0u8; 16];
    out[..8].copy_from_slice(&v[0].to_le_bytes());
    out[8..].copy_from_slice(&v[1].to_le_bytes());
    out
}

/// Sixteen bytes back into a register.
fn of_bytes(b: [u8; 16]) -> [u64; 2] {
    let mut lo = [0u8; 8];
    let mut hi = [0u8; 8];
    lo.copy_from_slice(&b[..8]);
    hi.copy_from_slice(&b[8..]);
    [u64::from_le_bytes(lo), u64::from_le_bytes(hi)]
}

/// A register's words as signed values, low first.
fn words(v: [u64; 2]) -> [i16; 8] {
    let b = bytes(v);
    core::array::from_fn(|i| i16::from_le_bytes([b[2 * i], b[2 * i + 1]]))
}

/// Eight words back into a register.
fn of_words(w: [i16; 8]) -> [u64; 2] {
    let mut b = [0u8; 16];
    for (i, w) in w.iter().enumerate() {
        b[2 * i..2 * i + 2].copy_from_slice(&w.to_le_bytes());
    }
    of_bytes(b)
}

/// A register's doublewords as signed values, low first.
fn dwords(v: [u64; 2]) -> [i32; 4] {
    [
        v[0] as i32,
        (v[0] >> 32) as i32,
        v[1] as i32,
        (v[1] >> 32) as i32,
    ]
}

/// Four doublewords back into a register.
fn of_dwords(d: [i32; 4]) -> [u64; 2] {
    [
        u64::from(d[0] as u32) | (u64::from(d[1] as u32) << 32),
        u64::from(d[2] as u32) | (u64::from(d[3] as u32) << 32),
    ]
}

/// Saturate to a signed word.
fn sat16(v: i32) -> i16 {
    v.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

/// One SSSE3 operation on `n` bytes of operand: 8 for the `MM` forms, 16
/// for the `XMM` ones. The result's bytes past `n` are zero.
///
/// Mostly the width does not matter, because the operation is per lane; it
/// matters to three families, which is why it is a parameter rather than the
/// `MM` forms being the low half of the `XMM` ones:
///
/// - **`PSHUFB`** indexes with as many bits as there are bytes — three for
///   an `MM` register, four for an `XMM` one — and a set bit 7 writes zero.
/// - **`PALIGNR`** shifts the *concatenation* `destination:source`, sixteen
///   bytes for `MM` operands and thirty-two for `XMM` ones, so the source's
///   bytes are not where a 128-bit shift would put them; a shift of the whole
///   concatenation or more yields zero.
/// - **The horizontal adds and subtracts** put the destination's pairs in
///   the low half of the result and the source's in the high half, and the
///   halves are half of `n`.
///
/// Every subtraction is the even lane minus the odd one above it, and
/// `PMADDUBSW` treats the *destination's* bytes as unsigned and the source's
/// as signed. `PMULHRSW` is `((a * b) >> 14) + 1`, then bits 16:1: a product
/// scaled by 2^-15 and rounded to nearest (*Intel SDM* volume 2, each
/// instruction's Operation section).
#[allow(clippy::too_many_lines)]
pub(super) fn ssse3(op: Op, dst: [u64; 2], src: [u64; 2], n: usize, imm: u8) -> [u64; 2] {
    let (d, s) = (bytes(dst), bytes(src));
    let (dw, sw) = (words(dst), words(src));
    let (dd, sd) = (dwords(dst), dwords(src));
    let nw = n / 2;
    let nd = n / 4;
    match op {
        Op::PSHUFB => {
            let mut out = [0u8; 16];
            for i in 0..n {
                let c = s[i];
                out[i] = if c & 0x80 != 0 {
                    0
                } else {
                    d[usize::from(c) & (n - 1)]
                };
            }
            of_bytes(out)
        }
        Op::PALIGNR => {
            let mut cat = [0u8; 32];
            cat[..n].copy_from_slice(&s[..n]);
            cat[n..2 * n].copy_from_slice(&d[..n]);
            let shift = usize::from(imm);
            let mut out = [0u8; 16];
            for (i, b) in out.iter_mut().take(n).enumerate() {
                *b = if i + shift < 2 * n { cat[i + shift] } else { 0 };
            }
            of_bytes(out)
        }
        Op::PSIGNB | Op::PABSB => {
            let mut out = [0u8; 16];
            for i in 0..n {
                let sign = s[i] as i8;
                out[i] = if op == Op::PABSB {
                    sign.unsigned_abs()
                } else {
                    match sign.signum() {
                        -1 => (d[i] as i8).wrapping_neg() as u8,
                        0 => 0,
                        _ => d[i],
                    }
                };
            }
            of_bytes(out)
        }
        Op::PSIGNW | Op::PABSW | Op::PMULHRSW => {
            let mut out = [0i16; 8];
            for i in 0..nw {
                out[i] = match op {
                    Op::PSIGNW => match sw[i].signum() {
                        -1 => dw[i].wrapping_neg(),
                        0 => 0,
                        _ => dw[i],
                    },
                    Op::PABSW => sw[i].unsigned_abs() as i16,
                    _ => {
                        let t = ((i32::from(dw[i]) * i32::from(sw[i])) >> 14) + 1;
                        (t >> 1) as i16
                    }
                };
            }
            of_words(out)
        }
        Op::PSIGND | Op::PABSD => {
            let mut out = [0i32; 4];
            for i in 0..nd {
                out[i] = if op == Op::PABSD {
                    sd[i].unsigned_abs() as i32
                } else {
                    match sd[i].signum() {
                        -1 => dd[i].wrapping_neg(),
                        0 => 0,
                        _ => dd[i],
                    }
                };
            }
            of_dwords(out)
        }
        Op::PMADDUBSW => {
            let mut out = [0i16; 8];
            for (i, w) in out.iter_mut().take(nw).enumerate() {
                let p = |k: usize| i32::from(d[k]) * i32::from(s[k] as i8);
                *w = sat16(p(2 * i) + p(2 * i + 1));
            }
            of_words(out)
        }
        Op::PHADDW | Op::PHADDSW | Op::PHSUBW | Op::PHSUBSW => {
            let mut out = [0i16; 8];
            let half = nw / 2;
            for (i, w) in out.iter_mut().take(nw).enumerate() {
                let from = if i < half { dw } else { sw };
                let k = 2 * (i % half);
                let (a, b) = (i32::from(from[k]), i32::from(from[k + 1]));
                *w = match op {
                    Op::PHADDW => (a + b) as i16,
                    Op::PHADDSW => sat16(a + b),
                    Op::PHSUBW => (a - b) as i16,
                    _ => sat16(a - b),
                };
            }
            of_words(out)
        }
        Op::PHADDD | Op::PHSUBD => {
            let mut out = [0i32; 4];
            let half = nd / 2;
            for (i, v) in out.iter_mut().take(nd).enumerate() {
                let from = if i < half { dd } else { sd };
                let k = 2 * (i % half);
                *v = if op == Op::PHADDD {
                    from[k].wrapping_add(from[k + 1])
                } else {
                    from[k].wrapping_sub(from[k + 1])
                };
            }
            of_dwords(out)
        }
        _ => dst,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A handful of exact single-precision constants.
    const ONE: u64 = 0x3f80_0000;
    const TWO: u64 = 0x4000_0000;
    const THREE: u64 = 0x4040_0000;
    const FOUR: u64 = 0x4080_0000;
    const FIVE: u64 = 0x40a0_0000;
    const SEVEN: u64 = 0x40e0_0000;
    const MINUS_ONE: u64 = 0xbf80_0000;

    #[test]
    fn haddps_pairs_the_destination_then_the_source() {
        // d = [1, 2, 3, 4], s = [1, 4, 3, 4]: [3, 7, 5, 7].
        let d = of_singles([ONE, TWO, THREE, FOUR]);
        let s = of_singles([ONE, FOUR, THREE, FOUR]);
        let (out, flags) = horizontal(Op::HADDPS, d, s, Env::X86_SSE);
        assert_eq!(out, of_singles([THREE, SEVEN, FIVE, SEVEN]));
        assert_eq!(flags, Flags::NONE);
        // HSUBPS is the even lane minus the odd: [-1, -1, -3, -1].
        let (out, _) = horizontal(Op::HSUBPS, d, s, Env::X86_SSE);
        assert_eq!(singles(out), [MINUS_ONE, MINUS_ONE, 0xc040_0000, MINUS_ONE]);
    }

    #[test]
    fn addsub_subtracts_the_even_lanes_and_adds_the_odd() {
        let d = of_singles([FOUR, FOUR, FOUR, FOUR]);
        let s = of_singles([ONE, ONE, ONE, ONE]);
        let (out, _) = horizontal(Op::ADDSUBPS, d, s, Env::X86_SSE);
        assert_eq!(out, of_singles([THREE, FIVE, THREE, FIVE]));
        // The doubles: 4 - 1 and 4 + 1.
        let d = [0x4010_0000_0000_0000, 0x4010_0000_0000_0000];
        let s = [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000];
        let (out, _) = horizontal(Op::ADDSUBPD, d, s, Env::X86_SSE);
        assert_eq!(out, [0x4008_0000_0000_0000, 0x4014_0000_0000_0000]);
        // HADDPD: 4 + 4 in the low lane, 1 + 1 in the high.
        let (out, _) = horizontal(Op::HADDPD, d, s, Env::X86_SSE);
        assert_eq!(out, [0x4020_0000_0000_0000, 0x4000_0000_0000_0000]);
        let (out, _) = horizontal(Op::HSUBPD, d, s, Env::X86_SSE);
        assert_eq!(out, [0, 0]);
    }

    #[test]
    fn pshufb_indexes_with_four_bits_or_three_and_zeroes_on_bit_seven() {
        let table = [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908];
        // Reverse the bytes, but zero byte 1 (bit 7 of its control) and
        // index 0x1f in byte 15, which an XMM register reads as 0x0f.
        let ctl = of_bytes([
            0x0f, 0x80, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
            0x01, 0x1f,
        ]);
        let out = bytes(ssse3(Op::PSHUFB, table, ctl, 16, 0));
        assert_eq!(out, [15, 0, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 15]);
        // On an MM register the index is three bits: 0x0f reads byte 7.
        let out = bytes(ssse3(Op::PSHUFB, table, ctl, 8, 0));
        assert_eq!(&out[..8], &[7, 0, 5, 4, 3, 2, 1, 0]);
        assert_eq!(&out[8..], &[0; 8]);
    }

    #[test]
    fn palignr_shifts_the_concatenation_with_the_source_low() {
        let d = [0x1716_1514_1312_1110, 0x1f1e_1d1c_1b1a_1918];
        let s = [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908];
        // Shift 4: source bytes 4..15, then destination bytes 0..3.
        let out = bytes(ssse3(Op::PALIGNR, d, s, 16, 4));
        let want: [u8; 16] = core::array::from_fn(|i| {
            let k = i + 4;
            if k < 16 {
                k as u8
            } else {
                0x10 + (k - 16) as u8
            }
        });
        assert_eq!(out, want);
        // 16 is the destination alone; 32 and over is zero.
        assert_eq!(ssse3(Op::PALIGNR, d, s, 16, 16), d);
        assert_eq!(ssse3(Op::PALIGNR, d, s, 16, 24)[1], 0);
        assert_eq!(ssse3(Op::PALIGNR, d, s, 16, 32), [0, 0]);
        // MM: the concatenation is sixteen bytes; shift 3.
        let out = ssse3(Op::PALIGNR, d, s, 8, 3)[0];
        assert_eq!(out, 0x1211_1007_0605_0403);
    }

    #[test]
    fn the_horizontal_word_operations_pair_destination_then_source() {
        let d = of_words([1, 2, 3, 4, 0x7fff, 1, -0x8000, 1]);
        let s = of_words([10, 20, 30, 40, 50, 60, 70, 80]);
        assert_eq!(
            words(ssse3(Op::PHADDW, d, s, 16, 0)),
            [3, 7, -0x8000, -0x7fff, 30, 70, 110, 150]
        );
        assert_eq!(
            words(ssse3(Op::PHADDSW, d, s, 16, 0)),
            [3, 7, 0x7fff, -0x7fff, 30, 70, 110, 150]
        );
        assert_eq!(
            words(ssse3(Op::PHSUBSW, d, s, 16, 0)),
            [-1, -1, 0x7ffe, -0x8000, -10, -10, -10, -10]
        );
        // On MM operands: two pairs from each, d = [1, 2, 3, 4].
        assert_eq!(
            words(ssse3(Op::PHSUBW, d, s, 8, 0)),
            [-1, -1, -10, -10, 0, 0, 0, 0]
        );
        let d = of_dwords([1, 2, 3, 4]);
        let s = of_dwords([5, 7, 11, 13]);
        assert_eq!(dwords(ssse3(Op::PHADDD, d, s, 16, 0)), [3, 7, 12, 24]);
        assert_eq!(dwords(ssse3(Op::PHSUBD, d, s, 8, 0)), [-1, -2, 0, 0]);
    }

    #[test]
    fn pmaddubsw_is_unsigned_times_signed_with_saturation() {
        // 255 * 127 + 255 * 127 = 64770, which saturates to 32767; and
        // 200 * -1 + 3 * 2 = -194.
        let mut d = [0u8; 16];
        let mut s = [0u8; 16];
        d[..4].copy_from_slice(&[255, 255, 200, 3]);
        s[..4].copy_from_slice(&[127, 127, 0xff, 2]);
        let out = words(ssse3(Op::PMADDUBSW, of_bytes(d), of_bytes(s), 16, 0));
        assert_eq!(&out[..2], &[0x7fff, -194]);
    }

    #[test]
    fn pmulhrsw_rounds_the_scaled_product() {
        // 0x4000 * 0x4000 = 2^28; >> 14 = 2^14; + 1; >> 1 = 0x2000.
        // -0x8000 * -0x8000 = 2^30 comes out as 0x8000 — the one case that
        // overflows a word, and it wraps to -0x8000.
        // 3 * 0x4000 = 0xc000; >> 14 = 3; + 1 = 4; >> 1 = 2. And -3 gives
        // -0xc000 >> 14 = -3; + 1 = -2; >> 1 = -1.
        let d = of_words([0x4000, -0x8000, 3, -3, 0, 0, 0, 0]);
        let s = of_words([0x4000, -0x8000, 0x4000, 0x4000, 0, 0, 0, 0]);
        assert_eq!(
            words(ssse3(Op::PMULHRSW, d, s, 16, 0)),
            [0x2000, -0x8000, 2, -1, 0, 0, 0, 0]
        );
    }

    #[test]
    fn psign_and_pabs_follow_the_sign_of_the_source() {
        let d = of_bytes([5; 16]);
        let mut s = [0u8; 16];
        s[..3].copy_from_slice(&[0xff, 0, 1]);
        let out = bytes(ssse3(Op::PSIGNB, d, of_bytes(s), 16, 0));
        assert_eq!(&out[..3], &[0xfb, 0, 5]);
        // -128 has no positive counterpart: its absolute value is 0x80,
        // read unsigned.
        let mut s = [0u8; 16];
        s[..3].copy_from_slice(&[0x80, 0xff, 1]);
        assert_eq!(
            &bytes(ssse3(Op::PABSB, d, of_bytes(s), 16, 0))[..3],
            &[0x80, 1, 1]
        );
        let s = of_dwords([i32::MIN, -7, 7, 0]);
        assert_eq!(
            ssse3(Op::PABSD, d, s, 16, 0),
            of_dwords([i32::MIN, 7, 7, 0])
        );
        assert_eq!(
            dwords(ssse3(Op::PSIGND, of_dwords([9, 9, 9, 9]), s, 16, 0)),
            [-9, -9, 9, 0]
        );
    }

    #[test]
    fn the_widening_moves_extend_exactly_the_lanes_they_read() {
        let src = 0x8877_6655_4433_2211;
        // Bytes 0x11, 0x22, ... widened to words: positive, so the same
        // either way, until 0x88, which is negative.
        assert_eq!(
            words(widen(src, 1, 2, true)),
            [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, -0x78]
        );
        assert_eq!(words(widen(src, 1, 2, false))[7], 0x88);
        // Two words of 0xfffe, 0x8001 sign-extended to quadwords.
        assert_eq!(
            widen(0x8001_fffe, 2, 8, true),
            [0xffff_ffff_ffff_fffe, 0xffff_ffff_ffff_8001]
        );
        assert_eq!(widen(0x8001_fffe, 2, 8, false), [0xfffe, 0x8001]);
        // Two bytes to quadwords reads only two bytes.
        assert_eq!(widen(0xffff_ff80, 1, 8, true), [u64::MAX - 0x7f, u64::MAX]);
        assert_eq!(
            widen(0x8000_0000_7fff_ffff, 4, 8, true),
            [0x7fff_ffff, 0xffff_ffff_8000_0000]
        );
    }

    #[test]
    fn the_integer_gaps_sse2_left() {
        let none = [0, 0];
        // PACKUSDW saturates signed doublewords to unsigned words.
        let d = of_dwords([-1, 0x1_0000, 0x1234, 0xffff]);
        let s = of_dwords([i32::MIN, i32::MAX, 0, 1]);
        assert_eq!(
            words(sse41(Op::PACKUSDW, d, s, none, 0)).map(|w| w as u16),
            [0, 0xffff, 0x1234, 0xffff, 0, 0xffff, 0, 1]
        );
        // PMULDQ: the even signed doublewords, to quadwords.
        let d = of_dwords([-2, 99, 3, 99]);
        let s = of_dwords([5, 99, -7, 99]);
        assert_eq!(
            sse41(Op::PMULDQ, d, s, none, 0),
            [-10i64 as u64, -21i64 as u64]
        );
        assert_eq!(
            dwords(sse41(Op::PMULLD, d, s, none, 0)),
            [-10, 9801, -21, 9801]
        );
        // The four new widths of minimum and maximum.
        let d = of_dwords([-1, 5, 0, 7]);
        let s = of_dwords([1, -5, 0, 8]);
        assert_eq!(dwords(sse41(Op::PMINSD, d, s, none, 0)), [-1, -5, 0, 7]);
        assert_eq!(dwords(sse41(Op::PMAXUD, d, s, none, 0)), [-1, -5, 0, 8]);
        assert_eq!(dwords(sse41(Op::PMINUD, d, s, none, 0)), [1, 5, 0, 7]);
        let d = of_words([-1, 2, 0, 0, 0, 0, 0, 0]);
        let s = of_words([1, -2, 0, 0, 0, 0, 0, 0]);
        assert_eq!(&words(sse41(Op::PMINUW, d, s, none, 0))[..2], &[1, 2]);
        assert_eq!(&words(sse41(Op::PMAXUW, d, s, none, 0))[..2], &[-1, -2]);
        // PCMPEQQ.
        assert_eq!(sse41(Op::PCMPEQQ, [1, 2], [1, 3], none, 0), [u64::MAX, 0]);
    }

    #[test]
    fn phminposuw_finds_the_first_smallest_word() {
        let s = of_words([5, 3, 9, 3, -1, 7, 8, 4]);
        // 3 at index 1 beats the 3 at index 3; -1 is 0xffff, the largest.
        assert_eq!(sse41(Op::PHMINPOSUW, [!0, !0], s, [0, 0], 0), [0x1_0003, 0]);
    }

    #[test]
    fn mpsadbw_slides_the_destination_block_against_a_fixed_source_one() {
        let d = of_bytes(core::array::from_fn(|i| i as u8));
        let s = of_bytes([0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0]);
        // imm 0: |i + j - 0| summed over j = 4i + 6.
        let want: [i16; 8] = core::array::from_fn(|i| 4 * i as i16 + 6);
        assert_eq!(words(sse41(Op::MPSADBW, d, s, [0, 0], 0)), want);
        // imm 0b101: the destination from byte 4, the source block from
        // byte 4 (all ones): |4 + i + j - 1| summed = 4i + 18.
        let want: [i16; 8] = core::array::from_fn(|i| 4 * i as i16 + 18);
        assert_eq!(words(sse41(Op::MPSADBW, d, s, [0, 0], 0b101)), want);
    }

    #[test]
    fn the_blends_select_by_immediate_or_by_xmm0s_signs() {
        let d = [0x1111_1111_1111_1111, 0x2222_2222_2222_2222];
        let s = [0xaaaa_aaaa_aaaa_aaaa, 0xbbbb_bbbb_bbbb_bbbb];
        assert_eq!(sse41(Op::BLENDPD, d, s, [0, 0], 0b10), [d[0], s[1]]);
        assert_eq!(
            sse41(Op::BLENDPS, d, s, [0, 0], 0b0110),
            [0xaaaa_aaaa_1111_1111, 0x2222_2222_bbbb_bbbb]
        );
        assert_eq!(
            sse41(Op::PBLENDW, d, s, [0, 0], 0b1000_0001),
            [0x1111_1111_1111_aaaa, 0xbbbb_2222_2222_2222]
        );
        // Variable: only the top bit of each mask lane counts.
        let mask = [0x0000_0000_8000_007f, 0x8000_0000_0000_0000];
        assert_eq!(
            sse41(Op::PBLENDVB, d, s, mask, 0),
            [0x1111_1111_aa11_1111, 0xbb22_2222_2222_2222]
        );
        assert_eq!(
            sse41(Op::BLENDVPS, d, s, mask, 0),
            [0x1111_1111_aaaa_aaaa, 0xbbbb_bbbb_2222_2222]
        );
        assert_eq!(sse41(Op::BLENDVPD, d, s, mask, 0), [d[0], s[1]]);
    }

    #[test]
    fn the_rounds_take_their_mode_from_the_immediate_or_from_mxcsr() {
        // 2.5 and -2.5 under each of the four modes: nearest-even gives 2
        // and -2, down 2 and -3, up 3 and -2, truncation 2 and -2.
        let src = of_singles([0x4020_0000, 0xc020_0000, 0, 0]);
        for (imm, want) in [
            (0u8, [0x4000_0000, 0xc000_0000]),
            (1, [0x4000_0000, 0xc040_0000]),
            (2, [0x4040_0000, 0xc000_0000]),
            (3, [0x4000_0000, 0xc000_0000]),
        ] {
            let (out, flags) = round(Op::ROUNDPS, [0, 0], src, imm, Env::X86_SSE);
            assert_eq!(&singles(out)[..2], &want, "imm {imm}");
            assert!(flags.contains(Flags::INEXACT), "imm {imm}");
        }
        // imm[2]: MXCSR's mode — here toward positive — whatever imm[1:0]
        // says; and imm[3] suppresses the precision exception.
        let up = Env::X86_SSE.round(Round::TowardPositive);
        let (out, flags) = round(Op::ROUNDPS, [0, 0], src, 0b1100, up);
        assert_eq!(&singles(out)[..2], &[0x4040_0000, 0xc000_0000]);
        assert_eq!(flags, Flags::NONE);
        // The scalar double form keeps the destination's high lane.
        let (out, _) = round(
            Op::ROUNDSD,
            [7, 0x1234],
            [0x3ff8_0000_0000_0000, 0],
            3,
            Env::X86_SSE,
        );
        assert_eq!(out, [0x3ff0_0000_0000_0000, 0x1234]);
        // A signalling NaN is invalid; a denormal is *not* a denormal
        // exception, which the arithmetic would raise.
        let (_, flags) = round(Op::ROUNDSS, [0, 0], [0x7fa0_0000, 0], 0, Env::X86_SSE);
        assert!(flags.contains(Flags::INVALID));
        let (out, flags) = round(Op::ROUNDSS, [0, 0], [1, 0], 8, Env::X86_SSE);
        assert_eq!(out, [0, 0]);
        assert_eq!(flags, Flags::NONE);
    }

    #[test]
    fn the_dot_products_select_inputs_and_broadcast_the_sum() {
        let d = of_singles([ONE, TWO, THREE, FOUR]);
        let s = of_singles([FIVE, 0x40c0_0000, SEVEN, 0x4100_0000]); // 5 6 7 8
        // All four products, the sum in lane 0: 5 + 12 + 21 + 32 = 70.
        let (out, flags) = dot(Op::DPPS, d, s, 0xf1, Env::X86_SSE);
        assert_eq!(singles(out), [0x428c_0000, 0, 0, 0]);
        assert_eq!(flags, Flags::NONE);
        // Only the low two products, broadcast everywhere: 17.
        let (out, _) = dot(Op::DPPS, d, s, 0x3f, Env::X86_SSE);
        assert_eq!(singles(out), [0x4188_0000; 4]);
        // DPPD: 1.5 * 2 + 2 * 4 = 11 into the high lane only.
        let d = [0x3ff8_0000_0000_0000, 0x4000_0000_0000_0000];
        let s = [0x4000_0000_0000_0000, 0x4010_0000_0000_0000];
        let (out, _) = dot(Op::DPPD, d, s, 0x32, Env::X86_SSE);
        assert_eq!(out, [0, 0x4026_0000_0000_0000]);
    }

    /// A string as a register, zero-padded.
    fn text(s: &[u8]) -> [u64; 2] {
        let mut b = [0u8; 16];
        b[..s.len()].copy_from_slice(s);
        of_bytes(b)
    }

    #[test]
    fn crc32c_of_the_check_string_is_the_castagnoli_check_value() {
        // The standard check: CRC-32C of "123456789", with the conventional
        // all-ones start and final inversion, is 0xE3069283 — a property of
        // the polynomial, which is iSCSI's (RFC 3720), and computable from it
        // alone. Byte at a time and in wider pieces agree, because the
        // instruction consumes its source least significant byte first.
        let msg = b"123456789";
        let bytewise = msg.iter().fold(!0u32, |c, &b| crc32c(c, u64::from(b), 1));
        assert_eq!(!bytewise, 0xe306_9283);
        let mut q = [0u8; 8];
        q.copy_from_slice(&msg[..8]);
        let wide = crc32c(!0, u64::from_le_bytes(q), 8);
        assert_eq!(!crc32c(wide, u64::from(msg[8]), 1), 0xe306_9283);
        let mut d = [0u8; 4];
        d.copy_from_slice(&msg[..4]);
        let c = crc32c(!0, u64::from(u32::from_le_bytes(d)), 4);
        let c = crc32c(c, u64::from(u16::from_le_bytes([msg[4], msg[5]])), 2);
        let c = crc32c(c, u64::from(u16::from_le_bytes([msg[6], msg[7]])), 2);
        assert_eq!(!crc32c(c, u64::from(msg[8]), 1), 0xe306_9283);
    }

    #[test]
    fn equal_any_finds_the_members_of_a_set() {
        // strpbrk("Hello, world!", "aeiou"): 'e' at 1, 'o' at 4 and 8.
        let set = text(b"aeiou");
        let s = text(b"Hello, world!");
        assert_eq!(pcmpstr(set, s, 5, 13, 0x00), 0b1_0001_0010);
        // Negated: every *other* bit, invalid tail included.
        assert_eq!(pcmpstr(set, s, 5, 13, 0x10), !0b1_0001_0010);
        // Masked negative: negated only within the string.
        assert_eq!(pcmpstr(set, s, 5, 13, 0x30), 0b1_1110_1110_1101);
        assert_eq!(implicit_length(set, false), 5);
        assert_eq!(implicit_length(s, false), 13);
    }

    #[test]
    fn ranges_take_the_first_operand_in_pairs() {
        // [a-z] over "Hello": the four lower-case letters.
        let r = text(b"az");
        assert_eq!(pcmpstr(r, text(b"Hello"), 2, 5, 0x04), 0b1_1110);
        // An odd count leaves the last bound unpaired, and unused.
        assert_eq!(pcmpstr(text(b"azH"), text(b"Hello"), 3, 5, 0x04), 0b1_1110);
        // Signed words: [-10, 10] over [-20, -5, 0, 15].
        let bounds = of_words([-10, 10, 0, 0, 0, 0, 0, 0]);
        let v = of_words([-20, -5, 0, 15, 0, 0, 0, 0]);
        assert_eq!(pcmpstr(bounds, v, 2, 4, 0x07), 0b0110);
        // The same bits as unsigned words: 0xfff6..0x000a is an empty range.
        assert_eq!(pcmpstr(bounds, v, 2, 4, 0x05), 0);
    }

    #[test]
    fn equal_each_is_a_string_compare() {
        // "hello" against "help!": the first difference is at 3, and past
        // both ends the strings agree.
        let r = pcmpstr(text(b"hello"), text(b"help!"), 5, 5, 0x08);
        assert_eq!(r, 0xffe7);
        // With negative polarity, the lowest set bit is the first mismatch.
        assert_eq!(pcmpstr(text(b"hello"), text(b"help!"), 5, 5, 0x18), 0x0018);
        // One string longer than the other: the extra element mismatches.
        assert_eq!(pcmpstr(text(b"ab"), text(b"abc"), 2, 3, 0x08), 0xfffb);
    }

    #[test]
    fn equal_ordered_is_a_substring_search_with_a_partial_tail() {
        // strstr("hello world", "lo") is at 3, and only there.
        assert_eq!(
            pcmpstr(text(b"lo"), text(b"hello world"), 2, 11, 0x0c),
            1 << 3
        );
        // A needle that runs off the end of the register matches as far as
        // it goes: "lo" against sixteen bytes ending in 'l' reports 15.
        let hay = text(b"xxxxxxxxxxxxxxxl");
        assert_eq!(pcmpstr(text(b"lo"), hay, 2, 16, 0x0c), 1 << 15);
        // But not when the haystack *string* has ended before the register:
        // the needle's next element is valid and the haystack's is not.
        assert_eq!(pcmpstr(text(b"lo"), text(b"hel"), 2, 3, 0x0c), 0);
    }

    #[test]
    fn the_mask_output_widens_to_elements() {
        assert_eq!(expand_mask(0b101, false), [0x0000_0000_00ff_00ff, 0]);
        assert_eq!(expand_mask(0x81, true), [0xffff, 0xffff_0000_0000_0000]);
    }

    #[test]
    fn a_signalling_nan_in_one_lane_reports_invalid() {
        let d = of_singles([ONE, 0x7fa0_0000, ONE, ONE]);
        let (_, flags) = horizontal(Op::HADDPS, d, d, Env::X86_SSE);
        assert!(flags.contains(Flags::INVALID));
    }
}
