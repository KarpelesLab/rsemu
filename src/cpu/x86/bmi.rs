//! Executing BMI1 and BMI2: the VEX-encoded general-register instructions.
//!
//! `ANDN`, `BEXTR`, `BLSI`, `BLSMSK` and `BLSR` (BMI1), and `BZHI`, `MULX`,
//! `PDEP`, `PEXT`, `RORX`, `SARX`, `SHLX` and `SHRX` (BMI2). `TZCNT` is
//! BMI1's too, but it is legacy-encoded and lives beside `LZCNT` in `simd`.
//!
//! # The gate
//!
//! They borrow VEX's encoding and none of AVX's state, so none of AVX's
//! gate: no `CR4.OSXSAVE`, no `XCR0`, no `#NM` for `CR0.TS`. What is left is
//! the type 13 exception class every one of their pages names (*Intel SDM*
//! volume 2 §2.5.1, table 2-29): `#UD` without the `CPUID` bit, with
//! `VEX.L = 1` — which the decoder has already turned into `Op::UD` — and in
//! real-address and virtual-8086 mode, where VEX is no prefix at all. A
//! `LOCK`, `66`, `F2`, `F3` or `REX` before the VEX prefix is the decoder's
//! `#UD` too.
//!
//! # Operand size
//!
//! Thirty-two bits, or sixty-four with `VEX.W` in 64-bit mode; outside it
//! `VEX.W` is ignored (each page's opcode table, "W1 ... 64-bit mode only",
//! and the note that `VEX.W1` is treated as `W0` in 32-bit modes). Not the
//! code segment's operand size: there is no `66` to change it, and a
//! 16-bit code segment still runs the 32-bit forms. The decoder folds
//! `VEX.W` into `Fields::rex` only in 64-bit mode, so [`Fields::rex_w`] is
//! the whole rule.
//!
//! # Undefined flags
//!
//! Where a page says a flag is undefined — `AF` and `PF` for most, `SF` for
//! `BEXTR` — it is left as it was, as `BSF`, `BSR`, `LZCNT` and `TZCNT`
//! leave theirs in this core: deterministic, and the cheapest answer. The
//! BMI2 instructions other than `BZHI` affect no flag at all.
//!
//! # Sources
//!
//! *Intel SDM* volume 2, each instruction's page (operation, flags affected,
//! exceptions) and table A-4/A-5/A-6 for the encodings; *AMD64 Architecture
//! Programmer's Manual* volume 3 for the same instructions. No copyleft
//! emulator was consulted (`CLAUDE.md`, provenance).

use super::exec::{Ex, Exec, Fault, VEC_UD};
use super::flags;
use super::isa::{Arg, Fields, Op};

impl Exec<'_> {
    /// Execute one VEX-encoded BMI1 or BMI2 instruction.
    pub(super) fn bmi(&mut self, f: &Fields) -> Ex<()> {
        if !self.has_extension(f.insn) || !self.protected() || self.v86() {
            return Err(Fault::bare(VEC_UD));
        }
        let size = if f.rex_w() { 8 } else { 4 };
        let bits = u32::from(size) * 8;
        let mask = Self::mask(size);
        let op = f.insn.op;
        // Every form reads ModRM r/m; read it first, so a faulting memory
        // operand leaves every register and flag as it was.
        let src = self.read_arg(f, Arg::Ey, size)?;
        match op {
            Op::BLSI | Op::BLSMSK | Op::BLSR => {
                let out = match op {
                    Op::BLSI => src & src.wrapping_neg(),
                    Op::BLSMSK => src ^ src.wrapping_sub(1),
                    _ => src & src.wrapping_sub(1),
                } & mask;
                self.write_arg(f, Arg::By, size, out)?;
                let sign = out & Self::msb(size) != 0;
                self.set_flag(flags::SF, sign);
                self.set_flag(flags::OF, false);
                // `BLSMSK`'s result is never zero, and its page says so by
                // clearing `ZF` outright; the other two report the result.
                self.set_flag(flags::ZF, op != Op::BLSMSK && out == 0);
                // `BLSI`: `CF` is "set if the source is not zero"; `BLSMSK`
                // and `BLSR`: "set if the source is zero".
                self.set_flag(flags::CF, (op == Op::BLSI) == (src != 0));
            }
            Op::ANDN => {
                let first = self.read_arg(f, Arg::By, size)?;
                let out = !first & src & mask;
                self.write_arg(f, Arg::Gy, size, out)?;
                self.set_flag(flags::SF, out & Self::msb(size) != 0);
                self.set_flag(flags::ZF, out == 0);
                self.set_flag(flags::CF | flags::OF, false);
            }
            Op::BEXTR => {
                let control = self.read_arg(f, Arg::By, size)?;
                let out = bextr(src, control, bits);
                self.write_arg(f, Arg::Gy, size, out)?;
                self.set_flag(flags::ZF, out == 0);
                self.set_flag(flags::CF | flags::OF, false);
            }
            Op::BZHI => {
                let index = self.read_arg(f, Arg::By, size)?;
                let (out, past) = bzhi(src, index, bits);
                self.write_arg(f, Arg::Gy, size, out)?;
                self.set_flag(flags::SF, out & Self::msb(size) != 0);
                self.set_flag(flags::ZF, out == 0);
                self.set_flag(flags::CF, past);
                self.set_flag(flags::OF, false);
            }
            Op::PDEP | Op::PEXT => {
                let first = self.read_arg(f, Arg::By, size)?;
                let out = if op == Op::PDEP {
                    pdep(first, src)
                } else {
                    pext(first, src)
                };
                self.write_arg(f, Arg::Gy, size, out & mask)?;
            }
            Op::MULX => {
                let factor = self.state.regs.read(2, size, false);
                let product = u128::from(factor) * u128::from(src);
                let low = product as u64 & mask;
                let high = (product >> bits) as u64 & mask;
                // The low half first, so that with both destinations the same
                // register the high half is what remains (*Intel SDM* volume
                // 2, `MULX`: "If the first and second operand are identical,
                // it will contain the high half of the multiplication
                // result").
                self.write_arg(f, Arg::By, size, low)?;
                self.write_arg(f, Arg::Gy, size, high)?;
            }
            Op::RORX => {
                let count = (f.imm as u32) & (bits - 1);
                self.write_arg(f, Arg::Gy, size, rotate_right(src, count, bits))?;
            }
            Op::SHLX | Op::SHRX | Op::SARX => {
                // The count is masked to five or six bits, as the legacy
                // shifts mask theirs, and nothing else about it is read.
                let count = (self.read_arg(f, Arg::By, size)? as u32) & (bits - 1);
                let out = match op {
                    Op::SHLX => src << count,
                    Op::SHRX => src >> count,
                    _ => (Self::sign_extend(src, size) as i64 >> count) as u64,
                };
                self.write_arg(f, Arg::Gy, size, out & mask)?;
            }
            _ => return Err(Fault::bare(VEC_UD)),
        }
        Ok(())
    }
}

/// `BEXTR`: `control[7:0]` is the start bit and `control[15:8]` the length,
/// over a source of `bits` bits zero-extended without limit — so a start at
/// or past the width, or a length of zero, extracts nothing, and a field
/// that runs off the top is cut there (*Intel SDM* volume 2, `BEXTR`,
/// "Operation": `TEMP := ZERO_EXTEND_TO_512(SRC1)`).
#[must_use]
pub(super) const fn bextr(src: u64, control: u64, bits: u32) -> u64 {
    let start = (control & 0xff) as u32;
    let len = ((control >> 8) & 0xff) as u32;
    if start >= bits {
        return 0;
    }
    let field = src >> start;
    if len >= 64 {
        field
    } else {
        field & ((1 << len) - 1)
    }
}

/// `BZHI`: the source with every bit from `index[7:0]` upward cleared, and
/// whether the index was past the operand — which is `CF`, and then leaves
/// the source whole (*Intel SDM* volume 2, `BZHI`).
#[must_use]
pub(super) const fn bzhi(src: u64, index: u64, bits: u32) -> (u64, bool) {
    let n = (index & 0xff) as u32;
    if n >= bits {
        (src, true)
    } else {
        (src & ((1 << n) - 1), false)
    }
}

/// `PDEP`: the low bits of `src`, in order, at the positions of `mask`'s
/// set bits, lowest first; every other bit zero.
#[must_use]
pub(super) const fn pdep(src: u64, mask: u64) -> u64 {
    let mut out = 0;
    let mut m = mask;
    let mut k = 0;
    while m != 0 {
        let lowest = m & m.wrapping_neg();
        if (src >> k) & 1 != 0 {
            out |= lowest;
        }
        m ^= lowest;
        k += 1;
    }
    out
}

/// `PEXT`: the bits of `src` at the positions of `mask`'s set bits, packed
/// into the low bits of the result in order, lowest first.
#[must_use]
pub(super) const fn pext(src: u64, mask: u64) -> u64 {
    let mut out = 0;
    let mut m = mask;
    let mut k = 0;
    while m != 0 {
        let lowest = m & m.wrapping_neg();
        if src & lowest != 0 {
            out |= 1 << k;
        }
        m ^= lowest;
        k += 1;
    }
    out
}

/// Rotate the low `bits` bits of `value` right by `count`, already reduced
/// below `bits`.
#[must_use]
const fn rotate_right(value: u64, count: u32, bits: u32) -> u64 {
    if bits == 64 {
        value.rotate_right(count)
    } else {
        (value as u32).rotate_right(count) as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pdep_and_pext_deposit_and_gather_in_order() {
        // The *Intel SDM* volume 2 `PEXT` figure's shape, checked by hand:
        // the mask's set nibbles pick 0x1, 0x2, 0x5, 0x6, 0x7 out of
        // 0x12345678 and pack them low.
        assert_eq!(pext(0x1234_5678, 0xff00_fff0), 0x1_2567);
        assert_eq!(pdep(0x1_2567, 0xff00_fff0), 0x1200_5670);
        // A dense mask is the identity on the bits it covers; an empty one
        // gives nothing.
        assert_eq!(pext(0xdead_beef, u64::MAX), 0xdead_beef);
        assert_eq!(pdep(0xdead_beef, u64::MAX), 0xdead_beef);
        assert_eq!(pext(u64::MAX, 0), 0);
        assert_eq!(pdep(u64::MAX, 0), 0);
        // Sparse: every other bit.
        assert_eq!(pdep(0b1011, 0b0101_0101), 0b0100_0101);
        assert_eq!(pext(0b0100_0101, 0b0101_0101), 0b1011);
        // The top bit of a 64-bit mask.
        assert_eq!(pdep(1, 1 << 63), 1 << 63);
        assert_eq!(pext(1 << 63, 1 << 63), 1);
        // The two invert one another on the mask's bits.
        for (src, mask) in [
            (0x0123_4567_89ab_cdef, 0xf0f0_0ff0_1234_8001u64),
            (!0, 0x8421),
        ] {
            assert_eq!(
                pext(pdep(src, mask), mask),
                src & ((1 << mask.count_ones()) - 1)
            );
        }
    }

    #[test]
    fn bextr_cuts_at_the_width_and_reads_eight_bit_fields() {
        // start 4, length 8.
        assert_eq!(bextr(0x1234_5678, 0x0804, 32), 0x67);
        // Length zero, start past the width, start exactly at it.
        assert_eq!(bextr(0x1234_5678, 0x0004, 32), 0);
        assert_eq!(bextr(u64::MAX, 0x0840, 64), 0);
        assert_eq!(bextr(0xffff_ffff, 0x0820, 32), 0);
        // A field that runs off the top stops there.
        assert_eq!(bextr(0xf000_0000, 0xff1c, 32), 0xf);
        assert_eq!(bextr(u64::MAX, 0xff00, 64), u64::MAX);
        // Bits above 15 of the control are not part of it.
        assert_eq!(bextr(0xff, 0xdead_0000_0404, 64), 0xf);
    }

    #[test]
    fn bzhi_keeps_the_bits_below_the_index() {
        assert_eq!(bzhi(0xffff_ffff, 4, 32), (0xf, false));
        assert_eq!(bzhi(0xffff_ffff, 0, 32), (0, false));
        assert_eq!(bzhi(0xffff_ffff, 31, 32), (0x7fff_ffff, false));
        // At or past the width: the source whole, and `CF`.
        assert_eq!(bzhi(0xffff_ffff, 32, 32), (0xffff_ffff, true));
        assert_eq!(bzhi(u64::MAX, 64, 64), (u64::MAX, true));
        assert_eq!(bzhi(u64::MAX, 63, 64), (u64::MAX >> 1, false));
        // Only the low byte of the index counts: 0x104 is 4.
        assert_eq!(bzhi(0xff, 0x104, 32), (0xf, false));
    }

    #[test]
    fn rotate_right_stays_inside_the_operand() {
        assert_eq!(rotate_right(0x1, 1, 32), 0x8000_0000);
        assert_eq!(rotate_right(0x1, 1, 64), 1 << 63);
        assert_eq!(rotate_right(0x1234_5678, 8, 32), 0x7812_3456);
    }
}
