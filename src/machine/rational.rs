//! Exact rational arithmetic, for frequencies that are not integers.
//!
//! The NES master clock is 236250000/11 Hz — 21477272.72… — and §5's example
//! writes it that way on purpose: the ratio between the CPU and PPU domains is
//! exact, and only the wall-clock rate is fractional. Rounding the literal at
//! parse time would throw away the exactness the clock forest exists to
//! preserve (`ROADMAP.md` §4.2, `CLAUDE.md` "no floats in the time path").
//!
//! This is a small, self-contained value type living in `machine/` because
//! `core::clock` does not exist yet. **Seam:** when the clock forest lands it
//! will want the same type, and this one should move there rather than being
//! duplicated.
//!
//! Every operation is checked. A machine file is untrusted input, so overflow
//! is an error to report, never a panic and never a wrap.

use core::fmt;

/// A rational number in lowest terms, with a strictly positive denominator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rational {
    num: i128,
    den: i128,
}

impl Rational {
    /// Zero.
    pub const ZERO: Rational = Rational { num: 0, den: 1 };

    /// `num / den`, reduced. `None` if `den` is zero or the sign cannot be
    /// normalised (`i128::MIN`).
    pub fn new(num: i128, den: i128) -> Option<Rational> {
        if den == 0 || num == i128::MIN || den == i128::MIN {
            return None;
        }
        let (num, den) = if den < 0 { (-num, -den) } else { (num, den) };
        let g = gcd(num.unsigned_abs(), den.unsigned_abs());
        let g = if g == 0 { 1 } else { i128::try_from(g).ok()? };
        Some(Rational {
            num: num / g,
            den: den / g,
        })
    }

    /// An integer as a rational.
    pub const fn from_int(n: i64) -> Rational {
        Rational {
            num: n as i128,
            den: 1,
        }
    }

    /// The numerator, in lowest terms.
    pub const fn numerator(self) -> i128 {
        self.num
    }

    /// The denominator, in lowest terms; always positive.
    pub const fn denominator(self) -> i128 {
        self.den
    }

    /// Whether the value is a whole number.
    pub const fn is_integer(self) -> bool {
        self.den == 1
    }

    /// The value as an integer, or `None` if it is not whole.
    pub const fn to_integer(self) -> Option<i128> {
        if self.den == 1 { Some(self.num) } else { None }
    }

    /// `self + rhs`, or `None` on overflow.
    pub fn checked_add(self, rhs: Rational) -> Option<Rational> {
        let num = self
            .num
            .checked_mul(rhs.den)?
            .checked_add(rhs.num.checked_mul(self.den)?)?;
        Rational::new(num, self.den.checked_mul(rhs.den)?)
    }

    /// `self - rhs`, or `None` on overflow.
    pub fn checked_sub(self, rhs: Rational) -> Option<Rational> {
        let num = self
            .num
            .checked_mul(rhs.den)?
            .checked_sub(rhs.num.checked_mul(self.den)?)?;
        Rational::new(num, self.den.checked_mul(rhs.den)?)
    }

    /// `self * rhs`, or `None` on overflow.
    pub fn checked_mul(self, rhs: Rational) -> Option<Rational> {
        Rational::new(
            self.num.checked_mul(rhs.num)?,
            self.den.checked_mul(rhs.den)?,
        )
    }

    /// `self / rhs`, or `None` on overflow or division by zero.
    pub fn checked_div(self, rhs: Rational) -> Option<Rational> {
        if rhs.num == 0 {
            return None;
        }
        Rational::new(
            self.num.checked_mul(rhs.den)?,
            self.den.checked_mul(rhs.num)?,
        )
    }

    /// `-self`, or `None` on overflow.
    pub fn checked_neg(self) -> Option<Rational> {
        Rational::new(self.num.checked_neg()?, self.den)
    }
}

impl fmt::Display for Rational {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.den == 1 {
            write!(f, "{}", self.num)
        } else {
            write!(f, "{}/{}", self.num, self.den)
        }
    }
}

/// Binary-free Euclid on magnitudes; `gcd(0, 0)` is 0.
fn gcd(mut a: u128, mut b: u128) -> u128 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn fractions_are_reduced_and_signs_normalised() {
        let r = Rational::new(4, -8).expect("valid");
        assert_eq!(r.numerator(), -1);
        assert_eq!(r.denominator(), 2);
        assert_eq!(r.to_string(), "-1/2");
        assert_eq!(Rational::new(6, 3).expect("valid").to_string(), "2");
    }

    #[test]
    fn the_nes_master_clock_stays_exact() {
        let f = Rational::new(236_250_000, 11).expect("valid");
        assert_eq!(f.numerator(), 236_250_000);
        assert_eq!(f.denominator(), 11);
        assert!(!f.is_integer());
        // master / 12 is the 6502 clock; master / 4 is the PPU dot clock, and
        // the ratio between them must come out as exactly 3.
        let cpu = f.checked_div(Rational::from_int(12)).expect("valid");
        let ppu = f.checked_div(Rational::from_int(4)).expect("valid");
        assert_eq!(ppu.checked_div(cpu).expect("valid").to_integer(), Some(3));
    }

    #[test]
    fn arithmetic_is_checked_not_wrapping() {
        let big = Rational::new(i128::MAX, 1).expect("valid");
        assert_eq!(big.checked_add(big), None);
        assert_eq!(big.checked_mul(big), None);
        assert_eq!(Rational::from_int(1).checked_div(Rational::ZERO), None);
        assert_eq!(Rational::new(1, 0), None);
        assert_eq!(Rational::new(i128::MIN, 1), None);
    }

    #[test]
    fn addition_and_subtraction_agree() {
        let a = Rational::new(1, 3).expect("valid");
        let b = Rational::new(1, 6).expect("valid");
        assert_eq!(a.checked_add(b), Rational::new(1, 2));
        assert_eq!(a.checked_sub(b), Rational::new(1, 6));
        assert_eq!(a.checked_neg(), Rational::new(-1, 3));
    }

    #[test]
    fn gcd_handles_zero() {
        assert_eq!(gcd(0, 0), 0);
        assert_eq!(gcd(0, 5), 5);
        assert_eq!(gcd(12, 18), 6);
        assert_eq!(Rational::new(0, 5).expect("valid"), Rational::ZERO);
    }
}
