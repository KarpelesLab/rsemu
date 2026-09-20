//! The transcendentals, against values computed independently.
//!
//! # How the references were produced
//!
//! Every row below is `(operation, argument, result)` in raw
//! extended-precision bits. The arguments were chosen to cover each
//! function's domain; the results were computed by **GNU `bc -l`** — a
//! different implementation, in a different language, at `scale=110`, which
//! is about ninety correct decimal digits against the nineteen an
//! extended-precision significand holds — and then rounded once to
//! sixty-four bits, ties to even, by exact rational arithmetic.
//!
//! For each argument `x`, given exactly as a decimal (an extended-precision
//! value is a binary fraction, so its decimal expansion terminates):
//!
//! ```text
//! bc -l <<'EOF'
//! scale=110
//! x = <the argument, exactly>
//! s(x)                    # FSIN      c(x)                    # FCOS
//! s(x)/c(x)               # FTAN      a(x)                    # FATAN
//! a(x/sqrt(1-x*x))        # FASIN     2*a(sqrt((1-x)/(1+x)))  # FACOS
//! (e(x)-e(0-x))/2         # FSINH     (e(x)+e(0-x))/2         # FCOSH
//! (e(2*x)-1)/(e(2*x)+1)   # FTANH     l((1+x)/(1-x))/2        # FATANH
//! e(x)                    # FETOX     e(x)-1                  # FETOXM1
//! e(x*l(2))               # FTWOTOX   e(x*l(10))              # FTENTOX
//! l(x)                    # FLOGN     l(1+x)                  # FLOGNP1
//! l(x)/l(2)               # FLOG2     l(x)/l(10)              # FLOG10
//! EOF
//! ```
//!
//! `bc`'s `-l` library is POSIX's: `s`, `c`, `a`, `l`, `e` and `sqrt`, each
//! computed to the current `scale`. Nothing here consults the host's libm,
//! which is neither correctly rounded nor the same twice.
//!
//! # What the test asserts
//!
//! That every result is the correctly rounded one — not within an ulp, the
//! same bits. M68881UM §4.3.2 allows a 68881 four thousand and ninety-six
//! ulp of extended precision in the worst case and sixty-four typically;
//! this core is inside half of one, which `docs/cpu/m68k.md` records as the
//! deviation it is.

use crate::float::x87::F80;

use super::isa::fp::FpOp;
use super::tests_68010::Board;
use super::{Coprocessor, Model};

/// `(operation, argument field, argument significand, result field, result
/// significand)`.
static REFERENCE: &[(FpOp, u16, u64, u16, u64)] = &[
    (
        FpOp::Sin,
        0x3ff8,
        0x8000000000000000,
        0x3ff7,
        0xffff555577777437,
    ),
    (
        FpOp::Sin,
        0x3ffd,
        0x9000000000000000,
        0x3ffd,
        0x8e1beb2635c3b28c,
    ),
    (
        FpOp::Sin,
        0x3ffe,
        0x8000000000000000,
        0x3ffd,
        0xf57743a2582f7f44,
    ),
    (
        FpOp::Sin,
        0x3fff,
        0x8000000000000000,
        0x3ffe,
        0xd76aa47848677021,
    ),
    (
        FpOp::Sin,
        0x3fff,
        0xc90fdaa22168c235,
        0x3fff,
        0x8000000000000000,
    ),
    (
        FpOp::Sin,
        0x4000,
        0xc000000000000000,
        0x3ffc,
        0x9081c36db6aada79,
    ),
    (
        FpOp::Sin,
        0x4004,
        0xa123456789abcdef,
        0x3ffe,
        0x872be99524472076,
    ),
    (
        FpOp::Sin,
        0x4010,
        0xf000000000000000,
        0xbffd,
        0xf9fe61aa55805bf9,
    ),
    (
        FpOp::Sin,
        0x4030,
        0x9876543210fedcba,
        0x3ffe,
        0xf77fa7a1612a5691,
    ),
    (
        FpOp::Cos,
        0x3ff8,
        0x8000000000000000,
        0x3ffe,
        0xfffe0000aaaa93e9,
    ),
    (
        FpOp::Cos,
        0x3ffd,
        0x9000000000000000,
        0x3ffe,
        0xf5f10a7bb77d3dfa,
    ),
    (
        FpOp::Cos,
        0x3ffe,
        0x8000000000000000,
        0x3ffe,
        0xe0a94032dbea7cee,
    ),
    (
        FpOp::Cos,
        0x3fff,
        0x8000000000000000,
        0x3ffe,
        0x8a51407da8345c92,
    ),
    (
        FpOp::Cos,
        0x3fff,
        0xc90fdaa22168c235,
        0xbfbd,
        0xece675d1fc8f8cbb,
    ),
    (
        FpOp::Cos,
        0x4000,
        0xc000000000000000,
        0xbffe,
        0xfd7025f42f2e9308,
    ),
    (
        FpOp::Cos,
        0x4004,
        0xa123456789abcdef,
        0xbffe,
        0xd967844730e20084,
    ),
    (
        FpOp::Cos,
        0x4010,
        0xf000000000000000,
        0x3ffe,
        0xdf68d2e0856fa22b,
    ),
    (
        FpOp::Cos,
        0x4030,
        0x9876543210fedcba,
        0x3ffd,
        0x82d98c6a57cb49c6,
    ),
    (
        FpOp::Tan,
        0x3ff8,
        0x8000000000000000,
        0x3ff8,
        0x8000aaabbbbd75da,
    ),
    (
        FpOp::Tan,
        0x3ffd,
        0x9000000000000000,
        0x3ffd,
        0x93ebc5a05bb823e7,
    ),
    (
        FpOp::Tan,
        0x3ffe,
        0x8000000000000000,
        0x3ffe,
        0x8bda7adf9a3a5219,
    ),
    (
        FpOp::Tan,
        0x3fff,
        0x8000000000000000,
        0x3fff,
        0xc75922e5f71d2dc5,
    ),
    (
        FpOp::Tan,
        0x3fff,
        0xc90fdaa22168c235,
        0xc040,
        0x8a51e04daabda35f,
    ),
    (
        FpOp::Tan,
        0x4000,
        0xc000000000000000,
        0xbffc,
        0x91f7b892a5c37866,
    ),
    (
        FpOp::Tan,
        0x4004,
        0xa123456789abcdef,
        0xbffe,
        0x9f2b1ea910a0a0b8,
    ),
    (
        FpOp::Tan,
        0x4010,
        0xf000000000000000,
        0xbffe,
        0x8f3b2b67c05cb1b9,
    ),
    (
        FpOp::Tan,
        0x4030,
        0x9876543210fedcba,
        0x4000,
        0xf21bbc14b86a4130,
    ),
    (
        FpOp::Asin,
        0xbffe,
        0xc000000000000000,
        0xbffe,
        0xd91a98ae3406e041,
    ),
    (
        FpOp::Asin,
        0xbffb,
        0x8000000000000000,
        0xbffb,
        0x80155ef4a9b0ca30,
    ),
    (
        FpOp::Asin,
        0x3ff0,
        0x8000000000000000,
        0x3ff0,
        0x8000000055555556,
    ),
    (
        FpOp::Asin,
        0x3ffd,
        0xb504f333f9de6484,
        0x3ffd,
        0xb9051c960ecaa428,
    ),
    (
        FpOp::Asin,
        0x3ffe,
        0x8000000000000000,
        0x3ffe,
        0x860a91c16b9b2c23,
    ),
    (
        FpOp::Asin,
        0x3ffe,
        0xfedcba9876543210,
        0x3fff,
        0xbcfd4fdf60e0aeeb,
    ),
    (
        FpOp::Acos,
        0xbffe,
        0xc000000000000000,
        0x4000,
        0x9ace937c9db6192b,
    ),
    (
        FpOp::Acos,
        0xbffb,
        0x8000000000000000,
        0x3fff,
        0xd11130916c03ced8,
    ),
    (
        FpOp::Acos,
        0x3ff0,
        0x8000000000000000,
        0x3fff,
        0xc90edaa22168178a,
    ),
    (
        FpOp::Acos,
        0x3ffd,
        0xb504f333f9de6484,
        0x3fff,
        0x9ace937c9db6192b,
    ),
    (
        FpOp::Acos,
        0x3ffe,
        0x8000000000000000,
        0x3fff,
        0x860a91c16b9b2c23,
    ),
    (
        FpOp::Acos,
        0x3ffe,
        0xfedcba9876543210,
        0x3ffb,
        0xc128ac2c08813497,
    ),
    (
        FpOp::Atan,
        0xbffe,
        0x8000000000000000,
        0xbffd,
        0xed63382b0dda7b45,
    ),
    (
        FpOp::Atan,
        0xbff0,
        0x8000000000000000,
        0xbfef,
        0xfffffffeaaaaaaae,
    ),
    (
        FpOp::Atan,
        0x3fe0,
        0x8000000000000000,
        0x3fdf,
        0xffffffffffffffff,
    ),
    (
        FpOp::Atan,
        0x3ffc,
        0xc000000000000000,
        0x3ffc,
        0xbdcbda5e72d81134,
    ),
    (
        FpOp::Atan,
        0x3fff,
        0x8000000000000000,
        0x3ffe,
        0xc90fdaa22168c235,
    ),
    (
        FpOp::Atan,
        0x4000,
        0xc000000000000000,
        0x3fff,
        0x9fe0bb5bd42affec,
    ),
    (
        FpOp::Atan,
        0x4005,
        0xfa00000000000000,
        0x3fff,
        0xc809b7336fb1da2b,
    ),
    (
        FpOp::Atan,
        0x4030,
        0x9876543210fedcba,
        0x3fff,
        0xc90fdaa221688c7a,
    ),
    (
        FpOp::Sinh,
        0xbffe,
        0x8000000000000000,
        0xbffe,
        0x8566807f31dcb652,
    ),
    (
        FpOp::Sinh,
        0xbff0,
        0x8000000000000000,
        0xbff0,
        0x8000000055555555,
    ),
    (
        FpOp::Sinh,
        0x3fe0,
        0x8000000000000000,
        0x3fe0,
        0x8000000000000000,
    ),
    (
        FpOp::Sinh,
        0x3ffc,
        0xc000000000000000,
        0x3ffc,
        0xc12081b5628ee9a4,
    ),
    (
        FpOp::Sinh,
        0x3fff,
        0x8000000000000000,
        0x3fff,
        0x966cfe2275cc12d4,
    ),
    (
        FpOp::Sinh,
        0x4000,
        0xc000000000000000,
        0x4002,
        0xa04937384a4d6cdc,
    ),
    (
        FpOp::Sinh,
        0x4005,
        0xfa00000000000000,
        0x40b2,
        0xa1aab6f749df6f8e,
    ),
    (
        FpOp::Cosh,
        0xbffe,
        0x8000000000000000,
        0x3fff,
        0x90560c3157468323,
    ),
    (
        FpOp::Cosh,
        0xbff0,
        0x8000000000000000,
        0x3fff,
        0x8000000100000000,
    ),
    (
        FpOp::Cosh,
        0x3fe0,
        0x8000000000000000,
        0x3fff,
        0x8000000000000001,
    ),
    (
        FpOp::Cosh,
        0x3ffc,
        0xc000000000000000,
        0x3fff,
        0x8241b081ae6fcc36,
    ),
    (
        FpOp::Cosh,
        0x3fff,
        0x8000000000000000,
        0x3fff,
        0xc583aa8ecfaa8261,
    ),
    (
        FpOp::Cosh,
        0x4000,
        0xc000000000000000,
        0x4002,
        0xa11524beb0c2f252,
    ),
    (
        FpOp::Cosh,
        0x4005,
        0xfa00000000000000,
        0x40b2,
        0xa1aab6f749df6f8e,
    ),
    (
        FpOp::Tanh,
        0xbffe,
        0x8000000000000000,
        0xbffd,
        0xec9a9ebab4579b29,
    ),
    (
        FpOp::Tanh,
        0xbff0,
        0x8000000000000000,
        0xbfef,
        0xfffffffeaaaaaaad,
    ),
    (
        FpOp::Tanh,
        0x3fe0,
        0x8000000000000000,
        0x3fdf,
        0xffffffffffffffff,
    ),
    (
        FpOp::Tanh,
        0x3ffc,
        0xc000000000000000,
        0x3ffc,
        0xbdc7fc81dfbbb194,
    ),
    (
        FpOp::Tanh,
        0x3fff,
        0x8000000000000000,
        0x3ffe,
        0xc2f7d5a8a79ca2ac,
    ),
    (
        FpOp::Tanh,
        0x4000,
        0xc000000000000000,
        0x3ffe,
        0xfebbe888d057ff10,
    ),
    (
        FpOp::Tanh,
        0x4005,
        0xfa00000000000000,
        0x3fff,
        0x8000000000000000,
    ),
    (
        FpOp::Atanh,
        0xbffe,
        0xc000000000000000,
        0xbffe,
        0xf913957192d2baa3,
    ),
    (
        FpOp::Atanh,
        0xbffb,
        0x8000000000000000,
        0xbffb,
        0x802ac4569bad66e6,
    ),
    (
        FpOp::Atanh,
        0x3ff0,
        0x8000000000000000,
        0x3ff0,
        0x80000000aaaaaaac,
    ),
    (
        FpOp::Atanh,
        0x3ffd,
        0xb504f333f9de6484,
        0x3ffd,
        0xbd2ef820a78acd00,
    ),
    (
        FpOp::Atanh,
        0x3ffe,
        0x8000000000000000,
        0x3ffe,
        0x8c9f53d5681854bb,
    ),
    (
        FpOp::Atanh,
        0x3ffe,
        0xfedcba9876543210,
        0x4000,
        0xc36cbb4358d8c5ba,
    ),
    (
        FpOp::Etox,
        0xc002,
        0xa000000000000000,
        0x3ff0,
        0xbe6bcdab23e4d4e3,
    ),
    (
        FpOp::Etox,
        0xbffe,
        0x8000000000000000,
        0x3ffe,
        0x9b4597e37cb04ff4,
    ),
    (
        FpOp::Etox,
        0x3ff8,
        0x8000000000000000,
        0x3fff,
        0x810100ab00222d86,
    ),
    (
        FpOp::Etox,
        0x3ffe,
        0x8000000000000000,
        0x3fff,
        0xd3094c70f034de4c,
    ),
    (
        FpOp::Etox,
        0x3fff,
        0x8000000000000000,
        0x4000,
        0xadf85458a2bb4a9b,
    ),
    (
        FpOp::Etox,
        0x4000,
        0xc000000000000000,
        0x4003,
        0xa0af2dfb7d882f97,
    ),
    (
        FpOp::Etox,
        0x4005,
        0x8000000000000000,
        0x405b,
        0xa12cc167acbe6903,
    ),
    (
        FpOp::Etox,
        0x4008,
        0xfa00000000000000,
        0x45a1,
        0xcf391bcd76b9d6c0,
    ),
    (
        FpOp::EtoxM1,
        0xbffe,
        0x8000000000000000,
        0xbffd,
        0xc974d039069f6018,
    ),
    (
        FpOp::EtoxM1,
        0xbff0,
        0x8000000000000000,
        0xbfef,
        0xffff0000aaaa5555,
    ),
    (
        FpOp::EtoxM1,
        0x3fe0,
        0x8000000000000000,
        0x3fe0,
        0x8000000080000000,
    ),
    (
        FpOp::EtoxM1,
        0x3ffc,
        0xc000000000000000,
        0x3ffc,
        0xd32e05c2d60d4b54,
    ),
    (
        FpOp::EtoxM1,
        0x3fff,
        0x8000000000000000,
        0x3fff,
        0xdbf0a8b145769535,
    ),
    (
        FpOp::EtoxM1,
        0x4000,
        0xc000000000000000,
        0x4003,
        0x98af2dfb7d882f97,
    ),
    (
        FpOp::EtoxM1,
        0x4005,
        0xfa00000000000000,
        0x40b3,
        0xa1aab6f749df6f8e,
    ),
    (
        FpOp::TwoToX,
        0xc002,
        0xa000000000000000,
        0x3ff5,
        0x8000000000000000,
    ),
    (
        FpOp::TwoToX,
        0xbffe,
        0x8000000000000000,
        0x3ffe,
        0xb504f333f9de6484,
    ),
    (
        FpOp::TwoToX,
        0x3ff8,
        0x8000000000000000,
        0x3fff,
        0x80b1ed4fd999ab6c,
    ),
    (
        FpOp::TwoToX,
        0x3ffe,
        0x8000000000000000,
        0x3fff,
        0xb504f333f9de6484,
    ),
    (
        FpOp::TwoToX,
        0x3fff,
        0x8000000000000000,
        0x4000,
        0x8000000000000000,
    ),
    (
        FpOp::TwoToX,
        0x4000,
        0xc000000000000000,
        0x4002,
        0x8000000000000000,
    ),
    (
        FpOp::TwoToX,
        0x4005,
        0x8000000000000000,
        0x403f,
        0x8000000000000000,
    ),
    (
        FpOp::TwoToX,
        0x4008,
        0xfa00000000000000,
        0x43e7,
        0x8000000000000000,
    ),
    (
        FpOp::TenToX,
        0xc002,
        0xa000000000000000,
        0x3fdd,
        0xdbe6fecebdedd5bf,
    ),
    (
        FpOp::TenToX,
        0x3ffe,
        0x8000000000000000,
        0x4000,
        0xca62c1d6d2da9490,
    ),
    (
        FpOp::TenToX,
        0x3fff,
        0x8000000000000000,
        0x4002,
        0xa000000000000000,
    ),
    (
        FpOp::TenToX,
        0x4000,
        0xc000000000000000,
        0x4008,
        0xfa00000000000000,
    ),
    (
        FpOp::TenToX,
        0x4002,
        0xa000000000000000,
        0x4020,
        0x9502f90000000000,
    ),
    (
        FpOp::Logn,
        0x3ff0,
        0x8000000000000000,
        0xc002,
        0xa65af67854b28211,
    ),
    (
        FpOp::Logn,
        0x3ffe,
        0x8000000000000000,
        0xbffe,
        0xb17217f7d1cf79ac,
    ),
    (
        FpOp::Logn,
        0x3fff,
        0x8000000000000000,
        0x0000,
        0x0000000000000000,
    ),
    (
        FpOp::Logn,
        0x3fff,
        0xb504f333f9de6484,
        0x3ffd,
        0xb17217f7d1cf79ab,
    ),
    (
        FpOp::Logn,
        0x4000,
        0x8000000000000000,
        0x3ffe,
        0xb17217f7d1cf79ac,
    ),
    (
        FpOp::Logn,
        0x4002,
        0xa000000000000000,
        0x4000,
        0x935d8dddaaa8ac17,
    ),
    (
        FpOp::Logn,
        0x400c,
        0x9c40000000000000,
        0x4002,
        0x935d8dddaaa8ac17,
    ),
    (
        FpOp::Logn,
        0x4040,
        0x8000000000000000,
        0x4004,
        0xb437e057b116b792,
    ),
    (
        FpOp::LognP1,
        0xbffe,
        0x8000000000000000,
        0xbffe,
        0xb17217f7d1cf79ac,
    ),
    (
        FpOp::LognP1,
        0xbff0,
        0x8000000000000000,
        0xbff0,
        0x80008000aaabaaac,
    ),
    (
        FpOp::LognP1,
        0x3fe0,
        0x8000000000000000,
        0x3fdf,
        0xffffffff00000001,
    ),
    (
        FpOp::LognP1,
        0x3ffc,
        0xc000000000000000,
        0x3ffc,
        0xaff983853c9e9e44,
    ),
    (
        FpOp::LognP1,
        0x3fff,
        0x8000000000000000,
        0x3ffe,
        0xb17217f7d1cf79ac,
    ),
    (
        FpOp::LognP1,
        0x4000,
        0xc000000000000000,
        0x3fff,
        0xb17217f7d1cf79ac,
    ),
    (
        FpOp::LognP1,
        0x4005,
        0xfa00000000000000,
        0x4001,
        0x9ac2d24612fac83c,
    ),
    (
        FpOp::Log2,
        0x3ff0,
        0x8000000000000000,
        0xc002,
        0xf000000000000000,
    ),
    (
        FpOp::Log2,
        0x3ffe,
        0x8000000000000000,
        0xbfff,
        0x8000000000000000,
    ),
    (
        FpOp::Log2,
        0x3fff,
        0x8000000000000000,
        0x0000,
        0x0000000000000000,
    ),
    (
        FpOp::Log2,
        0x3fff,
        0xb504f333f9de6484,
        0x3ffd,
        0xffffffffffffffff,
    ),
    (
        FpOp::Log2,
        0x4000,
        0x8000000000000000,
        0x3fff,
        0x8000000000000000,
    ),
    (
        FpOp::Log2,
        0x4002,
        0xa000000000000000,
        0x4000,
        0xd49a784bcd1b8afe,
    ),
    (
        FpOp::Log2,
        0x400c,
        0x9c40000000000000,
        0x4002,
        0xd49a784bcd1b8afe,
    ),
    (
        FpOp::Log2,
        0x4040,
        0x8000000000000000,
        0x4005,
        0x8200000000000000,
    ),
    (
        FpOp::Log10,
        0x3ff0,
        0x8000000000000000,
        0xc001,
        0x907e90dcac12f81f,
    ),
    (
        FpOp::Log10,
        0x3ffe,
        0x8000000000000000,
        0xbffd,
        0x9a209a84fbcff799,
    ),
    (
        FpOp::Log10,
        0x3fff,
        0x8000000000000000,
        0x0000,
        0x0000000000000000,
    ),
    (
        FpOp::Log10,
        0x3fff,
        0xb504f333f9de6484,
        0x3ffc,
        0x9a209a84fbcff798,
    ),
    (
        FpOp::Log10,
        0x4000,
        0x8000000000000000,
        0x3ffd,
        0x9a209a84fbcff799,
    ),
    (
        FpOp::Log10,
        0x4002,
        0xa000000000000000,
        0x3fff,
        0x8000000000000000,
    ),
    (
        FpOp::Log10,
        0x400c,
        0x9c40000000000000,
        0x4001,
        0x8000000000000000,
    ),
    (
        FpOp::Log10,
        0x4040,
        0x8000000000000000,
        0x4003,
        0x9c891cef0fbf3777,
    ),
];

/// The opmode that names an operation, for assembling the command word.
fn opmode(op: FpOp) -> u16 {
    match op {
        FpOp::Sin => 0x0e,
        FpOp::Cos => 0x1d,
        FpOp::Tan => 0x0f,
        FpOp::Asin => 0x0c,
        FpOp::Acos => 0x1c,
        FpOp::Atan => 0x0a,
        FpOp::Sinh => 0x02,
        FpOp::Cosh => 0x19,
        FpOp::Tanh => 0x09,
        FpOp::Atanh => 0x0d,
        FpOp::Etox => 0x10,
        FpOp::EtoxM1 => 0x08,
        FpOp::TwoToX => 0x11,
        FpOp::TenToX => 0x12,
        FpOp::Logn => 0x14,
        FpOp::LognP1 => 0x06,
        FpOp::Log2 => 0x16,
        FpOp::Log10 => 0x15,
        other => panic!("{other:?} has no opmode here"),
    }
}

/// How many representable values apart two numbers are — the usual
/// unit-in-the-last-place distance, computed on the encodings, which are
/// ordered within a sign.
fn ulps_between(a: F80, b: F80) -> i128 {
    let key = |v: F80| -> i128 {
        let field = i128::from(v.exp_field());
        let sig = i128::from(v.sig);
        // A normal's significand runs over `[2^63, 2^64)`, so one step of the
        // exponent is `2^63` representable values and not `2^64`; counting it
        // as the encoding would makes every binade boundary look like an
        // enormous error.
        let magnitude = if field == 0 {
            sig
        } else {
            field * (1i128 << 63) + (sig - (1i128 << 63))
        };
        if v.sign() { -magnitude } else { magnitude }
    };
    key(a) - key(b)
}

#[test]
fn every_transcendental_is_correctly_rounded() {
    let b = Board::with_fpu(Model::M68030, Coprocessor::M68881);
    b.boot(&[0x4e71]);
    let mut wrong = alloc::vec::Vec::new();
    for &(op, field, sig, want_field, want_sig) in REFERENCE {
        let expected = F80::new(want_field, want_sig);
        b.with_regs(|r| {
            r.fp[1] = F80::new(field, sig);
            r.fp[0] = F80::ZERO;
            r.fpcr = 0;
            r.fpsr = 0;
        });
        // The general class: source FP1, destination FP0, the opmode.
        b.load(0x500, &[0xf200, (1 << 10) | opmode(op)]);
        b.at(0x500);
        b.cpu.step();
        let got = b.cpu.regs().fp[0];
        if got != expected {
            wrong.push(alloc::format!(
                "{op:?}({field:04x}:{sig:016x}) = {:04x}:{:016x}, {} ulp from \
                 {want_field:04x}:{want_sig:016x}",
                got.sign_exp,
                got.sig,
                ulps_between(got, expected)
            ));
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {} not correctly rounded:\n{}",
        wrong.len(),
        REFERENCE.len(),
        wrong.join("\n")
    );
}

#[test]
fn the_special_cases_are_the_operation_tables() {
    // Each of these is an entry in the instruction's own operation table in
    // M68881UM §4, or in the operand-error list of Table 6-2.
    let b = Board::with_fpu(Model::M68030, Coprocessor::M68881);
    b.boot(&[0x4e71]);
    let one = F80::new(0x3fff, 1 << 63);
    let minus_one = F80::new(0xbfff, 1 << 63);
    let zero = F80::ZERO;
    let minus_zero = F80::new(0x8000, 0);
    let inf = F80::new(0x7fff, 1 << 63);
    let minus_inf = F80::new(0xffff, 1 << 63);
    let nan = F80::new(0x7fff, u64::MAX);
    let pi_2 = F80::new(0x3fff, 0xc90f_daa2_2168_c235);
    let pi = F80::new(0x4000, 0xc90f_daa2_2168_c235);
    let two = F80::new(0x4000, 1 << 63);
    let minus_two = F80::new(0xc000, 1 << 63);

    let run = |op: FpOp, src: F80| -> (F80, u16) {
        b.with_regs(|r| {
            r.fp[1] = src;
            r.fp[0] = zero;
            r.fpsr = 0;
        });
        b.load(0x500, &[0xf200, (1 << 10) | opmode(op)]);
        b.at(0x500);
        b.cpu.step();
        let regs = b.cpu.regs();
        (regs.fp[0], (regs.fpsr & 0x0000_ff00) as u16)
    };
    const OPERR: u16 = 0x2000;
    const DZ: u16 = 0x0400;

    // Zeros keep their sign where the function is odd and give one where it
    // is even.
    assert_eq!(run(FpOp::Sin, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::Cos, minus_zero), (one, 0));
    assert_eq!(run(FpOp::Tan, zero), (zero, 0));
    assert_eq!(run(FpOp::Sinh, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::Cosh, minus_zero), (one, 0));
    assert_eq!(run(FpOp::Tanh, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::Atan, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::Asin, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::Atanh, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::EtoxM1, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::LognP1, minus_zero), (minus_zero, 0));
    assert_eq!(run(FpOp::Etox, zero), (one, 0));
    assert_eq!(run(FpOp::TwoToX, zero), (one, 0));
    assert_eq!(run(FpOp::TenToX, zero), (one, 0));
    // acos(0) is π/2, which is not representable, so it is inexact — "the
    // INEX2 bit in the FPSR may be set even if an exact result is produced"
    // (§4.3.2) and here the result genuinely is not one.
    assert_eq!(run(FpOp::Acos, zero).0, pi_2);

    // "Source is ±infinity" is an operand error for the trigonometric and
    // inverse trigonometric functions (Table 6-2).
    for op in [FpOp::Sin, FpOp::Cos, FpOp::Tan, FpOp::Asin, FpOp::Acos] {
        assert_eq!(run(op, inf), (nan, OPERR), "{op:?} of +infinity");
        assert_eq!(run(op, minus_inf), (nan, OPERR), "{op:?} of -infinity");
    }
    // The inverse ones also reject anything outside [-1, 1].
    for op in [FpOp::Asin, FpOp::Acos, FpOp::Atanh] {
        assert_eq!(run(op, two), (nan, OPERR), "{op:?} of two");
        assert_eq!(run(op, minus_two), (nan, OPERR), "{op:?} of minus two");
    }
    // The exponentials and the hyperbolics reject nothing.
    assert_eq!(run(FpOp::Etox, inf), (inf, 0));
    assert_eq!(run(FpOp::Etox, minus_inf), (zero, 0));
    assert_eq!(run(FpOp::Sinh, minus_inf), (minus_inf, 0));
    assert_eq!(run(FpOp::Cosh, minus_inf), (inf, 0));
    assert_eq!(run(FpOp::Tanh, minus_inf), (minus_one, 0));
    assert_eq!(run(FpOp::Atan, inf).0, pi_2);

    // The boundaries of the inverse functions are the ones the limits give.
    assert_eq!(run(FpOp::Asin, one).0, pi_2);
    assert_eq!(run(FpOp::Acos, one), (zero, 0));
    assert_eq!(run(FpOp::Acos, minus_one).0, pi);

    // A logarithm of zero is a divide by zero returning minus infinity, and
    // of a negative an operand error (§6.1.6, Table 6-2).
    for op in [FpOp::Logn, FpOp::Log2, FpOp::Log10] {
        assert_eq!(run(op, zero), (minus_inf, DZ), "{op:?} of zero");
        assert_eq!(run(op, minus_one), (nan, OPERR), "{op:?} of minus one");
        assert_eq!(run(op, inf), (inf, 0), "{op:?} of infinity");
    }
    assert_eq!(run(FpOp::LognP1, minus_one), (minus_inf, DZ));
    assert_eq!(run(FpOp::LognP1, minus_two), (nan, OPERR));
    // atanh(±1) is ±infinity, which §6.1.6 lists as a divide by zero.
    assert_eq!(run(FpOp::Atanh, one), (inf, DZ));
    assert_eq!(run(FpOp::Atanh, minus_one), (minus_inf, DZ));

    // A NaN goes straight through every one of them.
    for op in [FpOp::Sin, FpOp::Etox, FpOp::Logn, FpOp::Atan] {
        let payload = F80::new(0x7fff, 0xc000_0000_0000_1234);
        assert_eq!(run(op, payload), (payload, 0), "{op:?} of a NaN");
    }
}

#[test]
fn fsincos_writes_both_registers() {
    // M68881UM §4, *FSINCOS*: the sine goes to FPs and the cosine to FPc,
    // and "if FPc and FPs specify the same floating-point data register, the
    // sine result is stored in the register, and the cosine result is
    // discarded".
    let b = Board::with_fpu(Model::M68030, Coprocessor::M68881);
    b.boot(&[0x4e71]);
    let half = F80::new(0x3ffe, 1 << 63);
    b.with_regs(|r| r.fp[1] = half);
    // FSINCOS.X FP1,FP3:FP2.
    b.load(0x500, &[0xf200, (1 << 10) | (2 << 7) | 0x30 | 3]);
    b.at(0x500);
    b.cpu.step();
    let regs = b.cpu.regs();
    // The values the table above has already checked against bc.
    let sine = REFERENCE
        .iter()
        .find(|(op, f, s, _, _)| *op == FpOp::Sin && *f == 0x3ffe && *s == 1 << 63)
        .map(|&(_, _, _, f, s)| F80::new(f, s))
        .expect("a reference sine of a half");
    let cosine = REFERENCE
        .iter()
        .find(|(op, f, s, _, _)| *op == FpOp::Cos && *f == 0x3ffe && *s == 1 << 63)
        .map(|&(_, _, _, f, s)| F80::new(f, s))
        .expect("a reference cosine of a half");
    assert_eq!(regs.fp[2], sine);
    assert_eq!(regs.fp[3], cosine);
    // Naming one register for both leaves the sine.
    b.with_regs(|r| r.fp[1] = half);
    b.load(0x500, &[0xf200, (1 << 10) | (4 << 7) | 0x30 | 4]);
    b.at(0x500);
    b.cpu.step();
    assert_eq!(b.cpu.regs().fp[4], sine);
}

#[test]
fn a_huge_argument_is_reduced_exactly() {
    // "Large arguments may lose accuracy during reduction, and very large
    // arguments (greater than approximately 10^20) lose all accuracy"
    // (M68881UM §4, *FCOS*). This core's reduction is exact for every
    // representable argument, so `sin² + cos² = 1` holds at any magnitude —
    // a deviation from the part, and a documented one.
    let b = Board::with_fpu(Model::M68030, Coprocessor::M68881);
    b.boot(&[0x4e71]);
    let one = F80::new(0x3fff, 1 << 63);
    for field in [0x3fffu16 + 40, 0x3fff + 100, 0x3fff + 900, 0x7ffe] {
        let x = F80::new(field, 0x9876_5432_10fe_dcba);
        b.with_regs(|r| r.fp[1] = x);
        b.load(
            0x500,
            &[
                0xf200,
                (1 << 10) | (2 << 7) | 0x30 | 3, // FSINCOS FP1,FP3:FP2
                0xf200,
                (2 << 10) | (2 << 7) | 0x23, // FMUL FP2,FP2
                0xf200,
                (3 << 10) | (3 << 7) | 0x23, // FMUL FP3,FP3
                0xf200,
                (3 << 10) | (2 << 7) | 0x22, // FADD FP3,FP2
            ],
        );
        b.at(0x500);
        for _ in 0..4 {
            b.cpu.step();
        }
        // The sine and the cosine are each rounded to sixty-four bits, then
        // squared and added with a rounding apiece, so the identity holds to
        // within a couple of units in the last place rather than exactly — a
        // reduction that had gone wrong would be nowhere near.
        let distance = ulps_between(b.cpu.regs().fp[2], one).abs();
        assert!(
            distance <= 2,
            "2^{} lost the identity by {distance} ulp",
            i32::from(field) - 0x3fff
        );
    }
}
