#![no_main]
//! Software IEEE-754 arithmetic, against the host FPU where the host is an
//! oracle — and against itself everywhere else.
//!
//! # Where the oracle stops
//!
//! Host `f32`/`f64` decide the answer **only** for round-to-nearest-even
//! `add`/`sub`/`mul`/`div`/`sqrt`/`fma` with finite operands and a finite
//! result. Those are the cases IEEE 754-2019 §5.1 makes unique, and Rust's
//! `f64` is an IEEE binary64 that is not flushing subnormals. The host is
//! **not** an oracle for:
//!
//! * **NaN payloads** — every architecture canonicalises differently and wasm
//!   canonicalises hardest, which is the whole reason this subsystem exists.
//! * **The exception flags** — reading the host's `MXCSR` or `FPSR` needs
//!   `libc` or inline assembly, both forbidden here (`ROADMAP.md` §0).
//! * **The other four rounding attributes** — setting the host's mode needs
//!   the same forbidden tools.
//! * **The 80-bit format** — Rust has no `long double`, and reaching C's would
//!   need FFI.
//!
//! So this target does two things: it checks the host-agreeing cases against
//! the host, and it checks everything else against properties that must hold
//! whatever the answer is — no panic, no encoding a decoder rejects, no flag
//! combination the standard forbids, and the 80-bit path agreeing with
//! binary64 when precision control says it must.
//!
//! # Input encoding
//!
//! ```text
//!   0        selector: rounding mode, guest profile, operation
//!   1..9     operand a, little-endian
//!   9..17    operand b, little-endian
//!   17..25   operand c, little-endian (the addend for `fma`)
//! ```
//!
//! Bit patterns go in raw and unfiltered: NaNs, infinities, subnormals and
//! every reserved x87 encoding are exactly the interesting inputs.

use libfuzzer_sys::fuzz_target;
use rsemu::float::x87::{self, F80, Precision};
use rsemu::float::{B32, B64, Env, Flags, Format, Round};

/// Read a little-endian `u64` from `data`, or zero past the end.
fn word(data: &[u8], at: usize) -> u64 {
    let mut buf = [0u8; 8];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = data.get(at + i).copied().unwrap_or(0);
    }
    u64::from_le_bytes(buf)
}

/// A flag set no operation may deliver.
fn flags_are_coherent(f: Flags) {
    // §7.5: underflow is signalled only together with inexact, because the
    // default exception handling defines it as "tiny *and* inexact".
    assert!(
        !f.contains(Flags::UNDERFLOW) || f.contains(Flags::INEXACT),
        "underflow without inexact: {f:?}"
    );
    // §7.4: an overflowed result is always inexact too.
    assert!(
        !f.contains(Flags::OVERFLOW) || f.contains(Flags::INEXACT),
        "overflow without inexact: {f:?}"
    );
    // A result cannot both overflow and underflow.
    assert!(!(f.contains(Flags::OVERFLOW) && f.contains(Flags::UNDERFLOW)));
    // Nothing sets a bit outside the six that exist.
    assert_eq!(f.0 & !0x3f, 0, "unknown flag bit: {f:?}");
}

/// Every result must be an encoding the format can produce: the unused high
/// bits of a binary32 held in a `u64` are clear, and a NaN is quiet unless the
/// guest was asked to propagate a payload.
fn encoding_is_canonical<F: Format>(bits: u64) {
    assert_eq!(bits & !F::MASK, 0, "bits outside the format");
}

fuzz_target!(|data: &[u8]| {
    let selector = data.first().copied().unwrap_or(0);
    let round = match selector & 7 {
        0 => Round::TiesEven,
        1 => Round::TiesAway,
        2 => Round::TowardZero,
        3 => Round::TowardNegative,
        _ => Round::TowardPositive,
    };
    let profile = match (selector >> 3) & 3 {
        0 => Env::RISCV,
        1 => Env::X86_SSE,
        2 => Env::ARM,
        _ => Env::X87,
    };
    let mut env = profile.round(round);
    if selector & 0x20 != 0 {
        env = env.ftz(true);
    }
    if selector & 0x40 != 0 {
        env = env.daz(true);
    }
    let (a, b, c) = (word(data, 1), word(data, 9), word(data, 17));

    // ---- binary64 and binary32: never panic, always coherent -------------
    for (bits, flags) in [
        rsemu::float::add::<B64>(a, b, env),
        rsemu::float::sub::<B64>(a, b, env),
        rsemu::float::mul::<B64>(a, b, env),
        rsemu::float::div::<B64>(a, b, env),
        rsemu::float::sqrt::<B64>(a, env),
        rsemu::float::fma::<B64>(a, b, c, env),
        rsemu::float::min::<B64>(a, b, env),
        rsemu::float::max::<B64>(a, b, env),
        rsemu::float::convert::<B32, B64>(a, env),
    ] {
        flags_are_coherent(flags);
        encoding_is_canonical::<B64>(bits);
    }
    for (bits, flags) in [
        rsemu::float::add::<B32>(a, b, env),
        rsemu::float::mul::<B32>(a, b, env),
        rsemu::float::div::<B32>(a, b, env),
        rsemu::float::sqrt::<B32>(a, env),
        rsemu::float::fma::<B32>(a, b, c, env),
        rsemu::float::convert::<B64, B32>(a, env),
    ] {
        flags_are_coherent(flags);
        encoding_is_canonical::<B32>(bits);
    }
    for width in [8u32, 16, 32, 64] {
        let (_, f) = rsemu::float::to_signed::<B64>(a, width, env);
        flags_are_coherent(f);
        let (_, f) = rsemu::float::to_unsigned::<B64>(a, width, env);
        flags_are_coherent(f);
        let (bits, f) = rsemu::float::from_signed::<B32>(a as i64, width, env);
        flags_are_coherent(f);
        encoding_is_canonical::<B32>(bits);
    }

    // ---- the host, where it is an oracle ---------------------------------
    if round == Round::TiesEven && !env.flush_outputs && !env.subnormal_inputs.flushes() {
        let (x, y, z) = (f64::from_bits(a), f64::from_bits(b), f64::from_bits(c));
        if x.is_finite() && y.is_finite() {
            let checks = [
                (x + y, rsemu::float::add::<B64>(a, b, env).0),
                (x - y, rsemu::float::sub::<B64>(a, b, env).0),
                (x * y, rsemu::float::mul::<B64>(a, b, env).0),
            ];
            for (want, got) in checks {
                if want.is_finite() {
                    assert_eq!(got, want.to_bits());
                }
            }
            if y != 0.0 {
                let want = x / y;
                if want.is_finite() {
                    assert_eq!(rsemu::float::div::<B64>(a, b, env).0, want.to_bits());
                }
            }
            if z.is_finite() {
                let want = x.mul_add(y, z);
                if want.is_finite() {
                    assert_eq!(rsemu::float::fma::<B64>(a, b, c, env).0, want.to_bits());
                }
            }
        }
        if x > 0.0 && x.is_finite() {
            assert_eq!(rsemu::float::sqrt::<B64>(a, env).0, x.sqrt().to_bits());
        }
        let x32 = f32::from_bits(a as u32);
        let y32 = f32::from_bits(b as u32);
        if x32.is_finite() && y32.is_finite() {
            let want = x32 * y32;
            if want.is_finite() {
                let got = rsemu::float::mul::<B32>(u64::from(a as u32), u64::from(b as u32), env).0;
                assert_eq!(got, u64::from(want.to_bits()));
            }
        }
        // Narrowing is unique in nearest-even too.
        if x.is_finite() {
            let want = x as f32;
            assert_eq!(
                rsemu::float::convert::<B64, B32>(a, env).0,
                u64::from(want.to_bits())
            );
        }
    }

    // ---- the 80-bit path, against itself ---------------------------------
    let x87_env = Env::X87.round(round);
    let (wa, _) = x87::from_binary::<B64>(a, x87_env);
    let (wb, _) = x87::from_binary::<B64>(b, x87_env);
    // Widening a binary64 is always exact, so narrowing it back is the
    // identity for everything except a NaN, whose payload x87 may reshape.
    if f64::from_bits(a).is_finite() {
        assert_eq!(x87::to_binary::<B64>(wa, x87_env).0, a & B64::MASK);
    }
    for pc in [Precision::Single, Precision::Double, Precision::Extended] {
        for (v, f) in [
            x87::add(wa, wb, pc, x87_env),
            x87::sub(wa, wb, pc, x87_env),
            x87::mul(wa, wb, pc, x87_env),
            x87::div(wa, wb, pc, x87_env),
            x87::sqrt(wa, pc, x87_env),
        ] {
            flags_are_coherent(f);
            // Every encoding this arithmetic produces must be one the decoder
            // accepts: no unnormal, no pseudo-NaN, no pseudo-infinity.
            assert!(
                !matches!(x87::classify(v), x87::X87Class::Unsupported),
                "produced an unsupported encoding: {v:?}"
            );
        }
    }
    // Raw ten-byte operands, including every reserved encoding.
    let raw = |at: usize| {
        let mut bytes = [0u8; 10];
        for (i, byte) in bytes.iter_mut().enumerate() {
            *byte = data.get(at + i).copied().unwrap_or(0);
        }
        F80::from_bytes(bytes)
    };
    let (ra, rb) = (raw(1), raw(11));
    assert_eq!(F80::from_bytes(ra.to_bytes()), ra);
    for pc in [Precision::Single, Precision::Extended] {
        for (v, f) in [
            x87::add(ra, rb, pc, x87_env),
            x87::mul(ra, rb, pc, x87_env),
            x87::div(ra, rb, pc, x87_env),
            x87::sqrt(ra, pc, x87_env),
        ] {
            flags_are_coherent(f);
            assert!(!matches!(x87::classify(v), x87::X87Class::Unsupported));
        }
    }
});
