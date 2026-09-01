//! Temporaries, their types, and immediate values.

use core::fmt;

/// The type of an IR temporary.
///
/// A real enum rather than the extensible-newtype pattern (CLAUDE.md, "Type
/// conventions"): exhaustiveness is genuinely wanted here, because every
/// backend must lower every type and a silently-unhandled one is a
/// miscompile rather than a missing feature.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Type {
    /// A one-bit value, held as 0 or 1.
    ///
    /// Not in `ROADMAP.md` §9's list, and required by the decision to make
    /// flags ordinary temporaries (see the module docs): a carry-in is one
    /// bit, `setcond` produces one bit, and `brcond` consumes one. Backends
    /// lower it to whatever the host uses for a boolean — a full register
    /// holding 0 or 1 on most, a flag on some — never to a bitfield within
    /// another temporary.
    I1,
    /// A 32-bit integer.
    I32,
    /// A 64-bit integer.
    I64,
    /// A 128-bit integer.
    ///
    /// Produced by the widening multiplies and consumed by the extract that
    /// takes their high half; not a general-purpose arithmetic type.
    I128,
    /// A 32-bit float, carried as bits.
    ///
    /// The IR never performs float arithmetic: tier-1 floating point is a
    /// helper call into soft-float (`ROADMAP.md` §9.1), because guest FP
    /// executed on host FP is not bit-reproducible across hosts. This type
    /// exists so a value can be *carried* — held in a register, spilled,
    /// passed to a helper — not so it can be added.
    F32,
    /// A 64-bit float, carried as bits. See [`Type::F32`].
    F64,
    /// A 128-bit vector.
    ///
    /// Reserved. `ROADMAP.md` §9 adds vector *ops* with the ARM/x86 SIMD work
    /// and not before; the type exists now so the shape of a helper call that
    /// returns one does not change later.
    V128,
}

impl Type {
    /// Width of a value of this type, in bits.
    #[inline]
    #[must_use]
    pub const fn bits(self) -> u32 {
        match self {
            Type::I1 => 1,
            Type::I32 | Type::F32 => 32,
            Type::I64 | Type::F64 => 64,
            Type::I128 | Type::V128 => 128,
        }
    }

    /// Whether this is one of the integer types.
    ///
    /// [`Type::I1`] counts: it is an integer one bit wide, and the arithmetic
    /// ops that take a carry take it as an integer.
    #[inline]
    #[must_use]
    pub const fn is_int(self) -> bool {
        matches!(self, Type::I1 | Type::I32 | Type::I64 | Type::I128)
    }

    /// Whether this is one of the float types.
    #[inline]
    #[must_use]
    pub const fn is_float(self) -> bool {
        matches!(self, Type::F32 | Type::F64)
    }
}

impl fmt::Display for Type {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Type::I1 => "i1",
            Type::I32 => "i32",
            Type::I64 => "i64",
            Type::I128 => "i128",
            Type::F32 => "f32",
            Type::F64 => "f64",
            Type::V128 => "v128",
        })
    }
}

/// An SSA temporary, numbered within its [`Block`](crate::ir::Block).
///
/// Assigned exactly once, which the verifier checks. The number is an index
/// into the block's type table, so a `Temp` is only meaningful alongside the
/// block that defined it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Temp(pub u32);

impl Temp {
    /// This temporary's index within its block.
    #[inline]
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

impl fmt::Display for Temp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "t{}", self.0)
    }
}

/// An immediate operand.
///
/// Integers are carried as `u128` whatever their type, and floats as **bits**
/// rather than as `f32`/`f64`. That is a determinism rule, not a convenience:
/// `ROADMAP.md` §0 forbids floats in anything that decides guest-visible
/// state, and a NaN payload that survived a host `f64` round-trip would be a
/// host-dependent constant baked into a translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Const {
    /// An integer immediate, zero-extended into the low bits.
    Int(u128),
    /// A 32-bit float immediate, as bits.
    F32Bits(u32),
    /// A 64-bit float immediate, as bits.
    F64Bits(u64),
}

impl Const {
    /// The immediate's bits, zero-extended to 128.
    #[inline]
    #[must_use]
    pub const fn bits(self) -> u128 {
        match self {
            Const::Int(v) => v,
            Const::F32Bits(v) => v as u128,
            Const::F64Bits(v) => v as u128,
        }
    }
}

impl fmt::Display for Const {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Const::Int(v) => write!(f, "{v:#x}"),
            Const::F32Bits(v) => write!(f, "f32:{v:#010x}"),
            Const::F64Bits(v) => write!(f, "f64:{v:#018x}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;

    #[test]
    fn every_type_reports_its_width() {
        assert_eq!(Type::I1.bits(), 1);
        assert_eq!(Type::I32.bits(), 32);
        assert_eq!(Type::V128.bits(), 128);
        assert!(Type::I1.is_int());
        assert!(!Type::I1.is_float());
        assert!(Type::F64.is_float());
        assert!(!Type::V128.is_int() && !Type::V128.is_float());
    }

    #[test]
    fn a_float_immediate_keeps_its_bits() {
        // The point of carrying bits: a signalling NaN payload survives, where
        // a host f64 round-trip may quietly canonicalise it.
        let snan = Const::F64Bits(0x7ff0_0000_0000_0001);
        assert_eq!(snan.bits(), 0x7ff0_0000_0000_0001);
        assert_eq!(format!("{snan}"), "f64:0x7ff0000000000001");
    }

    #[test]
    fn temps_display_as_they_index() {
        assert_eq!(format!("{}", Temp(7)), "t7");
        assert_eq!(Temp(7).index(), 7);
    }
}
