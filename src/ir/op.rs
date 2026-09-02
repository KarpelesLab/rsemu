//! Opcodes, and the descriptors that ride on them.

use crate::core::value::Width;
use core::fmt;

/// An IR opcode.
///
/// An **extensible enumeration** (CLAUDE.md, "Type conventions"): a
/// `#[repr(transparent)]` newtype with `pub const` variants, so a later
/// frontend can be given an op without that being a breaking change and
/// without an `unreachable!()` in any backend's lowering.
///
/// The numbering is grouped, and the groups are stable: a backend's dispatch
/// can range-test rather than match one op at a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct Opcode(pub u16);

impl Opcode {
    // ---- Data movement: 0x01..0x10 ----------------------------------------

    /// Copy a temporary, or materialize an immediate.
    pub const MOV: Opcode = Opcode(0x01);
    /// Sign-extend a narrower value into a wider type.
    pub const EXT_S: Opcode = Opcode(0x02);
    /// Zero-extend a narrower value into a wider type.
    pub const EXT_Z: Opcode = Opcode(0x03);
    /// Truncate a wider value into a narrower type.
    pub const TRUNC: Opcode = Opcode(0x04);
    /// Reverse byte order within each lane.
    ///
    /// Carries a lane width, because ARM's `REV16` and `REVSH` swap *within*
    /// halfword lanes rather than across the whole word; a whole-word swap is
    /// this op with a lane width equal to the type.
    pub const BSWAP: Opcode = Opcode(0x05);
    /// Insert a bitfield into a value.
    ///
    /// Load-bearing rather than decorative: x86's sub-register aliasing is
    /// exactly this. A lifter holding `EAX` as one temporary expresses every
    /// byte operand as a deposit or an extract — and `AH` is register number
    /// four, not one, so a lifter that skips this mis-handles it silently.
    pub const DEPOSIT: Opcode = Opcode(0x06);
    /// Read a bitfield out of a value.
    pub const EXTRACT: Opcode = Opcode(0x07);
    /// Read a piece of guest architectural state into a temporary.
    ///
    /// The slot number rides in [`Inst::aux`](crate::ir::Inst::aux).
    ///
    /// **Asymmetric on purpose: there is no `set_slot`.** A write to guest
    /// state is a *rebinding* — the slot maps to a new temporary from that
    /// point on, and the boundary marker records the mapping — which is what
    /// lets a value stay in a host register across several guest instructions
    /// instead of being stored and reloaded. Reads cannot work the same way,
    /// because a block's first use of a register has to come from somewhere,
    /// and `InsnStart::live` can only ever publish *outward*: the verifier
    /// requires the temporaries it names to be assigned already.
    ///
    /// Both the first frontend and the first backend independently invented a
    /// private convention for this before the op existed, which is the case
    /// for it being an op.
    ///
    /// Ordering: a pass may delete a `get_slot` nothing consumes, but may not
    /// move one across an [`Opcode::INSN_START`], which publishes live
    /// temporaries back into guest state and can therefore change what a slot
    /// holds.
    pub const GET_SLOT: Opcode = Opcode(0x08);

    // ---- Arithmetic: 0x10..0x20 -------------------------------------------

    /// Add.
    pub const ADD: Opcode = Opcode(0x10);
    /// Subtract.
    pub const SUB: Opcode = Opcode(0x11);
    /// Multiply, keeping the low half.
    pub const MUL: Opcode = Opcode(0x12);
    /// Signed divide.
    ///
    /// A primitive, not an ISA-level divide: every one of our guests defines
    /// divide-by-zero and signed overflow differently and none of them the way
    /// the host does, so a frontend wraps this in a guard or calls a helper.
    pub const DIV_S: Opcode = Opcode(0x13);
    /// Unsigned divide. See [`Opcode::DIV_S`].
    pub const DIV_U: Opcode = Opcode(0x14);
    /// Signed remainder.
    pub const REM_S: Opcode = Opcode(0x15);
    /// Unsigned remainder.
    pub const REM_U: Opcode = Opcode(0x16);
    /// Negate.
    pub const NEG: Opcode = Opcode(0x17);
    /// Add with a one-bit carry in, producing a result and a one-bit carry out.
    ///
    /// Not `add2`: that is a carry *chain* over a 2N-bit value in two N-bit
    /// temporaries, which is a different shape. This is `ADC`, and it is the
    /// only add the 6502 has.
    pub const ADDC: Opcode = Opcode(0x18);
    /// Subtract with a one-bit borrow in, producing a result and a borrow out.
    pub const SUBB: Opcode = Opcode(0x19);
    /// Unsigned widening multiply, producing the full double-width product.
    pub const MULU2: Opcode = Opcode(0x1a);
    /// Signed widening multiply, producing the full double-width product.
    pub const MULS2: Opcode = Opcode(0x1b);
    /// Signed-by-unsigned high multiply.
    ///
    /// RISC-V `mulhsu`, which neither widening multiply expresses.
    pub const MULHSU: Opcode = Opcode(0x1c);

    // ---- Logic and shifts: 0x20..0x30 -------------------------------------

    /// Bitwise and.
    pub const AND: Opcode = Opcode(0x20);
    /// Bitwise or.
    pub const OR: Opcode = Opcode(0x21);
    /// Bitwise exclusive or.
    pub const XOR: Opcode = Opcode(0x22);
    /// Bitwise complement.
    pub const NOT: Opcode = Opcode(0x23);
    /// `a & !b` — ARM's `BIC`.
    pub const ANDC: Opcode = Opcode(0x24);
    /// Shift left.
    ///
    /// **Undefined when the shift amount is at least the type's width**, which
    /// is what x86-64 and aarch64 give for free. Every guest disagrees about
    /// the out-of-range case — ARM's `LSL #32` yields zero with the carry from
    /// bit 0, the 386 masks the count to five bits, the 8086 does not mask at
    /// all, the m68k takes it modulo 64 — so each frontend emits its own
    /// guard. A shared default here would be wrong for all of them.
    pub const SHL: Opcode = Opcode(0x25);
    /// Logical shift right. Out-of-range amounts are undefined; see [`Opcode::SHL`].
    pub const SHR: Opcode = Opcode(0x26);
    /// Arithmetic shift right. Out-of-range amounts are undefined; see [`Opcode::SHL`].
    pub const SAR: Opcode = Opcode(0x27);
    /// Rotate left.
    pub const ROTL: Opcode = Opcode(0x28);
    /// Rotate right.
    pub const ROTR: Opcode = Opcode(0x29);
    /// Rotate left by one *through* a carry bit — an (N+1)-bit rotate.
    ///
    /// Takes a one-bit carry in and produces a one-bit carry out. Six of our
    /// nine ISAs need it, and on the 8-bit ones it is among the most common
    /// instructions on the chip.
    pub const ROTLC: Opcode = Opcode(0x2a);
    /// Rotate right by one through a carry bit. ARM's `RRX` is this op.
    pub const ROTRC: Opcode = Opcode(0x2b);

    // ---- Bit counting: 0x30..0x38 -----------------------------------------

    /// Count leading zeros. The result at zero input is the type's width.
    pub const CLZ: Opcode = Opcode(0x30);
    /// Count trailing zeros. The result at zero input is the type's width.
    pub const CTZ: Opcode = Opcode(0x31);
    /// Population count.
    ///
    /// On the flag path rather than exotic: it is x86's `PF` and the Z80's
    /// `P/V`, computed on nearly every arithmetic and logical instruction —
    /// and read almost never, which is the case for letting dead-code
    /// elimination see it.
    pub const POPCOUNT: Opcode = Opcode(0x32);

    // ---- Compare and branch: 0x38..0x40 -----------------------------------

    /// Materialize a condition as a one-bit value.
    pub const SETCOND: Opcode = Opcode(0x38);
    /// Select between two values on a condition.
    pub const MOVCOND: Opcode = Opcode(0x39);
    /// Branch within the block on a one-bit value.
    pub const BRCOND: Opcode = Opcode(0x3a);

    // ---- Memory: 0x40..0x48 -----------------------------------------------

    /// Load from guest memory, described by a [`MemOp`].
    pub const LD: Opcode = Opcode(0x40);
    /// Store to guest memory, described by a [`MemOp`].
    pub const ST: Opcode = Opcode(0x41);

    // ---- Atomics: 0x48..0x60 ----------------------------------------------

    /// Compare and exchange.
    pub const CMPXCHG: Opcode = Opcode(0x48);
    /// Exchange.
    pub const XCHG: Opcode = Opcode(0x49);
    /// Atomic add, returning the previous value.
    pub const FETCH_ADD: Opcode = Opcode(0x4a);
    /// Atomic and, returning the previous value.
    pub const FETCH_AND: Opcode = Opcode(0x4b);
    /// Atomic or, returning the previous value.
    pub const FETCH_OR: Opcode = Opcode(0x4c);
    /// Atomic exclusive or, returning the previous value.
    pub const FETCH_XOR: Opcode = Opcode(0x4d);
    /// Atomic signed minimum, returning the previous value.
    pub const FETCH_SMIN: Opcode = Opcode(0x4e);
    /// Atomic signed maximum, returning the previous value.
    pub const FETCH_SMAX: Opcode = Opcode(0x4f);
    /// Atomic unsigned minimum, returning the previous value.
    pub const FETCH_UMIN: Opcode = Opcode(0x50);
    /// Atomic unsigned maximum, returning the previous value.
    pub const FETCH_UMAX: Opcode = Opcode(0x51);
    /// A memory fence.
    pub const FENCE: Opcode = Opcode(0x52);
    /// Load-exclusive, taking a reservation.
    ///
    /// Not expressible as [`Opcode::CMPXCHG`]: a reservation can fail
    /// *because a trap happened in between*, and both cores that have one
    /// (RISC-V `LR`/`SC`, ARMv7-M `LDREX`/`STREX`) keep the monitor in
    /// architectural CPU state where a trap breaks it.
    pub const LD_EXCL: Opcode = Opcode(0x53);
    /// Store-exclusive, succeeding only if the reservation still holds.
    pub const ST_EXCL: Opcode = Opcode(0x54);

    // ---- Control and side effects: 0x60..0x70 -----------------------------

    /// Jump to a known successor block.
    pub const GOTO_TB: Opcode = Opcode(0x60);
    /// Leave the block, returning to the dispatcher.
    pub const EXIT_TB: Opcode = Opcode(0x61);
    /// Look a successor up by guest PC and jump to it.
    pub const LOOKUP_AND_GOTO: Opcode = Opcode(0x62);
    /// Call a helper.
    ///
    /// Where breadth lives. Every soft-float entry point returns a value *and*
    /// its exception flags, so a helper may produce two results.
    pub const CALL_HELPER: Opcode = Opcode(0x63);
    /// Charge guest ticks.
    ///
    /// The tick count is a hashed output rather than a budget (see the module
    /// docs), so it is explicit in the IR where the verifier can check it,
    /// instead of being a backend convention. A frontend emits the charges its
    /// interpreter makes, at the same points.
    pub const CHARGE: Opcode = Opcode(0x64);
    /// Mark a guest instruction boundary.
    ///
    /// Refers to an [`InsnStart`](crate::ir::InsnStart) in the block. Nothing
    /// may be reordered across one, which is what makes a fault deliverable at
    /// the architecturally correct PC with the architecturally correct tick
    /// count.
    pub const INSN_START: Opcode = Opcode(0x65);

    // ---- SSA glue: 0x70.. --------------------------------------------------

    /// An SSA phi. Required: superblocks span branches.
    pub const PHI: Opcode = Opcode(0x70);

    /// This opcode's mnemonic.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Opcode::MOV => "mov",
            Opcode::EXT_S => "ext_s",
            Opcode::EXT_Z => "ext_z",
            Opcode::TRUNC => "trunc",
            Opcode::BSWAP => "bswap",
            Opcode::DEPOSIT => "deposit",
            Opcode::EXTRACT => "extract",
            Opcode::GET_SLOT => "get_slot",
            Opcode::ADD => "add",
            Opcode::SUB => "sub",
            Opcode::MUL => "mul",
            Opcode::DIV_S => "div_s",
            Opcode::DIV_U => "div_u",
            Opcode::REM_S => "rem_s",
            Opcode::REM_U => "rem_u",
            Opcode::NEG => "neg",
            Opcode::ADDC => "addc",
            Opcode::SUBB => "subb",
            Opcode::MULU2 => "mulu2",
            Opcode::MULS2 => "muls2",
            Opcode::MULHSU => "mulhsu",
            Opcode::AND => "and",
            Opcode::OR => "or",
            Opcode::XOR => "xor",
            Opcode::NOT => "not",
            Opcode::ANDC => "andc",
            Opcode::SHL => "shl",
            Opcode::SHR => "shr",
            Opcode::SAR => "sar",
            Opcode::ROTL => "rotl",
            Opcode::ROTR => "rotr",
            Opcode::ROTLC => "rotlc",
            Opcode::ROTRC => "rotrc",
            Opcode::CLZ => "clz",
            Opcode::CTZ => "ctz",
            Opcode::POPCOUNT => "popcount",
            Opcode::SETCOND => "setcond",
            Opcode::MOVCOND => "movcond",
            Opcode::BRCOND => "brcond",
            Opcode::LD => "ld",
            Opcode::ST => "st",
            Opcode::CMPXCHG => "cmpxchg",
            Opcode::XCHG => "xchg",
            Opcode::FETCH_ADD => "fetch_add",
            Opcode::FETCH_AND => "fetch_and",
            Opcode::FETCH_OR => "fetch_or",
            Opcode::FETCH_XOR => "fetch_xor",
            Opcode::FETCH_SMIN => "fetch_smin",
            Opcode::FETCH_SMAX => "fetch_smax",
            Opcode::FETCH_UMIN => "fetch_umin",
            Opcode::FETCH_UMAX => "fetch_umax",
            Opcode::FENCE => "fence",
            Opcode::LD_EXCL => "ld_excl",
            Opcode::ST_EXCL => "st_excl",
            Opcode::GOTO_TB => "goto_tb",
            Opcode::EXIT_TB => "exit_tb",
            Opcode::LOOKUP_AND_GOTO => "lookup_and_goto",
            Opcode::CALL_HELPER => "call_helper",
            Opcode::CHARGE => "charge",
            Opcode::INSN_START => "insn_start",
            Opcode::PHI => "phi",
            _ => "unknown",
        }
    }

    /// Whether this op ends a block.
    #[inline]
    #[must_use]
    pub fn is_terminator(self) -> bool {
        matches!(
            self,
            Opcode::GOTO_TB | Opcode::EXIT_TB | Opcode::LOOKUP_AND_GOTO
        )
    }

    /// Whether this op has an effect the optimiser may not remove.
    ///
    /// A store, an atomic, a helper, a charge and a boundary marker all stay
    /// even when nothing consumes their result. A load does **not** appear
    /// here — its own [`MemOp::volatile`] decides, because a 6502 dummy read
    /// is a real bus cycle whose value is discarded and DCE would otherwise
    /// eat it.
    #[inline]
    #[must_use]
    pub fn has_side_effect(self) -> bool {
        matches!(
            self,
            Opcode::ST
                | Opcode::CMPXCHG
                | Opcode::XCHG
                | Opcode::FENCE
                | Opcode::ST_EXCL
                | Opcode::LD_EXCL
                | Opcode::CALL_HELPER
                | Opcode::CHARGE
                | Opcode::INSN_START
        ) || (self.0 >= Opcode::FETCH_ADD.0 && self.0 <= Opcode::FETCH_UMAX.0)
    }
}

impl fmt::Display for Opcode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Pack a bitfield position and length into an instruction's `aux`.
///
/// [`Opcode::DEPOSIT`] and [`Opcode::EXTRACT`] need two numbers and `Inst` has
/// one `u32` to spare, so the two are packed. This lives here rather than in
/// any one backend because a convention every backend must independently agree
/// to is not a convention — it is a divergence waiting to happen.
#[inline]
#[must_use]
pub const fn bitfield_aux(pos: u32, len: u32) -> u32 {
    (pos & 0xffff) | ((len & 0xffff) << 16)
}

/// Unpack what [`bitfield_aux`] packed, as `(position, length)`.
#[inline]
#[must_use]
pub const fn bitfield_parts(aux: u32) -> (u32, u32) {
    (aux & 0xffff, (aux >> 16) & 0xffff)
}

/// A comparison, for [`Opcode::SETCOND`], [`Opcode::MOVCOND`] and
/// [`Opcode::BRCOND`].
///
/// The guests' own four-bit condition codes are not represented here: ARM's
/// `LE` is `Z | (N != V)` over flag temporaries, which is one or two of these
/// rather than a dedicated encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Cond {
    /// Equal.
    Eq,
    /// Not equal.
    Ne,
    /// Signed less than.
    LtS,
    /// Signed less than or equal.
    LeS,
    /// Signed greater than.
    GtS,
    /// Signed greater than or equal.
    GeS,
    /// Unsigned less than.
    LtU,
    /// Unsigned less than or equal.
    LeU,
    /// Unsigned greater than.
    GtU,
    /// Unsigned greater than or equal.
    GeU,
}

impl Cond {
    /// The condition that holds exactly when this one does not.
    ///
    /// Needed the moment a frontend builds a **side exit**: a superblock
    /// inlines one side of a branch and leaves the other as an
    /// [`Opcode::BRCOND`] that jumps *over* the exit sequence, which tests the
    /// negation of the branch the guest wrote. Every condition here has its
    /// negation in the same set — that is why the set has ten members rather
    /// than the six a comparison needs — so this is total and needs no
    /// `swap the operands` fallback that would break a frontend's operand
    /// order.
    #[inline]
    #[must_use]
    pub const fn invert(self) -> Cond {
        match self {
            Cond::Eq => Cond::Ne,
            Cond::Ne => Cond::Eq,
            Cond::LtS => Cond::GeS,
            Cond::GeS => Cond::LtS,
            Cond::LeS => Cond::GtS,
            Cond::GtS => Cond::LeS,
            Cond::LtU => Cond::GeU,
            Cond::GeU => Cond::LtU,
            Cond::LeU => Cond::GtU,
            Cond::GtU => Cond::LeU,
        }
    }
}

/// Whether a load sign-extends.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Sign {
    /// Zero-extend into the destination type.
    Unsigned,
    /// Sign-extend into the destination type.
    Signed,
}

/// Which address space an access reaches.
///
/// Not every machine has one memory space: the Z80 keeps a separate 64 KiB
/// I/O space that `IN`/`OUT` reach and ordinary loads do not, and x86 the
/// same. A machine that wires only the first gets a floating bus on every
/// port rather than a fault storm, so the distinction is real and belongs in
/// the op.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct MemSpace(pub u8);

impl MemSpace {
    /// The core's ordinary memory space.
    pub const MEM: MemSpace = MemSpace(0);
    /// A separate I/O space, where the machine has one.
    pub const IO: MemSpace = MemSpace(1);
}

/// An x86 segment register, by its architectural number.
///
/// Carried rather than folded into the address, because the descriptor is
/// hidden state a `MOV DS,ax` between two instructions can change, and
/// because the fault differs by register: a stack-segment violation raises
/// `#SS` where every other segment raises `#GP`. In real mode the fold is
/// legal and a frontend may do it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct SegId(pub u8);

/// The byte order of an access.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Endian {
    /// Little-endian.
    Little,
    /// Big-endian.
    Big,
    /// Whatever the region answering this address says.
    ///
    /// Not a hedge: ARM resolves byte order against the address space at the
    /// *physical* address at run time, and MIPS decides it per access from a
    /// status bit crossed with privilege. Resolved through the software-TLB
    /// entry rather than baked into the translation.
    AsRegion,
}

/// What an access does when it is not naturally aligned.
///
/// Four distinct behaviours across our cores, and on two of them the choice is
/// a run-time control-register bit rather than a property of the ISA.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Align {
    /// No alignment requirement.
    None,
    /// Raise an alignment fault, before translation.
    Fault,
    /// Split into aligned pieces, translating every piece before writing any.
    ///
    /// x86's rule, and the reason it is a policy rather than a loop: a fault
    /// on the second page must not leave the first half written.
    Split,
    /// Rotate the loaded value, as ARMv5 does by default.
    Rotate,
}

/// What the access is for.
///
/// The software TLB is split by access type, and MIPS picks its exception code
/// family from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AccessKind {
    /// An instruction fetch.
    Fetch,
    /// A data load.
    Load,
    /// A data store.
    Store,
}

/// Everything a [`Opcode::LD`] or [`Opcode::ST`] needs to know.
///
/// `ROADMAP.md` §9 sketches `{ size, sign, endianness, alignment, index }`.
/// Surveying our nine cores found that short by several fields, and found two
/// of the original five to be run-time rather than static. What is *not* here
/// is the rest of `MemAttrs` — `privileged`, `exclusive`, `bus`, `core_bus`,
/// `requester` — because those come from CPU state at execution time, and two
/// of them (the 6502's open-bus latches) change on nearly every access.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MemOp {
    /// The access width.
    pub size: Width,
    /// Whether a load sign-extends.
    pub sign: Sign,
    /// Which address space to reach.
    pub space: MemSpace,
    /// The segment, on a guest that has them.
    pub seg: Option<SegId>,
    /// Byte order.
    pub endian: Endian,
    /// The misalignment policy.
    pub align: Align,
    /// What the access is for.
    pub kind: AccessKind,
    /// Whether the access must survive dead-code elimination and reordering.
    ///
    /// True for every MMIO access, and for the reads whose *value* is
    /// discarded but whose bus cycle is not: the 6502's dummy reads — the
    /// internal cycle `PLA`, `RTS`, `RTI` and `JSR` spend — and its index
    /// fix-up read, which on the NMOS part lands on the unfixed address,
    /// which is why `STA $20ff,X` touches `$2000`-page hardware.
    pub volatile: bool,
}

impl MemOp {
    /// A plain little-endian load of `size` from the ordinary memory space.
    #[must_use]
    pub const fn load(size: Width) -> MemOp {
        MemOp {
            size,
            sign: Sign::Unsigned,
            space: MemSpace::MEM,
            seg: None,
            endian: Endian::Little,
            align: Align::None,
            kind: AccessKind::Load,
            volatile: false,
        }
    }

    /// A plain little-endian store of `size` to the ordinary memory space.
    #[must_use]
    pub const fn store(size: Width) -> MemOp {
        MemOp {
            kind: AccessKind::Store,
            ..MemOp::load(size)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn opcodes_name_themselves() {
        assert_eq!(Opcode::ADDC.name(), "addc");
        assert_eq!(Opcode::LOOKUP_AND_GOTO.name(), "lookup_and_goto");
        // An op from outside the defined set is not a panic: the whole point
        // of the extensible-enum pattern is that a later frontend can carry
        // one through code that predates it.
        assert_eq!(Opcode(0xfff).name(), "unknown");
    }

    #[test]
    fn terminators_and_effects_are_disjoint_from_pure_arithmetic() {
        assert!(Opcode::EXIT_TB.is_terminator());
        assert!(!Opcode::ADD.is_terminator());
        assert!(!Opcode::ADD.has_side_effect());
        assert!(Opcode::ST.has_side_effect());
        assert!(Opcode::CHARGE.has_side_effect());
        assert!(Opcode::INSN_START.has_side_effect());
        // Every atomic read-modify-write, by range rather than one at a time.
        for op in [
            Opcode::FETCH_ADD,
            Opcode::FETCH_XOR,
            Opcode::FETCH_SMIN,
            Opcode::FETCH_UMAX,
        ] {
            assert!(op.has_side_effect(), "{op} must not be eliminable");
        }
        // A plain load is eliminable; a volatile one is the MemOp's business.
        assert!(!Opcode::LD.has_side_effect());
    }

    #[test]
    fn every_condition_has_its_negation_in_the_set() {
        // A side exit branches over itself on the *negation* of the guest's
        // branch, so a condition whose negation were missing would force a
        // frontend to swap the operands and get the signedness wrong.
        for cond in [
            Cond::Eq,
            Cond::Ne,
            Cond::LtS,
            Cond::LeS,
            Cond::GtS,
            Cond::GeS,
            Cond::LtU,
            Cond::LeU,
            Cond::GtU,
            Cond::GeU,
        ] {
            assert_ne!(cond.invert(), cond);
            assert_eq!(cond.invert().invert(), cond);
        }
        assert_eq!(Cond::LtS.invert(), Cond::GeS);
        assert_eq!(Cond::GtU.invert(), Cond::LeU);
    }

    #[test]
    fn a_store_descriptor_differs_from_a_load_only_in_kind() {
        let ld = MemOp::load(Width::U16);
        let st = MemOp::store(Width::U16);
        assert_eq!(ld.kind, AccessKind::Load);
        assert_eq!(st.kind, AccessKind::Store);
        assert_eq!(ld.size, st.size);
        assert!(!ld.volatile);
    }
}
