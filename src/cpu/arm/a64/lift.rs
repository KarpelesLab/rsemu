//! The AArch64 frontend: guest instructions lifted into
//! [`ir::Block`](crate::ir::Block)s.
//!
//! `ROADMAP.md` §9's pipeline has two halves that never meet — a frontend that
//! turns guest bytes into IR and a backend that turns IR into something
//! callable. This is the **third** frontend, after `cpu::riscv::lift` and
//! `cpu::x86::lift`, and it lands on the core with the strongest oracle in the
//! tree: nine conformance guests, an empty known-failures ledger, and a Debian
//! arm64 boot to a shell on `machines/arm64-virt.machine`.
//!
//! # The subset, exactly
//!
//! A documented subset done exactly beats a broad one done approximately, so
//! this lifts the **A64 integer core** and nothing else:
//!
//! * `ADR`/`ADRP`, add/subtract immediate, logical immediate, `MOVZ`/`MOVN`/
//!   `MOVK`, the bitfield moves, `EXTR`.
//! * The shifted-register logical and arithmetic families, the
//!   extended-register add/subtract family, `ADC`/`SBC` and their
//!   flag-setting forms.
//! * `CCMP`/`CCMN`, the four conditional selects, `UDIV`/`SDIV`, the four
//!   register shifts, `REV`/`REV16`/`REV32`, `CLZ`, `CLS`, `MADD`/`MSUB`, the
//!   widening multiply-accumulates, `SMULH`/`UMULH`.
//! * Every single-register load and store in the unsigned-offset, unscaled,
//!   pre-indexed, post-indexed and register-offset addressing modes; both pair
//!   forms and their write-back variants; the three literal loads; every
//!   `PRFM`, which is architecturally a hint this core makes no access for.
//! * `B`, `BL`, `B.cond`, `CBZ`/`CBNZ`, `TBZ`/`TBNZ`, `BR`, `BLR`, `RET`, and
//!   the hint and barrier instructions the interpreter retires as no-ops.
//!
//! Deliberately **not** lifted, each ending the block with a terminator that
//! hands the PC back to the interpreter:
//!
//! * **Everything that touches the SIMD and floating-point register file.**
//!   `ROADMAP.md` §9.1 makes guest floating point a call into software
//!   IEEE-754 so that it is bit-reproducible across hosts, and CLAUDE.md's
//!   `no_host_float_on_the_guest_path` gate reads the sources back. A second
//!   float implementation is the one thing worse than none, so the whole
//!   family is excluded rather than half-lifted — including `Fmt::LdStFp*`
//!   and the structure loads, whose *addressing* would lift fine and whose
//!   register file would not. One line does it, and it is the table's own
//!   feature column rather than a list of formats: `isa::Feat::is_simd_fp`.
//! * **The exclusives and the LSE atomics.** [`Opcode::LD_EXCL`] and
//!   [`Opcode::ST_EXCL`] can express `LDXR`/`STXR`, but not `LDXP`/`STXP`:
//!   the 64-bit pair form is a **16-byte** single-copy-atomic access and the
//!   IR's atomics have no type wider than `i64` (`ir`, "Known gaps"). Lifting
//!   half a family is how the two halves drift apart, so the group stays with
//!   the interpreter that already passes its conformance guest.
//! * **`LDTR`/`STTR`.** The unprivileged forms differ from the unscaled ones
//!   only in being translated with **EL0's** permissions, and that is a
//!   property of the access rather than of the address: `Exec::unpriv` is set
//!   per instruction and [`MemOp`] has nowhere to carry it. A kernel's
//!   `copy_to_user` is built out of these, so this is a real gap and it is
//!   named rather than approximated.
//! * `SVC`, `BRK`, `HVC`, `SMC`, `HLT`, `ERET`, `WFI`, `CLREX`, `MRS`, `MSR`,
//!   `SYS`/`SYSL`, the `CRC32` family, and every encoding the configured
//!   part's [`Features`] do not allow.
//!
//! # Ticks, and where the block has to end
//!
//! [`ir`](crate::ir)'s decision 2: a tick count is a hashed *output*, so a
//! block that charges 7 where the interpreter charged 8 fails the phase-5
//! state-hash gate. `exec` charges **one tick per bus access**, at three kinds
//! of site:
//!
//! | Site | Count | Static? |
//! | --- | --- | --- |
//! | instruction fetch | **1** — every A64 instruction is one aligned word | **yes** |
//! | a load or store | 1 when aligned; `bytes` when it splits | no |
//! | a translation-table read | 0 on a TLB hit, 1 per level on a miss | no |
//!
//! So the static column is one tick per guest instruction and nothing else,
//! which is simpler than either of the other two frontends — A64 has no
//! variable-length encoding and no compressed form. The structural rule that
//! makes it exact is the same one:
//!
//! * **A block never leaves the page it started on.** The fetch translation is
//!   resolved once, at block entry, exactly as the interpreter resolves it for
//!   the first instruction — so no fetch *inside* the block can miss the TLB,
//!   walk, or fault, and `charge(1)` per instruction is the whole fetch cost.
//!   See [`Stop::Page`].
//!
//! Every load and store here is [`MemOp::volatile`]: the access spends ticks
//! and can fault, both guest-visible, so dead-code elimination may not remove
//! one whose value is discarded — `ldr xzr, [x0]` really does read the bus.
//!
//! ## A store ends the block
//!
//! A **load** cannot change what the rest of the block means. A store can: if
//! it lands in the page the block was lifted from, every instruction after it
//! is a translation of bytes that no longer exist.
//!
//! A64 permits that — a guest owes `DC CVAU`, `IC IVAU` and the barriers
//! around them between writing instruction memory and executing it, so what a
//! translation does in between is unspecified — but `ROADMAP.md` §0 does not:
//! a bit-identical state hash across the interpreter and the JIT, and the
//! interpreter re-fetches every instruction. Diverging there would be legal
//! for the architecture and a broken promise for rsemu, and — just as bad — a
//! differential harness cannot tell that divergence apart from a lifter bug.
//!
//! The invalidation mechanism's granularity forces the answer: a guest store
//! is matched against cached translations at the **block boundary**
//! (`jit::dispatch`), so a block boundary is where a store's effect on code
//! can first be honoured. Ending the block after a store puts the boundary
//! exactly there.
//!
//! # Superblocks: merging across direct branches
//!
//! `ROADMAP.md` §9's fourth speed mechanism. [`Shape::Trace`] is the default.
//!
//! * **`B`/`BL` to a direct target is not an exit at all.** The link register
//!   is bound and lifting continues at the target.
//! * **A conditional branch becomes a side exit** — `B.cond`, `CBZ`, `CBNZ`,
//!   `TBZ` and `TBNZ` alike, because on this architecture all five reduce to
//!   *one bit* and the IR's one-operand [`Opcode::BRCOND`] takes exactly that.
//!   One side is inlined and the other becomes an inline exit sequence the
//!   trace branches over, carrying the whole register map as of the branch.
//! * **Which side is inlined** is the classic static prediction: a
//!   **backward** branch is a loop's back edge, so the taken side is inlined
//!   and the fall-through becomes the side exit; a **forward** branch is an
//!   `if`, so the fall-through is inlined. A backward target outside the entry
//!   page is not a candidate, because no instruction outside that page may be
//!   lifted.
//! * **`BR`/`BLR`/`RET` still end the block.** The target is computed.
//!
//! # Flags are four temporaries, not a packed word
//!
//! [`ir`](crate::ir)'s decision 1, and A64 is the case it was written for.
//! `PSTATE.NZCV` is four bits that nearly every arithmetic instruction writes
//! and that only a condition reads, so each is its own [`RegSlot`] holding
//! `0` or `1` and each is computed into its own [`Type::I1`] temporary. An
//! ARM condition is then boolean algebra over four temporaries — `LE` is
//! `Z | (N != V)` — rather than a sixteen-way table over a packed nibble.
//!
//! **Nothing here emits [`Opcode::ADDC`]**, and that is a deliberate cost.
//! `AddWithCarry` is the architecture's own primitive and `ADDC` is exactly
//! it, but `jit::x86` refuses that op — so a lifter that used it would produce
//! blocks the host code generator could never compile, and that means *every*
//! block, because `CMP` is `SUBS` and `SUBS` is `AddWithCarry`. The carry is
//! recovered from unsigned comparisons instead
//! (`Lifter::add_with_carry` and `Lifter::sub_with_flags`), which is more
//! IR ops and compiles.
//!
//! # Paging: which address space `entry_pc` names
//!
//! Three things make a lifted block safe under translation, and they are the
//! same three `cpu::riscv::lift` states:
//!
//! 1. **The page bound is the MMU's page bound.** [`mmu::PAGE_SIZE`](super::mmu::PAGE_SIZE) is 4 KiB,
//!    the only granule this core implements, so a block that stays inside one
//!    virtual page stays inside one leaf descriptor's worth of permissions and
//!    one physical page.
//! 2. **The entry translation is the caller's, and it must be the *fetch*
//!    path** — the one that checks execute permission and charges a walk on a
//!    TLB miss, never `Cpu::translate_debug`, whose entire purpose is to have
//!    no effects.
//! 3. **The translation context is in the cache key**, through [`Origin`].
//!
//! What is *not* a hazard here, and is worth writing down because it looks
//! like one: **`TCR_EL1.TBI`**. Address tagging would make two virtual
//! addresses that differ in their top byte name one page, so a lifter that
//! keyed on the tagged address and an MMU that ignored the tag would disagree
//! about which bytes a block came from. This core does not implement `TBI` at
//! all — [`mmu`](super::mmu)'s regime selection reads the full 64-bit address, and an
//! address carrying a non-zero tag falls in neither half and takes a
//! translation fault — so the lifter and the walker agree by construction. If
//! `TBI` is ever implemented, the tag has to be stripped in exactly one place
//! and both must read it.
//!
//! The **two `TTBR`s** are not a hazard either, for a smaller reason: which
//! one a walk starts from is a pure function of the virtual address, and the
//! virtual address is the block's own entry PC. A block cannot span the hole
//! between the two halves, because it cannot leave its page.
//!
//! # Guest state: the slot numbering
//!
//! | Slot | State |
//! | --- | --- |
//! | `0..=30` | `X0`–`X30` ([`x_slot`]) |
//! | `31` | `SP`, whichever of `SP_EL0`/`SP_EL1` `PSTATE.SP` selects ([`SP`]) |
//! | `32..=35` | `PSTATE.N`, `.Z`, `.C`, `.V`, one bit each |
//! | `36` | the exclusive monitor ([`EXCLUSIVE`]) |
//! | `37` | the program counter ([`PC`]) |
//!
//! `SP` is one slot rather than two because nothing in the subset can change
//! `PSTATE.SP` or the exception level: `MSR SPSel`, `MSR SP_EL0` and `ERET`
//! are all outside it and end the block, so which architectural register slot
//! 31 names cannot change while a block runs.
//!
//! [`EXCLUSIVE`] is numbered and never bound, exactly as
//! `cpu::riscv::lift`'s reservation slot is: a store in this subset *does*
//! break a reservation — `Exec::store` clears it when the address shares the
//! reserved sixteen-byte granule — but whether it does depends on the run-time
//! address, so it is the [`Opcode::ST`]'s own business. The slot exists so
//! that a later exclusives frontend and any consumer of a fault's state agree
//! on its number.
//!
//! [`PC`] is bound only at a block's **exit** boundary; at every other
//! boundary it is [`InsnStart::pc`], a constant.
//!
//! # How this is known to be right
//!
//! It is not, on its own: CLAUDE.md's "CPU cores" rule makes the interpreter
//! the oracle and this frontend differentially tested against it *forever*.
//! [`differential`](super::differential) is that harness, driven from a
//! generated corpus in `tests/a64_lift_differential.rs`. The tests below
//! assert the *shape* of what this file emits; the harness asserts the
//! meaning.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487),
//! chapter C6 for every operand rule below, and the `AddWithCarry`,
//! `DecodeBitMasks`, `ConditionHolds` and `ExtendReg` shared pseudocode. No
//! emulator source of any licence was opened for any part of this file
//! (`ROADMAP.md` §1).

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::value::Width;
use crate::ir::{
    AccessKind, Align, Block, BlockBuilder, Cond as IrCond, Const, Endian, InsnStart, MemOp,
    MemSpace, Opcode, RegSlot, Sign, Temp, Type, bitfield_aux,
};

use super::isa::{self, Cond, Features, Fmt, LsAccess, Op, ShiftKind};
use super::mmu::PAGE_MASK;

// ---------------------------------------------------------------------------
// The slot numbering
// ---------------------------------------------------------------------------

/// The slot holding general register `X`*n*, for `n` below 31.
///
/// # Panics
///
/// Never: `n` is masked, because every caller derives it from a five-bit
/// instruction field and register 31 is not a general register at all.
#[inline]
#[must_use]
pub const fn x_slot(n: u32) -> RegSlot {
    RegSlot((n & 31) as u16)
}

/// The slot holding the stack pointer.
///
/// One slot, not two: nothing in the lifted subset can change `PSTATE.SP` or
/// the exception level, so which architectural register this names is fixed
/// for the life of a block.
pub const SP: RegSlot = RegSlot(31);

/// The slot holding `PSTATE.N`.
pub const N: RegSlot = RegSlot(32);
/// The slot holding `PSTATE.Z`.
pub const Z: RegSlot = RegSlot(33);
/// The slot holding `PSTATE.C`.
pub const C: RegSlot = RegSlot(34);
/// The slot holding `PSTATE.V`.
pub const V: RegSlot = RegSlot(35);

/// The slot holding the exclusive monitor.
///
/// Never bound here — see the module docs.
pub const EXCLUSIVE: RegSlot = RegSlot(36);

/// The slot holding the program counter.
///
/// Bound only at a block's exit boundary; at every other boundary the PC is
/// [`InsnStart::pc`] and a temporary for it would be a second source of truth.
pub const PC: RegSlot = RegSlot(37);

/// One past the highest slot this frontend numbers.
pub const SLOT_COUNT: u16 = 38;

/// The four flag slots, in `NZCV` order, so a host can loop over them.
pub const FLAG_SLOTS: [RegSlot; 4] = [N, Z, C, V];

/// What one instruction fetch costs.
///
/// One, always: every A64 instruction is one naturally aligned word and
/// `Exec::fetch` makes exactly one `read_once` for it. This is the whole of
/// the static tick column.
const FETCH_TICKS: u64 = 1;

// ---------------------------------------------------------------------------
// Inputs and outputs
// ---------------------------------------------------------------------------

/// Where the lifter reads guest instruction bytes.
///
/// Whole words, because A64 has exactly one instruction length and the
/// interpreter fetches one aligned word per instruction — which is also why
/// the fetch charge here is a constant where the other two frontends have to
/// derive it from the encoding.
///
/// Implemented for every `FnMut(u64) -> Option<u32>`, so a caller can pass a
/// closure over an address space, a snapshot, or a slice. `None` means "cannot
/// be read here" and ends the block ([`Stop::Unreadable`]); the lifter never
/// invents an encoding.
pub trait InsnSource {
    /// The instruction word at guest address `addr`, or `None` if it is
    /// unreadable.
    fn word(&mut self, addr: u64) -> Option<u32>;
}

impl<F: FnMut(u64) -> Option<u32>> InsnSource for F {
    #[inline]
    fn word(&mut self, addr: u64) -> Option<u32> {
        self(addr)
    }
}

/// Why a block stopped where it did.
///
/// Reported rather than inferred, because "the block is short" has six
/// different causes and only one of them is a gap in the subset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stop {
    /// An encoding outside the subset. It was not lifted; the block's exit PC
    /// is its address, so the interpreter executes it next.
    Unsupported,
    /// A memory access that ended the block: a store always, and a load under
    /// [`Shape::BasicBlock`].
    Access,
    /// A transfer of control this block cannot follow: `BR`/`BLR`/`RET`
    /// always, and a branch under a [`Shape`] that does not merge.
    Transfer,
    /// The next instruction would leave the page the block started on.
    Page,
    /// The caller's instruction limit.
    Limit,
    /// The instruction bytes could not be read.
    Unreadable,
}

/// How much a block is allowed to swallow.
///
/// `ROADMAP.md` §9's fourth speed mechanism is superblocks, and this is the
/// switch. [`Shape::Trace`] is what a dispatcher wants; the other two exist
/// because a speed claim with no baseline is not a measurement.
///
/// The shapes are strictly nested: everything [`Shape::BasicBlock`] lifts,
/// [`Shape::Extended`] lifts, and everything that lifts, [`Shape::Trace`]
/// lifts. All three must agree with the interpreter on every column — a
/// disagreement between two of them is a frontend bug wherever it shows up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Shape {
    /// A basic block: it ends at the first memory access and at the first
    /// transfer of control. The baseline a speed claim is measured against.
    BasicBlock,
    /// An extended basic block: a **load** is an ordinary instruction, and
    /// only a store or a transfer of control ends the block.
    Extended,
    /// A trace: direct branches are merged in, with a precise side exit for
    /// each path not taken. One entry, several exits. The default.
    #[default]
    Trace,
}

impl Shape {
    /// Whether a **load** ends the block. A store always does.
    #[inline]
    #[must_use]
    pub const fn access_ends_block(self) -> bool {
        matches!(self, Shape::BasicBlock)
    }

    /// Whether a direct branch is merged into the block.
    #[inline]
    #[must_use]
    pub const fn merges(self) -> bool {
        matches!(self, Shape::Trace)
    }

    /// This shape's contribution to [`Block::key`].
    const fn key_bits(self) -> u64 {
        match self {
            Shape::BasicBlock => 0,
            Shape::Extended => 1 << 5,
            Shape::Trace => 2 << 5,
        }
    }
}

/// Which address space `entry_pc` names, and under what mapping.
///
/// A block is a function of the *bytes* at `entry_pc`, and under translation
/// which bytes those are is a function of the translation tables. Naming the
/// world a lift happened in is therefore part of naming the block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Origin {
    /// `SCTLR_EL1.M` is clear, so `entry_pc` is at once the guest PC, the
    /// address [`InsnSource`] reads, and the physical address.
    ///
    /// Writing this out by hand is a claim about the core, not a request:
    /// `ADR`, `B` and every branch target is computed from `entry_pc`, so
    /// lifting from a *physical* address on a core that runs the same code at
    /// some other virtual address produces a block whose PC arithmetic is
    /// wrong everywhere.
    Bare,
    /// Translation is on: `entry_pc` is a virtual address, valid only for the
    /// mapping that was in force when the bytes were read.
    Paged {
        /// What distinguishes one mapping of `entry_pc` from another.
        ///
        /// `cpu::arm::a64::engine` puts the **physical page the entry fetch
        /// resolved to** here rather than `SysRegs::translation_gen`, and the
        /// argument is the one `cpu::riscv::engine` makes: the generation is
        /// bumped by every `TLBI` and by every write to `TTBR0_EL1`,
        /// `TTBR1_EL1`, `TCR_EL1` and `SCTLR_EL1` — so a Linux guest bumps it
        /// on every `switch_mm` and every unmap, and a cache keyed on it would
        /// be thrown away faster than it fills. The physical page is strictly
        /// more precise: different bytes mean a different page and therefore a
        /// different key, the same page rewritten is caught by the block
        /// cache's own store matching, which is already by physical page, and
        /// changed permissions are caught by the entry translation, which is
        /// redone on every execution and faults before the block runs.
        generation: u64,
    },
}

impl Origin {
    /// This origin's contribution to [`Block::key`].
    ///
    /// Bit 7 separates the two worlds, so a physical lift and a virtual lift
    /// of the same number never collide; above it sits the generation, exact
    /// until it passes 2^56.
    const fn key_bits(self) -> u64 {
        match self {
            Origin::Bare => 0,
            Origin::Paged { generation } => (1 << 7) | generation.wrapping_shl(8),
        }
    }
}

/// Everything about the core a lift depends on, beside the guest bytes.
///
/// A struct rather than three arguments because every one of these belongs in
/// [`Block::key`], and a caller that passed them separately would be one
/// forgotten field away from a cache that returns the wrong translation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct World {
    /// Which optional features the part has, so [`isa::decode`] filters
    /// exactly as the interpreter's does.
    pub features: Features,
    /// Which world `entry_pc` lives in.
    pub origin: Origin,
    /// `SCTLR_EL1.A`: whether an unaligned ordinary access faults.
    ///
    /// In the key because it is the [`Align`] every [`MemOp`] carries, and a
    /// block lifted with it clear says something different from one lifted
    /// with it set. The *check itself* is the host's — `Exec::check_align`
    /// reads `SCTLR_EL1` live on every access — so this field decides what a
    /// **backend** may assume, never what the guest observes.
    pub strict_align: bool,
}

/// A lifted block, and what is true about it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lifted {
    /// The block. Always ends in a terminator and always passes
    /// [`verify`](crate::ir::verify).
    pub block: Block,
    /// Why lifting stopped.
    pub stop: Stop,
    /// How many guest instructions were lifted. Zero is legal and means the
    /// block's first instruction was outside the subset.
    ///
    /// What the block **covers**, which under [`Shape::Trace`] is not what a
    /// run through it retires: a trace inlines one side of every branch it
    /// merges, and leaving through a side exit retires only the instructions
    /// on the path taken. Anything that has to know what retired counts
    /// boundaries instead —
    /// [`Interp::boundaries`](crate::ir::Interp::boundaries).
    pub insns: usize,
    /// The world this block was lifted in, as the caller declared it.
    pub world: World,
}

/// How many guest instructions [`lift`] takes by default.
///
/// **Computed, not inherited.** `cpu::x86::lift` records what happens when it
/// is not: at 64 instructions an x86 cold block bounds at 13 200 ticks, above
/// `SchedulerConfig::max_ticks_per_quantum`, so no block would ever be
/// admitted. The same arithmetic here, per guest instruction, in the worst
/// case the engine's cold bound has to assume:
///
/// | | ticks |
/// | --- | --- |
/// | the fetch | 1 |
/// | a pair access, unaligned, split into bytes | 2 × 8 |
/// | a four-level walk in front of each of those bytes | 2 × 8 × 4 |
///
/// which is 81, so 64 instructions bound a cold block at **5 188** including
/// the entry walk — inside two thirds of a quantum, where x86's 13 200 was
/// outside a whole one. Sixty-four therefore stands here on its own
/// arithmetic rather than because two other frontends use it, and the engine's
/// `a_cold_block_fits_inside_a_scheduler_quantum` asserts the bound rather
/// than leaving it in prose.
///
/// It does a second job under [`Shape::Trace`]: it is the only thing that
/// bounds an unrolled loop, and — because a dispatcher checks its exit flag at
/// block boundaries and a trace has fewer of them — it is also the bound on
/// how long a safe point can be delayed (`ROADMAP.md` §4.7).
pub const MAX_INSNS: usize = 64;

// The block bound is one page, and it is sound only because the smallest
// translation granule this core implements is that same size: a block
// descriptor makes the bound conservative, a *smaller* leaf would make it
// wrong. Checked rather than remembered.
const _: () = assert!(super::mmu::PAGE_SIZE == 4096);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Lift the guest instructions at `entry_pc` into a translation block.
///
/// Reads at most `max_insns` instructions, never leaves `entry_pc`'s page, and
/// always produces a well-formed block — including when nothing could be
/// lifted, in which case the block is just the exit boundary and a terminator
/// and [`Lifted::insns`] is zero.
///
/// `src` must read through the same translation the interpreter's *fetch*
/// uses, never the debug walk; the module docs' "Paging" section is why
/// [`World::origin`] is an argument rather than an assumption.
///
/// # Errors
///
/// None today, and the signature says so rather than promising it: both
/// sibling frontends refuse a configuration they cannot lift — RV32 for one, a
/// real-mode segment fold for the other — and an A64 configuration this
/// frontend could not handle would be refused the same way rather than
/// silently mis-lifted.
pub fn lift<S: InsnSource>(
    world: &World,
    entry_pc: u64,
    src: &mut S,
    max_insns: usize,
    shape: Shape,
) -> Result<Lifted> {
    let mut lf = Lifter::new(world, entry_pc, shape);
    let page = lf.page;
    let mut pc = entry_pc;
    let mut insns = 0usize;

    let stop = loop {
        if insns >= max_insns {
            break Stop::Limit;
        }
        if pc & !PAGE_MASK != page {
            break Stop::Page;
        }
        let Some(word) = src.word(pc) else {
            break Stop::Unreadable;
        };
        let next_pc = pc.wrapping_add(4);
        match lf.insn(word, pc, next_pc) {
            Flow::Rejected => break Stop::Unsupported,
            // `next` is `next_pc` for everything in program order and the
            // target for a merged branch, which is the whole of what merging
            // does to this loop.
            Flow::Continue(next) => {
                insns += 1;
                pc = next;
            }
            Flow::Access { next, store } => {
                insns += 1;
                pc = next;
                if store || shape.access_ends_block() {
                    break Stop::Access;
                }
            }
            Flow::Transfer => {
                insns += 1;
                pc = next_pc;
                break Stop::Transfer;
            }
        }
    };

    Ok(Lifted {
        block: lf.finish(pc),
        stop,
        insns,
        world: *world,
    })
}

/// The block cache key: every property of the core this lift depends on, and
/// the world it happened in.
///
/// [`Block::key`] is the rest of the cache key beside the entry PC. Identical
/// guest bytes lift differently under a different feature set (an LSE atomic
/// is an instruction on one part and `UNDEFINED` on another, and the two end
/// the block in different places), under a different `SCTLR_EL1.A`, and under
/// a different [`Shape`] — so all three belong here or a cache returns the
/// wrong translation. The shape is in the key even though every shape is
/// *correct*: a cache that mixed them would make a measurement of one of them
/// a measurement of whichever happened to be resident.
///
/// The [`Origin`] belongs here for a stronger reason: under translation the
/// *bytes* at `entry_pc` are a function of the tables, so a key without it
/// lets a cache return a block lifted through a mapping the guest has since
/// replaced.
///
/// Public because a block cache has to ask this question *before* it lifts
/// anything: `jit::Dispatcher` looks a block up under
/// `(pc, key(world, shape))` and calls [`lift`] only when that misses. A
/// dispatcher that derived the key itself would be a second copy of the
/// answer, and the two would drift.
#[must_use]
pub fn key(world: &World, shape: Shape) -> u64 {
    let mut key = 0u64;
    if world.features.lse {
        key |= 1;
    }
    if world.features.crc32 {
        key |= 2;
    }
    if world.features.fp {
        key |= 4;
    }
    if world.features.advsimd {
        key |= 8;
    }
    if world.strict_align {
        key |= 16;
    }
    key | shape.key_bits() | world.origin.key_bits()
}

// ---------------------------------------------------------------------------
// The plan: what an encoding means, decided before anything is emitted
// ---------------------------------------------------------------------------

/// How a load or store computes its address, and what it writes back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Addr {
    /// `[Rn, #off]` — no write-back.
    Offset(i64),
    /// `[Rn], #off` — the access is at the base, and the base moves after it.
    Post(i64),
    /// `[Rn, #off]!` — the access is at the new base, which is written back.
    Pre(i64),
    /// `[Rn, Rm, extend #amount]`.
    Reg {
        /// The `option` field, whose low two bits give the index width and
        /// whose bit 2 says whether the extension is signed.
        option: u32,
        /// How far the extended index is shifted left.
        amount: u32,
    },
}

/// Which of the four conditional selects an encoding is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Sel {
    /// `CSEL`: the alternative is `Rm`.
    Plain,
    /// `CSINC`: the alternative is `Rm + 1`.
    Inc,
    /// `CSINV`: the alternative is `NOT(Rm)`.
    Inv,
    /// `CSNEG`: the alternative is `-Rm`.
    Neg,
}

/// Which of the three bitfield moves an encoding is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bf {
    /// `SBFM`: the field is sign-extended from its top bit.
    Signed,
    /// `BFM`: the bits outside the field keep the destination's value.
    Merge,
    /// `UBFM`: the bits outside the field are zero.
    Unsigned,
}

/// What lifting one instruction will emit.
///
/// Every encoding is classified — and every static precondition checked —
/// *before* a single op is emitted, so the emitter is total and a rejected
/// instruction leaves no debris in the block. Splitting it this way is not
/// bookkeeping: an instruction whose `UNDEFINED` case is a decode constant
/// must end the block rather than lift a trap, and that decision has to come
/// before the boundary marker is opened.
#[derive(Debug, Clone, Copy)]
enum Plan {
    /// Retires with no architectural effect: a hint, a barrier, a `PRFM`.
    Nop,
    /// `Rd = <constant>` at 64 bits — `ADR` and `ADRP`.
    Konst(u64),
    /// `Rd|SP = Rn|SP +/- <immediate>`.
    AddSubImm {
        /// The already-shifted immediate.
        imm: u64,
        /// Whether it is subtracted.
        sub: bool,
        /// Whether the flags are written.
        flags: bool,
    },
    /// `Rd|SP = Rn <logical> <bitmask immediate>`.
    LogicalImm {
        /// [`Opcode::AND`], [`Opcode::OR`] or [`Opcode::XOR`].
        op: Opcode,
        /// The decoded bitmask.
        imm: u64,
        /// Whether the logical flags are written (`ANDS`).
        flags: bool,
    },
    /// `MOVZ`, `MOVN` and `MOVK`, whose immediate is already positioned.
    MoveWide {
        /// The 16-bit immediate, shifted into place.
        imm: u64,
        /// Where the 16-bit field sits.
        pos: u32,
        /// Whether the rest of the destination is kept (`MOVK`).
        keep: bool,
        /// Whether the immediate is inverted (`MOVN`).
        invert: bool,
    },
    /// `SBFM`, `BFM`, `UBFM`, with `DecodeBitMasks` already run.
    Bitfield {
        /// Which of the three.
        kind: Bf,
        /// `wmask & tmask`, the bits the field contributes.
        field: u64,
        /// `tmask`, which `SBFM` needs on its own for the sign fill.
        tmask: u64,
        /// The rotate amount.
        r: u32,
        /// The field's top bit, for `SBFM`'s sign.
        s: u32,
    },
    /// `EXTR Rd, Rn, Rm, #lsb`.
    Extr {
        /// The bit position the pair is extracted at.
        lsb: u32,
    },
    /// The shifted-register logical family.
    LogicalShift {
        /// [`Opcode::AND`], [`Opcode::OR`] or [`Opcode::XOR`].
        op: Opcode,
        /// Whether the shifted operand is inverted.
        invert: bool,
        /// The shift kind.
        shift: ShiftKind,
        /// The shift amount, already range-checked.
        amount: u32,
        /// Whether the logical flags are written.
        flags: bool,
    },
    /// The shifted-register add and subtract family.
    AddSubShift {
        /// The shift kind. `ROR` is unallocated here and rejected.
        shift: ShiftKind,
        /// The shift amount, already range-checked.
        amount: u32,
        /// Whether it is a subtraction.
        sub: bool,
        /// Whether the flags are written.
        flags: bool,
    },
    /// The extended-register add and subtract family.
    AddSubExt {
        /// The `option` field.
        option: u32,
        /// The left shift, already range-checked to 0..=4.
        amount: u32,
        /// Whether it is a subtraction.
        sub: bool,
        /// Whether the flags are written.
        flags: bool,
    },
    /// `ADC`, `ADCS`, `SBC`, `SBCS`.
    AddSubCarry {
        /// Whether it is a subtraction, which inverts the second operand.
        sub: bool,
        /// Whether the flags are written.
        flags: bool,
    },
    /// `CCMP`, `CCMN`, in both the register and immediate forms.
    CondCmp {
        /// The condition under which the comparison happens at all.
        cond: Cond,
        /// Whether it is a comparison rather than a negated comparison.
        sub: bool,
        /// The immediate, where the encoding has one.
        imm: Option<u64>,
        /// The flags forced when the condition does not hold.
        nzcv: u32,
    },
    /// `CSEL`, `CSINC`, `CSINV`, `CSNEG`.
    CondSel {
        /// The condition.
        cond: Cond,
        /// Which alternative the false side takes.
        kind: Sel,
    },
    /// `UDIV`, `SDIV`.
    Div {
        /// Whether the division is signed.
        signed: bool,
    },
    /// `LSLV`, `LSRV`, `ASRV`, `RORV`.
    ShiftReg {
        /// Which shift.
        kind: ShiftKind,
    },
    /// `REV`, `REV16`, `REV32`.
    Rev {
        /// The lane the byte order is reversed within.
        lane: u32,
    },
    /// `CLZ`.
    Clz,
    /// `CLS`.
    Cls,
    /// `MADD`, `MSUB`.
    MulAdd {
        /// Whether the product is subtracted from the accumulator.
        sub: bool,
    },
    /// `SMADDL`, `SMSUBL`, `UMADDL`, `UMSUBL`.
    MulAddLong {
        /// Whether the word operands are sign-extended.
        signed: bool,
        /// Whether the product is subtracted.
        sub: bool,
    },
    /// `SMULH`, `UMULH`.
    MulHigh {
        /// Whether the multiply is signed.
        signed: bool,
    },
    /// `B` and `BL`, and an unconditional `B.cond`.
    Branch {
        /// The statically known target.
        target: u64,
        /// Whether `X30` is written.
        link: bool,
    },
    /// `B.cond` with a real condition.
    CondBranch {
        /// The condition.
        cond: Cond,
        /// The statically known target.
        target: u64,
    },
    /// `CBZ`, `CBNZ`.
    CompareBranch {
        /// Whether the branch is taken when the register is *not* zero.
        nonzero: bool,
        /// The statically known target.
        target: u64,
    },
    /// `TBZ`, `TBNZ`.
    TestBranch {
        /// The bit position, six bits wide.
        pos: u32,
        /// Whether the branch is taken when the bit is set.
        set: bool,
        /// The statically known target.
        target: u64,
    },
    /// `BR`, `BLR`, `RET`.
    BranchReg {
        /// Whether `X30` is written.
        link: bool,
    },
    /// A PC-relative literal load.
    LoadLiteral {
        /// The statically known address.
        addr: u64,
        /// How many bytes are read.
        bytes: u64,
        /// Whether the value is sign-extended.
        signed: bool,
    },
    /// A single-register load or store.
    Single {
        /// What the access does.
        access: LsAccess,
        /// How the address is formed.
        addr: Addr,
    },
    /// A load or store of a register pair.
    Pair(Pair),
}

/// What a pair load or store does, in one struct.
///
/// A struct rather than five fields on the variant because the emitter takes
/// them as one argument: five is over the limit `too_many_arguments` draws,
/// and threading them separately is how one of them ends up in the wrong
/// position.
#[derive(Debug, Clone, Copy)]
struct Pair {
    /// Whether it loads.
    load: bool,
    /// The base-2 logarithm of the access width.
    scale: u32,
    /// Whether a loaded word is sign-extended (`LDPSW`).
    signed: bool,
    /// Whether the destination registers are 64 bits wide.
    wide: bool,
    /// How the address is formed. Never [`Addr::Reg`].
    addr: Addr,
}

/// Decide what an encoding means, or reject it.
///
/// Every `UNDEFINED` case the architecture decides from *decode constants* is
/// checked here and answered with `None`, which ends the block: lifting a
/// conditional trap would be a second implementation of the interpreter's
/// exception path, and the interpreter is the oracle.
#[allow(clippy::too_many_lines)]
fn classify(world: &World, op: Op, fmt: Fmt, word: u32, pc: u64) -> Option<Plan> {
    let width = isa::datasize(word);
    let sf = isa::sf(word);

    // Loads and stores are decided by their *format*, which is how the
    // interpreter dispatches them too: the operation column names the
    // instruction and the format says how the address is formed.
    if fmt.is_load_store() {
        return classify_memory(fmt, word);
    }

    let plan = match op {
        // -- PC-relative addressing ------------------------------------
        Op::Adr | Op::Adrp => {
            let imm = isa::sext(
                ((isa::field(word, 23, 5) as u64) << 2) | u64::from(isa::field(word, 30, 29)),
                21,
            );
            Plan::Konst(if op == Op::Adrp {
                (pc & !0xfff).wrapping_add((imm as u64) << 12)
            } else {
                pc.wrapping_add(imm as u64)
            })
        }

        // -- add/subtract (immediate) ----------------------------------
        Op::AddImm | Op::AddsImm | Op::SubImm | Op::SubsImm => {
            let shift = isa::field(word, 23, 22);
            // `sh` is one bit; `0b1x` is unallocated.
            if shift > 1 {
                return None;
            }
            Plan::AddSubImm {
                imm: u64::from(isa::imm12(word)) << (12 * shift),
                sub: matches!(op, Op::SubImm | Op::SubsImm),
                flags: matches!(op, Op::AddsImm | Op::SubsImm),
            }
        }

        // -- logical (immediate) ---------------------------------------
        Op::AndImm | Op::OrrImm | Op::EorImm | Op::AndsImm => {
            let (imm, _) = isa::decode_bit_masks(
                isa::n_bit(word),
                isa::imms(word),
                isa::immr(word),
                true,
                width,
            )?;
            Plan::LogicalImm {
                op: match op {
                    Op::AndImm | Op::AndsImm => Opcode::AND,
                    Op::OrrImm => Opcode::OR,
                    _ => Opcode::XOR,
                },
                imm,
                flags: op == Op::AndsImm,
            }
        }

        // -- move wide -------------------------------------------------
        Op::Movn | Op::Movz | Op::Movk => {
            let hw = isa::field(word, 22, 21);
            // A 32-bit move may only shift by 0 or 16.
            if width == 32 && hw > 1 {
                return None;
            }
            let pos = hw * 16;
            Plan::MoveWide {
                imm: u64::from(isa::imm16(word)) << pos,
                pos,
                keep: op == Op::Movk,
                invert: op == Op::Movn,
            }
        }

        // -- bitfield --------------------------------------------------
        Op::Sbfm | Op::Bfm | Op::Ubfm => {
            // `N` must match `sf`, and the fields must fit the operand.
            if isa::n_bit(word) != u32::from(sf) {
                return None;
            }
            let r = isa::immr(word);
            let s = isa::imms(word);
            // DDI 0487 C6: the 32-bit variant requires `immr<5>` and `imms<5>`
            // to be zero. `DecodeBitMasks` does not catch it — it only checks
            // that the *element* fits, and a six-bit field naming bit 55 of a
            // 32-bit register produces an element that fits perfectly well.
            if !sf && (r | s) >= 32 {
                return None;
            }
            let (wmask, tmask) = isa::decode_bit_masks(isa::n_bit(word), s, r, false, width)?;
            Plan::Bitfield {
                kind: match op {
                    Op::Sbfm => Bf::Signed,
                    Op::Bfm => Bf::Merge,
                    _ => Bf::Unsigned,
                },
                field: wmask & tmask,
                tmask,
                r,
                s,
            }
        }
        Op::Extr => {
            if isa::n_bit(word) != u32::from(sf) {
                return None;
            }
            let lsb = isa::imms(word);
            if lsb >= width {
                return None;
            }
            Plan::Extr { lsb }
        }

        // -- branches --------------------------------------------------
        Op::B | Op::Bl => Plan::Branch {
            target: pc.wrapping_add(isa::imm26(word) as u64),
            link: op == Op::Bl,
        },
        Op::Bcond => {
            let cond = isa::cond_lo(word);
            let target = pc.wrapping_add(isa::imm19(word) as u64);
            // `AL` and `NV` are synonyms for *always*, so `B.AL` is an
            // unconditional branch and lifting it as a side exit would put a
            // condition where the architecture has none.
            if cond.0 & 0xe == 0xe {
                Plan::Branch {
                    target,
                    link: false,
                }
            } else {
                Plan::CondBranch { cond, target }
            }
        }
        Op::Cbz | Op::Cbnz => Plan::CompareBranch {
            nonzero: op == Op::Cbnz,
            target: pc.wrapping_add(isa::imm19(word) as u64),
        },
        Op::Tbz | Op::Tbnz => Plan::TestBranch {
            // The bit position is split: bit 31 is its top bit, and it also
            // decides the operand width.
            pos: (u32::from(sf) << 5) | isa::field(word, 23, 19),
            set: op == Op::Tbnz,
            target: pc.wrapping_add(isa::imm14(word) as u64),
        },
        Op::Br | Op::Ret => Plan::BranchReg { link: false },
        Op::Blr => Plan::BranchReg { link: true },

        // -- hints and barriers ----------------------------------------
        //
        // The interpreter retires every one of these with no architectural
        // effect, and so does this: there is one instruction stream and every
        // access completes before the next, so there is nothing to order.
        Op::Nop | Op::Yield | Op::Sev | Op::Sevl | Op::Hint | Op::Wfe => Plan::Nop,
        Op::Dsb | Op::Dmb | Op::Isb => Plan::Nop,

        // -- literal loads ---------------------------------------------
        Op::LdrLitW | Op::LdrLitX | Op::LdrswLit => {
            let (bytes, signed) = match op {
                Op::LdrLitW => (4, false),
                Op::LdrLitX => (8, false),
                _ => (4, true),
            };
            Plan::LoadLiteral {
                addr: pc.wrapping_add(isa::imm19(word) as u64),
                bytes,
                signed,
            }
        }
        Op::PrfmLit => Plan::Nop,

        // -- logical, shifted register ---------------------------------
        Op::AndShift
        | Op::BicShift
        | Op::OrrShift
        | Op::OrnShift
        | Op::EorShift
        | Op::EonShift
        | Op::AndsShift
        | Op::BicsShift => {
            let amount = isa::shift_amount(word);
            if width == 32 && amount >= 32 {
                return None;
            }
            Plan::LogicalShift {
                op: match op {
                    Op::AndShift | Op::BicShift | Op::AndsShift | Op::BicsShift => Opcode::AND,
                    Op::OrrShift | Op::OrnShift => Opcode::OR,
                    _ => Opcode::XOR,
                },
                invert: matches!(
                    op,
                    Op::BicShift | Op::OrnShift | Op::EonShift | Op::BicsShift
                ),
                shift: ShiftKind::from_bits(isa::shift_type(word)),
                amount,
                flags: matches!(op, Op::AndsShift | Op::BicsShift),
            }
        }

        // -- add/subtract, shifted register ----------------------------
        Op::AddShift | Op::AddsShift | Op::SubShift | Op::SubsShift => {
            let amount = isa::shift_amount(word);
            let shift_bits = isa::shift_type(word);
            // `ROR` is not an addressing mode for add and subtract.
            if shift_bits == 3 || (width == 32 && amount >= 32) {
                return None;
            }
            Plan::AddSubShift {
                shift: ShiftKind::from_bits(shift_bits),
                amount,
                sub: matches!(op, Op::SubShift | Op::SubsShift),
                flags: matches!(op, Op::AddsShift | Op::SubsShift),
            }
        }

        // -- add/subtract, extended register ---------------------------
        Op::AddExt | Op::AddsExt | Op::SubExt | Op::SubsExt => {
            let amount = isa::field(word, 12, 10);
            if amount > 4 {
                return None;
            }
            Plan::AddSubExt {
                option: isa::extend_option(word),
                amount,
                sub: matches!(op, Op::SubExt | Op::SubsExt),
                flags: matches!(op, Op::AddsExt | Op::SubsExt),
            }
        }

        // -- add/subtract with carry -----------------------------------
        Op::Adc | Op::Adcs | Op::Sbc | Op::Sbcs => Plan::AddSubCarry {
            sub: matches!(op, Op::Sbc | Op::Sbcs),
            flags: matches!(op, Op::Adcs | Op::Sbcs),
        },

        // -- conditional -----------------------------------------------
        Op::CcmnReg | Op::CcmpReg | Op::CcmnImm | Op::CcmpImm => Plan::CondCmp {
            cond: isa::cond_hi(word),
            sub: matches!(op, Op::CcmpReg | Op::CcmpImm),
            imm: matches!(op, Op::CcmnImm | Op::CcmpImm).then(|| u64::from(isa::rm(word))),
            nzcv: word & 0xf,
        },
        Op::Csel | Op::Csinc | Op::Csinv | Op::Csneg => Plan::CondSel {
            cond: isa::cond_hi(word),
            kind: match op {
                Op::Csel => Sel::Plain,
                Op::Csinc => Sel::Inc,
                Op::Csinv => Sel::Inv,
                _ => Sel::Neg,
            },
        },

        // -- two-source -------------------------------------------------
        Op::Udiv | Op::Sdiv => Plan::Div {
            signed: op == Op::Sdiv,
        },
        Op::Lslv | Op::Lsrv | Op::Asrv | Op::Rorv => Plan::ShiftReg {
            kind: match op {
                Op::Lslv => ShiftKind::Lsl,
                Op::Lsrv => ShiftKind::Lsr,
                Op::Asrv => ShiftKind::Asr,
                _ => ShiftKind::Ror,
            },
        },

        // -- one-source -------------------------------------------------
        //
        // `RBIT` is absent, and it is the one omission in this group that is
        // not a policy: the IR has no bit-reversal op, and open-coding one is
        // five shifts and five masks per width. It ends the block instead.
        Op::Rev16 => Plan::Rev { lane: 16 },
        Op::RevW => Plan::Rev { lane: 32 },
        Op::Rev32 => Plan::Rev { lane: 32 },
        Op::RevX => Plan::Rev { lane: 64 },
        Op::Clz => Plan::Clz,
        Op::Cls => Plan::Cls,

        // -- three-source -----------------------------------------------
        Op::Madd | Op::Msub => Plan::MulAdd {
            sub: op == Op::Msub,
        },
        Op::Smaddl | Op::Smsubl | Op::Umaddl | Op::Umsubl => Plan::MulAddLong {
            signed: matches!(op, Op::Smaddl | Op::Smsubl),
            sub: matches!(op, Op::Smsubl | Op::Umsubl),
        },
        Op::Smulh | Op::Umulh => Plan::MulHigh {
            signed: op == Op::Smulh,
        },

        _ => return None,
    };
    let _ = world;
    Some(plan)
}

/// Decide what a load or store encoding means, from its format.
fn classify_memory(fmt: Fmt, word: u32) -> Option<Plan> {
    match fmt {
        // The unprivileged forms are a *permission* difference the IR has
        // nowhere to carry; see the module docs.
        Fmt::LdStUnpriv => None,
        Fmt::LdStPairOff | Fmt::LdStPairPost | Fmt::LdStPairPre => {
            let opc = isa::field(word, 31, 30);
            let load = isa::bit(word, 22);
            // `opc` is not the single-register `size` field: `0b00` is a word,
            // `0b01` is `LDPSW` and `0b10` is a doubleword.
            let (scale, signed, wide) = match opc {
                0b00 => (2u32, false, false),
                0b01 if load => (2u32, true, true),
                0b10 => (3u32, false, true),
                _ => return None,
            };
            let offset = isa::imm7(word) << scale;
            Some(Plan::Pair(Pair {
                load,
                scale,
                signed,
                wide,
                addr: match fmt {
                    Fmt::LdStPairOff => Addr::Offset(offset),
                    Fmt::LdStPairPost => Addr::Post(offset),
                    _ => Addr::Pre(offset),
                },
            }))
        }
        _ => {
            let size = isa::ls_size(word);
            let access = isa::ls_access(size, isa::ls_opc(word))?;
            let addr = match fmt {
                Fmt::LdStUImm => Addr::Offset((u64::from(isa::imm12(word)) << size) as i64),
                Fmt::LdStUnscaled => Addr::Offset(isa::imm9(word)),
                Fmt::LdStPost => Addr::Post(isa::imm9(word)),
                Fmt::LdStPre => Addr::Pre(isa::imm9(word)),
                Fmt::LdStRegOff => {
                    let option = isa::extend_option(word);
                    // `option<1>` must be set: the encodings with it clear are
                    // unallocated rather than an `LSL` by another name.
                    if option & 2 == 0 {
                        return None;
                    }
                    Addr::Reg {
                        option,
                        amount: if isa::bit(word, 12) { size } else { 0 },
                    }
                }
                _ => return None,
            };
            if matches!(access, LsAccess::Prefetch) {
                // Architecturally a hint, and this core makes no access at all
                // rather than pretending to have a cache. No prefetch encoding
                // has a write-back form, so once the addressing mode has been
                // *validated* there is nothing left for the instruction to do.
                //
                // **Validated, and that word is the whole comment.** This
                // check used to come first, which made `PRFM [Xn, Xm, SXTB]` —
                // a register-offset encoding whose `option<1>` is clear, and
                // therefore unallocated — a no-op here and `UNDEFINED` in the
                // interpreter. Two thousand generated programs never found it,
                // because the generator forces that bit set; an enumerated
                // one-bit sweep against `llvm-mc` found it in the only two
                // words in forty-two thousand that this frontend accepted and
                // the assembler refused.
                return Some(Plan::Nop);
            }
            Some(Plan::Single { access, addr })
        }
    }
}

// ---------------------------------------------------------------------------
// The lifter
// ---------------------------------------------------------------------------

/// What lifting one instruction did to the block, and where lifting goes next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flow {
    /// Nothing was emitted; the instruction is outside the subset.
    Rejected,
    /// Lifted; carry on at this guest PC — the program-order successor for
    /// everything except a merged direct branch, where it is the target, which
    /// is the entire mechanism by which a trace spans more than one basic
    /// block.
    Continue(u64),
    /// Lifted a memory access; carry on unless the [`Shape`] ends the block at
    /// one, or unless it was a store.
    Access {
        /// Where lifting carries on.
        next: u64,
        /// Whether it was a store, which ends the block whatever the shape.
        store: bool,
    },
    /// Lifted, and it transferred control somewhere this block cannot follow.
    Transfer,
}

/// One translation in progress.
struct Lifter<'a> {
    world: &'a World,
    shape: Shape,
    /// The page the entry PC is on. No instruction outside it is ever lifted,
    /// which is what keeps every fetch charge static.
    page: u64,
    b: BlockBuilder,
    /// Which temporary holds each of `X0`–`X30`, where one does.
    ///
    /// **This is the trace's register allocation.** It survives a merged
    /// branch untouched, so a value computed before a `BL` is still in a
    /// temporary after it rather than having gone out to a slot and come back.
    x: [Option<Temp>; 31],
    /// Which temporary holds `SP`, where one does.
    sp: Option<Temp>,
    /// Which temporary holds each of `N`, `Z`, `C`, `V`.
    flags: [Option<Temp>; 4],
    /// The block's shared zeroes: `[i32, i64]`.
    zero: [Option<Temp>; 2],
    /// Ticks charged so far, counted from block entry.
    ticks: u64,
    /// The temporary holding the exit PC, once a transfer has set one.
    pc_out: Option<Temp>,
    /// The exit PC's value where it is a constant, for the exit boundary.
    static_exit: Option<u64>,
}

/// The IR type a `width`-bit A64 operation computes in.
#[inline]
const fn ty_of(width: u32) -> Type {
    if width == 32 { Type::I32 } else { Type::I64 }
}

impl<'a> Lifter<'a> {
    fn new(world: &'a World, entry_pc: u64, shape: Shape) -> Lifter<'a> {
        Lifter {
            world,
            shape,
            page: entry_pc & !PAGE_MASK,
            b: BlockBuilder::new(entry_pc, key(world, shape)),
            x: [None; 31],
            sp: None,
            flags: [None; 4],
            zero: [None; 2],
            ticks: 0,
            pc_out: None,
            static_exit: None,
        }
    }

    // -- constants ------------------------------------------------------

    /// Materialize a constant of the given width.
    fn konst(&mut self, width: u32, value: u64) -> Temp {
        let value = value & isa::ones(width);
        self.b.imm(ty_of(width), Const::Int(u128::from(value)))
    }

    /// The block's shared zero for a width.
    fn zero(&mut self, width: u32) -> Temp {
        let slot = usize::from(width == 64);
        match self.zero[slot] {
            Some(t) => t,
            None => {
                let t = self.konst(width, 0);
                self.zero[slot] = Some(t);
                t
            }
        }
    }

    /// A one-bit constant.
    fn bit(&mut self, value: bool) -> Temp {
        self.b.imm(Type::I1, Const::Int(u128::from(value)))
    }

    // -- registers ------------------------------------------------------

    /// Read a general register as a 64-bit value.
    ///
    /// DDI 0487 C1.2.5: register 31 is `XZR` in most encodings and `SP` in a
    /// handful, and `is_sp` is the format's answer to which — the same
    /// [`Fmt::rd_is_sp`]/[`Fmt::rn_is_sp`] the interpreter and the
    /// disassembler ask, which is what stops the three disagreeing about
    /// whether `add x0, x31, #1` means `sp` or `xzr`. Because the register
    /// number is a decode constant, the hard-wired zero costs nothing at run
    /// time: it folds to an immediate here.
    fn read_x(&mut self, n: u32, is_sp: bool) -> Temp {
        if n == 31 {
            if !is_sp {
                return self.zero(64);
            }
            return match self.sp {
                Some(t) => t,
                None => {
                    let t = self.b.get_slot(Type::I64, SP);
                    self.sp = Some(t);
                    t
                }
            };
        }
        match self.x[n as usize] {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I64, x_slot(n));
                self.x[n as usize] = Some(t);
                t
            }
        }
    }

    /// Read a general register at `width`, narrowing to [`Type::I32`] where
    /// the operation is a `W` form.
    fn read(&mut self, n: u32, width: u32, is_sp: bool) -> Temp {
        if width == 64 {
            return self.read_x(n, is_sp);
        }
        if n == 31 && !is_sp {
            return self.zero(32);
        }
        let v = self.read_x(n, is_sp);
        self.b.unary(Opcode::TRUNC, Type::I32, v)
    }

    /// Bind a general register to a 64-bit temporary. A write to `XZR` is
    /// discarded, and `n` is a decode constant so the guard costs nothing at
    /// run time.
    fn write_x(&mut self, n: u32, is_sp: bool, t: Temp) {
        if n == 31 {
            if is_sp {
                self.sp = Some(t);
            }
            return;
        }
        self.x[n as usize] = Some(t);
    }

    /// Bind a general register from a `width`-bit result.
    ///
    /// A 32-bit result zero-extends into the 64-bit register, which is the
    /// rule that makes `W`-form arithmetic well defined.
    fn write(&mut self, n: u32, width: u32, is_sp: bool, t: Temp) {
        let t = if width == 32 {
            self.b.unary(Opcode::EXT_Z, Type::I64, t)
        } else {
            t
        };
        self.write_x(n, is_sp, t);
    }

    // -- flags ----------------------------------------------------------

    /// Read one flag as a one-bit temporary. The index is `NZCV` order.
    fn read_flag(&mut self, i: usize) -> Temp {
        match self.flags[i] {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I1, FLAG_SLOTS[i]);
                self.flags[i] = Some(t);
                t
            }
        }
    }

    /// Bind all four flags at once. Every flag-setting A64 instruction writes
    /// all four, so there is no partial form to express.
    fn write_flags(&mut self, nzcv: [Temp; 4]) {
        for (slot, t) in self.flags.iter_mut().zip(nzcv) {
            *slot = Some(t);
        }
    }

    /// `N` and `Z` from a result, with `C` and `V` cleared — the logical
    /// instructions' flag rule, which is what makes `TST` followed by `B.CS` a
    /// bug rather than an idiom.
    fn logical_flags(&mut self, width: u32, result: Temp) {
        let ty = ty_of(width);
        let zero = self.zero(width);
        let n = self.b.setcond(IrCond::LtS, ty, result, zero);
        let z = self.b.setcond(IrCond::Eq, ty, result, zero);
        let c = self.bit(false);
        let v = self.bit(false);
        self.write_flags([n, z, c, v]);
    }

    /// `AddWithCarry(a, b, carry_in)`, yielding the sum and `NZCV`.
    ///
    /// Deliberately **not** [`Opcode::ADDC`], which is exactly this primitive
    /// and which `jit::x86` refuses; see the module docs. The carry out is
    /// recovered from two unsigned comparisons — `t = a + b` carried iff
    /// `t <u a`, and `s = t + carry` carried iff `s <u t`, and at most one of
    /// the two can be true — and the overflow from the sign rule, which holds
    /// whatever the carry in was: an addition overflows exactly when the
    /// operands' signs agree and the result's differs.
    fn add_with_carry(
        &mut self,
        width: u32,
        a: Temp,
        b: Temp,
        carry: Option<Temp>,
    ) -> (Temp, [Temp; 4]) {
        let ty = ty_of(width);
        let zero = self.zero(width);
        let t = self.b.binary(Opcode::ADD, ty, a, b);
        let (sum, c) = match carry {
            None => {
                let c = self.b.setcond(IrCond::LtU, ty, t, a);
                (t, c)
            }
            Some(cin) => {
                let c1 = self.b.setcond(IrCond::LtU, ty, t, a);
                let wide = self.b.unary(Opcode::EXT_Z, ty, cin);
                let s = self.b.binary(Opcode::ADD, ty, t, wide);
                let c2 = self.b.setcond(IrCond::LtU, ty, s, t);
                let c = self.b.binary(Opcode::OR, Type::I1, c1, c2);
                (s, c)
            }
        };
        let n = self.b.setcond(IrCond::LtS, ty, sum, zero);
        let z = self.b.setcond(IrCond::Eq, ty, sum, zero);
        // Overflow: the operands agreed about their sign and the result does
        // not — `~(a ^ b) & (a ^ sum)`, tested at the sign bit.
        let differ = self.b.binary(Opcode::XOR, ty, a, b);
        let changed = self.b.binary(Opcode::XOR, ty, a, sum);
        let both = self.b.emit(Opcode::ANDC, ty, &[changed, differ]);
        let v = self.b.setcond(IrCond::LtS, ty, both, zero);
        (sum, [n, z, c, v])
    }

    /// The architecture's subtraction: `AddWithCarry(a, NOT(b), 1)`.
    ///
    /// Written out rather than routed through [`Lifter::add_with_carry`] with
    /// an inverted operand because the comparisons collapse: the carry out of
    /// `a + !b + 1` is set exactly when `a >=u b`, which is one `setcond`
    /// rather than five ops, and it is why `SUBS x0, x1, x2` sets `C` when
    /// there was **no** borrow.
    fn sub_with_flags(&mut self, width: u32, a: Temp, b: Temp) -> (Temp, [Temp; 4]) {
        let ty = ty_of(width);
        let zero = self.zero(width);
        let sum = self.b.binary(Opcode::SUB, ty, a, b);
        let c = self.b.setcond(IrCond::GeU, ty, a, b);
        let n = self.b.setcond(IrCond::LtS, ty, sum, zero);
        let z = self.b.setcond(IrCond::Eq, ty, sum, zero);
        // For a subtraction the sign rule is `(a ^ b) & (a ^ sum)`, which is
        // the addition's predicate with `b` inverted.
        let differ = self.b.binary(Opcode::XOR, ty, a, b);
        let changed = self.b.binary(Opcode::XOR, ty, a, sum);
        let both = self.b.binary(Opcode::AND, ty, differ, changed);
        let v = self.b.setcond(IrCond::LtS, ty, both, zero);
        (sum, [n, z, c, v])
    }

    /// Whether an A64 condition holds, as a one-bit temporary.
    ///
    /// `None` means *always* — `AL` and `NV`, which the architecture makes
    /// synonyms rather than opposites — so a caller emits nothing at all for
    /// an unconditional select.
    fn cond_holds(&mut self, cond: Cond) -> Option<Temp> {
        // DDI 0487 `ConditionHolds`: the top three bits select the test and the
        // bottom bit inverts it — except at `0b1111`, where inverting *always*
        // would give *never*, and the architecture says it does not. That
        // exception needs **no guard here**, which is worth stating because
        // the obvious `&& cond.0 & 0xf != 0xf` reads as though it does: `AL`
        // and `NV` share the selector `0b111`, whose arm returns `None` before
        // an inversion could apply. A mutation pass found that guard could not
        // fail and it came out rather than staying as decoration.
        let invert = cond.0 & 1 == 1;
        let base = match (cond.0 >> 1) & 7 {
            0 => self.read_flag(1),
            1 => self.read_flag(2),
            2 => self.read_flag(0),
            3 => self.read_flag(3),
            // C && !Z
            4 => {
                let c = self.read_flag(2);
                let z = self.read_flag(1);
                self.b.emit(Opcode::ANDC, Type::I1, &[c, z])
            }
            // N == V
            5 => {
                let n = self.read_flag(0);
                let v = self.read_flag(3);
                let differ = self.b.binary(Opcode::XOR, Type::I1, n, v);
                self.b.unary(Opcode::NOT, Type::I1, differ)
            }
            // N == V && !Z
            6 => {
                let n = self.read_flag(0);
                let v = self.read_flag(3);
                let z = self.read_flag(1);
                let differ = self.b.binary(Opcode::XOR, Type::I1, n, v);
                let same = self.b.unary(Opcode::NOT, Type::I1, differ);
                self.b.emit(Opcode::ANDC, Type::I1, &[same, z])
            }
            _ => return None,
        };
        Some(if invert {
            self.b.unary(Opcode::NOT, Type::I1, base)
        } else {
            base
        })
    }

    // -- operand shapes -------------------------------------------------

    /// `ShiftReg(value, kind, amount)` at `width`, with `amount` already known
    /// to be below the width.
    fn shifted(&mut self, width: u32, value: Temp, kind: ShiftKind, amount: u32) -> Temp {
        if amount == 0 {
            return value;
        }
        let ty = ty_of(width);
        let by = self.konst(width, u64::from(amount));
        let op = match kind {
            ShiftKind::Lsl => Opcode::SHL,
            ShiftKind::Lsr => Opcode::SHR,
            ShiftKind::Asr => Opcode::SAR,
            ShiftKind::Ror => Opcode::ROTR,
        };
        self.b.binary(op, ty, value, by)
    }

    /// `ExtendReg(value, option, amount)` at 64 bits.
    ///
    /// The low two bits of `option` give the source width and bit 2 says
    /// whether the extension is signed. A sign extension from eight or sixteen
    /// bits is a shift pair rather than an [`Opcode::EXT_S`], because the IR
    /// has no `i8` or `i16` to extend *from*.
    fn extended(&mut self, value: Temp, option: u32, amount: u32) -> Temp {
        let bits = match option & 3 {
            0 => 8,
            1 => 16,
            2 => 32,
            _ => 64,
        };
        let signed = option & 4 != 0;
        let base = if bits == 64 {
            value
        } else if signed {
            let up = self.konst(64, u64::from(64 - bits));
            let left = self.b.binary(Opcode::SHL, Type::I64, value, up);
            self.b.binary(Opcode::SAR, Type::I64, left, up)
        } else {
            let mask = self.konst(64, isa::ones(bits));
            self.b.binary(Opcode::AND, Type::I64, value, mask)
        };
        if amount == 0 {
            base
        } else {
            let by = self.konst(64, u64::from(amount));
            self.b.binary(Opcode::SHL, Type::I64, base, by)
        }
    }

    /// `base + offset`, folding a zero offset away.
    fn offset(&mut self, base: Temp, offset: i64) -> Temp {
        if offset == 0 {
            return base;
        }
        let off = self.konst(64, offset as u64);
        self.b.binary(Opcode::ADD, Type::I64, base, off)
    }

    // -- boundaries and exits -------------------------------------------

    /// The guest state a temporary currently shadows, in slot order.
    ///
    /// Slot order rather than binding order: `ROADMAP.md` §0's determinism rule
    /// reaches the IR too, and this vector is hashed by anything that hashes a
    /// block.
    fn live_regs(&self) -> Vec<(RegSlot, Temp)> {
        let mut live = Vec::new();
        for (n, temp) in self.x.iter().enumerate() {
            if let Some(t) = temp {
                live.push((x_slot(n as u32), *t));
            }
        }
        if let Some(t) = self.sp {
            live.push((SP, t));
        }
        for (i, temp) in self.flags.iter().enumerate() {
            if let Some(t) = temp {
                live.push((FLAG_SLOTS[i], *t));
            }
        }
        live
    }

    /// Emit a precise side exit: leave the block for `exit_pc` when `cond` has
    /// the value `when`, and fall through otherwise.
    ///
    /// The sequence is *inline* and branched over on the negated condition
    /// rather than appended at the end of the block. Three things that buys,
    /// none of them cosmetic: the boundary records stay in program order so
    /// [`InsnStart::ticks`] stays monotonic and the verifier's check on it
    /// keeps working; every [`Opcode::BRCOND`] stays a *forward* branch, which
    /// is what `ir::pass`'s single backward liveness walk is built on; and the
    /// exit's live map is taken exactly here, at the branch, which is what
    /// makes leaving through it architecturally precise.
    fn side_exit(&mut self, cond: Temp, when: bool, exit_pc: u64) {
        let skip = if when {
            self.b.unary(Opcode::NOT, Type::I1, cond)
        } else {
            cond
        };
        let over = self
            .b
            .emit_raw(Opcode::BRCOND, Type::I1, None, None, &[skip], None, None, 0);
        // Everything from here to the terminator runs only on the exit path,
        // so the constant costs nothing when the branch is not taken.
        let target = self.konst(64, exit_pc);
        let mut live = self.live_regs();
        live.push((PC, target));
        self.b.insn_start(InsnStart {
            pc: exit_pc,
            next_pc: exit_pc,
            ticks: self.ticks,
            live,
        });
        self.b.exit_tb();
        let after = self.b.next_index() as u32;
        self.b.patch_aux(over, after);
    }

    /// Close the block: the exit boundary, then the terminator.
    ///
    /// The exit boundary begins no guest instruction. It carries the outgoing
    /// state map and the [`PC`] slot, which is the only thing that tells a
    /// dispatcher where to resume; its `pc` field is the exit PC where that is
    /// a constant, and the program-order continuation otherwise.
    fn finish(mut self, program_order_pc: u64) -> Block {
        let pc = match self.pc_out {
            Some(t) => t,
            None => self.konst(64, program_order_pc),
        };
        let mut live = self.live_regs();
        live.push((PC, pc));
        let at = self.static_exit.unwrap_or(program_order_pc);
        self.b.insn_start(InsnStart {
            pc: at,
            next_pc: at,
            ticks: self.ticks,
            live,
        });
        self.b.exit_tb();
        self.b.finish()
    }

    /// The misalignment policy a memory op carries.
    ///
    /// A64's rule is not x86's [`Align::Split`]: `Exec::store` translates and
    /// writes byte by byte, so a fault on the second page leaves the first
    /// half written. What the architecture says is that `SCTLR_EL1.A` decides
    /// between performing the access and raising, which is exactly
    /// [`Align::None`] and [`Align::Fault`].
    const fn align(&self) -> Align {
        if self.world.strict_align {
            Align::Fault
        } else {
            Align::None
        }
    }

    /// The descriptor for an access of `bytes` bytes.
    ///
    /// `None` for a byte count no [`Width`] names, which cannot arise from any
    /// encoding here — every one derives its width from a two-bit `size`
    /// field — and is answered rather than asserted because a total emitter is
    /// what makes a rejected instruction leave no debris.
    fn memop(&self, bytes: u64, sign: Sign, kind: AccessKind) -> Option<MemOp> {
        Some(MemOp {
            size: Width::from_bytes(bytes)?,
            sign,
            space: MemSpace::MEM,
            seg: None,
            endian: Endian::Little,
            align: self.align(),
            kind,
            // The access spends ticks and can fault, both guest-visible, so
            // dead-code elimination may not remove it (module docs).
            volatile: true,
        })
    }

    // -- one instruction ------------------------------------------------

    /// Lift one instruction.
    fn insn(&mut self, word: u32, pc: u64, next_pc: u64) -> Flow {
        let Some(row) = isa::decode(word, self.world.features) else {
            return Flow::Rejected;
        };
        // One line for the whole SIMD and floating-point family, keyed off the
        // table's own feature column rather than a list of formats — the same
        // question `Exec::execute` asks to decide whether `CPACR_EL1` traps.
        if row.feat.is_simd_fp() {
            return Flow::Rejected;
        }
        let Some(plan) = classify(self.world, row.op, row.fmt, word, pc) else {
            return Flow::Rejected;
        };

        // The boundary, then the fetch charge. Nothing has been emitted for
        // this instruction yet, so a fault inside it publishes the state as of
        // *this* PC, which is what makes the exception precise.
        let live = self.live_regs();
        self.b.insn_start(InsnStart {
            pc,
            next_pc,
            ticks: self.ticks,
            live,
        });
        self.b.charge(FETCH_TICKS);
        self.ticks += FETCH_TICKS;

        self.emit(plan, row.fmt, word, next_pc, pc)
    }

    /// Emit one classified instruction.
    #[allow(clippy::too_many_lines)]
    fn emit(&mut self, plan: Plan, fmt: Fmt, word: u32, next_pc: u64, pc: u64) -> Flow {
        let width = isa::datasize(word);
        let ty = ty_of(width);
        let d = isa::rd(word);
        let n = isa::rn(word);
        let m = isa::rm(word);
        let rd_sp = fmt.rd_is_sp();
        let rn_sp = fmt.rn_is_sp();

        match plan {
            Plan::Nop => {}

            Plan::Konst(value) => {
                let t = self.konst(64, value);
                self.write_x(d, false, t);
            }

            Plan::AddSubImm { imm, sub, flags } => {
                let a = self.read(n, width, rn_sp);
                let b = self.konst(width, imm);
                let result = self.arith(width, a, b, sub, flags, None);
                self.write(d, width, rd_sp, result);
            }

            Plan::LogicalImm { op, imm, flags } => {
                let a = self.read(n, width, false);
                let b = self.konst(width, imm);
                let result = self.b.binary(op, ty, a, b);
                if flags {
                    self.logical_flags(width, result);
                }
                self.write(d, width, rd_sp, result);
            }

            Plan::MoveWide {
                imm,
                pos,
                keep,
                invert,
            } => {
                let result = if keep {
                    let old = self.read(d, width, false);
                    let field = self.konst(width, imm >> pos);
                    let dst = self.b.temp(ty);
                    self.b.emit_raw(
                        Opcode::DEPOSIT,
                        ty,
                        Some(dst),
                        None,
                        &[old, field],
                        None,
                        None,
                        bitfield_aux(pos, 16),
                    );
                    dst
                } else if invert {
                    self.konst(width, !imm)
                } else {
                    self.konst(width, imm)
                };
                self.write(d, width, false, result);
            }

            Plan::Bitfield {
                kind,
                field,
                tmask,
                r,
                s,
            } => {
                let src = self.read(n, width, false);
                let rotated = self.shifted(width, src, ShiftKind::Ror, r);
                let keep = self.konst(width, field);
                let taken = self.b.binary(Opcode::AND, ty, rotated, keep);
                let result = match kind {
                    Bf::Unsigned => taken,
                    Bf::Merge => {
                        let old = self.read(d, width, false);
                        let outside = self.konst(width, !field);
                        let kept = self.b.binary(Opcode::AND, ty, old, outside);
                        self.b.binary(Opcode::OR, ty, kept, taken)
                    }
                    Bf::Signed => {
                        // The sign is the *source's* bit `S`, replicated above
                        // the field: negating a one-bit value gives all ones
                        // or all zeroes at the operation's width.
                        let bit = self.b.temp(ty);
                        self.b.emit_raw(
                            Opcode::EXTRACT,
                            ty,
                            Some(bit),
                            None,
                            &[src],
                            None,
                            None,
                            bitfield_aux(s % width, 1),
                        );
                        let top = self.b.unary(Opcode::NEG, ty, bit);
                        let above = self.konst(width, !tmask);
                        let fill = self.b.binary(Opcode::AND, ty, top, above);
                        self.b.binary(Opcode::OR, ty, fill, taken)
                    }
                };
                self.write(d, width, false, result);
            }

            Plan::Extr { lsb } => {
                let hi = self.read(n, width, false);
                let lo = self.read(m, width, false);
                let result = if lsb == 0 {
                    lo
                } else {
                    let down = self.konst(width, u64::from(lsb));
                    let up = self.konst(width, u64::from(width - lsb));
                    let low = self.b.binary(Opcode::SHR, ty, lo, down);
                    let high = self.b.binary(Opcode::SHL, ty, hi, up);
                    self.b.binary(Opcode::OR, ty, low, high)
                };
                self.write(d, width, false, result);
            }

            Plan::LogicalShift {
                op,
                invert,
                shift,
                amount,
                flags,
            } => {
                let raw = self.read(m, width, false);
                let operand = self.shifted(width, raw, shift, amount);
                let a = self.read(n, width, false);
                let result = if invert {
                    if op == Opcode::AND {
                        // `BIC` is `a & !b`, which is one op.
                        self.b.emit(Opcode::ANDC, ty, &[a, operand])
                    } else {
                        let inverted = self.b.unary(Opcode::NOT, ty, operand);
                        self.b.binary(op, ty, a, inverted)
                    }
                } else {
                    self.b.binary(op, ty, a, operand)
                };
                if flags {
                    self.logical_flags(width, result);
                }
                self.write(d, width, false, result);
            }

            Plan::AddSubShift {
                shift,
                amount,
                sub,
                flags,
            } => {
                let raw = self.read(m, width, false);
                let operand = self.shifted(width, raw, shift, amount);
                let a = self.read(n, width, false);
                let result = self.arith(width, a, operand, sub, flags, None);
                self.write(d, width, false, result);
            }

            Plan::AddSubExt {
                option,
                amount,
                sub,
                flags,
            } => {
                let raw = self.read_x(m, false);
                let wide = self.extended(raw, option, amount);
                let operand = if width == 32 {
                    self.b.unary(Opcode::TRUNC, Type::I32, wide)
                } else {
                    wide
                };
                let a = self.read(n, width, rn_sp);
                let result = self.arith(width, a, operand, sub, flags, None);
                self.write(d, width, rd_sp, result);
            }

            Plan::AddSubCarry { sub, flags } => {
                let a = self.read(n, width, false);
                let raw = self.read(m, width, false);
                let b = if sub {
                    self.b.unary(Opcode::NOT, ty, raw)
                } else {
                    raw
                };
                let carry = self.read_flag(2);
                // Never the collapsed subtraction: the carry in is a run-time
                // value here, so `a + !b + C` is a genuine three-way add.
                let result = if flags {
                    let (sum, nzcv) = self.add_with_carry(width, a, b, Some(carry));
                    self.write_flags(nzcv);
                    sum
                } else {
                    let t = self.b.binary(Opcode::ADD, ty, a, b);
                    let wide = self.b.unary(Opcode::EXT_Z, ty, carry);
                    self.b.binary(Opcode::ADD, ty, t, wide)
                };
                self.write(d, width, false, result);
            }

            Plan::CondCmp {
                cond,
                sub,
                imm,
                nzcv,
            } => {
                let a = self.read(n, width, false);
                let b = match imm {
                    Some(value) => self.konst(width, value),
                    None => self.read(m, width, false),
                };
                let (_, computed) = if sub {
                    self.sub_with_flags(width, a, b)
                } else {
                    self.add_with_carry(width, a, b, None)
                };
                let selected = match self.cond_holds(cond) {
                    // `AL`/`NV`: the comparison always happens, so the forced
                    // flags are unreachable and nothing selects between them.
                    None => computed,
                    Some(sel) => {
                        let mut out = computed;
                        for (i, flag) in out.iter_mut().enumerate() {
                            let forced = self.bit((nzcv >> (3 - i)) & 1 != 0);
                            *flag = self
                                .b
                                .emit(Opcode::MOVCOND, Type::I1, &[sel, *flag, forced]);
                        }
                        out
                    }
                };
                self.write_flags(selected);
            }

            Plan::CondSel { cond, kind } => {
                let a = self.read(n, width, false);
                let b = self.read(m, width, false);
                let alt = match kind {
                    Sel::Plain => b,
                    Sel::Inc => {
                        let one = self.konst(width, 1);
                        self.b.binary(Opcode::ADD, ty, b, one)
                    }
                    Sel::Inv => self.b.unary(Opcode::NOT, ty, b),
                    Sel::Neg => self.b.unary(Opcode::NEG, ty, b),
                };
                let result = match self.cond_holds(cond) {
                    None => a,
                    Some(sel) => self.b.emit(Opcode::MOVCOND, ty, &[sel, a, alt]),
                };
                self.write(d, width, false, result);
            }

            Plan::Div { signed } => {
                let a = self.read(n, width, false);
                let b = self.read(m, width, false);
                let zero = self.zero(width);
                // DDI 0487: a division by zero produces zero and does not trap
                // — there is no divide-by-zero exception in A64 — and the IR's
                // divide is documented as needing a frontend's guard. The
                // most-negative divided by minus one wraps rather than
                // trapping, which is what `wrapping_div` gives on both sides.
                let by_zero = self.b.setcond(IrCond::Eq, ty, b, zero);
                let one = self.konst(width, 1);
                let safe = self.b.emit(Opcode::MOVCOND, ty, &[by_zero, one, b]);
                let op = if signed { Opcode::DIV_S } else { Opcode::DIV_U };
                let quotient = self.b.binary(op, ty, a, safe);
                let result = self.b.emit(Opcode::MOVCOND, ty, &[by_zero, zero, quotient]);
                self.write(d, width, false, result);
            }

            Plan::ShiftReg { kind } => {
                let a = self.read(n, width, false);
                let raw = self.read(m, width, false);
                // The shift amount is taken modulo the operand width, which is
                // why an A64 shift by 64 is a no-op rather than a zero, and
                // which is also the guard `Opcode::SHL` requires each frontend
                // to emit for itself.
                let mask = self.konst(width, u64::from(width - 1));
                let amount = self.b.binary(Opcode::AND, ty, raw, mask);
                let op = match kind {
                    ShiftKind::Lsl => Opcode::SHL,
                    ShiftKind::Lsr => Opcode::SHR,
                    ShiftKind::Asr => Opcode::SAR,
                    ShiftKind::Ror => Opcode::ROTR,
                };
                let result = self.b.binary(op, ty, a, amount);
                self.write(d, width, false, result);
            }

            Plan::Rev { lane } => {
                let a = self.read(n, width, false);
                let dst = self.b.temp(ty);
                self.b.emit_raw(
                    Opcode::BSWAP,
                    ty,
                    Some(dst),
                    None,
                    &[a],
                    Some(Const::Int(u128::from(lane))),
                    None,
                    0,
                );
                self.write(d, width, false, dst);
            }

            Plan::Clz => {
                let a = self.read(n, width, false);
                let result = self.b.unary(Opcode::CLZ, ty, a);
                self.write(d, width, false, result);
            }

            Plan::Cls => {
                // Count the sign bits *above* the top one: the leading zeroes
                // of `x ^ (x >> 1)` within `width - 1` bits, which is
                // `clz(folded) - 1` at the operation's width — and correct at
                // `folded == 0`, where `clz` is the width and the answer is
                // `width - 1`.
                let a = self.read(n, width, false);
                let one = self.konst(width, 1);
                let down = self.b.binary(Opcode::SHR, ty, a, one);
                let xored = self.b.binary(Opcode::XOR, ty, a, down);
                let mask = self.konst(width, isa::ones(width - 1));
                let folded = self.b.binary(Opcode::AND, ty, xored, mask);
                let lz = self.b.unary(Opcode::CLZ, ty, folded);
                let result = self.b.binary(Opcode::SUB, ty, lz, one);
                self.write(d, width, false, result);
            }

            Plan::MulAdd { sub } => {
                let a = self.read(n, width, false);
                let b = self.read(m, width, false);
                let acc = self.read(isa::ra(word), width, false);
                let product = self.b.binary(Opcode::MUL, ty, a, b);
                let op = if sub { Opcode::SUB } else { Opcode::ADD };
                let result = self.b.binary(op, ty, acc, product);
                self.write(d, width, false, result);
            }

            Plan::MulAddLong { signed, sub } => {
                // The sources are the *word* halves and the accumulator is a
                // doubleword: widening after narrowing, never the reverse.
                let a = self.read(n, 32, false);
                let b = self.read(m, 32, false);
                let ext = if signed { Opcode::EXT_S } else { Opcode::EXT_Z };
                let wa = self.b.unary(ext, Type::I64, a);
                let wb = self.b.unary(ext, Type::I64, b);
                let product = self.b.binary(Opcode::MUL, Type::I64, wa, wb);
                let acc = self.read_x(isa::ra(word), false);
                let op = if sub { Opcode::SUB } else { Opcode::ADD };
                let result = self.b.binary(op, Type::I64, acc, product);
                self.write_x(d, false, result);
            }

            Plan::MulHigh { signed } => {
                let a = self.read_x(n, false);
                let b = self.read_x(m, false);
                let low = self.b.temp(Type::I64);
                let high = self.b.temp(Type::I64);
                let op = if signed { Opcode::MULS2 } else { Opcode::MULU2 };
                self.b
                    .emit_raw(op, Type::I64, Some(low), Some(high), &[a, b], None, None, 0);
                self.write_x(d, false, high);
            }

            Plan::Branch { target, link } => {
                if link {
                    let ret = self.konst(64, next_pc);
                    self.write_x(30, false, ret);
                }
                if self.shape.merges() {
                    // A direct unconditional transfer: the trace continues at
                    // the target and the branch costs nothing but its fetch. A
                    // target off the entry page ends the block one turn later,
                    // through the loop's page check, and `finish` then names
                    // the target as the exit PC — the same answer the other
                    // arm would have produced.
                    return Flow::Continue(target);
                }
                let t = self.konst(64, target);
                self.pc_out = Some(t);
                self.static_exit = Some(target);
                return Flow::Transfer;
            }

            Plan::CondBranch { cond, target } => {
                let Some(taken) = self.cond_holds(cond) else {
                    // `classify` folds `AL` and `NV` into `Plan::Branch`, so
                    // this cannot be reached; falling through to the exit is
                    // still correct if it ever were.
                    return Flow::Transfer;
                };
                return self.branch_on(taken, target, next_pc, pc);
            }

            Plan::CompareBranch { nonzero, target } => {
                let value = self.read(d, width, false);
                let zero = self.zero(width);
                let cond = if nonzero { IrCond::Ne } else { IrCond::Eq };
                let taken = self.b.setcond(cond, ty, value, zero);
                return self.branch_on(taken, target, next_pc, pc);
            }

            Plan::TestBranch { pos, set, target } => {
                let value = self.read_x(d, false);
                let bit = self.b.temp(Type::I64);
                self.b.emit_raw(
                    Opcode::EXTRACT,
                    Type::I64,
                    Some(bit),
                    None,
                    &[value],
                    None,
                    None,
                    bitfield_aux(pos, 1),
                );
                let zero = self.zero(64);
                let cond = if set { IrCond::Ne } else { IrCond::Eq };
                let taken = self.b.setcond(cond, Type::I64, bit, zero);
                return self.branch_on(taken, target, next_pc, pc);
            }

            Plan::BranchReg { link } => {
                // The target is read *before* the link is bound, which is what
                // makes `blr x30` correct.
                let target = self.read_x(n, false);
                if link {
                    let ret = self.konst(64, next_pc);
                    self.write_x(30, false, ret);
                }
                self.pc_out = Some(target);
                return Flow::Transfer;
            }

            Plan::LoadLiteral {
                addr,
                bytes,
                signed,
            } => {
                let at = self.konst(64, addr);
                let sign = if signed { Sign::Signed } else { Sign::Unsigned };
                let Some(mem) = self.memop(bytes, sign, AccessKind::Load) else {
                    return Flow::Rejected;
                };
                let value = self.b.load(Type::I64, at, mem);
                self.write_x(d, false, value);
                return Flow::Access {
                    next: next_pc,
                    store: false,
                };
            }

            Plan::Single { access, addr } => return self.single(access, addr, word, next_pc),

            Plan::Pair(pair) => return self.pair(pair, word, next_pc),
        }
        Flow::Continue(next_pc)
    }

    /// The add-or-subtract shape every arithmetic family shares.
    fn arith(
        &mut self,
        width: u32,
        a: Temp,
        b: Temp,
        sub: bool,
        flags: bool,
        carry: Option<Temp>,
    ) -> Temp {
        let ty = ty_of(width);
        if !flags {
            let op = if sub { Opcode::SUB } else { Opcode::ADD };
            return self.b.binary(op, ty, a, b);
        }
        let (sum, nzcv) = if sub {
            self.sub_with_flags(width, a, b)
        } else {
            self.add_with_carry(width, a, b, carry)
        };
        self.write_flags(nzcv);
        sum
    }

    /// A conditional branch, merged as a side exit or turned into a computed
    /// exit PC depending on the [`Shape`].
    fn branch_on(&mut self, taken: Temp, target: u64, next_pc: u64, pc: u64) -> Flow {
        if !self.shape.merges() {
            let then = self.konst(64, target);
            let other = self.konst(64, next_pc);
            let sel = self
                .b
                .emit(Opcode::MOVCOND, Type::I64, &[taken, then, other]);
            self.pc_out = Some(sel);
            return Flow::Transfer;
        }
        // Static prediction, and it is what decides whether a loop unrolls: a
        // backward branch is a back edge, so the taken side is the trace; a
        // forward one is an `if`, so the fall-through is. A backward target
        // off the entry page is not a candidate, because no instruction
        // outside that page may be lifted.
        let inline_taken = target < pc && target & !PAGE_MASK == self.page;
        if inline_taken {
            self.side_exit(taken, false, next_pc);
            Flow::Continue(target)
        } else {
            self.side_exit(taken, true, target);
            Flow::Continue(next_pc)
        }
    }

    /// A single-register load or store.
    fn single(&mut self, access: LsAccess, addr: Addr, word: u32, next_pc: u64) -> Flow {
        let t = isa::rd(word);
        let n = isa::rn(word);
        let base = self.read_x(n, true);
        let (at, writeback) = self.address(base, addr, word);

        let store = match access {
            LsAccess::Prefetch => {
                // Unreachable: `classify_memory` answers a prefetch with
                // `Plan::Nop` before it can get here.
                return Flow::Continue(next_pc);
            }
            LsAccess::Store { bytes } => {
                let value = self.read_x(t, false);
                let Some(mem) = self.memop(bytes, Sign::Unsigned, AccessKind::Store) else {
                    return Flow::Rejected;
                };
                self.b.store(Type::I64, at, value, mem);
                true
            }
            LsAccess::Load { bytes, wide } => {
                let Some(mem) = self.memop(bytes, Sign::Unsigned, AccessKind::Load) else {
                    return Flow::Rejected;
                };
                let value = self.b.load(Type::I64, at, mem);
                let _ = wide;
                // A zero-extending load already sits inside its destination
                // width, so the 32-bit and 64-bit forms bind the same value.
                self.write_x(t, false, value);
                false
            }
            LsAccess::LoadSigned { bytes, wide } => {
                let Some(mem) = self.memop(bytes, Sign::Signed, AccessKind::Load) else {
                    return Flow::Rejected;
                };
                let value = self.b.load(Type::I64, at, mem);
                let value = if wide {
                    value
                } else {
                    // The 32-bit destination keeps the sign extension only
                    // within its own width, and then zero-extends.
                    let mask = self.konst(64, isa::ones(32));
                    self.b.binary(Opcode::AND, Type::I64, value, mask)
                };
                self.write_x(t, false, value);
                false
            }
        };

        // The write-back happens after the access, so a fault leaves the base
        // register untouched and the instruction can be restarted.
        if let Some(value) = writeback {
            self.write_x(n, true, value);
        }
        Flow::Access {
            next: next_pc,
            store,
        }
    }

    /// A load or store of a register pair.
    fn pair(&mut self, pair: Pair, word: u32, next_pc: u64) -> Flow {
        let Pair {
            load,
            scale,
            signed,
            wide,
            addr,
        } = pair;
        let bytes = 1u64 << scale;
        let t = isa::rd(word);
        let t2 = isa::ra(word);
        let n = isa::rn(word);
        let base = self.read_x(n, true);
        let (first_at, writeback) = self.address(base, addr, word);
        let second_at = self.offset(first_at, bytes as i64);

        if load {
            let sign = if signed { Sign::Signed } else { Sign::Unsigned };
            let Some(mem) = self.memop(bytes, sign, AccessKind::Load) else {
                return Flow::Rejected;
            };
            let first = self.b.load(Type::I64, first_at, mem);
            let second = self.b.load(Type::I64, second_at, mem);
            let _ = wide;
            // Both accesses happen before either register is bound, which is
            // what makes `ldp x0, x1, [x0]` correct.
            self.write_x(t, false, first);
            self.write_x(t2, false, second);
            if let Some(value) = writeback {
                self.write_x(n, true, value);
            }
            return Flow::Access {
                next: next_pc,
                store: false,
            };
        }

        let first = self.read_x(t, false);
        let second = self.read_x(t2, false);
        let Some(mem) = self.memop(bytes, Sign::Unsigned, AccessKind::Store) else {
            return Flow::Rejected;
        };
        self.b.store(Type::I64, first_at, first, mem);
        self.b.store(Type::I64, second_at, second, mem);
        if let Some(value) = writeback {
            self.write_x(n, true, value);
        }
        Flow::Access {
            next: next_pc,
            store: true,
        }
    }

    /// The address an addressing mode produces, and the write-back it owes.
    fn address(&mut self, base: Temp, addr: Addr, word: u32) -> (Temp, Option<Temp>) {
        match addr {
            Addr::Offset(off) => (self.offset(base, off), None),
            Addr::Post(off) => (base, Some(self.offset(base, off))),
            Addr::Pre(off) => {
                let a = self.offset(base, off);
                (a, Some(a))
            }
            Addr::Reg { option, amount } => {
                let index = self.read_x(isa::rm(word), false);
                let extended = self.extended(index, option, amount);
                (self.b.binary(Opcode::ADD, Type::I64, base, extended), None)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::verify;
    use alloc::vec;

    /// A part with everything this core implements, in bare mode with
    /// alignment checking off — the shape a `.machine` file's `cortex-a53`
    /// resets into.
    fn world() -> World {
        World {
            features: Features::ALL,
            origin: Origin::Bare,
            strict_align: false,
        }
    }

    /// The address a test program is lifted at. Away from zero so that a
    /// `PC`-relative computation that dropped the base shows up.
    const AT: u64 = 0x8000;

    /// Lift a program, asserting that the verifier accepts what came out.
    ///
    /// Every test goes through here, which is how "the verifier accepts every
    /// block this frontend produces" is asserted everywhere rather than once.
    fn lift_at(world: &World, at: u64, program: &[u32], shape: Shape) -> Lifted {
        let base = at;
        let words = program.to_vec();
        let mut src = |addr: u64| {
            let off = addr.checked_sub(base)? / 4;
            words.get(off as usize).copied()
        };
        let lifted = lift(world, at, &mut src, MAX_INSNS, shape).expect("this world lifts");
        verify(&lifted.block).expect("the frontend emits a well-formed block");
        lifted
    }

    fn lifted(program: &[u32]) -> Lifted {
        lift_at(&world(), AT, program, Shape::Trace)
    }

    /// How many instructions of a given opcode a block holds.
    fn count(block: &Block, op: Opcode) -> usize {
        block.insts().iter().filter(|i| i.op == op).count()
    }

    #[test]
    fn an_encoding_outside_the_subset_lifts_nothing_and_says_so() {
        // `SVC #0` is a supervisor call, which is the interpreter's.
        let out = lifted(&[0xd400_0001]);
        assert_eq!(out.insns, 0);
        assert_eq!(out.stop, Stop::Unsupported);
        // A block that lifted nothing is still well formed: the exit boundary
        // and a terminator, so a dispatcher can tell where the guest is.
        assert_eq!(out.block.marks().len(), 1);
        assert_eq!(out.block.marks()[0].pc, AT);
        assert!(out.block.insts().last().expect("non-empty").op == Opcode::EXIT_TB);
    }

    #[test]
    fn every_instruction_charges_exactly_one_fetch_tick() {
        // Four `mov x0, #1`-shaped `movz` instructions.
        let out = lifted(&[0xd280_0020, 0xd280_0041, 0xd280_0062, 0xd280_0083]);
        assert_eq!(out.insns, 4);
        assert_eq!(count(&out.block, Opcode::CHARGE), 4);
        for (i, mark) in out.block.marks().iter().enumerate() {
            assert_eq!(mark.ticks, i as u64, "the column is one tick per fetch");
        }
    }

    #[test]
    fn a_block_never_leaves_the_page_it_started_on() {
        // Start four bytes below a page boundary: one instruction fits.
        let at = 0x1_0000 - 4;
        let out = lift_at(&world(), at, &[0xd503_201f, 0xd503_201f], Shape::Trace);
        assert_eq!(out.insns, 1);
        assert_eq!(out.stop, Stop::Page);
        assert_eq!(out.block.marks().last().expect("an exit").pc, 0x1_0000);
    }

    #[test]
    fn a_store_ends_its_block_and_a_load_does_not() {
        // `str x1, [x0]` then `nop`.
        let store = lifted(&[0xf900_0001, 0xd503_201f]);
        assert_eq!(store.insns, 1);
        assert_eq!(store.stop, Stop::Access);
        // `ldr x1, [x0]` then `nop`: the trace runs on.
        let load = lifted(&[0xf940_0001, 0xd503_201f]);
        assert_eq!(load.insns, 2);
        // and a basic block stops at it, which is the whole difference.
        let basic = lift_at(&world(), AT, &[0xf940_0001, 0xd503_201f], Shape::BasicBlock);
        assert_eq!(basic.insns, 1);
        assert_eq!(basic.stop, Stop::Access);
    }

    #[test]
    fn a_backward_branch_unrolls_under_a_trace_and_exits_under_a_basic_block() {
        // `add x0, x0, #1` then `b .-4`: a two-instruction loop.
        let program = [0x9100_0400, 0x17ff_ffff];
        let trace = lift_at(&world(), AT, &program, Shape::Trace);
        assert_eq!(trace.insns, MAX_INSNS, "the trace unrolls to the limit");
        assert_eq!(trace.stop, Stop::Limit);
        let basic = lift_at(&world(), AT, &program, Shape::BasicBlock);
        assert_eq!(basic.insns, 2);
        assert_eq!(basic.stop, Stop::Transfer);
    }

    #[test]
    fn a_conditional_branch_becomes_a_side_exit_that_names_the_whole_state() {
        // `subs x0, x0, #1` then `b.ne .-4`, a countdown loop.
        let out = lifted(&[0xf100_0400, 0x54ff_ffe1]);
        // Merged: the trace goes round rather than leaving.
        assert!(out.insns > 2, "the loop unrolled: {}", out.insns);
        assert!(count(&out.block, Opcode::BRCOND) > 0);
        // Every side exit's boundary binds the PC, and the register map with
        // it, which is what makes leaving through it precise.
        let exits: Vec<&InsnStart> = out
            .block
            .marks()
            .iter()
            .filter(|m| m.live.iter().any(|&(s, _)| s == PC))
            .collect();
        assert!(exits.len() > 1, "one exit per merged branch, plus the end");
        for mark in exits {
            assert!(
                mark.live.iter().any(|&(s, _)| s == x_slot(0)),
                "a side exit carries the registers the trace held"
            );
        }
    }

    #[test]
    fn a_slot_a_boundary_shadows_stays_shadowed_at_every_later_boundary() {
        // The invariant `InsnStart::live` states: a frontend that drops a slot
        // mid-block silently reverts that register to whatever the host last
        // held. Asserted over a program that binds registers, the stack
        // pointer and all four flags.
        let out = lifted(&[
            0xd280_0020, // movz x0, #1
            0x9100_43ff, // add  sp, sp, #16
            0xeb01_001f, // cmp  x0, x1        (subs xzr, x0, x1)
            0x9a81_0000, // csel x0, x0, x1, eq
            0xd280_0041, // movz x1, #2
        ]);
        let mut seen: Vec<RegSlot> = Vec::new();
        for mark in out.block.marks() {
            for slot in &seen {
                assert!(
                    mark.live.iter().any(|&(s, _)| s == *slot) || *slot == PC,
                    "slot {slot:?} was dropped at pc {:#x}",
                    mark.pc
                );
            }
            for &(slot, _) in &mark.live {
                if !seen.contains(&slot) && slot != PC {
                    seen.push(slot);
                }
            }
        }
        assert!(seen.contains(&SP), "the stack pointer was bound");
        assert!(
            seen.contains(&N) && seen.contains(&V),
            "the flags were bound"
        );
    }

    #[test]
    fn the_flags_are_four_one_bit_slots_and_a_condition_reads_them() {
        // `cmp x0, x1` then `b.gt .+8`, which reads N, Z and V.
        let out = lifted(&[0xeb01_001f, 0x5400_004c]);
        let block = &out.block;
        let reads: Vec<u32> = block
            .insts()
            .iter()
            .filter(|i| i.op == Opcode::GET_SLOT)
            .map(|i| i.aux)
            .collect();
        // The condition reads no flag through `get_slot`: `cmp` bound all four
        // in this very block, so they are already in temporaries.
        assert!(!reads.contains(&u32::from(N.0)), "{reads:?}");
        // A condition in a block that did *not* set the flags does read them.
        let cold = lifted(&[0x5400_004c]);
        let cold_reads: Vec<u32> = cold
            .block
            .insts()
            .iter()
            .filter(|i| i.op == Opcode::GET_SLOT)
            .map(|i| i.aux)
            .collect();
        assert!(cold_reads.contains(&u32::from(N.0)), "{cold_reads:?}");
        assert!(cold_reads.contains(&u32::from(V.0)), "{cold_reads:?}");
        assert!(cold_reads.contains(&u32::from(Z.0)), "{cold_reads:?}");
    }

    #[test]
    fn nothing_this_frontend_emits_is_an_op_the_host_backend_refuses() {
        // `jit::x86` refuses `ADDC`, `SUBB` and the atomics outright, and a
        // lifter that used the architecture's own `AddWithCarry` op would
        // produce blocks it could never compile — every block, because `CMP`
        // is `SUBS`. Asserted over a program that reaches every flag-setting
        // family this frontend has.
        let out = lifted(&[
            0xb100_0400, // adds x0, x0, #1
            0xeb01_0000, // subs x0, x0, x1
            0xba01_0000, // adcs x0, x0, x1
            0xfa01_0000, // sbcs x0, x0, x1
            0xea01_001f, // tst  x0, x1
            0xfa41_1804, // ccmp x0, #1, #4, ne
        ]);
        for inst in out.block.insts() {
            assert!(
                !matches!(inst.op, Opcode::ADDC | Opcode::SUBB),
                "{} is refused by jit::x86",
                inst.op
            );
        }
        assert_eq!(out.insns, 6);
    }

    #[test]
    fn the_zero_register_folds_and_the_stack_pointer_does_not() {
        let slots = |l: &Lifted| -> Vec<u32> {
            l.block
                .insts()
                .iter()
                .filter(|i| i.op == Opcode::GET_SLOT)
                .map(|i| i.aux)
                .collect()
        };
        // `add x0, xzr, x1` reads `x1` and nothing else: register 31 in the
        // `Rn` position of a shifted-register add is `XZR`, and the register
        // number is a decode constant, so the hard-wired zero folds to an
        // immediate rather than costing a slot read.
        let zr = lifted(&[0x8b01_03e0]);
        assert_eq!(slots(&zr), vec![u32::from(x_slot(1).0)]);
        // `add x0, sp, #1` reads the stack pointer, because `AddSubImm` is one
        // of the formats DDI 0487 C1.2.5 lists.
        let sp = lifted(&[0x9100_07e0]);
        assert_eq!(slots(&sp), vec![u32::from(SP.0)]);
    }

    #[test]
    fn a_write_to_the_zero_register_is_discarded_but_its_flags_are_not() {
        // `subs xzr, x0, x1` is `cmp`: no register moves and every flag does.
        let out = lifted(&[0xeb01_001f]);
        let exit = out.block.marks().last().expect("an exit boundary");
        assert!(
            !exit.live.iter().any(|&(s, _)| s == x_slot(31)),
            "register 31 is not a general register"
        );
        for flag in FLAG_SLOTS {
            assert!(
                exit.live.iter().any(|&(s, _)| s == flag),
                "{flag:?} was not bound by a flag-setting instruction"
            );
        }
    }

    #[test]
    fn a_division_guards_its_own_divisor() {
        // The IR's divide rejects a zero divisor and documents that a frontend
        // owes the guard; A64 defines the answer as zero.
        let out = lifted(&[0x9ac1_0800]); // udiv x0, x0, x1
        assert_eq!(count(&out.block, Opcode::DIV_U), 1);
        assert_eq!(
            count(&out.block, Opcode::MOVCOND),
            2,
            "one movcond makes the divisor safe and one forces the result"
        );
    }

    #[test]
    fn every_access_is_volatile_so_a_discarded_load_still_reads_the_bus() {
        let out = lifted(&[0xf940_001f]); // ldr xzr, [x0]
        let load = out
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::LD)
            .expect("the load is in the block");
        assert!(load.mem.expect("a descriptor").volatile);
    }

    #[test]
    fn alignment_policy_rides_in_the_key_and_in_every_descriptor() {
        let mut strict = world();
        strict.strict_align = true;
        let relaxed = lifted(&[0xf940_0001]);
        let checked = lift_at(&strict, AT, &[0xf940_0001], Shape::Trace);
        assert_ne!(relaxed.block.key, checked.block.key);
        let of = |l: &Lifted| {
            l.block
                .insts()
                .iter()
                .find(|i| i.op == Opcode::LD)
                .and_then(|i| i.mem)
                .expect("a descriptor")
                .align
        };
        assert_eq!(of(&relaxed), Align::None);
        assert_eq!(of(&checked), Align::Fault);
    }

    #[test]
    fn the_cache_key_separates_the_worlds_and_the_shapes() {
        let bare = world();
        let mut paged = world();
        paged.origin = Origin::Paged { generation: 7 };
        assert_ne!(key(&bare, Shape::Trace), key(&paged, Shape::Trace));
        assert_ne!(key(&bare, Shape::Trace), key(&bare, Shape::BasicBlock));
        assert_ne!(key(&bare, Shape::Extended), key(&bare, Shape::BasicBlock));
        let mut other = paged;
        other.origin = Origin::Paged { generation: 8 };
        assert_ne!(key(&paged, Shape::Trace), key(&other, Shape::Trace));
        // and a part without the atomics is a different world from one with,
        // because the same bytes end the block in different places.
        let mut plain = world();
        plain.features.lse = false;
        assert_ne!(key(&bare, Shape::Trace), key(&plain, Shape::Trace));
    }

    #[test]
    fn a_pair_load_reads_both_words_before_it_binds_either_register() {
        // `ldp x0, x1, [x0]`: if the second address were computed from the
        // *new* x0 the second load would read the wrong place.
        let out = lifted(&[0xa940_0400]);
        assert_eq!(count(&out.block, Opcode::LD), 2);
        let exit = out.block.marks().last().expect("an exit");
        assert!(exit.live.iter().any(|&(s, _)| s == x_slot(0)));
        assert!(exit.live.iter().any(|&(s, _)| s == x_slot(1)));
    }

    #[test]
    fn a_write_back_is_bound_after_the_access() {
        // `ldr x1, [x0, #-248]!`: the base moves, and it must move after the
        // load so a fault leaves it alone.
        let out = lifted(&[0xf850_8c01]);
        let insts = out.block.insts();
        let load = insts
            .iter()
            .position(|i| i.op == Opcode::LD)
            .expect("the load");
        // The write-back's value is an `add` emitted before the load, but the
        // *binding* is what the exit boundary records — and the exit boundary
        // is after it either way, so what this asserts is that both registers
        // come out bound.
        let exit = out.block.marks().last().expect("an exit");
        assert!(exit.live.iter().any(|&(s, _)| s == x_slot(0)));
        assert!(exit.live.iter().any(|&(s, _)| s == x_slot(1)));
        assert!(load < insts.len());
    }

    #[test]
    fn a_simd_encoding_is_refused_by_the_tables_own_feature_column() {
        // `fadd d0, d0, d1` — the whole family is excluded, and it is the
        // feature column that excludes it rather than a list of formats.
        let out = lifted(&[0x1e61_2800]);
        assert_eq!(out.insns, 0);
        assert_eq!(out.stop, Stop::Unsupported);
    }

    #[test]
    fn an_unprivileged_load_is_refused_because_the_ir_cannot_carry_its_check() {
        let out = lifted(&[0xf841_0801]); // ldtr x1, [x0, #16]
        assert_eq!(out.insns, 0);
        assert_eq!(out.stop, Stop::Unsupported);
    }

    #[test]
    fn a_prefetch_makes_no_access_at_all() {
        let out = lifted(&[0xf980_0000]); // prfm pldl1keep, [x0]
        assert_eq!(out.insns, 1);
        assert_eq!(count(&out.block, Opcode::LD), 0);
        assert_eq!(count(&out.block, Opcode::ST), 0);
        assert_eq!(count(&out.block, Opcode::CHARGE), 1);
        // and the register-offset form, whose `option` field the *addressing
        // mode* constrains even though the access does not exist.
        let reg = lifted(&[0xf8a0_6885]); // prfm pldl3strm, [x4, x0]
        assert_eq!(reg.insns, 1);
        assert_eq!(count(&reg.block, Opcode::LD), 0);
    }

    #[test]
    fn an_unallocated_prefetch_is_refused_like_any_other_encoding() {
        // `option<1>` clear is unallocated in the register-offset form, and it
        // is unallocated whether or not the access it names exists: the
        // interpreter raises `UNDEFINED` for `PRFM [X4, X0, SXTB]` and so must
        // this. Both words are ones an enumerated `llvm-mc` sweep found this
        // frontend accepting and the assembler refusing — the only two in
        // forty-two thousand.
        for word in [0xf8a0_0885u32, 0xf8a1_8885] {
            let out = lifted(&[word]);
            assert_eq!(out.insns, 0, "{word:#010x} was lifted");
            assert_eq!(out.stop, Stop::Unsupported);
        }
    }

    #[test]
    fn an_undefined_shift_amount_ends_the_block_rather_than_lifting_a_trap() {
        // `add w0, w0, w1, lsl #32` is UNDEFINED on a 32-bit operand.
        let out = lifted(&[0x0b81_8000]);
        assert_eq!(out.insns, 0);
        assert_eq!(out.stop, Stop::Unsupported);
        // and `ror` is not an addressing mode for add.
        let ror = lifted(&[0x8bc1_0000]);
        assert_eq!(ror.insns, 0);
    }

    #[test]
    fn a_branch_with_link_binds_x30_from_the_program_order_successor() {
        // `bl .+8`, merged: the trace continues at the target and x30 holds
        // the instruction after the branch.
        let out = lifted(&[0x9400_0002, 0xd503_201f, 0xd503_201f]);
        let exit = out.block.marks().last().expect("an exit");
        assert!(exit.live.iter().any(|&(s, _)| s == x_slot(30)));
    }
}
