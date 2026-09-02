//! The architecture-neutral translation IR.
//!
//! `ROADMAP.md` §9 is the design; this module is where it becomes types. The
//! shape is deliberately small and low-level: typed temporaries, SSA within a
//! translation block, and a helper call for anything rare. The op set is
//! chosen so the *common* case of every target ISA lowers to one or two host
//! instructions, and everything else becomes a [`Opcode::CALL_HELPER`] rather
//! than a new opcode.
//!
//! Nothing here knows what a guest is. A frontend lifts guest bytes into a
//! [`Block`]; a backend lowers a [`Block`] into something callable. The two
//! never meet.
//!
//! # Why this module is `no_std`
//!
//! `ROADMAP.md` §11's target table has a bare-metal row whose engine is the
//! **portable IR interpreter**, so the IR and that interpreter must both build
//! without `std`. Host code generation — W^X buffers, wasm module
//! instantiation — lives in `jit/`, above the `std` line (CLAUDE.md, "`no_std`").
//!
//! # Six decisions, and what they were derived from
//!
//! Each of these was settled by surveying **our own** nine interpreters under
//! `cpu/`, never another project's translator (CLAUDE.md, "Provenance"). Where
//! a decision departs from §9's sketch, it says so and why.
//!
//! ## 1. Flags are ordinary temporaries, not a packed word and not a deferred triple
//!
//! Six of our nine cores are flag machines, and all six store flags eagerly in
//! a guest-visible packed word that `save`/`load` and gdb read — `Regs::p`,
//! `Regs::f`, `Regs::eflags`, `Regs::sr`, `Regs::cpsr`, `Regs::xpsr`. The IR
//! nonetheless computes each flag into its own temporary and materializes the
//! packed word only where something can observe it (see decision 3).
//!
//! The alternative — a deferred `{op, a, b, result}` triple, computed on
//! demand — cannot express what our cores actually do:
//!
//! * **x86 needs a flavour tag per site, not one op tag.** `AF` comes from
//!   `(a ^ b ^ r) & 0x10` for `ADD`, from bit 4 of the result for `SHL`
//!   (the microcode is `ADD dst,dst`), is cleared by `AND`, and falls out of
//!   the division loop for `DIV`. Every one of those is asserted by the
//!   SingleStepTests corpus, so none can be approximated.
//! * **The Z80's `SCF`/`CCF` read the *previous* instruction's flag output**
//!   through `Q` (`cpu::z80::exec`), which is not a function of the current
//!   instruction's operands at all.
//! * **The m68k's `ADDX`/`SUBX`/`ABCD` carry a sticky `Z`** that is only ever
//!   cleared, never set: `Z_new & Z_old`, a loop-carried dependency.
//! * **6502 decimal `ADC` takes three flags from three different
//!   intermediates** — N and V from the pre-correction sum, Z from the plain
//!   binary sum, because the zero flag is computed by hardware that never sees
//!   the decimal correction.
//!
//! As temporaries each of those is simply a different expression, and the
//! optimiser earns the speed back the other way: x86's `PF` popcount and the
//! Z80's undocumented `XF`/`YF` bits are computed on nearly every ALU
//! instruction and read almost never, so dead-code elimination removes them
//! wherever the packed word is not observed. ARM's cores are already written
//! this way — `alu` returns `(result, n, z, c, v)` and the caller decides
//! whether to commit — so the `S` bit becomes "these temporaries have no
//! consumers" rather than a branch.
//!
//! The cost is honest and worth stating: **this design is strictly worse than
//! eager packing until liveness and DCE exist.** They are in §9's pass list;
//! they are not optional.
//!
//! ## 2. Ticks are an output, not a budget
//!
//! A core's cycle counter is in its snapshot, and `Machine::state_hash` is
//! `fnv1a` over that snapshot. `ROADMAP.md` §0 requires a bit-identical state
//! hash *across the interpreter and the JIT for the same guest*, so a
//! translated block that charges 7 ticks where the interpreter charged 8 does
//! not drift subtly — it fails the phase-5 gate as a hash mismatch.
//!
//! So the IR carries ticks explicitly, in [`Opcode::CHARGE`], rather than
//! leaving them to a backend convention. A frontend emits the same charges its
//! interpreter makes, at the same points, and the verifier can check that a
//! charge is never folded across a guest instruction boundary. Deferring
//! *materialization* into a host register is a backend optimization; deferring
//! the *count* is a bug.
//!
//! This is why per-access accounting survives into the JIT at all
//! (CLAUDE.md, "CPU cores"): the count is data-dependent — a misaligned RISC-V
//! access splits into per-byte accesses that each charge and each translate
//! separately — so no post-hoc table of instruction lengths can reproduce it.
//!
//! ## 3. `insn_start` names the whole architectural state, and carries ticks
//!
//! §9 requires a marker at every guest instruction boundary recording the
//! guest PC and the live guest-register-to-temporary mapping, so a fault
//! halfway through a block can materialize exactly the state the ISA
//! specifies. Two additions fell out of the survey:
//!
//! * **`next_pc`, not a length.** `Exit::len` is derived as
//!   `next_pc - this_pc`, and on x86 the length is not a static property of
//!   the opcode.
//! * **A tick column.** Decision 2 means the exact retired-tick count must be
//!   reconstructible at the same offsets the PC is, or a snapshot taken at a
//!   fault hashes differently.
//!
//! And the mapping is over a [`RegSlot`] space the *frontend* numbers, not
//! over "the sixteen or thirty-two obvious registers", because five of our
//! nine cores keep guest-visible state outside their register struct: the
//! Z80's `WZ` (read out by `BIT n,(HL)`) and `Q`, MIPS's `in_delay` (`EPC`
//! names the branch, not the delay slot), the 6502's open-bus latches (inputs
//! to `MemAttrs` on every access), RISC-V's `reservation`, the m68k's
//! prefetch queue.
//!
//! ## 4. A mode change is a barrier, not a call
//!
//! ARM banks `r13`/`r14` per mode, ARMv7-M swaps `sp` between MSP and PSP, the
//! m68k swaps `a7` on any write to `SR`. A mode change moves the *meaning* of
//! a register, not just its value, so every instruction that can cause one is
//! both a helper call **and** a hard barrier for the register-to-temporary
//! mapping. It may not be treated as an opaque call that leaves the mapping
//! intact.
//!
//! ## 5. Four ops from §9's list are not defined here
//!
//! `orc`, `eqv`, `nand` and `nor` have no consumer in any of our nine cores —
//! they are PowerPC-shaped, and every backend would owe a lowering for an op
//! nothing emits. `andc` is kept, because ARM's `BIC` is exactly that. If a
//! guest that needs them lands, they are two lines each; until then they are
//! recorded here as deliberately absent rather than forgotten.
//!
//! ## 6. Four ops from outside §9's list are defined here
//!
//! Each is justified by a real instruction in one of our `isa.rs` tables, and
//! each replaces three-to-five ops plus a flag dance in a *common* case:
//!
//! | Added | Because |
//! | --- | --- |
//! | [`Opcode::ADDC`] / [`Opcode::SUBB`] | add/subtract with a one-bit carry *in* and *out*. §9's `add2`/`sub2` are carry **chains** — a 2N-bit value in two N-bit temps — which is a different shape. The 6502's only add is `ADC`; ARM expresses its entire ALU as add-with-carry (`SUB` is `add(a, !b, true)`); x86's `add` takes a `carry: bool` parameter. |
//! | [`Opcode::ROTLC`] / [`Opcode::ROTRC`] | rotate through carry by one — an (N+1)-bit rotate. Six of nine ISAs: 6502 `ROL`/`ROR`, Z80 `RL`/`RR`, SM83, x86 `RCL`/`RCR` at count 1, m68k `ROXL`/`ROXR`, and ARM `RRX`, which is *encoded* as `ROR #0`. |
//! | [`Opcode::MULHSU`] | RISC-V's `mulhsu` is signed-by-unsigned high multiply, which neither `mulu2` nor `muls2` expresses, and it appears in ordinary compiler output. |
//! | [`Opcode::LD_EXCL`] / [`Opcode::ST_EXCL`] | `cmpxchg` cannot express a reservation that fails *because a trap happened in between*. RISC-V `LR`/`SC` and ARMv7-M `LDREX`/`STREX` both keep a monitor in CPU state that a trap or a foreign store breaks. |
//!
//! # Known gaps, recorded rather than discovered twice
//!
//! Found by building the first frontend and the first backend against this
//! IR. None of them blocks the RV64I path; each is written down here so the
//! next person meets a note instead of a surprise.
//!
//! * **`PHI` cannot be executed as defined**, and superblocks landed without
//!   needing it — which corrects what this note used to say. Nothing in
//!   [`Inst`] records which predecessor each operand arrived from, and §9
//!   lists `phi` as *"required — superblocks span branches"*. They do, but a
//!   superblock is one entry and **many exits**: `cpu::riscv::lift` inlines one
//!   side of every branch and turns the other into a side exit that leaves the
//!   block, so control never *rejoins* and there is no merge point for a `phi`
//!   to name. `phi` becomes necessary at the first construction that merges
//!   paths back together — an if-conversion, or a tier-2 region — and an edge
//!   encoding is owed then rather than now.
//! * **Atomics carry no [`MemOp`]**, so they take their width from the
//!   instruction type and have no endianness or address space of their own.
//!   Since the IR has no `i8`/`i16`, x86's `lock xadd byte` and ARM's
//!   `LDREXB` are not expressible at all today.
//! * **[`Opcode::BSWAP`]'s lane width has no field of its own** and is read
//!   from the immediate.
//! * **[`Opcode::MOVCOND`] accepts two shapes** — select on a one-bit value,
//!   or compare a pair and select. Both are natural; the IR should pick one.
//! * **Nothing checks that [`InsnStart::ticks`] agrees with the charges
//!   before it.** Deliberate, because a helper may charge through the host and
//!   legitimately break the equality — but it means the differential harness,
//!   not the verifier, is what catches a frontend that miscounts. For the
//!   first frontend that harness is
//!   `cpu::riscv::differential`, which compares the column
//!   against the ticks the interpreter charged on every case it runs.
//!
//! ## 7. A block is straight-line SSA with forward branches, and several exits
//!
//! Not a decision that was made up front — it is what superblocks turned out
//! to need, and it is recorded here because three separate things depend on it
//! and none of them says so locally.
//!
//! * A [`Opcode::BRCOND`] branches **forward** and lands inside the block. The
//!   verifier enforces it because [`Liveness`] is a single backward walk, which
//!   is exact for forward control flow and silently *wrong* for a loop rather
//!   than being an error.
//! * A terminator may appear **anywhere**, and the last instruction must be
//!   one. That is what a side exit is: an inline `insn_start`/`exit_tb` pair
//!   the trace branches over.
//! * A branch target is an instruction *index*, so
//!   [`eliminate_dead_code`] repoints every branch when it drops
//!   instructions ahead of one. That was a latent bug for as long as no
//!   frontend emitted a branch.
//!
//! # What is deliberately not here yet
//!
//! Vector ops (`v128` exists as a [`Type`] so the IR can carry the values;
//! §9 adds the ops with the SIMD work, not before), the rest of the pass
//! pipeline — constant folding, copy propagation, the load/store reordering
//! rules — and every *host* backend. [`Interp`] is
//! here, because §11's bare-metal row runs on it and it is the oracle the host
//! backends are differentially tested against; **liveness and dead-code
//! elimination are here**, in [`Liveness`] and [`eliminate_dead_code`],
//! because decision 1 is a debt until they exist rather than after; and
//! **so is the register allocator**, in [`linear_scan`], because everything it
//! decides — which intervals overlap, which definitions a forward branch can
//! jump over, which values outlive a call — is a property of the block rather
//! than of a host. A backend contributes two lists of register numbers and
//! gets back one [`Home`] per temporary.
//! [`Type::V128`] and the float types
//! carry values today so that a helper call can take and return them —
//! tier-1 floating point is helper calls into the soft-float implementation
//! (§9.1), which is what makes guest FP bit-reproducible across hosts.

mod block;
mod interp;
mod op;
mod pass;
mod regalloc;
mod types;
mod verify;

pub use block::{Block, BlockBuilder, InsnStart, Inst, RegSlot};
pub use interp::{Fault, Interp, IrHost, Outcome};
pub use op::{
    AccessKind, Align, Cond, Endian, MemOp, MemSpace, Opcode, SegId, Sign, bitfield_aux,
    bitfield_parts,
};
pub use pass::{Liveness, TempLife, eliminate_dead_code};
pub use regalloc::{Allocation, Home, MAX_REGS, RegBanks, linear_scan};
pub use types::{Const, Temp, Type};
pub use verify::verify;
