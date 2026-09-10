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
//! ### What decision 2 costs, and what a comparison against QEMU is measuring
//!
//! `ROADMAP.md` §8's gate is **within 2× of QEMU wall-clock**, black-box.
//! QEMU does not do per-access cycle accounting by default — `-icount` is off
//! — so a straight comparison prices this decision as if it were overhead.
//! Whether that gate is fair is a judgement, but it cannot be made at all
//! without the number, so here is the number.
//!
//! `benches/a64_linux_boot.rs`, twenty guest seconds of Linux 6.12 `arm64` on
//! `machines/arm64-virt.machine` under `engine = "jit-host"`, host
//! instructions by callgrind with `--smc-check=all-non-file`:
//! **44 219 859 493** host instructions for **154 233 958** guest
//! instructions retired in 14 283 856 blocks — 286.7 host instructions per
//! guest instruction, of which the code the JIT generated is 28.8 and
//! replaying the deferred charges and boundaries is **62.1**. The replay is
//! **2.2× the cost of the translated guest code it is bookkeeping for**, and
//! it is the largest single row in the profile.
//!
//! It is that large because it is **two events per guest instruction on every
//! frontend in this tree** and always exactly two: `cpu::arm::a64::lift`,
//! `cpu::riscv::lift` and `cpu::x86::lift` each emit one
//! [`Opcode::INSN_START`] and one [`Opcode::CHARGE`] per guest instruction and
//! no other charge at all — the fetch tick, static on A64, one or two on
//! RISC-V, computed on x86 — and every other tick a guest spends is charged by
//! the *host* inside an access. So the event stream a backend replays is a run
//! of `(boundary, charge)` pairs — 168 507 057 boundaries and 154 223 996
//! charges over this boot, the boundaries exceeding the guest instructions by
//! exactly the one exit boundary each block closes with — and the shape is a
//! property of the IR rather than of any one frontend.
//!
//! Where the 9 577 607 664 host instructions go, by the file each machine
//! instruction was compiled from (`jit::x86`'s `flush_thunk` inlines the whole
//! of `IrHost`, so the callee columns are that host's own work):
//!
//! | | Ir | share |
//! | --- | --- | --- |
//! | `jit::x86::rt` — the context bookkeeping, the match, the arms | 4 797 390 200 | 50.1% |
//! | `cpu::arm::a64::engine` — `charge`, `insn_start` and `spent`, inlined | 1 456 632 299 | 15.2% |
//! | `core::slice::index` — `all[lo..hi]` and `marks().get(i)` | 797 845 746 | 8.3% |
//! | `cpu::arm::a64::exec` — `Exec::charge`, the two counters themselves | 785 384 900 | 8.2% |
//! | the event iterator (`slice::iter`, `ptr::non_null`) | 769 243 342 | 8.0% |
//! | `core::iter::range` — **`for _ in 0..ticks` inside `Host::charge`** | 524 562 606 | 5.5% |
//! | `core::cmp` — the two range clamps | 278 041 514 | 2.9% |
//! | `alloc::raw_vec` — `Block::marks()` | 168 507 057 | 1.8% |
//!
//! Counted off the compiled code instead, one **boundary** event is 38 host
//! instructions and one **charge** event is 17, with 13 more to enter and
//! leave each of the 61 908 759 replays; that model over-predicts the measured
//! total by 2.6%, which is the boundaries that leave the guard early. Of a
//! boundary's 38: nine are finding the
//! [`InsnStart`] record and bounds-checking the index, seven are the context
//! stores a fault or an exit reads back, three are [`IrHost::insn_start`],
//! fourteen are asking whether the tick allowance is gone, and five are the
//! loop. Of a charge's 17: three are the context, nine are
//! [`IrHost::charge`], five are the loop.
//!
//! ### The counterfactual, which is the number the comparison needs
//!
//! A build that accounts per **region** instead of per guest instruction —
//! one summed charge and one boundary per flush, which is what a translator
//! with no accuracy guarantee keeps for its own budget — was built in a
//! scratch tree and thrown away. It is not shippable: it gives up the tick
//! stream, the preemption point and the fault site all at once. It ran the
//! same guest work, and that is checkable rather than asserted — 61 899 909
//! flush calls against 61 908 759, 18 945 326 inlined loads against
//! 18 946 842, 14 278 681 blocks against 14 283 856, every one within 0.04%.
//!
//! | | baseline | per-region | |
//! | --- | --- | --- | --- |
//! | host instructions | 44 219 859 493 | 37 805 131 461 | **−14.51%** |
//! | the replay | 9 577 607 664 | 4 311 268 755 | **−54.99%** |
//! | the code the JIT generated | 4 449 445 570 | 4 240 949 395 | −4.69% |
//!
//! So **per-guest-instruction accounting costs 5.27 G host instructions on
//! this boot — 11.9% of it, 34.1 per guest instruction** — over accounting at
//! the granularity a translator would keep anyway. (The whole-run figure is
//! larger than the replay's because the counterfactual also lifted 17% fewer
//! distinct blocks, a second-order effect of slightly different block ends;
//! 11.9% is the number that is only about this decision.) It is a *lower*
//! bound on what the guarantee costs: the per-access ticks the host charges
//! inside `IrHost::load` and `FastMem::note_fast_load`, and the `MemAttrs`
//! plumbing that carries them, are not removed by it.
//!
//! ### What of that is not buying accuracy
//!
//! Two things, each measured on its own against the same boot and each
//! reaching the same `Machine::state_hash` (`0x9cc4de4dee51678b`) — which is
//! the acceptance test for anything in this paragraph, since a saving that
//! changes the hash is not a saving:
//!
//! * **`Host::charge` loops.** `cpu::arm::a64::engine`'s implementation is
//!   `for _ in 0..ticks { self.exec.charge() }` over two counters that both
//!   just add. The loop scaffolding alone is 524 562 606 host instructions —
//!   5.5% of the replay for a loop whose trip count is one — and replacing it
//!   with an addition each is **−0.93%** of the whole run.
//! * **The pair is replayed as two events.** A boundary and its charge are
//!   adjacent in the event list by construction: every frontend emits them
//!   together, and neither a region split nor a branch target can fall between
//!   them except in a shape no frontend produces. A backend that fuses them
//!   pays one dispatch, one loop tail and one `host_of` instead of two:
//!   **−1.40%**, and `flush_thunk` gets *shorter*, from 98 compiled
//!   instructions to 80.
//!
//! ### And one that looked like it and is not, which is the more useful result
//!
//! [`IrHost::spent`] is asked once per boundary — fourteen of a boundary's
//! thirty-eight instructions — and inside a region the answer cannot change:
//! a region contains no call site, so nothing but [`Opcode::CHARGE`] moves the
//! host's tick count and the charges in a region are static. So a host could
//! publish its *headroom* instead, a backend could take that once per replay,
//! and every `spent()` until the headroom is spent down could be skipped. The
//! answer would be the same answer, exactly; the seam even licenses it, since
//! `spent` is documented as an observation the two backends already "ask a
//! different number of times on paths that must stay indistinguishable".
//!
//! **It was built, on top of the two savings above, and it is slower.** Two
//! shapes were measured, both reaching the same state hash:
//!
//! | | host instructions | against the two savings above |
//! | --- | --- | --- |
//! | the two savings above | 43 358 070 089 | — |
//! | headroom taken lazily, at the first boundary that would ask | 45 100 550 983 | **+4.02%** |
//! | headroom taken eagerly, once per replay | 44 159 528 013 | **+1.85%** |
//!
//! The mechanism is visible in the compiled code and it is not about the
//! arithmetic. `flush_thunk` today uses **no callee-saved registers at all** —
//! it has no prologue and no epilogue, and `Ctx` in `rdi` plus the event
//! cursor is the whole of its state. A headroom counter is one more value live
//! across the loop, and that one value buys a three-push, three-pop frame on
//! every one of the 61 908 759 calls plus the spills around it: 98 compiled
//! instructions become 103 eager and 119 lazy. The `spent()` calls avoided are
//! worth less than the frame.
//!
//! Recorded here rather than left to be rediscovered, with the thing that
//! would have to change for it to work: the headroom would have to live in
//! the backend's execution context rather than in a local — refreshed at
//! block entry and by the thunks that can charge, which are paying a call
//! anyway — so that nothing new is live across the replay loop. That is a
//! backend design, it is unproven, and the two savings above are worth more
//! than it is.
//!
//! ### And what would have to be given up to do better than that
//!
//! The rest of the 62.1 is buying something, and it is worth naming what,
//! because each is a `ROADMAP.md` §0 non-negotiable rather than a preference:
//!
//! * **The seven context stores and the nine finding the mark** buy a fault
//!   delivered at the guest instruction that took it, with the architectural
//!   state that instruction started with. Give them up and an exception
//!   arrives at the block's entry PC.
//! * **The allowance question at every boundary** buys a translated block that
//!   leaves at exactly the boundary the interpreter leaves at.
//!   `cpu::arm::a64::engine`'s `spent` folds its generic timer's comparator
//!   into that question for precisely this reason: the interpreter samples the
//!   timer once per instruction, and a chain that ran on to a region end would
//!   take the interrupt tens of ticks later at a different `ELR_EL1`. That is
//!   a divergence, and it is the one the twenty-second boot in
//!   `tests/engine_longrun.rs` exists to catch.
//! * **The charge itself** buys the state hash. Decision 2, above.
//!
//! There is no version of this that is cheap *and* keeps all three.
//! **11.9% of a real arm64 Linux boot is what per-guest-instruction
//! accounting costs; 1.95% of it — a sixth — is implementation rather than
//! guarantee, and the two changes that recover it are named above.** A
//! wall-clock comparison against a translator that does not make the promise
//! is measuring the promise, and it should say so.
//!
//! Every number here is reproducible from `benches/a64_linux_boot.rs`, whose
//! docs give the callgrind invocation; the per-file split wants
//! `CARGO_PROFILE_BENCH_DEBUG=line-tables-only` on the build, which does not
//! change the instruction count.
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
//!   instructions ahead of one, and [`hoist_slot_reads`] repoints every branch
//!   when it moves one. That was a latent bug for as long as no
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
//! because decision 1 is a debt until they exist rather than after;
//! [`hoist_slot_reads`] is here because a slot read is where a backend that
//! defers its bookkeeping has to stop deferring, and a frontend that emits one
//! per guest instruction turns "per region" back into "per instruction" — a
//! Linux boot on `arm64-virt` spent 19.4% of its host instructions replaying
//! that bookkeeping against 7.7% in the code the JIT generated; and
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
pub use pass::{Liveness, TempLife, eliminate_dead_code, hoist_slot_reads};
pub use regalloc::{Allocation, CallSites, Home, MAX_REGS, RegBanks, linear_scan};
pub use types::{Const, Temp, Type};
pub use verify::verify;
