//! The x86 frontend: guest instructions lifted into [`ir::Block`](crate::ir::Block)s.
//!
//! The second frontend for `ROADMAP.md` §9's translation IR, after
//! `cpu::riscv::lift`. RISC-V went first because it has **no condition flags**
//! and a fixed instruction length, so it exercised the IR's structure —
//! boundaries, ticks, the register mapping — without the two things x86 makes
//! unavoidable. Those two things are this file's subject.
//!
//! # The subset, exactly
//!
//! **32-bit protected mode, integer instructions, with or without paging.** A
//! documented subset done
//! exactly beats a broad one done approximately, so the world this frontend
//! lifts in is checked rather than assumed — [`World::of`] refuses everything
//! else — and within it the instruction list is closed:
//!
//! * The eight ALU operations in all four of their ModRM forms, their
//!   accumulator-immediate forms, and groups `80`/`81`/`83`. Plus `TEST`,
//!   `NOT`, `NEG`, `INC`, `DEC`.
//! * `MOV` in every general-register form, `MOVZX`, `MOVSX`, `LEA`, `BSWAP`,
//!   `CBW`/`CWDE`, `CWD`/`CDQ`, `BSF`, `BSR`, `SETcc`, `CMOVcc`,
//!   `LAHF`/`SAHF`, `NOP`.
//! * Every shift and rotate except `RCL`/`RCR` by more than one:
//!   `SHL`/`SHR`/`SAR`/`ROL`/`ROR` by an immediate and by `CL`, `RCL`/`RCR`
//!   by one, and the undocumented `SETMO`.
//! * `MUL`, `IMUL` — one-operand, two-operand and three-operand.
//! * `PUSH`, `POP`, `LEAVE`, `CALL rel32`, `CALL r/m`, `RET`, `RET imm16`,
//!   `JMP rel8`/`rel32`/`r/m`, all sixteen `Jcc` in both encodings.
//! * `CLC`, `STC`, `CMC`, `CLD`, `STD`.
//!
//! Everything else ends the block with a terminator that hands the PC back to
//! the interpreter, which stays the oracle (CLAUDE.md, "CPU cores"). The
//! exclusions that are *decisions* rather than gaps:
//!
//! | Excluded | Why |
//! | --- | --- |
//! | x87, SSE, MMX | the IR has no vector ops (`ROADMAP.md` §9 adds them with the SIMD work) and tier-1 floating point is a helper call into soft-float that no x87 entry point exists for yet |
//! | long mode, `REX` | a second operand-size and address-size lattice on top of the one below, **and a carry the IR cannot compute at this width** — see "Widening the world" |
//! | real mode and virtual-8086 | `segment << 4 + offset` with a *16-bit offset wrap* is a different address path in `exec`, checked against three million hardware vectors, and generalising it is how that accuracy gets lost |
//! | a unified translation buffer | paging itself is **in** the subset now ([`Origin::Paged`]); what is out is paging on a part whose instruction and data translations share one array, where a data operand evicts the code page and the *next fetch* pays for a walk no static analysis can place — see "Paging" |
//! | [`Smc::Guard`] under paging | the in-block guard compares **linear** pages and two of them may alias one physical page. [`Smc::EndBlock`] is the answer, and [`lift`] refuses the other combination rather than emitting a check that can miss |
//! | `DIV`, `IDIV` | `#DE` is an exception the block cannot deliver, and the undefined flags come out of `exec::cord`'s trial-subtraction **loop**, which a block with only forward branches cannot express |
//! | the string primitives, `REP` | a loop inside a block; the IR's verifier rejects a backward [`Opcode::BRCOND`] because `ir::pass`'s liveness is a single backward walk |
//! | `LOCK`, `XCHG` with memory | the IR's atomics carry no [`MemOp`], so a byte- or word-wide atomic has no type to name (`ir`'s "Known gaps") |
//! | segment loads, `LES`/`LDS`/`LSS`, far transfers, `INT`, `IRET`, `HLT` | a descriptor load is a mode change, which `ir`'s decision 4 makes a helper call **and** a hard barrier |
//! | `PUSHF`/`POPF` | they observe the packed word, and `POPF`'s `IOPL` and `POPF_FORBIDDEN` rules are privilege logic rather than arithmetic. `LAHF`/`SAHF` *are* lifted, so the packed low byte is not untested |
//! | a `67` address-size prefix | 16-bit addressing inside a 32-bit segment is a second effective-address form for no guest that matters |
//! | `RCL`/`RCR` by more than one | an N-bit rotate through carry is a loop. See "What the IR could not say" |
//!
//! This frontend is the **third** consumer of `isa`'s declarative rows, never a
//! second decoder: [`isa::decode_stream_as`] is the same function `exec` and
//! `disasm` drive, so an encoding's length, its ModRM byte, its group
//! resolution and its immediates are decided in one place (CLAUDE.md,
//! "CPU cores").
//!
//! # Flags: computed as temporaries, published lazily, and elided where dead
//!
//! [`ir`](crate::ir)'s decision 1 says flags are ordinary temporaries rather
//! than a packed word or a deferred `{op, a, b, result}` triple, and names x86
//! as the reason the triple cannot work: `AF` comes from `(a ^ b ^ r) & 0x10`
//! for `ADD`, from bit 4 of the *result* for `SHL` because the microcode is an
//! `ADD dst,dst`, and is cleared outright by `AND`. Every one of those is
//! asserted by the SingleStepTests corpus, so none can be approximated.
//!
//! So each of `CF`, `PF`, `AF`, `ZF`, `SF` and `OF` is its own [`RegSlot`] and
//! its own [`Type::I1`] temporary, and `EFLAGS` is never assembled inside a
//! block. Three consequences, and the third is the one that had to be got
//! right:
//!
//! 1. **The packed word is materialized only where something observes it.**
//!    The host's slot storage is written at the block's exit, or at a fault,
//!    and not once per instruction (`ir::interp`, "Materializing guest state").
//! 2. **`PF` costs a popcount on nearly every ALU instruction**, which
//!    decision 1 promised dead-code elimination would pay for. [`lift`]
//!    therefore runs [`eliminate_dead_code`] over its own output before
//!    returning it — the RISC-V frontend does not, because RISC-V has no
//!    flags to eliminate.
//! 3. **Dead-code elimination alone is not enough, and cannot be.** A
//!    temporary named in *any* [`InsnStart::live`] mapping is live by
//!    definition, because that mapping is what a mid-block fault reconstructs
//!    architectural state from. Name all six flags at every boundary and not
//!    one of them is ever removable, whatever nothing reads: the popcount
//!    stays, and decision 1's cost is paid with none of its saving.
//!
//! ## Flag elision, and exactly when it is sound
//!
//! [`Flags::Elide`] is the answer, and it is a claim about what can be
//! *observed* rather than about what is computed:
//!
//! > The boundary opened for guest instruction *i* may omit a flag slot when
//! > instruction *i* **unconditionally writes** that flag and instruction *i*
//! > **cannot fault**.
//!
//! Nothing between two boundaries can observe guest state except a fault (the
//! dispatcher checks its exit flag and drains guest stores at block
//! boundaries, `jit::dispatch`; there is no helper call in this subset, and a
//! [`Opcode::GET_SLOT`] of a flag never happens for a flag the lifter holds a
//! temporary for). So a flag value that the next instruction overwrites
//! before any fault site is unobservable, and dead-code elimination then
//! removes the whole chain that computed it — the popcount, its mask and its
//! comparison, in one backward walk.
//!
//! Both halves of the condition are load-bearing. A shift by `CL` writes no
//! flag at all when the count is zero, so its write is *conditional* and it
//! never elides. An instruction with a memory operand can fault on a segment
//! limit **before** it writes any flag, so the pre-instruction flags must
//! still be recoverable. `INC` and `DEC` deliberately preserve `CF`, so they
//! elide five flags and not the sixth.
//!
//! One of those halves is, today, conservative rather than observable, and
//! saying which is more useful than implying both are equally load-bearing: a
//! `CL` shift with a *memory* destination is out of the subset, so a `CL` shift
//! cannot fault, so eliding at its boundary would change nothing anyone could
//! see. Injecting exactly that bug is the one mutation
//! [`differential`](super::differential) does not catch, and it is asserted by
//! shape instead — `a_shift_by_cl_elides_nothing_because_it_may_write_nothing`
//! below. It becomes differentially observable the day the memory form is
//! lifted.
//!
//! This deliberately spends the "once a boundary names a slot, every later
//! boundary must name it too" invariant [`InsnStart::live`] states — which
//! that documentation also says is *"asserted per frontend"* rather than
//! checked. It is asserted here by
//! `a_flag_an_instruction_can_fault_at_is_never_elided` and
//! `inc_elides_five_flags_and_never_the_carry`, and it is asserted where it
//! matters by
//! [`differential`](super::differential), which faults in the middle of a
//! trace and compares every flag against the interpreter. [`Flags::Eager`]
//! keeps every flag at every boundary and exists so that the difference is a
//! measurement rather than an assertion: both settings are in the cache key,
//! the differential corpus runs both, and `benches/x86_dispatch.rs` reports
//! both.
//!
//! # Ticks, and where a block has to end
//!
//! [`ir`](crate::ir)'s decision 2: a tick count is a hashed *output*, so a
//! block that charges differently from the interpreter fails the phase-5
//! state-hash gate. On a 386 or 486 `exec` charges at exactly
//! three kinds of site:
//!
//! | Site | Count | Static? |
//! | --- | --- | --- |
//! | instruction fetch | `bus_clocks()` **per byte**, prefixes included — a 386 fetches one byte at a time through `Exec::fetch_at` and its prefetch queue is not modelled | **yes**, and it is why the block is page-bounded |
//! | the operation itself | `Op::clocks()`, charged once by `Exec::instruction` before execution | **yes**, from the resolved row |
//! | a data access | `bus_clocks()` per bus transaction | no — the operand's address decides how many there are |
//!
//! `Exec::prepare_ea`'s effective-address charge is an 8086 microcode cost and
//! is **not** charged on a 386, which is one of the reasons this frontend
//! starts at the 386 rather than at the part with the hardware corpus.
//!
//! So the static charge is `bus_clocks() * len + op.clocks()`, emitted as one
//! [`Opcode::CHARGE`] at the head of each guest instruction — which is exact,
//! because `Exec::instruction` makes every static charge before it makes any
//! access.
//!
//! Two structural rules follow:
//!
//! * **A block never leaves the linear page it started on.** Not the `EIP`
//!   page: `CS.base` need not be page-aligned, and the linear page is what
//!   bounds the bytes. What a guest store is matched against is the
//!   **physical** page ([`Lifted::page`], and `jit::Translation::page`), which
//!   with paging off is the same number.
//! * **The entry fetch is the only fetch translation a block can need**, and
//!   the caller charges it. With paging on a fetch may miss the guest's own
//!   translation buffer and charge a page-table walk — two to four bus reads
//!   and possibly an accessed-bit write — and the whole of a page-bounded
//!   block translates through one entry. That holds only where the
//!   instruction and data buffers are separate
//!   ([`paging::Buffers`](super::paging::Buffers)), which is why
//!   [`World::of`] refuses paging on a part where they are not: there a data
//!   operand evicts the code page's translation and the *next* fetch pays for
//!   a walk no static analysis of the block can place. See "Paging" below.
//!
//! ## Page-straddling instructions
//!
//! An x86 instruction is 1 to 15 bytes and may begin on one page and end on
//! the next, where the second page can fault mid-instruction. This frontend
//! does not lift such an instruction: the byte source refuses to leave the
//! entry page, [`isa::decode_stream_as`] reports the stream as truncated, and
//! the block ends with [`Stop::Page`] *before* the straddling instruction.
//! The interpreter then executes it and takes whatever fault it takes, at the
//! architecturally correct point, because it is the oracle and it re-fetches
//! byte by byte.
//!
//! That is the same answer RISC-V gives for a 32-bit instruction whose second
//! halfword is on the next page, and it costs the same thing: one short block
//! per page boundary. What it buys is that no instruction in a block can fault
//! *during its own fetch*, which is what makes the static charge above a
//! charge rather than a guess. A block whose **first** instruction straddles
//! lifts nothing at all, reports zero instructions, and the dispatcher answers
//! `Stop::Untranslatable` — which is the contract that already existed.
//!
//! # Paging, and what naming a block under it costs
//!
//! Paging is **in** the world this frontend lifts, on a part whose translation
//! buffers are split ([`paging::Buffers::Split`](super::paging::Buffers) — a
//! Pentium and after, which here is [`Variant::X86_64`]). Getting there took
//! four separate things, and the order below is the order they had to be done
//! in rather than a list of features.
//!
//! **1. The tick charge.** A block never leaves its page, so the whole of it
//! translates through one entry. What used to break that was a *second* walk
//! inside the block: `paging::Tlb` was one 32-entry array indexed
//! `page % 32` shared by fetches and data accesses, so a guest instruction
//! whose data operand lay 128 KiB from the code page evicted the code page's
//! own translation and the interpreter's *next* fetch re-walked and charged
//! two to four bus reads for it, at a point no static analysis of the block
//! could predict. That is fixed at the source rather than worked around here:
//! the buffer is now two buffers, which is what every part since the Pentium
//! has, and a 386 and a 486 keep the single buffer their manuals document —
//! which is exactly why they are **not** in the paged world and
//! [`World::of`] checks the arrangement rather than the part number.
//!
//! **2. Somewhere for a block's data accesses to be translated.** An IR
//! `ld`/`st` carries a linear address; under paging somebody has to turn it
//! into a physical one, charge the guest walk when it misses, and take the
//! fall-through walk that sets a page's dirty bit on the first write. That
//! somebody is the [`IrHost`](crate::ir::IrHost), and the answer is
//! `cpu::riscv::engine`'s: **not a memory path that agrees with the
//! interpreter's, the interpreter's**. `Exec::read_mem` and `Exec::write_mem`
//! are what `Exec::step` itself calls — the segment check, the translation
//! with its accessed and dirty bits, the page-crossing split, and the bus
//! transaction, each charging through the same `Exec::charge`. A host that
//! reimplemented them would have to reproduce a walk's tick cost and the
//! accessed-bit write-back, and `differential`'s own host is the evidence
//! that reproducing those is a job rather than a line. So this frontend asks
//! for nothing new: the accesses it already emits are chargeable because each
//! happens at an access the block really makes.
//!
//! **3. The self-modifying-code guard cannot stay linear.** `jit::cache`'s
//! slot records the guest-**physical** page the bytes were read from, and the
//! [`Smc::Guard`] sequence below compares a store's **linear** page against
//! the block's. With paging off those are the same number. With it on they
//! are not: two linear pages may alias one physical page, so a store through
//! the other mapping would miss a guard that can only see linear addresses,
//! and one linear page names a different physical page after a `CR3` reload.
//! A guard that can miss is worse than no guard, so under [`Origin::Paged`]
//! [`lift`] **refuses** [`Smc::Guard`] and [`Smc::EndBlock`] is the policy: a
//! store is the last guest instruction in its block, the dispatcher's
//! page-drain at the next boundary is reached before anything the store
//! changed can execute, and the drain is by physical page at both ends —
//! [`Lifted::page`] is the physical page the entry resolved to, and a host
//! notes the physical address its store reached.
//!
//! **4. The key has to name the mapping, and a `CR3` generation is the wrong
//! thing to name it with.** [`World::generation`] covers the segment bases;
//! nothing bumps it when the page tables change, and `INVLPG`, a `CR3` write
//! and an accessed-bit transition all change what the same `EIP` means. The
//! obvious repair is a translation generation like `cpu::riscv::lift`'s
//! `Origin::Paged { generation }` — and `cpu::riscv::engine` measured that
//! being unusable on a real guest: Linux bumps its counter on every `SRET`
//! and `MRET`, so a cache keyed on it missed every time and ran four times
//! slower than the interpreter it replaced. What actually names a block is
//! **the physical page its bytes came from**, which the entry translation has
//! just resolved, and that is what [`Origin::Paged`] carries:
//!
//! * a different mapping means different bytes, a different physical page and
//!   a different key, so a stale block cannot be served;
//! * the same physical page with its bytes rewritten is caught by the block
//!   cache's own invalidation, which is already by physical page;
//! * the same physical page with changed permissions is caught by the entry
//!   translation, which is redone on every execution and faults before the
//!   block runs;
//! * every access inside the block translates live, through the core's MMU.
//!
//! [`Origin::Paged`] carries the whole physical **address**, not its page:
//! the twelve bits below the frame say which byte of it the entry is, and
//! with a page-granular key a `CS` base moved by one page would give the same
//! `EIP` a different offset in the same frame — different bytes, and a
//! different distance to the end of the page — under one key.
//!
//! It also subsumes `CS.base`, which is why [`World::generation`] is left out
//! of a paged key rather than added to it: under [`Smc::EndBlock`] no segment
//! base appears as a constant in the emitted IR at all — [`MemOp::seg`]
//! carries the register and the host folds the base — so the only thing
//! `CS.base` decides is *which bytes* the entry names, and the physical page
//! it resolved to decides that exactly.
//!
//! ## The contract a caller owes, which is the one that looks like a working JIT
//!
//! **The entry page must be read through the *fetch* path, on every execution
//! of the block and not once at lift time.** `Exec::translate_access` with
//! [`paging::Access::fetch`](super::paging::Access) is what charges the walk
//! on a miss, sets the accessed bit and checks execute permission; a debug
//! walk has deliberately none of those effects. A caller that lifts through
//! the debug walk gets a block that is *correct* and a clock that is short by
//! one walk per entry — which is the failure mode that passes every test that
//! does not compare cycles. [`Origin::Paged`]'s field is the physical page
//! that translation produced, so the two cannot come apart: there is nothing
//! to key a paged block on except the answer the fetch path gave.
//!
//! `differential`'s `PagedHost` is that contract implemented, and
//! `cpu::riscv::engine::admit` is the same contract on the other core.
//!
//! ## What paging still does not buy
//!
//! Bytes written by something that is **not** this core — a DMA engine
//! filling a page cache, another CPU — are outside `jit::dispatch`'s
//! contract, which is that a host accumulates the pages *it* wrote. That is
//! the same known gap `cpu::riscv::engine` states, from the same cause, and
//! it is not made worse or better by paging.
//!
//! # Widening the world further: what long mode costs
//!
//! **Paging was a prerequisite rather than an alternative**, which is worth
//! stating because "long mode without paging" sounds like a smaller job and is
//! not a machine: `EFER.LMA` is set only when `CR0.PG` goes on with
//! `EFER.LME`, and IA-32e paging requires `CR4.PAE` (*Intel SDM* volume 3
//! §9.8.5, *AMD64 Architecture Programmer's Manual* volume 2 §14.6). A
//! processor in long mode is a processor with paging on, always. So the
//! section above is the first half of this one.
//!
//! ## Long mode: `REX` is not the hard part either
//!
//! The register file and the prefix are mechanical — [`isa::decode_stream_as`]
//! already decodes `Bits::B64`, because `exec` runs it. Four things behind
//! them are not:
//!
//! 1. ~~**The carry of a 64-bit `ADD` is bit 64, and there is no `i65`.**~~
//!    **Done.** `Lifter::add` read `CF` off `self.bit(wide, bits)` — the bit
//!    *above* the operand's width — which at a 64-bit operand size does not
//!    exist and would have come out zero; `Lifter::sub` formed `b + borrow`
//!    and compared against it, which wraps to zero on an all-ones subtrahend
//!    and turns a borrow into no borrow. Both are now the unsigned-compare
//!    formulation **at the operand's width** — `r < a || (r == a && carry)`
//!    for an add, `a < b || (a == b && borrow)` for a subtract — which has no
//!    width to be wrong at. That is a change to the arithmetic core rather
//!    than a case beside it, so it is proved where every other flag is: the
//!    two forms are equal at eight, sixteen and thirty-two bits, and
//!    [`differential`](super::differential)'s corpus runs them.
//! 2. **A 64-bit `MUL` needs [`Opcode::MULU2`]/[`Opcode::MULS2`]**, which
//!    "What the IR could not say" below records as unexercised precisely
//!    because a 32-bit subset never reaches them.
//! 3. **The slot numbering doubles.** Sixteen general registers pushes `RIP`
//!    and the six flags up the numbering, which every consumer of
//!    [`SLOT_COUNT`] and [`FLAG_SLOTS`] — [`differential`](super::differential)
//!    included — reads.
//! 4. **`RIP`-relative addressing makes the effective address depend on the
//!    instruction's own length**, and `Lifter`'s effective-address helper masks
//!    every sum to thirty-two bits.
//!
//! ## And it is not sufficient on its own
//!
//! Worth saying because it is easy to assume otherwise: widening this world
//! makes a 64-bit block *translatable*, and nothing more. No CPU core has a
//! JIT execution path — `X86`'s `Runnable::run` calls the interpreter's `step`
//! in a loop, and `jit::Dispatcher` appears only in
//! [`differential`](super::differential) and `cpu::riscv::differential`, driven
//! by their own hosts. `docs/platforms/pc64.md` measures the consequence: a
//! kernel boot with `jit-x86` and without it is byte-for-byte identical.
//! Both halves are needed, and this one is the second.
//!
//! # Self-modifying code, which x86 makes architectural
//!
//! `ROADMAP.md` §9.1's third mechanism, and `jit::dispatch` already records
//! that *"an x86 frontend needs the check **within** a block — x86 makes
//! coherent instruction caches architectural — and will need a finer hook than
//! this one"*. That note is acted on here rather than deferred, and the hook
//! turned out not to be needed: the check is expressible in the IR.
//!
//! * [`Smc::EndBlock`] is RISC-V's answer — a store is the last guest
//!   instruction in its block, so the dispatcher's page-drain at the next
//!   boundary is reached before anything the store changed can execute. It is
//!   correct and it costs a block per store, which on x86 is expensive: a
//!   store is not a rare instruction there the way it is in a register-rich
//!   RISC.
//! * [`Smc::Guard`] is the default **with paging off**. Under
//!   [`Origin::Paged`] it is refused rather than emitted: the comparison below
//!   is in linear space, two linear pages may alias one physical page, and a
//!   guard that can be walked past is worse than no guard. A store is an
//!   ordinary instruction, and
//!   after it the block tests the store's **linear** page against its own:
//!
//!   ```text
//!     st   ...                      ; the guest's store
//!     t = and(linear_address, ~0xfff)
//!     brcond ne t, <this block's page> -> after   ; the common case, skipped
//!     mov  pc = <where this instruction resumes>
//!     insn_start  live = <every register and flag> + EIP
//!     exit_tb                       ; the store hit this block's own code
//!   after:
//!     ...
//!   ```
//!
//!   Leaving through that exit puts a block boundary between the store and the
//!   next instruction, which is exactly where the dispatcher drains the store
//!   and invalidates the translation. The guest then re-enters at the same PC
//!   and the block is lifted again from the bytes the store left. Three IR
//!   instructions per store in the common case, against a whole extra block
//!   dispatch under [`Smc::EndBlock`].
//!
//!   "Where this instruction resumes" is the next instruction for every store
//!   in the subset but one. `CALL` pushes its return address and *then*
//!   transfers, so its store is not its last effect and its exit has to resume
//!   at the call's **target** — which is why the lifter carries a `Resume`
//!   rather than assuming, and is the mutation
//!   `a_call_that_rewrites_its_own_target_resumes_at_the_target` exists for.
//!
//! The comparison is in **linear** space and the block's own page is a
//! lift-time constant, which is why [`World`] carries `CS.base` and the six
//! segment bases and why the world's generation is in [`Block::key`]: a store
//! through `DS` and a fetch through `CS` are only comparable once both bases
//! are known. It is also why [`World::of`] takes the A20 gate's state and
//! refuses a machine with it **closed** — `Exec::masked` folds bit 20 out of
//! the physical address there, so two different linear pages alias one physical
//! page and a linear comparison would miss one of them.
//!
//! # Precise state at a fault
//!
//! `ROADMAP.md` §9: *"when a load faults halfway through a translated block,
//! the guest must observe exactly the architectural state its ISA specifies at
//! that instruction"*. x86 has more state and more faulting instructions than
//! RISC-V, and one rule that makes the answer simpler than it looks:
//! `Exec::step` restores the whole register file from a snapshot taken before
//! decoding, so **a faulting instruction is architecturally as if it had never
//! started**. The IR's lazy publication gives the same answer for free — a
//! fault publishes the boundary's map, which is the state before the
//! instruction — and that equivalence is what
//! [`differential::compare`](super::differential::compare) checks, register by
//! register and flag by flag, against a snapshot of the interpreter taken
//! before the same step.
//!
//! Since `CS` is required to be a flat 4 GiB segment (see [`World::of`]), the
//! only fault a lifted instruction can take is on a **data** operand: a
//! segment-limit or access-rights violation — `#GP` through most registers and
//! `#SS` through the stack, which is why [`MemOp::seg`] carries the register
//! rather than the frontend folding the base into the address — and, under
//! [`Origin::Paged`], a `#PF` from the host's own translation. All three
//! arrive as one thing here, because the vector is the interpreter's business
//! and what the block owes is the state *at* the instruction.
//!
//! # Guest state: the slot numbering
//!
//! | Slot | State | Held as |
//! | --- | --- | --- |
//! | `0..=7` | `EAX`..`EDI` in ModRM order ([`r_slot`]) | the 32-bit value, zero-extended |
//! | `8` | `EIP` ([`EIP`]) | the 32-bit value |
//! | `9..=14` | `CF PF AF ZF SF OF` ([`FLAG_SLOTS`]) | 0 or 1 |
//! | `15` | everything else in `EFLAGS` ([`EFLAGS_REST`]) | `eflags & !`[`ARITH_MASK`] |
//!
//! Segment registers and their hidden descriptors have no slots because
//! nothing in the subset writes one, and their bases are lift-time constants
//! in [`World`] rather than run-time state. `EIP` is bound only at a block's
//! **exit** boundary; at every other boundary it is [`InsnStart::pc`], a
//! constant.
//!
//! **Every temporary that becomes a register value is already masked to its
//! register's width**, which is the invariant that lets a 32-bit write be a
//! rebinding with no masking op of its own. Sub-register writes are
//! [`Opcode::DEPOSIT`] and sub-register reads are [`Opcode::EXTRACT`], which
//! is what that opcode's documentation says x86 needs it for — and `AH` is
//! register number four rather than one, so a lifter that skipped it would be
//! wrong silently.
//!
//! # What the IR could not say
//!
//! Recorded here rather than discovered twice, in the shape `ir`'s own "Known
//! gaps" section uses.
//!
//! * **[`Opcode::ROTLC`]/[`Opcode::ROTRC`] cannot express `RCL`/`RCR`.** They
//!   are an (N+1)-bit rotate at the *type's* width, and x86's is at the
//!   operand width — 8, 16 or 32 bits, only one of which the IR has a type
//!   for. The same gap the atomics have, from the same cause: there is no
//!   `i8` or `i16`. `RCL`/`RCR` by one is therefore lifted by hand as a shift,
//!   an or and a bit test, and by more than one is out of the subset.
//! * **A 128-bit widening multiply is not needed and a 64-bit one is not
//!   used.** Every product in a 32-bit subset fits in [`Type::I64`], so
//!   [`Opcode::MULU2`] and [`Opcode::MULS2`] stay unexercised here; they would
//!   be what a 64-bit `MUL` lowers to.
//! * **Arithmetic is at [`Type::I64`] whatever the operand size**, because
//!   `CF` is the bit *above* the operand's width and an `i32` add loses it.
//!   The 8- and 16-bit forms would want an `i8`/`i16` and the 32-bit form an
//!   `i33`; one type that holds all three is the honest answer, and every
//!   result is masked back to its operand width before it becomes state.
//!
//! # How this is known to be right
//!
//! It is not, on its own. [`differential`](super::differential) is the
//! harness — one guest program through both engines, comparing the eight
//! registers, `EIP`, all six flags, the tick count, the block's static tick
//! column, guest RAM byte for byte, and whether both agreed about faulting —
//! driven from a generated corpus in `tests/x86_lift_differential.rs` and from
//! `fuzz/fuzz_targets/x86_lift.rs`. The tests at the bottom of this file
//! assert the *shape* of what is emitted; the harness asserts the meaning.
//!
//! # Sources
//!
//! Intel's *80386 Programmer's Reference Manual* for the protected-mode
//! address path, the segment-limit checks and the exception model; the *Intel
//! SDM* volume 2 for the operand-size rules, the shift count mask, and the
//! flag definitions of every instruction above; volume 1 §3.4.1.1 for the
//! sub-register write rules. The undefined-flag behaviour is
//! `exec`'s, measured there against `SingleStepTests` — this
//! file reproduces it rather than re-deriving it, which is what makes the
//! differential comparison meaningful. No emulator source of any licence was
//! opened for any part of this file (`ROADMAP.md` §1).

use alloc::vec::Vec;

use crate::core::error::{Error, Result};
use crate::core::value::Width;
use crate::ir::{
    AccessKind, Align, Block, BlockBuilder, Cond, Const, Endian, InsnStart, MemOp, MemSpace,
    Opcode, RegSlot, SegId, Sign, Temp, Type, bitfield_aux, eliminate_dead_code,
};

use super::isa::{self, Arg, Bits, Fields, Op, seg};
use super::paging::Buffers;
use super::prot::Sys;
use super::{Config, Regs, Variant, flags};

// ---------------------------------------------------------------------------
// The slot numbering
// ---------------------------------------------------------------------------

/// The slot holding general register `n`, in ModRM order.
///
/// `0` is `EAX`, `4` is `ESP`, `7` is `EDI`. The value is the architectural
/// 32-bit register, zero-extended into the slot.
#[inline]
#[must_use]
pub const fn r_slot(n: u8) -> RegSlot {
    RegSlot((n & 7) as u16)
}

/// The slot holding `EIP`.
///
/// Bound only at a block's exit boundary; at every other boundary the program
/// counter is [`InsnStart::pc`] and a temporary for it would be a second
/// source of truth.
pub const EIP: RegSlot = RegSlot(8);

/// The carry flag.
pub const CF: RegSlot = RegSlot(9);
/// The parity flag.
pub const PF: RegSlot = RegSlot(10);
/// The auxiliary-carry flag.
pub const AF: RegSlot = RegSlot(11);
/// The zero flag.
pub const ZF: RegSlot = RegSlot(12);
/// The sign flag.
pub const SF: RegSlot = RegSlot(13);
/// The overflow flag.
pub const OF: RegSlot = RegSlot(14);

/// Everything in `EFLAGS` that is not one of the six arithmetic flags.
///
/// Held as `eflags & !`[`ARITH_MASK`], so the six that *are* modelled
/// separately never appear twice. `CLD` and `STD` are the only instructions in
/// the subset that write it, and `LAHF` the only one that reads it.
pub const EFLAGS_REST: RegSlot = RegSlot(15);

/// One past the highest slot this frontend numbers.
pub const SLOT_COUNT: u16 = 16;

/// The six arithmetic flag slots, in this frontend's own order.
pub const FLAG_SLOTS: [RegSlot; 6] = [CF, PF, AF, ZF, SF, OF];

/// The `EFLAGS` bit each of [`FLAG_SLOTS`] holds.
pub const FLAG_BITS: [u32; 6] = [
    flags::CF,
    flags::PF,
    flags::AF,
    flags::ZF,
    flags::SF,
    flags::OF,
];

/// Every `EFLAGS` bit that lives in its own slot rather than in
/// [`EFLAGS_REST`].
pub const ARITH_MASK: u32 = flags::CF | flags::PF | flags::AF | flags::ZF | flags::SF | flags::OF;

const F_CF: usize = 0;
const F_PF: usize = 1;
const F_AF: usize = 2;
const F_ZF: usize = 3;
const F_SF: usize = 4;
const F_OF: usize = 5;

/// Every flag, as a mask over [`FLAG_SLOTS`].
const ALL_FLAGS: u8 = 0x3f;
/// The five flags an `INC` or `DEC` writes: everything but `CF`.
const NOT_CARRY: u8 = ALL_FLAGS & !(1 << F_CF);
/// The five status flags `SAHF` moves out of `AH`: `CF PF AF ZF SF`.
const LOW_FIVE: u8 = (1 << F_CF) | (1 << F_PF) | (1 << F_AF) | (1 << F_ZF) | (1 << F_SF);

/// The page a block is bounded by, which is the translation runtime's.
///
/// Named here rather than imported because `jit` is a separate feature and
/// `cpu/` may not depend on it; a `const` assertion below checks the two agree
/// wherever both are compiled, because a frontend whose blocks were bounded by
/// a *larger* page than the block cache matches guest stores against would go
/// silently stale.
pub const PAGE_SIZE: u64 = 4096;

/// The mask selecting the offset within a [`PAGE_SIZE`] page.
pub const PAGE_MASK: u64 = PAGE_SIZE - 1;

/// How many guest instructions [`lift`] takes by default.
///
/// A block is bounded by its page anyway; this bounds a block in a tight page
/// and, under [`Shape::Trace`], is the only thing that bounds an unrolled
/// loop. It is therefore also the bound on how long a safe point can be
/// delayed, because a dispatcher checks its exit flag at block boundaries
/// (`ROADMAP.md` §4.7).
pub const MAX_INSNS: usize = 64;

// ---------------------------------------------------------------------------
// The world a lift happens in
// ---------------------------------------------------------------------------

/// Which address space the entry `EIP` names, and what names the block in it.
///
/// The x86 analogue of `cpu::riscv::lift::Origin`, and the same two-armed
/// shape for the same reason: a block lifted from a *virtual* address is valid
/// only for the mapping it was lifted under, and the guest may change that
/// mapping without changing any address.
///
/// What is deliberately **not** here is a translation generation. See the
/// module docs, "Paging": `cpu::riscv::engine` measured a `CR3`-style counter
/// being unusable on a real guest, and the physical page the entry translation
/// resolved to is both narrower and exact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Origin {
    /// `CR0.PG` is clear: a linear address *is* a physical one, and the
    /// entry `EIP` plus [`World::generation`] name the block on their own.
    Flat,
    /// `CR0.PG` is set, and the entry translated here.
    Paged {
        /// The **physical** address the entry `EIP`'s linear address resolved
        /// to, through the *fetch* path. The whole address, not its page.
        ///
        /// The caller owes that path — see the module docs, "The contract a
        /// caller owes". Its page is [`Lifted::page`], which is what a guest
        /// store is matched against; the twelve bits below that are in
        /// [`key`] too, and deliberately: the offset within the page decides
        /// which bytes the entry names and how far the block may run before
        /// it leaves the page, and two `CS` bases a page apart would
        /// otherwise give one key to two different blocks.
        phys: u64,
    },
}

impl Origin {
    /// Whether translation is on.
    #[inline]
    #[must_use]
    pub const fn paged(self) -> bool {
        matches!(self, Origin::Paged { .. })
    }

    /// The bits this origin contributes to [`key`], above the policies.
    ///
    /// Bit 7 separates the two worlds, so a flat lift and a paged lift of the
    /// same address never collide; above bit 8 sits the physical address or
    /// the world generation.
    ///
    /// Exact until either passes 2^56 — the bound
    /// `cpu::riscv::lift::Origin::key_bits` states for the same encoding. For
    /// the physical address that bound is unreachable rather than merely
    /// distant: a page-table entry carries a 52-bit frame
    /// ([`pte::FRAME64`](super::paging::pte)), so no address this core can
    /// produce reaches it.
    const fn key_bits(self, generation: u64) -> u64 {
        match self {
            Origin::Flat => generation.wrapping_shl(8),
            Origin::Paged { phys } => (1 << 7) | phys.wrapping_shl(8),
        }
    }
}

/// Everything outside the instruction bytes that a lift depends on.
///
/// The x86 analogue of `cpu::riscv::lift::Origin`, and larger for the reason
/// that file's "Paging" section gives: a block is a function of the *bytes* at
/// its entry PC, and on x86 which bytes those are depends on `CS.base`, while
/// what its addresses *mean* depends on five more segment bases. A cache keyed
/// without them would hand back a block lifted in a world that no longer
/// exists.
///
/// [`World::generation`] is how that is made exact rather than hashed: a
/// frontend keeps one counter and bumps it whenever any field here changes,
/// and [`key`] folds the counter into [`Block::key`]. That is the same
/// mechanism `Origin::Paged { generation }` uses, and it has the same stated
/// limit — the counter occupies the key above bit 8, so it is exact until it
/// passes 2^56.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct World {
    /// Which part this is: the opcode map, the clock table and the flag mask.
    pub variant: Variant,
    /// `CS.base`, added to `EIP` to reach the linear address of a fetch.
    pub cs_base: u64,
    /// The cached base of each of the six segment registers.
    pub seg_base: [u64; seg::COUNT],
    /// Whether `CMOVcc` decodes on this part.
    pub cmov: bool,
    /// A counter naming this world, bumped by whoever builds it whenever any
    /// field above changes. See the type's own documentation.
    ///
    /// Folded into [`key`] only under [`Origin::Flat`]: a paged block is named
    /// by the physical page its bytes came from, which subsumes `CS.base`.
    pub generation: u64,
    /// Which address space the entry `EIP` names.
    pub origin: Origin,
}

impl World {
    /// The world a core is in, or `None` if it is not one this frontend lifts.
    ///
    /// Every refusal below is a *correctness* condition rather than a missing
    /// feature, and each is stated where it is checked. Deriving the answer
    /// from the core's own registers is the point: the only way to claim a
    /// world wrongly is to write one out by hand.
    #[must_use]
    pub fn of(
        regs: &Regs,
        sys: &Sys,
        cfg: &Config,
        a20_open: bool,
        generation: u64,
        origin: Origin,
    ) -> Option<World> {
        // The A20 gate folds bit 20 out of every *physical* address
        // (`Exec::masked`), so with it shut two linear pages a megabyte apart
        // alias one physical page — and the self-modifying-code guard compares
        // linear pages. It comes in as an argument rather than being read here
        // because it lives on the core's interrupt lines rather than in its
        // registers: it is a pin, and a device drives it.
        if !a20_open {
            return None;
        }
        let variant = cfg.variant;
        // A part with 16-bit registers has a different address path entirely.
        if !variant.is_32bit() {
            return None;
        }
        // Protected mode, not real mode and not virtual-8086: both of those
        // compute an address as `segment << 4 + offset` with a 16-bit offset
        // wrap, which is `Exec`'s legacy path.
        if !sys.protected() || regs.flag(flags::VM) {
            return None;
        }
        // Long mode would need `REX` and a third operand-size lattice.
        if sys.sixty_four() {
            return None;
        }
        // Paging is in the subset, and the origin has to say the same thing
        // the control registers do: a caller that claimed `Origin::Flat` on a
        // paged core would get a block keyed as though a linear address were a
        // physical one, which is the stale-translation bug this type exists to
        // make unstatable.
        if sys.paging() != origin.paged() {
            return None;
        }
        // A part whose instruction and data translations share one array —
        // a 386 and a 486 — puts a page-table walk in front of a *fetch* the
        // block does not make, at a point no static analysis can predict: a
        // data operand far enough from the code page evicts the code page's
        // own translation and the next instruction fetch pays for the walk.
        // The arrangement is asked about rather than the part number, because
        // that is the fact the refusal is actually about.
        if sys.paging() && !matches!(cfg.variant.buffers(), Buffers::Split) {
            return None;
        }
        let cs = sys.seg(seg::CS);
        // A 32-bit code segment: `Bits::B32` is what the decoder is driven
        // with, and a 16-bit one would decode entirely differently.
        if !cs.big() {
            return None;
        }
        // A flat code segment. `Exec::jump_near` raises `#GP` for a target
        // outside `CS`, and a *computed* transfer — `RET`, `JMP r/m` — would
        // then need a conditional exception this IR cannot express. With a
        // 4 GiB segment the check is discharged for every target at once.
        if cs.limit != 0xffff_ffff {
            return None;
        }
        // A 32-bit stack. `Exec::set_sp` preserves the bits the stack's width
        // does not reach, so a 16-bit stack makes every push a deposit and
        // every pop a partial write.
        if !sys.seg(seg::SS).big() {
            return None;
        }
        let mut seg_base = [0u64; seg::COUNT];
        for (n, base) in seg_base.iter_mut().enumerate() {
            *base = sys.seg(n as u8).base;
        }
        Some(World {
            variant,
            cs_base: cs.base,
            seg_base,
            // A property of the *instance* rather than of the part number
            // (`ROADMAP.md` §6.1.1): `Exec` raises `#UD` for a `CMOVcc` when
            // this is clear, and a 386 and a 486 both have it clear by
            // default, so a lifter that assumed otherwise would lift an
            // instruction the interpreter refuses.
            cmov: cfg.features.cmov,
            generation,
            origin,
        })
    }

    /// The same world on a part without `CMOVcc`.
    ///
    /// `Exec` raises `#UD` for a `CMOV` when `Features::cmov` is clear, so
    /// whether the instruction is in the subset is a property of the core
    /// rather than of the encoding.
    #[must_use]
    pub const fn without_cmov(mut self) -> World {
        self.cmov = false;
        self
    }

    /// The linear address `EIP` names.
    ///
    /// Deliberately **not** masked to 32 bits, because `Exec::fetch_at` does
    /// not mask either: it computes `cs.base.wrapping_add(offset)` and hands
    /// the result to the bus. Matching the interpreter is the whole contract,
    /// so a difference here would be a divergence rather than a tidying.
    #[inline]
    #[must_use]
    pub const fn linear(&self, eip: u64) -> u64 {
        self.cs_base.wrapping_add(eip)
    }
}

// ---------------------------------------------------------------------------
// Shapes, policies, and the cache key
// ---------------------------------------------------------------------------

/// How much a block is allowed to swallow.
///
/// `ROADMAP.md` §9's fourth speed mechanism is superblocks, and this is the
/// switch. The shapes are strictly nested and all three must agree with the
/// interpreter on every column, so a disagreement between two of them is a
/// frontend bug wherever it shows up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Shape {
    /// A basic block: it ends at the first memory access and at the first
    /// transfer of control.
    BasicBlock,
    /// An extended basic block: a **load** is an ordinary instruction, and
    /// only a store or a transfer of control ends the block.
    Extended,
    /// A trace: direct branches are merged in, with a precise side exit for
    /// each path not taken. The default.
    #[default]
    Trace,
}

impl Shape {
    /// Whether a **load** ends the block.
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

    const fn key_bits(self) -> u64 {
        match self {
            Shape::BasicBlock => 0,
            Shape::Extended => 1,
            Shape::Trace => 2,
        }
    }
}

/// What a store does to the block it is in.
///
/// x86 makes coherent instruction caches architectural, so a store into the
/// page a running block was lifted from must be honoured before the next
/// instruction executes. See the module docs for the two answers and why
/// [`Smc::Guard`] is the default.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Smc {
    /// A store is the last guest instruction in its block.
    ///
    /// The only policy under [`Origin::Paged`], because the alternative
    /// compares linear pages and two of them may alias one physical page.
    EndBlock,
    /// A store is an ordinary instruction, followed by a run-time test of its
    /// linear page against the block's own.
    ///
    /// The default, and refused by [`lift`] under [`Origin::Paged`].
    #[default]
    Guard,
}

impl Smc {
    const fn key_bits(self) -> u64 {
        match self {
            Smc::EndBlock => 0,
            Smc::Guard => 1 << 2,
        }
    }
}

/// Whether a boundary names every flag, or only the ones something can observe.
///
/// See the module docs, "Flag elision, and exactly when it is sound".
/// [`Flags::Eager`] exists so the difference is a measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Flags {
    /// Every bound flag is named at every boundary.
    Eager,
    /// A flag the next instruction unconditionally overwrites, at a boundary
    /// no fault can be taken at, is left out — and dead-code elimination then
    /// removes the arithmetic that produced it. The default.
    #[default]
    Elide,
}

impl Flags {
    const fn key_bits(self) -> u64 {
        match self {
            Flags::Eager => 0,
            Flags::Elide => 1 << 3,
        }
    }
}

/// The block cache key: every setting and every world bit this lift depends on.
///
/// [`Block::key`] is the rest of the cache key beside the entry `EIP`. The
/// three policies are here even though all of them are *correct*, for the
/// reason the RISC-V lifter gives about its own [`Shape`]: a cache that mixed
/// them would make a measurement of one a measurement of whichever happened to
/// be resident.
///
/// Public because a block cache has to ask this question *before* it lifts
/// anything — `jit::Dispatcher` looks a block up under `(pc, key(..))` and
/// calls [`lift`] only when that misses — and a dispatcher that derived the
/// key itself would be a second copy of the answer.
#[must_use]
pub fn key(world: &World, shape: Shape, smc: Smc, flags: Flags) -> u64 {
    // Two bits rather than one. Only three parts reach here — a 16-bit one is
    // refused by `World::of` — and until paging landed a 386 and an x86-64
    // both encoded as zero. That was not a bug, because the two agree on
    // every input a block depends on (`Variant::map`, `bus_clocks`, and an
    // `Op::clocks` table that does not vary by part), but it was one feature
    // away from being one: they differ in their translation-buffer
    // arrangement, which is exactly what decides whether a paged world is
    // liftable at all. A separate number per part costs a bit.
    let variant = match world.variant {
        Variant::I80486 => 1u64,
        Variant::X86_64 => 2u64,
        _ => 0u64,
    };
    shape.key_bits()
        | smc.key_bits()
        | flags.key_bits()
        | (u64::from(world.cmov) << 4)
        | (variant << 5)
        | world.origin.key_bits(world.generation)
}

// ---------------------------------------------------------------------------
// Inputs and outputs
// ---------------------------------------------------------------------------

/// Where the lifter reads guest instruction bytes.
///
/// Bytes at **linear** addresses, which is what `Exec::fetch_at` reaches after
/// it has added `CS.base`. A caller must read them through the same path the
/// interpreter's *fetch* uses, and never through a debug walk that would skip
/// the effects a fetch has.
pub trait InsnSource {
    /// The byte at linear address `addr`, or `None` if it is unreadable.
    fn byte(&mut self, addr: u64) -> Option<u8>;
}

impl<F: FnMut(u64) -> Option<u8>> InsnSource for F {
    #[inline]
    fn byte(&mut self, addr: u64) -> Option<u8> {
        self(addr)
    }
}

/// Why a block stopped where it did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stop {
    /// An encoding outside the subset. It was not lifted; the block's exit
    /// `EIP` is its address, so the interpreter executes it next.
    Unsupported,
    /// A memory access that ended the block: a store under [`Smc::EndBlock`],
    /// or any access under [`Shape::BasicBlock`].
    Access,
    /// A transfer of control this block cannot follow.
    Transfer,
    /// The next instruction would leave the linear page the block started on,
    /// or would straddle its end.
    Page,
    /// The caller's instruction limit.
    Limit,
    /// The instruction bytes could not be read.
    Unreadable,
}

/// A lifted block, and what is true about it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lifted {
    /// The block. Always ends in a terminator and always passes
    /// [`verify`](crate::ir::verify).
    pub block: Block,
    /// Why lifting stopped.
    pub stop: Stop,
    /// How many guest instructions the block **covers**. Zero is legal and
    /// means the first instruction was outside the subset or straddled the
    /// page.
    ///
    /// Under [`Shape::Trace`] this is not what a run through the block
    /// retires: a trace inlines one side of every branch it merges, and
    /// leaving through a side exit retires only the instructions on the path
    /// taken. Anything that needs what retired counts boundaries instead —
    /// [`Interp::boundaries`](crate::ir::Interp::boundaries).
    pub insns: usize,
    /// The **physical** page the bytes were read from — what a guest store is
    /// matched against, and what `jit::Translation::page` wants.
    ///
    /// Under [`Origin::Flat`] that is the linear page, because a linear
    /// address is a physical one with `CR0.PG` clear. Under [`Origin::Paged`]
    /// it is the page of [`Origin::Paged`]'s address — the page the entry
    /// resolved to — and it has to be physical at *both* ends, because
    /// `jit::cache` invalidates by physical page and a host notes the physical
    /// address its store reached.
    pub page: u64,
    /// The linear page the block is bounded by.
    ///
    /// Equal to [`Lifted::page`] under [`Origin::Flat`]. Separate under
    /// paging, where the two are different numbers and confusing them is the
    /// stale-translation bug.
    pub linear_page: u64,
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Lift the guest instructions at `entry_eip` into a translation block.
///
/// Reads at most `max_insns` instructions, never leaves the linear page
/// `entry_eip` starts on, and always produces a well-formed block — including
/// when nothing could be lifted, in which case the block is the exit boundary
/// and a terminator and [`Lifted::insns`] is zero.
///
/// The block is run through [`eliminate_dead_code`] before it is returned;
/// see the module docs, "Flags".
///
/// # Errors
///
/// [`Error::Unimplemented`] if `max_insns` is zero, which would produce a
/// block a dispatcher must not cache, and if [`Smc::Guard`] is asked for under
/// [`Origin::Paged`] — the in-block guard compares linear pages and two of
/// them may alias one physical page, so it is refused rather than emitted with
/// a hole in it. See the module docs, "Paging".
pub fn lift<S: InsnSource>(
    world: &World,
    entry_eip: u64,
    src: &mut S,
    max_insns: usize,
    shape: Shape,
    smc: Smc,
    flag_policy: Flags,
) -> Result<Lifted> {
    if max_insns == 0 {
        return Err(Error::Unimplemented("an x86 lift of zero instructions"));
    }
    // The guard is a comparison of linear pages and under paging two of them
    // may name one physical page, so a store through the other mapping would
    // slip past it. `Smc::EndBlock` is the policy that holds there.
    if world.origin.paged() && matches!(smc, Smc::Guard) {
        return Err(Error::Unimplemented(
            "an x86 lift with the in-block store guard under paging",
        ));
    }
    let mut lf = Lifter::new(world, entry_eip, shape, smc, flag_policy);
    let page = lf.page;
    let map = world.variant.map();
    let mut eip = entry_eip & 0xffff_ffff;
    let mut insns = 0usize;

    let stop = loop {
        if insns >= max_insns {
            break Stop::Limit;
        }
        let lin = world.linear(eip);
        if lin & !PAGE_MASK != page {
            break Stop::Page;
        }

        // The byte source refuses to leave the entry page, which is what turns
        // a page-straddling instruction into a clean end-of-block rather than
        // a fetch this block would have to be able to fault on.
        let mut at = lin;
        let mut off_page = false;
        let fields = {
            let src = &mut *src;
            isa::decode_stream_as(map, Bits::B32, &mut || {
                if at & !PAGE_MASK != page {
                    off_page = true;
                    return None;
                }
                let byte = src.byte(at)?;
                at = at.wrapping_add(1);
                Some(byte)
            })
        };
        if fields.truncated {
            break if off_page {
                Stop::Page
            } else {
                Stop::Unreadable
            };
        }
        // A 386 raises `#GP` rather than executing an arbitrarily long prefix
        // run, so a sixteen-byte encoding is not an instruction and the block
        // ends before it — the interpreter delivers the exception.
        if fields.len > 15 {
            break Stop::Unsupported;
        }

        let next_eip = eip.wrapping_add(u64::from(fields.len)) & 0xffff_ffff;
        match lf.insn(&fields, eip, next_eip) {
            Flow::Rejected => break Stop::Unsupported,
            Flow::Continue(next) => {
                insns += 1;
                eip = next;
            }
            Flow::Access { next, store } => {
                insns += 1;
                eip = next;
                if shape.access_ends_block() || (store && matches!(smc, Smc::EndBlock)) {
                    break Stop::Access;
                }
            }
            Flow::Transfer => {
                insns += 1;
                eip = next_eip;
                break Stop::Transfer;
            }
        }
    };

    let block = lf.finish(eip);
    // Decision 1's debt, settled where it is incurred: the popcount `PF` costs
    // on nearly every ALU instruction is removable exactly when no boundary
    // names it, and this is what removes it.
    let block = eliminate_dead_code(&block);
    Ok(Lifted {
        block,
        stop,
        insns,
        // Physical at both ends: what a guest store is matched against.
        page: match world.origin {
            Origin::Flat => page,
            Origin::Paged { phys } => phys & !PAGE_MASK,
        },
        linear_page: page,
    })
}

// The block bound is one page and the translation runtime matches blocks to
// guest stores by page, so the two numbers have to be the same one.
#[cfg(feature = "jit")]
const _: () = assert!(PAGE_SIZE == crate::jit::PAGE_SIZE);

// ---------------------------------------------------------------------------
// The plan: what an encoding means, decided before anything is emitted
// ---------------------------------------------------------------------------

/// Where a shift or rotate takes its count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Count {
    /// A lift-time constant, already masked to five bits and never zero.
    Fixed(u32),
    /// `CL`, masked to five bits at run time — so the instruction writes
    /// nothing at all when it is zero, which is why such a shift never elides
    /// a flag and never reaches a memory operand.
    Cl,
}

/// What lifting one instruction will emit.
///
/// Every encoding is classified — and every static precondition checked —
/// *before* a single op is emitted, so the emitter is total and a rejected
/// instruction leaves no debris in the block.
#[derive(Debug, Clone, Copy)]
enum Plan {
    /// `ADD ADC SUB SBB CMP AND OR XOR TEST`, by their `Op`.
    Alu(Op),
    /// `INC` (`true`) or `DEC`.
    IncDec(bool),
    /// `NOT`, which writes no flag at all.
    Not,
    /// `NEG`, which is `0 - operand`.
    Neg,
    /// `MOV` in its general-register forms.
    Mov,
    /// `MOVZX` (`signed` false) and `MOVSX`, from `src_size` bytes.
    MovX { signed: bool, src_size: u8 },
    /// `LEA`.
    Lea,
    /// A shift or rotate.
    Shift { op: Op, count: Count },
    /// One-operand `MUL` (`signed` false) and `IMUL`.
    Mul { signed: bool },
    /// The two- and three-operand `IMUL`.
    ImulShort,
    /// `BSF` (`false`) and `BSR`.
    BitScan { reverse: bool },
    /// `SETcc`.
    SetCc(u8),
    /// `CMOVcc`.
    CmovCc(u8),
    /// A conditional jump to a statically known target.
    Jcc { cc: u8, target: u64 },
    /// `JMP rel`.
    JmpRel { target: u64 },
    /// `JMP r/m`.
    JmpInd,
    /// `CALL rel`.
    CallRel { target: u64 },
    /// `CALL r/m`.
    CallInd,
    /// `RET`, with the extra bytes it pops off the stack afterwards.
    Ret { extra: u64 },
    /// `PUSH`.
    Push,
    /// `POP`.
    Pop,
    /// `LEAVE`.
    Leave,
    /// `CLC`/`STC` (`Some`) and `CMC` (`None`).
    Carry(Option<bool>),
    /// `CLD`/`STD`.
    Direction(bool),
    /// `LAHF`.
    Lahf,
    /// `SAHF`.
    Sahf,
    /// `CBW`/`CWDE`.
    Cbw,
    /// `CWD`/`CDQ`.
    Cwd,
    /// `BSWAP`.
    Bswap,
    /// `NOP`, and nothing else.
    Nop,
}

impl Plan {
    /// Which flags this plan writes **unconditionally**, as a mask over
    /// [`FLAG_SLOTS`].
    ///
    /// Only unconditional writes count, and that is the whole subtlety of the
    /// elision rule: a shift by `CL` writes nothing when the count is zero, so
    /// it reports nothing however many flags it usually sets.
    const fn writes(self) -> u8 {
        match self {
            Plan::Alu(_) | Plan::Neg | Plan::Mul { .. } | Plan::ImulShort => ALL_FLAGS,
            // `INC` and `DEC` deliberately preserve the carry, which is why
            // they can be used to walk a pointer inside an add-with-carry
            // chain.
            Plan::IncDec(_) => NOT_CARRY,
            Plan::Shift {
                count: Count::Fixed(_),
                ..
            } => ALL_FLAGS,
            // A `CL` count of zero writes nothing at all.
            Plan::Shift {
                count: Count::Cl, ..
            } => 0,
            // The other five are documented undefined and `Exec` leaves them
            // alone, so the frontend must too.
            Plan::BitScan { .. } => 1 << F_ZF,
            Plan::Carry(_) => 1 << F_CF,
            Plan::Sahf => LOW_FIVE,
            _ => 0,
        }
    }
}

// ---------------------------------------------------------------------------
// The lifter
// ---------------------------------------------------------------------------

/// What lifting one instruction did, and where lifting goes next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flow {
    /// Nothing was emitted; the instruction is outside the subset.
    Rejected,
    /// Lifted; carry on at this guest `EIP`.
    ///
    /// The `EIP` is the program-order successor for everything except a merged
    /// direct branch, where it is the branch's target — which is the entire
    /// mechanism by which a trace spans more than one basic block.
    Continue(u64),
    /// Lifted a memory access; carry on unless the [`Shape`] or the [`Smc`]
    /// policy ends the block here.
    Access {
        /// Where lifting carries on.
        next: u64,
        /// Whether anything was written, which is what the self-modifying-code
        /// policy turns on.
        store: bool,
    },
    /// Lifted, and control went somewhere this block cannot follow.
    Transfer,
}

/// Where a self-modifying-code side exit resumes.
///
/// Almost always the next instruction, because a store is almost always the
/// last thing an instruction does. `CALL` is the exception that makes this a
/// field rather than a constant: it pushes the return address and *then*
/// transfers, so a store that hit this block's own page must resume at the
/// call's target and not after the call.
#[derive(Debug, Clone, Copy)]
struct Resume {
    /// The `EIP` the boundary records. Informational at an exit boundary — the
    /// slot is what a dispatcher reads — but it is what a dump shows.
    at: u64,
    /// The temporary holding the resume `EIP`, where it is computed. `None`
    /// means [`Resume::at`] is the whole answer and a constant is materialized
    /// on the exit path, where it costs nothing when the branch is not taken.
    pc: Option<Temp>,
}

/// One translation in progress.
struct Lifter<'a> {
    world: &'a World,
    shape: Shape,
    smc: Smc,
    policy: Flags,
    /// The linear page the entry `EIP` is on. No instruction outside it is
    /// ever lifted, which is what keeps every fetch charge static.
    page: u64,
    b: BlockBuilder,
    /// Which temporary holds each general register, where one does.
    ///
    /// **This is the trace's register allocation.** It survives a merged
    /// branch untouched, so a value computed before a `JMP` is still in a
    /// temporary after it rather than having gone out to a slot and come back.
    r: [Option<Temp>; 8],
    /// Which temporary holds each of the six arithmetic flags.
    fl: [Option<Temp>; 6],
    /// Which temporary holds the rest of `EFLAGS`.
    rest: Option<Temp>,
    /// The block's shared zero, and its two one-bit constants.
    zero: Option<Temp>,
    bit0: Option<Temp>,
    bit1: Option<Temp>,
    /// The effective address of the instruction being lifted, computed once.
    ea: Option<(u8, Temp)>,
    /// Where a self-modifying-code exit inside this instruction resumes.
    resume: Resume,
    /// Ticks charged so far, counted from block entry.
    ticks: u64,
    /// The temporary holding the exit `EIP`, once a transfer has set one.
    pc_out: Option<Temp>,
    /// The exit `EIP` where it is a constant, for the exit boundary's `pc`.
    static_exit: Option<u64>,
}

impl<'a> Lifter<'a> {
    fn new(world: &'a World, entry_eip: u64, shape: Shape, smc: Smc, policy: Flags) -> Lifter<'a> {
        Lifter {
            world,
            shape,
            smc,
            policy,
            page: world.linear(entry_eip) & !PAGE_MASK,
            b: BlockBuilder::new(entry_eip & 0xffff_ffff, key(world, shape, smc, policy)),
            r: [None; 8],
            fl: [None; 6],
            rest: None,
            zero: None,
            bit0: None,
            bit1: None,
            ea: None,
            resume: Resume { at: 0, pc: None },
            ticks: 0,
            pc_out: None,
            static_exit: None,
        }
    }

    // -- constants and small shapes -------------------------------------

    fn konst(&mut self, value: u64) -> Temp {
        self.b.imm(Type::I64, Const::Int(u128::from(value)))
    }

    fn zero(&mut self) -> Temp {
        match self.zero {
            Some(t) => t,
            None => {
                let t = self.konst(0);
                self.zero = Some(t);
                t
            }
        }
    }

    fn kbit(&mut self, value: bool) -> Temp {
        let slot = if value { self.bit1 } else { self.bit0 };
        match slot {
            Some(t) => t,
            None => {
                let t = self.b.imm(Type::I1, Const::Int(u128::from(value)));
                if value {
                    self.bit1 = Some(t);
                } else {
                    self.bit0 = Some(t);
                }
                t
            }
        }
    }

    /// A `len`-bit field at `pos`, as a [`Type::I64`] value.
    fn extract(&mut self, v: Temp, pos: u32, len: u32) -> Temp {
        let dst = self.b.temp(Type::I64);
        self.b.emit_raw(
            Opcode::EXTRACT,
            Type::I64,
            Some(dst),
            None,
            &[v],
            None,
            None,
            bitfield_aux(pos, len),
        );
        dst
    }

    /// `into` with its `len`-bit field at `pos` replaced by `what`.
    fn deposit(&mut self, into: Temp, what: Temp, pos: u32, len: u32) -> Temp {
        let dst = self.b.temp(Type::I64);
        self.b.emit_raw(
            Opcode::DEPOSIT,
            Type::I64,
            Some(dst),
            None,
            &[into, what],
            None,
            None,
            bitfield_aux(pos, len),
        );
        dst
    }

    /// One bit of a value, as a [`Type::I1`].
    fn bit(&mut self, v: Temp, pos: u32) -> Temp {
        let field = self.extract(v, pos, 1);
        self.b.unary(Opcode::TRUNC, Type::I1, field)
    }

    /// A one-bit value widened into [`Type::I64`].
    fn widen(&mut self, bit: Temp) -> Temp {
        self.b.unary(Opcode::EXT_Z, Type::I64, bit)
    }

    fn and_const(&mut self, v: Temp, m: u64) -> Temp {
        let k = self.konst(m);
        self.b.binary(Opcode::AND, Type::I64, v, k)
    }

    fn shl_const(&mut self, v: Temp, n: u32) -> Temp {
        let k = self.konst(u64::from(n));
        self.b.binary(Opcode::SHL, Type::I64, v, k)
    }

    fn shr_const(&mut self, v: Temp, n: u32) -> Temp {
        let k = self.konst(u64::from(n));
        self.b.binary(Opcode::SHR, Type::I64, v, k)
    }

    /// Sign-extend the low `bits` of a value across all sixty-four.
    ///
    /// A shift pair rather than [`Opcode::EXT_S`], which takes its source width
    /// from the temporary's *type* — and every temporary here is [`Type::I64`]
    /// whatever the operand size, for the reason the module docs give.
    fn sext(&mut self, v: Temp, bits: u32) -> Temp {
        if bits >= 64 {
            return v;
        }
        let up = self.shl_const(v, 64 - bits);
        let k = self.konst(u64::from(64 - bits));
        self.b.binary(Opcode::SAR, Type::I64, up, k)
    }

    /// Whether a value is zero, as a one-bit temporary.
    fn is_zero(&mut self, v: Temp) -> Temp {
        let z = self.zero();
        self.b.setcond(Cond::Eq, Type::I64, v, z)
    }

    // -- guest state -----------------------------------------------------

    fn read_r(&mut self, n: u8) -> Temp {
        let n = (n & 7) as usize;
        match self.r[n] {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I64, r_slot(n as u8));
                self.r[n] = Some(t);
                t
            }
        }
    }

    fn write_r(&mut self, n: u8, t: Temp) {
        self.r[(n & 7) as usize] = Some(t);
    }

    fn read_flag(&mut self, i: usize) -> Temp {
        match self.fl[i] {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I1, FLAG_SLOTS[i]);
                self.fl[i] = Some(t);
                t
            }
        }
    }

    fn write_flag(&mut self, i: usize, t: Temp) {
        self.fl[i] = Some(t);
    }

    fn read_rest(&mut self) -> Temp {
        match self.rest {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I64, EFLAGS_REST);
                self.rest = Some(t);
                t
            }
        }
    }

    /// Read a general register at 1, 2 or 4 bytes.
    ///
    /// The byte encoding is the one with no `REX` prefix, which this subset
    /// never has: numbers 0-3 are `AL`-`BL` and 4-7 are `AH`-`BH`, so `AH` is
    /// register four and not one — which is exactly why
    /// [`Opcode::EXTRACT`]'s documentation names x86.
    fn read_reg(&mut self, index: u8, size: u8) -> Temp {
        match size {
            1 => {
                let pos = if index & 4 == 0 { 0 } else { 8 };
                let whole = self.read_r(index & 3);
                self.extract(whole, pos, 8)
            }
            2 => {
                let whole = self.read_r(index);
                self.extract(whole, 0, 16)
            }
            _ => self.read_r(index),
        }
    }

    /// Write a general register at 1, 2 or 4 bytes.
    ///
    /// A narrow write **preserves** the bits above it, which is the 386's rule
    /// and not a convenience: `mov ax, 0` leaves the top of `EAX` alone, and
    /// code that switches operand sizes depends on it.
    fn write_reg(&mut self, index: u8, size: u8, value: Temp) {
        match size {
            1 => {
                let pos = if index & 4 == 0 { 0 } else { 8 };
                let old = self.read_r(index & 3);
                let merged = self.deposit(old, value, pos, 8);
                self.write_r(index & 3, merged);
            }
            2 => {
                let old = self.read_r(index);
                let merged = self.deposit(old, value, 0, 16);
                self.write_r(index, merged);
            }
            _ => self.write_r(index, value),
        }
    }

    /// The slots a temporary currently shadows, in slot order.
    ///
    /// Slot order rather than binding order: `ROADMAP.md` §0's determinism rule
    /// reaches the IR too, and this vector is hashed by anything that hashes a
    /// block.
    ///
    /// `skip_flags` is the elision mask — see the module docs. It is always
    /// empty at an exit boundary, because leaving the block publishes.
    fn live_state(&self, skip_flags: u8) -> Vec<(RegSlot, Temp)> {
        let mut live = Vec::new();
        for (n, temp) in self.r.iter().enumerate() {
            if let Some(t) = temp {
                live.push((r_slot(n as u8), *t));
            }
        }
        for (i, temp) in self.fl.iter().enumerate() {
            if let Some(t) = temp
                && skip_flags & (1 << i) == 0
            {
                live.push((FLAG_SLOTS[i], *t));
            }
        }
        if let Some(t) = self.rest {
            live.push((EFLAGS_REST, t));
        }
        live
    }

    // -- memory ----------------------------------------------------------

    const fn width_of(size: u8) -> Width {
        match size {
            1 => Width::U8,
            2 => Width::U16,
            4 => Width::U32,
            _ => Width::U64,
        }
    }

    fn mem_op(size: u8, sr: u8, kind: AccessKind) -> MemOp {
        MemOp {
            size: Self::width_of(size),
            sign: Sign::Unsigned,
            space: MemSpace::MEM,
            // Carried rather than folded into the address: the descriptor is
            // hidden state, and the fault differs by register — a stack
            // violation is `#SS` where every other segment raises `#GP`.
            seg: Some(SegId(sr)),
            endian: Endian::Little,
            // Alignment is the *host's* business, not a constraint the block
            // states: an unaligned access is one bus transaction unless it
            // crosses a page, and only the host knows whether it did — under
            // paging `Exec::linear_read` splits such an access into bytes and
            // translates each one.
            align: Align::None,
            kind,
            // The access spends ticks and can fault, both guest-visible, so
            // dead-code elimination may not remove one whose value is unused.
            volatile: true,
        }
    }

    fn mem_load(&mut self, sr: u8, addr: Temp, size: u8) -> Temp {
        let mem = Self::mem_op(size, sr, AccessKind::Load);
        self.b.load(Type::I64, addr, mem)
    }

    fn mem_store(&mut self, sr: u8, addr: Temp, value: Temp, size: u8) {
        let mem = Self::mem_op(size, sr, AccessKind::Store);
        self.b.store(Type::I64, addr, value, mem);
        // Under `Smc::EndBlock` nothing after this store exists to be
        // modified, because the block ends here; under `Shape::BasicBlock` the
        // access ends the block for its own reason and the guard would be
        // unreachable. Everywhere else the guard is what makes x86's coherent
        // instruction cache architectural rather than aspirational.
        if matches!(self.smc, Smc::Guard) && !self.shape.access_ends_block() {
            self.smc_guard(sr, addr);
        }
    }

    /// The self-modifying-code check: leave the block when this store landed in
    /// the page the block was lifted from.
    ///
    /// See the module docs. The comparison is in linear space, and the block's
    /// own page is a lift-time constant, which is why [`World`] carries the
    /// segment bases and why its generation is in the key.
    fn smc_guard(&mut self, sr: u8, addr: Temp) {
        let base = self.world.seg_base[sr as usize];
        let lin = if base == 0 {
            addr
        } else {
            let k = self.konst(base);
            self.b.binary(Opcode::ADD, Type::I64, addr, k)
        };
        let page = self.and_const(lin, !PAGE_MASK);
        let mine = self.konst(self.page);
        // Branch *over* the exit when the store missed this page, which is the
        // common case and costs one not-taken test.
        let over = self.b.emit_raw(
            Opcode::BRCOND,
            Type::I64,
            None,
            None,
            &[page, mine],
            None,
            Some(Cond::Ne),
            0,
        );
        let resume = self.resume;
        self.exit_sequence(resume);
        let after = self.b.next_index() as u32;
        self.b.patch_aux(over, after);
    }

    /// An inline exit: the target `EIP`, a boundary carrying the whole map, and
    /// a terminator.
    fn exit_sequence(&mut self, resume: Resume) {
        let target = match resume.pc {
            Some(t) => t,
            None => self.konst(resume.at),
        };
        let mut live = self.live_state(0);
        live.push((EIP, target));
        self.b.insn_start(InsnStart {
            pc: resume.at,
            next_pc: resume.at,
            ticks: self.ticks,
            live,
        });
        self.b.exit_tb();
    }

    /// A precise side exit taken when `cond` holds (or when it does not).
    ///
    /// The sequence is *inline* and branched over on the negated condition
    /// rather than appended at the end of the block, for the three reasons
    /// `cpu::riscv::lift::side_exit` gives: the boundary records stay in
    /// program order so the tick column stays monotonic, every
    /// [`Opcode::BRCOND`] stays a forward branch — which is what `ir::pass`'s
    /// single backward liveness walk is built on — and the exit's live map is
    /// taken exactly at the branch, which is what makes leaving through it
    /// architecturally precise.
    fn side_exit(&mut self, cond: Temp, exit_when: bool, exit_eip: u64) {
        let zero_bit = self.kbit(false);
        // Skip the exit when the condition says the *other* side is taken.
        let skip = if exit_when { Cond::Eq } else { Cond::Ne };
        let over = self.b.emit_raw(
            Opcode::BRCOND,
            Type::I1,
            None,
            None,
            &[cond, zero_bit],
            None,
            Some(skip),
            0,
        );
        self.exit_sequence(Resume {
            at: exit_eip,
            pc: None,
        });
        let after = self.b.next_index() as u32;
        self.b.patch_aux(over, after);
    }

    // -- effective addresses ---------------------------------------------

    /// The effective address this instruction's memory operand names, computed
    /// once and cached for the instruction.
    ///
    /// Every term is summed at [`Type::I64`] and the sum masked to thirty-two
    /// bits at the end. That is exact rather than a shortcut: each term is
    /// below 2^32, so the 64-bit sum's low thirty-two bits are the 32-bit
    /// wrapping sum. What would *not* be exact is widening a term and never
    /// masking the sum, which is the wrap bug CLAUDE.md's "Arithmetic" section
    /// names.
    fn ea(&mut self, f: &Fields) -> (u8, Temp) {
        if let Some(cached) = self.ea {
            return cached;
        }
        let sr = f.mem_segment();
        let addr = match f.modrm {
            Some(m) if !m.is_register() => {
                let mut terms: Option<Temp> = None;
                if m.rm == 4 {
                    let sib = f.sib.unwrap_or(isa::Sib::new(0));
                    // Base 5 with mode 0 is "no base": the displacement stands
                    // alone.
                    if !(sib.base == 5 && m.md == 0) {
                        terms = Some(self.read_r(f.base_num()));
                    }
                    if f.has_index() {
                        let index = self.read_r(f.index_num());
                        let scaled = if sib.scale == 0 {
                            index
                        } else {
                            self.shl_const(index, u32::from(sib.scale))
                        };
                        terms = Some(match terms {
                            Some(t) => self.b.binary(Opcode::ADD, Type::I64, t, scaled),
                            None => scaled,
                        });
                    }
                } else if !(m.rm == 5 && m.md == 0) {
                    terms = Some(self.read_r(f.rm_num()));
                }
                let disp = f.disp as i64 as u64;
                let sum = match (terms, disp) {
                    (Some(t), 0) => t,
                    (Some(t), d) => {
                        let k = self.konst(d);
                        self.b.binary(Opcode::ADD, Type::I64, t, k)
                    }
                    (None, d) => self.konst(d),
                };
                self.and_const(sum, 0xffff_ffff)
            }
            // `Ob`/`Ov`: the address is the immediate and there is no ModRM
            // byte at all. A ModRM byte that selects a *register* reaches here
            // only through an encoding with no memory form — `LEA` with a mode
            // field of three, which a 386 answers with the address it never
            // computed — and `Exec::ea` answers zero for the same reason.
            Some(_) => self.zero(),
            None => self.konst(f.imm & 0xffff_ffff),
        };
        self.ea = Some((sr, addr));
        (sr, addr)
    }

    // -- operands ---------------------------------------------------------

    /// The operand width this encoding fixes, in bytes.
    fn width(f: &Fields) -> u8 {
        f.insn.width_bytes(f.opsize).unwrap_or(f.opsize)
    }

    /// Read one operand at `size` bytes, mirroring `Exec::read_arg`.
    fn read_arg(&mut self, f: &Fields, arg: Arg, size: u8) -> Option<Temp> {
        let t = match arg {
            Arg::Eb | Arg::Ev | Arg::Ew | Arg::Ed => match f.modrm {
                Some(m) if m.is_register() => self.read_reg(f.rm_num(), size),
                _ => {
                    let (sr, addr) = self.ea(f);
                    self.mem_load(sr, addr, size)
                }
            },
            Arg::Gb | Arg::Gv | Arg::Gw => self.read_reg(f.reg_num(), size),
            Arg::Ib | Arg::Iw | Arg::Iv | Arg::Iz | Arg::Ibs => self.konst(f.imm & mask_of(size)),
            Arg::Rb | Arg::Rv => self.read_reg(f.opcode_reg(), size),
            Arg::Al => self.read_reg(0, 1),
            Arg::Ax => self.read_reg(0, size),
            Arg::Cl => self.read_reg(1, 1),
            Arg::One => self.konst(1),
            // `LEA`'s operand is the address itself, never what is there.
            Arg::M => self.ea(f).1,
            Arg::Ob | Arg::Ov => {
                let (sr, addr) = self.ea(f);
                self.mem_load(sr, addr, size)
            }
            _ => return None,
        };
        Some(t)
    }

    /// Write one operand at `size` bytes, mirroring `Exec::write_arg`.
    fn write_arg(&mut self, f: &Fields, arg: Arg, size: u8, value: Temp) -> bool {
        match arg {
            Arg::Eb | Arg::Ev | Arg::Ew | Arg::Ed => match f.modrm {
                Some(m) if m.is_register() => self.write_reg(f.rm_num(), size, value),
                _ => {
                    let (sr, addr) = self.ea(f);
                    self.mem_store(sr, addr, value, size);
                }
            },
            Arg::Gb | Arg::Gv | Arg::Gw => self.write_reg(f.reg_num(), size, value),
            Arg::Rb | Arg::Rv => self.write_reg(f.opcode_reg(), size, value),
            Arg::Al => self.write_reg(0, 1, value),
            Arg::Ax => self.write_reg(0, size, value),
            Arg::Cl => self.write_reg(1, 1, value),
            Arg::Ob | Arg::Ov => {
                let (sr, addr) = self.ea(f);
                self.mem_store(sr, addr, value, size);
            }
            _ => return false,
        }
        true
    }

    /// Whether this operand reaches memory rather than a register.
    fn is_memory(f: &Fields, arg: Arg) -> bool {
        match arg {
            Arg::Eb | Arg::Ev | Arg::Ew | Arg::Ed => !f.rm_is_register(),
            Arg::Ob | Arg::Ov => true,
            _ => false,
        }
    }

    /// Whether any of this encoding's operands reaches memory.
    fn touches_memory(f: &Fields) -> bool {
        let insn = f.insn;
        [insn.dst, insn.src, insn.aux]
            .into_iter()
            .any(|a| Self::is_memory(f, a))
    }

    // -- flags ------------------------------------------------------------

    /// `PF`: even parity of the low eight bits, which is the only parity x86
    /// computes — and, on nearly every ALU instruction, the one nothing reads.
    fn parity(&mut self, r: Temp) -> Temp {
        let low = self.and_const(r, 0xff);
        let ones = self.b.unary(Opcode::POPCOUNT, Type::I64, low);
        let odd = self.and_const(ones, 1);
        self.is_zero(odd)
    }

    /// `SF`, `ZF` and `PF` from a result already masked to `size`.
    fn set_szp(&mut self, r: Temp, size: u8) {
        let bits = u32::from(size) * 8;
        let zf = self.is_zero(r);
        self.write_flag(F_ZF, zf);
        let sf = self.bit(r, bits - 1);
        self.write_flag(F_SF, sf);
        let pf = self.parity(r);
        self.write_flag(F_PF, pf);
    }

    /// The flags after `AND`, `OR`, `XOR` and `TEST`.
    ///
    /// Carry and overflow are documented as cleared; `AF` is documented as
    /// undefined and is cleared on hardware, on every one of the tens of
    /// thousands of corpus vectors `exec` measured.
    fn logic_flags(&mut self, r: Temp, size: u8) {
        let off = self.kbit(false);
        self.write_flag(F_CF, off);
        self.write_flag(F_OF, off);
        self.write_flag(F_AF, off);
        self.set_szp(r, size);
    }

    /// `a + b + carry` at `size` bytes, with all six flags. Returns the masked
    /// result.
    fn add(&mut self, a: Temp, b: Temp, carry: Option<Temp>, size: u8) -> Temp {
        let bits = u32::from(size) * 8;
        let mask = mask_of(size);
        let partial = self.b.binary(Opcode::ADD, Type::I64, a, b);
        let wide = match carry {
            None => partial,
            Some(c) => {
                let c64 = self.widen(c);
                self.b.binary(Opcode::ADD, Type::I64, partial, c64)
            }
        };
        let r = self.and_const(wide, mask);
        // The carry out, as an unsigned comparison **at the operand's width**
        // rather than as the bit above it. `r < a`, or `r == a` with a carry
        // in — because the true sum is `r + CF * 2^n`, so the only way a
        // wrapped sum equals its own addend is a carry in on an all-ones
        // addend. Reading bit `n` of a wider sum says the same thing at eight,
        // sixteen and thirty-two bits and **nothing at all at sixty-four**,
        // where that bit does not exist and the flag would come out zero. This
        // form has no such width, which is what makes a 64-bit `ADD` lift.
        let lt = self.b.setcond(Cond::LtU, Type::I64, r, a);
        let cf = match carry {
            None => lt,
            Some(c) => {
                let eq = self.b.setcond(Cond::Eq, Type::I64, r, a);
                let wrapped = self.b.binary(Opcode::AND, Type::I1, eq, c);
                self.b.binary(Opcode::OR, Type::I1, lt, wrapped)
            }
        };
        self.write_flag(F_CF, cf);
        let ab = self.b.binary(Opcode::XOR, Type::I64, a, b);
        let abr = self.b.binary(Opcode::XOR, Type::I64, ab, r);
        let af = self.bit(abr, 4);
        self.write_flag(F_AF, af);
        // Signed overflow: the operands agreed in sign and the result did not.
        let same = self.b.unary(Opcode::NOT, Type::I64, ab);
        let ar = self.b.binary(Opcode::XOR, Type::I64, a, r);
        let ov = self.b.binary(Opcode::AND, Type::I64, same, ar);
        let of = self.bit(ov, bits - 1);
        self.write_flag(F_OF, of);
        self.set_szp(r, size);
        r
    }

    /// `a - b - borrow` at `size` bytes, with all six flags.
    fn sub(&mut self, a: Temp, b: Temp, borrow: Option<Temp>, size: u8) -> Temp {
        let bits = u32::from(size) * 8;
        let mask = mask_of(size);
        let rhs = match borrow {
            None => b,
            Some(c) => {
                let c64 = self.widen(c);
                self.b.binary(Opcode::ADD, Type::I64, b, c64)
            }
        };
        let diff = self.b.binary(Opcode::SUB, Type::I64, a, rhs);
        let r = self.and_const(diff, mask);
        // The borrow out, at the operand's width and never through `rhs`.
        // `a < b + borrow` is the definition, and forming that sum is what
        // fails at sixty-four bits: an all-ones subtrahend with a borrow in
        // wraps `rhs` to zero and `a < 0` is false where the answer is a
        // borrow. Split it instead — `a < b`, or `a == b` with a borrow in —
        // which is the same statement with nothing to wrap.
        let lt = self.b.setcond(Cond::LtU, Type::I64, a, b);
        let cf = match borrow {
            None => lt,
            Some(c) => {
                let eq = self.b.setcond(Cond::Eq, Type::I64, a, b);
                let exact = self.b.binary(Opcode::AND, Type::I1, eq, c);
                self.b.binary(Opcode::OR, Type::I1, lt, exact)
            }
        };
        self.write_flag(F_CF, cf);
        let ab = self.b.binary(Opcode::XOR, Type::I64, a, b);
        let abr = self.b.binary(Opcode::XOR, Type::I64, ab, r);
        let af = self.bit(abr, 4);
        self.write_flag(F_AF, af);
        let ar = self.b.binary(Opcode::XOR, Type::I64, a, r);
        let ov = self.b.binary(Opcode::AND, Type::I64, ab, ar);
        let of = self.bit(ov, bits - 1);
        self.write_flag(F_OF, of);
        self.set_szp(r, size);
        r
    }

    /// Evaluate one of the sixteen condition codes into a one-bit temporary.
    ///
    /// The guests' own condition codes are deliberately not an IR concept
    /// (`ir::op`, [`Cond`]), so each one is one or two ops over flag
    /// temporaries — which is also what makes a `Jcc` whose flags were computed
    /// two instructions earlier cost nothing but the branch.
    fn condition(&mut self, cc: u8) -> Temp {
        match cc & 15 {
            0 | 1 => {
                let t = self.read_flag(F_OF);
                self.maybe_not(t, cc & 1 == 1)
            }
            2 | 3 => {
                let t = self.read_flag(F_CF);
                self.maybe_not(t, cc & 1 == 1)
            }
            4 | 5 => {
                let t = self.read_flag(F_ZF);
                self.maybe_not(t, cc & 1 == 1)
            }
            6 | 7 => {
                let cf = self.read_flag(F_CF);
                let zf = self.read_flag(F_ZF);
                let t = self.b.binary(Opcode::OR, Type::I1, cf, zf);
                self.maybe_not(t, cc & 1 == 1)
            }
            8 | 9 => {
                let t = self.read_flag(F_SF);
                self.maybe_not(t, cc & 1 == 1)
            }
            10 | 11 => {
                let t = self.read_flag(F_PF);
                self.maybe_not(t, cc & 1 == 1)
            }
            12 | 13 => {
                let sf = self.read_flag(F_SF);
                let of = self.read_flag(F_OF);
                let t = self.b.binary(Opcode::XOR, Type::I1, sf, of);
                self.maybe_not(t, cc & 1 == 1)
            }
            _ => {
                let sf = self.read_flag(F_SF);
                let of = self.read_flag(F_OF);
                let ne = self.b.binary(Opcode::XOR, Type::I1, sf, of);
                let zf = self.read_flag(F_ZF);
                let t = self.b.binary(Opcode::OR, Type::I1, ne, zf);
                self.maybe_not(t, cc & 1 == 1)
            }
        }
    }

    fn maybe_not(&mut self, t: Temp, invert: bool) -> Temp {
        if invert {
            self.b.unary(Opcode::NOT, Type::I1, t)
        } else {
            t
        }
    }

    // -- the stack --------------------------------------------------------

    /// `ESP -= size`, then the store.
    ///
    /// That order is the architecture's, and it is why a fault on the store
    /// reports the *old* `ESP`: the boundary that publishes the map was opened
    /// before the pointer moved, and `Exec::step` restores its own snapshot for
    /// the same reason.
    fn push(&mut self, value: Temp, size: u8) {
        let esp = self.read_r(4);
        let k = self.konst(u64::from(size));
        let moved = self.b.binary(Opcode::SUB, Type::I64, esp, k);
        let sp = self.and_const(moved, 0xffff_ffff);
        self.write_r(4, sp);
        self.mem_store(seg::SS, sp, value, size);
    }

    /// The load, then `ESP += size`.
    fn pop(&mut self, size: u8) -> Temp {
        let esp = self.read_r(4);
        let value = self.mem_load(seg::SS, esp, size);
        let k = self.konst(u64::from(size));
        let moved = self.b.binary(Opcode::ADD, Type::I64, esp, k);
        let sp = self.and_const(moved, 0xffff_ffff);
        self.write_r(4, sp);
        value
    }

    // -- one instruction --------------------------------------------------

    fn insn(&mut self, f: &Fields, eip: u64, next_eip: u64) -> Flow {
        let Some(plan) = classify(self.world, f, next_eip) else {
            return Flow::Rejected;
        };
        self.ea = None;
        self.resume = Resume {
            at: next_eip,
            pc: None,
        };

        // The boundary, then the static charge. `Exec::instruction` makes every
        // static charge before it makes any access, so one charge here is
        // exact rather than an approximation of two.
        let skip = self.elidable(f, plan);
        let live = self.live_state(skip);
        self.b.insn_start(InsnStart {
            pc: eip,
            next_pc: next_eip,
            ticks: self.ticks,
            live,
        });
        let bus = u64::from(self.world.variant.bus_clocks());
        let charge = bus * u64::from(f.len) + u64::from(f.insn.op.clocks());
        self.b.charge(charge);
        self.ticks += charge;

        // The effective address is computed **before** execution, from the
        // register file as it stands at the start of the instruction — which is
        // where `Exec::prepare_ea` sits, and it is not a detail: `push [esp]`
        // reads its operand and then moves the pointer, so the address is the
        // one the instruction began with. `POP` is the exception the
        // architecture makes and `Plan::Pop` handles it by dropping this
        // cache; nothing else in the subset rebinds a register before it
        // reaches memory, which is exactly the kind of "nothing else" that
        // stops being true one instruction later.
        if Self::touches_memory(f) {
            let _ = self.ea(f);
        }

        self.emit(f, plan, eip, next_eip)
    }

    /// Which flag slots this boundary may leave out.
    ///
    /// The rule, and the argument that it is sound, are in the module docs.
    /// Both halves matter: an instruction that can fault must leave every flag
    /// recoverable, because the fault is delivered *at* this boundary.
    fn elidable(&self, f: &Fields, plan: Plan) -> u8 {
        if matches!(self.policy, Flags::Eager) {
            return 0;
        }
        let stack = matches!(
            plan,
            Plan::Push
                | Plan::Pop
                | Plan::Leave
                | Plan::CallRel { .. }
                | Plan::CallInd
                | Plan::Ret { .. }
        );
        if stack || Self::touches_memory(f) {
            return 0;
        }
        plan.writes()
    }

    #[allow(clippy::too_many_lines)]
    fn emit(&mut self, f: &Fields, plan: Plan, eip: u64, next_eip: u64) -> Flow {
        let insn = f.insn;
        let size = Self::width(f);
        let mem = Self::touches_memory(f);

        match plan {
            Plan::Alu(op) => {
                let Some(a) = self.read_arg(f, insn.dst, size) else {
                    return Flow::Rejected;
                };
                let Some(b) = self.read_arg(f, insn.src, size) else {
                    return Flow::Rejected;
                };
                let carry = if matches!(op, Op::ADC | Op::SBB) {
                    Some(self.read_flag(F_CF))
                } else {
                    None
                };
                let r = match op {
                    Op::ADD => self.add(a, b, None, size),
                    Op::ADC => self.add(a, b, carry, size),
                    Op::SUB | Op::CMP => self.sub(a, b, None, size),
                    Op::SBB => self.sub(a, b, carry, size),
                    Op::AND | Op::TEST => {
                        let r = self.b.binary(Opcode::AND, Type::I64, a, b);
                        self.logic_flags(r, size);
                        r
                    }
                    Op::OR => {
                        let r = self.b.binary(Opcode::OR, Type::I64, a, b);
                        self.logic_flags(r, size);
                        r
                    }
                    _ => {
                        let r = self.b.binary(Opcode::XOR, Type::I64, a, b);
                        self.logic_flags(r, size);
                        r
                    }
                };
                // `CMP` and `TEST` are the two that compute a result and throw
                // it away, which is why they are here rather than in their own
                // arm: the flags are the instruction.
                let stored = if matches!(op, Op::CMP | Op::TEST) {
                    false
                } else {
                    if !self.write_arg(f, insn.dst, size, r) {
                        return Flow::Rejected;
                    }
                    Self::is_memory(f, insn.dst)
                };
                Self::flow(next_eip, mem, stored)
            }
            Plan::IncDec(inc) => {
                // The carry is preserved, which is what makes `INC` usable
                // inside an add-with-carry chain — and is why this plan reports
                // five writable flags rather than six.
                let carry = self.read_flag(F_CF);
                let Some(a) = self.read_arg(f, insn.dst, size) else {
                    return Flow::Rejected;
                };
                let one = self.konst(1);
                let r = if inc {
                    self.add(a, one, None, size)
                } else {
                    self.sub(a, one, None, size)
                };
                self.write_flag(F_CF, carry);
                if !self.write_arg(f, insn.dst, size, r) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, Self::is_memory(f, insn.dst))
            }
            Plan::Not => {
                let Some(a) = self.read_arg(f, insn.dst, size) else {
                    return Flow::Rejected;
                };
                let inverted = self.b.unary(Opcode::NOT, Type::I64, a);
                let r = self.and_const(inverted, mask_of(size));
                if !self.write_arg(f, insn.dst, size, r) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, Self::is_memory(f, insn.dst))
            }
            Plan::Neg => {
                let Some(a) = self.read_arg(f, insn.dst, size) else {
                    return Flow::Rejected;
                };
                let zero = self.zero();
                let r = self.sub(zero, a, None, size);
                if !self.write_arg(f, insn.dst, size, r) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, Self::is_memory(f, insn.dst))
            }
            Plan::Mov => {
                let Some(v) = self.read_arg(f, insn.src, size) else {
                    return Flow::Rejected;
                };
                if !self.write_arg(f, insn.dst, size, v) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, Self::is_memory(f, insn.dst))
            }
            Plan::MovX { signed, src_size } => {
                let Some(raw) = self.read_arg(f, insn.src, src_size) else {
                    return Flow::Rejected;
                };
                let v = if signed {
                    let wide = self.sext(raw, u32::from(src_size) * 8);
                    self.and_const(wide, mask_of(f.opsize))
                } else {
                    raw
                };
                if !self.write_arg(f, insn.dst, f.opsize, v) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, false)
            }
            Plan::Lea => {
                let (_, addr) = self.ea(f);
                // The address size decides how much of the address exists; the
                // operand size decides how much of it is stored.
                let v = self.and_const(addr, mask_of(f.opsize));
                if !self.write_arg(f, insn.dst, f.opsize, v) {
                    return Flow::Rejected;
                }
                // `LEA` computes an address and never uses it, so it is not an
                // access however memory-shaped its operand looks.
                Flow::Continue(next_eip)
            }
            Plan::Shift { op, count } => {
                if !self.shift(f, op, count, size) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, Self::is_memory(f, insn.dst))
            }
            Plan::Mul { signed } => {
                if !self.multiply(f, signed, size) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, false)
            }
            Plan::ImulShort => {
                if !self.imul_short(f, size) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, false)
            }
            Plan::BitScan { reverse } => {
                let Some(src) = self.read_arg(f, insn.src, size) else {
                    return Flow::Rejected;
                };
                let zf = self.is_zero(src);
                self.write_flag(F_ZF, zf);
                let index = if reverse {
                    // The source is masked to the operand size, so the highest
                    // set bit is `63 - clz` at sixty-four bits.
                    let clz = self.b.unary(Opcode::CLZ, Type::I64, src);
                    let k = self.konst(63);
                    self.b.binary(Opcode::SUB, Type::I64, k, clz)
                } else {
                    self.b.unary(Opcode::CTZ, Type::I64, src)
                };
                // A source of zero leaves the destination untouched — the
                // manual calls the result undefined and the silicon writes
                // nothing. The destination is a register in every encoding, so
                // selecting the old value is the same thing and needs no
                // conditional store.
                let Some(old) = self.read_arg(f, insn.dst, size) else {
                    return Flow::Rejected;
                };
                let v = self.b.emit(Opcode::MOVCOND, Type::I64, &[zf, old, index]);
                if !self.write_arg(f, insn.dst, size, v) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, false)
            }
            Plan::SetCc(cc) => {
                let cond = self.condition(cc);
                let v = self.widen(cond);
                if !self.write_arg(f, insn.dst, 1, v) {
                    return Flow::Rejected;
                }
                let stored = Self::is_memory(f, insn.dst);
                Self::flow(next_eip, stored, stored)
            }
            Plan::CmovCc(cc) => {
                // The source is read unconditionally — `Exec` reads it before
                // it evaluates the condition, and on a memory operand that is a
                // bus cycle the guest can see. The destination is written
                // either way for the same reason.
                let Some(v) = self.read_arg(f, insn.src, size) else {
                    return Flow::Rejected;
                };
                let cond = self.condition(cc);
                let Some(old) = self.read_arg(f, insn.dst, size) else {
                    return Flow::Rejected;
                };
                let chosen = self.b.emit(Opcode::MOVCOND, Type::I64, &[cond, v, old]);
                if !self.write_arg(f, insn.dst, size, chosen) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, mem, false)
            }
            Plan::Jcc { cc, target } => {
                let cond = self.condition(cc);
                if !self.shape.merges() {
                    let then = self.konst(target);
                    let other = self.konst(next_eip);
                    let sel = self
                        .b
                        .emit(Opcode::MOVCOND, Type::I64, &[cond, then, other]);
                    self.pc_out = Some(sel);
                    return Flow::Transfer;
                }
                // Static prediction, and it is what decides whether a loop
                // unrolls: a backward branch is a back edge, so the taken side
                // is the trace; a forward one is an `if`, so the fall-through
                // is. A target off the entry page is never a candidate, because
                // no instruction outside that page may be lifted.
                let target_here = self.world.linear(target) & !PAGE_MASK == self.page;
                let inline_taken = target < eip && target_here;
                let (exit_pc, next, exit_when) = if inline_taken {
                    (next_eip, target, false)
                } else if target_here {
                    (target, next_eip, true)
                } else {
                    // The taken side leaves the page, so it has to be the exit
                    // whatever the direction says.
                    (target, next_eip, true)
                };
                self.side_exit(cond, exit_when, exit_pc);
                Flow::Continue(next)
            }
            Plan::JmpRel { target } => {
                if self.shape.merges() && self.world.linear(target) & !PAGE_MASK == self.page {
                    // A direct unconditional transfer: the trace continues at
                    // the target and the jump costs nothing but its fetch.
                    return Flow::Continue(target);
                }
                let t = self.konst(target);
                self.pc_out = Some(t);
                self.static_exit = Some(target);
                Flow::Transfer
            }
            Plan::JmpInd => {
                let Some(t) = self.read_arg(f, insn.dst, f.opsize) else {
                    return Flow::Rejected;
                };
                let target = self.and_const(t, mask_of(f.opsize));
                self.pc_out = Some(target);
                Flow::Transfer
            }
            Plan::CallRel { target } => {
                // The return address is pushed before the transfer, so a store
                // that hit this block's own page must resume at the *target*
                // rather than after the call. That is the whole reason
                // `Resume` is a field.
                let merged = self.shape.merges()
                    && matches!(self.smc, Smc::Guard)
                    && self.world.linear(target) & !PAGE_MASK == self.page;
                if merged {
                    self.resume = Resume {
                        at: target,
                        pc: None,
                    };
                }
                let ret = self.konst(next_eip);
                self.push(ret, f.opsize);
                if merged {
                    return Flow::Continue(target);
                }
                let t = self.konst(target);
                self.pc_out = Some(t);
                self.static_exit = Some(target);
                Flow::Transfer
            }
            Plan::CallInd => {
                // The target is read before the return address is pushed, which
                // is what makes `call [esp]` correct.
                let Some(raw) = self.read_arg(f, insn.dst, f.opsize) else {
                    return Flow::Rejected;
                };
                let target = self.and_const(raw, mask_of(f.opsize));
                self.resume = Resume {
                    at: next_eip,
                    pc: Some(target),
                };
                let ret = self.konst(next_eip);
                self.push(ret, f.opsize);
                self.pc_out = Some(target);
                Flow::Transfer
            }
            Plan::Ret { extra } => {
                let ip = self.pop(f.opsize);
                if extra != 0 {
                    let esp = self.read_r(4);
                    let k = self.konst(extra);
                    let moved = self.b.binary(Opcode::ADD, Type::I64, esp, k);
                    let sp = self.and_const(moved, 0xffff_ffff);
                    self.write_r(4, sp);
                }
                let target = self.and_const(ip, mask_of(f.opsize));
                self.pc_out = Some(target);
                Flow::Transfer
            }
            Plan::Push => {
                let Some(v) = self.read_arg(f, insn.dst, f.opsize) else {
                    return Flow::Rejected;
                };
                self.push(v, f.opsize);
                Self::flow(next_eip, true, true)
            }
            Plan::Pop => {
                let v = self.pop(f.opsize);
                // The one place the pre-computed address has to be thrown
                // away. *Intel SDM* volume 2, `POP`: a destination addressed
                // through the stack pointer is computed **after** the
                // increment, so `pop [esp+4]` stores four bytes above where
                // the address taken at the start of the instruction points.
                // Dropping the cache here makes `write_arg` recompute from the
                // stack pointer `pop` has just moved, which is what
                // `Exec::POP` does on the other side of the differential.
                if Self::is_memory(f, insn.dst) {
                    self.ea = None;
                }
                if !self.write_arg(f, insn.dst, f.opsize, v) {
                    return Flow::Rejected;
                }
                Self::flow(next_eip, true, Self::is_memory(f, insn.dst))
            }
            Plan::Leave => {
                // `LEAVE` is `mov esp, ebp` then `pop ebp`.
                let ebp = self.read_r(5);
                self.write_r(4, ebp);
                let v = self.pop(f.opsize);
                self.write_reg(5, f.opsize, v);
                Self::flow(next_eip, true, false)
            }
            Plan::Carry(set) => {
                let cf = match set {
                    Some(value) => self.kbit(value),
                    None => {
                        let old = self.read_flag(F_CF);
                        self.b.unary(Opcode::NOT, Type::I1, old)
                    }
                };
                self.write_flag(F_CF, cf);
                Flow::Continue(next_eip)
            }
            Plan::Direction(set) => {
                let rest = self.read_rest();
                let bit = self.kbit(set);
                let wide = self.widen(bit);
                let updated = self.deposit(rest, wide, flags::DF.trailing_zeros(), 1);
                self.rest = Some(updated);
                Flow::Continue(next_eip)
            }
            Plan::Lahf => {
                // The packed low byte, assembled here and nowhere else. Bits 3
                // and 5 have no storage and bit 1 always reads as one, all of
                // which is already true of `EFLAGS_REST`.
                let rest = self.read_rest();
                let mut ah = self.and_const(rest, 0xff);
                for (i, shift) in LOW_BYTE_LAYOUT {
                    let bit = self.read_flag(i);
                    let wide = self.widen(bit);
                    ah = self.deposit(ah, wide, shift, 1);
                }
                self.write_reg(4, 1, ah);
                Flow::Continue(next_eip)
            }
            Plan::Sahf => {
                let ah = self.read_reg(4, 1);
                for (i, shift) in LOW_BYTE_LAYOUT {
                    let bit = self.bit(ah, shift);
                    self.write_flag(i, bit);
                }
                Flow::Continue(next_eip)
            }
            Plan::Cbw => {
                let (from_bytes, to) = if f.opsize == 2 { (1u8, 2u8) } else { (2, 4) };
                let src = self.read_reg(0, from_bytes);
                let wide = self.sext(src, u32::from(from_bytes) * 8);
                let v = self.and_const(wide, mask_of(to));
                self.write_reg(0, to, v);
                Flow::Continue(next_eip)
            }
            Plan::Cwd => {
                let bits = u32::from(f.opsize) * 8;
                let acc = self.read_reg(0, f.opsize);
                let sign = self.bit(acc, bits - 1);
                let all = self.konst(mask_of(f.opsize));
                let zero = self.zero();
                let fill = self.b.emit(Opcode::MOVCOND, Type::I64, &[sign, all, zero]);
                self.write_reg(2, f.opsize, fill);
                Flow::Continue(next_eip)
            }
            Plan::Bswap => {
                let index = f.opcode_reg();
                let v = self.read_r(index);
                let dst = self.b.temp(Type::I64);
                // Lane width 32: the swap happens within the low doubleword,
                // which is the whole register on a 386. `Exec` performs the
                // doubleword swap at every operand size below eight, so this
                // does too.
                self.b.emit_raw(
                    Opcode::BSWAP,
                    Type::I64,
                    Some(dst),
                    None,
                    &[v],
                    Some(Const::Int(32)),
                    None,
                    0,
                );
                let masked = self.and_const(dst, 0xffff_ffff);
                self.write_r(index, masked);
                Flow::Continue(next_eip)
            }
            Plan::Nop => Flow::Continue(next_eip),
        }
    }

    /// Turn "this instruction touched memory" into the right [`Flow`].
    const fn flow(next: u64, touched: bool, store: bool) -> Flow {
        if touched || store {
            Flow::Access { next, store }
        } else {
            Flow::Continue(next)
        }
    }

    // -- shifts and rotates ------------------------------------------------

    /// One shift or rotate.
    ///
    /// `Exec::shift_value` iterates one bit at a time because the overflow flag
    /// of a multi-bit rotate is the *last* iteration's, and `RCL`/`RCR` rotate
    /// through a carry that changes under them. Every one of those
    /// last-iteration values has a closed form, and each is derived in a
    /// comment beside it — an iteration count is not something a translated
    /// block can afford, and "it looked right" is not something a flag can be.
    #[allow(clippy::too_many_lines)]
    fn shift(&mut self, f: &Fields, op: Op, count: Count, size: u8) -> bool {
        let insn = f.insn;
        let bits = u32::from(size) * 8;
        let mask = mask_of(size);
        let Some(a) = self.read_arg(f, insn.dst, size) else {
            return false;
        };

        // The flag bindings as they stood before this instruction. A `CL` count
        // of zero leaves every one of them alone, and selecting between the two
        // is the only way to say so without a branch.
        let before = self.fl;

        let (n, nz) = match count {
            Count::Fixed(k) => (Amount::Fixed(k), None),
            Count::Cl => {
                let cl = self.read_reg(1, 1);
                let masked = self.and_const(cl, 31);
                let zero = self.zero();
                let nz = self.b.setcond(Cond::Ne, Type::I64, masked, zero);
                (Amount::Dynamic(masked), Some(nz))
            }
        };

        let r = match op {
            Op::SHL => self.shl(a, n, bits, mask),
            Op::SHR => self.shr(a, n, bits, mask),
            Op::SAR => self.sar(a, n, bits, mask),
            Op::ROL => self.rol(a, n, bits, mask),
            Op::ROR => self.ror(a, n, bits, mask),
            Op::RCL => self.rcl1(a, bits, mask),
            Op::RCR => self.rcr1(a, bits, mask),
            _ => {
                // `SETMO`: the operand becomes all ones and the flags follow it
                // as a logical result would. Undocumented, and every one of
                // those flags is a lift-time constant.
                let ones = self.konst(mask);
                self.logic_flags(ones, size);
                ones
            }
        };

        // Only the three arithmetic shifts touch sign, zero, parity and the
        // auxiliary carry; the rotates leave all four alone, which is a rule
        // people get wrong by symmetry.
        if matches!(op, Op::SHL | Op::SHR | Op::SAR) {
            self.set_szp(r, size);
            let af = if matches!(op, Op::SHL) {
                // The microcode is an `ADD dst,dst`, so bit 4 of the result is
                // the real auxiliary carry — measured against hardware, not
                // guessed; see `exec`'s undefined-flag table.
                self.bit(r, 4)
            } else {
                self.kbit(false)
            };
            self.write_flag(F_AF, af);
        }

        let value = match nz {
            None => r,
            Some(nz) => {
                for i in 0..6 {
                    if self.fl[i] == before[i] {
                        continue;
                    }
                    let old = match before[i] {
                        Some(t) => t,
                        // Nothing in this block has bound the flag, so the
                        // host's own copy is still the architectural one and a
                        // slot read is the right way to reach it. Emitted here
                        // rather than up front so that a shift whose count is
                        // never zero costs nothing for it.
                        None => self.b.get_slot(Type::I1, FLAG_SLOTS[i]),
                    };
                    let new = match self.fl[i] {
                        Some(t) => t,
                        None => continue,
                    };
                    let chosen = self.b.emit(Opcode::MOVCOND, Type::I1, &[nz, new, old]);
                    self.write_flag(i, chosen);
                }
                self.b.emit(Opcode::MOVCOND, Type::I64, &[nz, r, a])
            }
        };
        self.write_arg(f, insn.dst, size, value)
    }

    fn shl(&mut self, a: Temp, n: Amount, bits: u32, mask: u64) -> Temp {
        let wide = self.shift_by(Opcode::SHL, a, n);
        let r = self.and_const(wide, mask);
        // The last bit shifted out is bit `bits` of the *unmasked* product,
        // which stays true when the count exceeds the operand width: the
        // product is then zero below `bits` as well.
        let cf = self.bit(wide, bits);
        self.write_flag(F_CF, cf);
        let msb = self.bit(r, bits - 1);
        let of = self.b.binary(Opcode::XOR, Type::I1, msb, cf);
        self.write_flag(F_OF, of);
        r
    }

    fn shr(&mut self, a: Temp, n: Amount, bits: u32, mask: u64) -> Temp {
        let shifted = self.shift_by(Opcode::SHR, a, n);
        let r = self.and_const(shifted, mask);
        // `CF` is bit `n - 1` of the source. The subtraction is masked so the
        // amount stays inside the type even on the discarded `n == 0` path of
        // the `CL` form, which the IR requires: an out-of-range shift is
        // *undefined*, so emitting one would let two backends disagree about a
        // value one of them throws away.
        let below = self.amount_minus_one(n);
        let out = self.shift_by(Opcode::SHR, a, below);
        let cf = self.bit(out, 0);
        self.write_flag(F_CF, cf);
        // `OF` is the source's sign bit, and only for a count of exactly one:
        // every later iteration sees a value whose top bit is already zero.
        let of = self.only_at_one(n, a, bits - 1);
        self.write_flag(F_OF, of);
        r
    }

    fn sar(&mut self, a: Temp, n: Amount, bits: u32, mask: u64) -> Temp {
        let wide = self.sext(a, bits);
        let shifted = self.shift_by(Opcode::SAR, wide, n);
        let r = self.and_const(shifted, mask);
        let below = self.amount_minus_one(n);
        let out = self.shift_by(Opcode::SAR, wide, below);
        let cf = self.bit(out, 0);
        self.write_flag(F_CF, cf);
        // `SAR` never sets overflow: the sign cannot change.
        let of = self.kbit(false);
        self.write_flag(F_OF, of);
        r
    }

    fn rol(&mut self, a: Temp, n: Amount, bits: u32, mask: u64) -> Temp {
        let e = self.reduce(n, bits);
        let left = self.shift_by(Opcode::SHL, a, e);
        let complement = self.complement(e, bits);
        let right = self.shift_by(Opcode::SHR, a, complement);
        let joined = self.b.binary(Opcode::OR, Type::I64, left, right);
        let r = self.and_const(joined, mask);
        // The bit that came round into position zero is the one that left the
        // top, which is the carry the last iteration set.
        let cf = self.bit(r, 0);
        self.write_flag(F_CF, cf);
        let msb = self.bit(r, bits - 1);
        let of = self.b.binary(Opcode::XOR, Type::I1, msb, cf);
        self.write_flag(F_OF, of);
        r
    }

    fn ror(&mut self, a: Temp, n: Amount, bits: u32, mask: u64) -> Temp {
        let e = self.reduce(n, bits);
        let right = self.shift_by(Opcode::SHR, a, e);
        let complement = self.complement(e, bits);
        let left = self.shift_by(Opcode::SHL, a, complement);
        let joined = self.b.binary(Opcode::OR, Type::I64, left, right);
        let r = self.and_const(joined, mask);
        let cf = self.bit(r, bits - 1);
        self.write_flag(F_CF, cf);
        let below = self.bit(r, bits - 2);
        let of = self.b.binary(Opcode::XOR, Type::I1, cf, below);
        self.write_flag(F_OF, of);
        r
    }

    /// `RCL` by one — an (N+1)-bit rotate at the *operand's* width.
    ///
    /// [`Opcode::ROTLC`] is exactly this shape and cannot be used: it rotates at
    /// the temporary's type width, and the IR has no `i8` or `i16` for the two
    /// narrower operand sizes. See the module docs.
    fn rcl1(&mut self, a: Temp, bits: u32, mask: u64) -> Temp {
        let carry_in = self.read_flag(F_CF);
        let cin = self.widen(carry_in);
        let up = self.shl_const(a, 1);
        let joined = self.b.binary(Opcode::OR, Type::I64, up, cin);
        let r = self.and_const(joined, mask);
        let cf = self.bit(a, bits - 1);
        self.write_flag(F_CF, cf);
        let msb = self.bit(r, bits - 1);
        let of = self.b.binary(Opcode::XOR, Type::I1, msb, cf);
        self.write_flag(F_OF, of);
        r
    }

    /// `RCR` by one. The overflow flag is computed from the value *before* the
    /// shift, which is the order `Exec::shift_value` uses and the opposite of
    /// `RCL`'s.
    fn rcr1(&mut self, a: Temp, bits: u32, mask: u64) -> Temp {
        let carry_in = self.read_flag(F_CF);
        let msb = self.bit(a, bits - 1);
        let of = self.b.binary(Opcode::XOR, Type::I1, msb, carry_in);
        self.write_flag(F_OF, of);
        let cf = self.bit(a, 0);
        let cin = self.widen(carry_in);
        let top = self.shl_const(cin, bits - 1);
        let down = self.shr_const(a, 1);
        let joined = self.b.binary(Opcode::OR, Type::I64, down, top);
        let r = self.and_const(joined, mask);
        self.write_flag(F_CF, cf);
        r
    }

    /// A shift by an amount that is a lift-time constant more often than not.
    fn shift_by(&mut self, op: Opcode, v: Temp, n: Amount) -> Temp {
        match n {
            Amount::Fixed(k) => {
                let t = self.konst(u64::from(k));
                self.b.binary(op, Type::I64, v, t)
            }
            Amount::Dynamic(t) => self.b.binary(op, Type::I64, v, t),
        }
    }

    /// `n - 1`, kept inside the type. See [`Lifter::shr`] for why the mask is
    /// not optional.
    fn amount_minus_one(&mut self, n: Amount) -> Amount {
        match n {
            Amount::Fixed(k) => Amount::Fixed(k.saturating_sub(1)),
            Amount::Dynamic(t) => {
                let one = self.konst(1);
                let d = self.b.binary(Opcode::SUB, Type::I64, t, one);
                Amount::Dynamic(self.and_const(d, 31))
            }
        }
    }

    /// `n % bits`, where `bits` is a power of two.
    fn reduce(&mut self, n: Amount, bits: u32) -> Amount {
        match n {
            Amount::Fixed(k) => Amount::Fixed(k % bits),
            Amount::Dynamic(t) => Amount::Dynamic(self.and_const(t, u64::from(bits - 1))),
        }
    }

    /// `bits - n`, for an `n` already reduced below `bits`. The result is
    /// between one and `bits`, which is always a legal shift at `i64`.
    fn complement(&mut self, n: Amount, bits: u32) -> Amount {
        match n {
            Amount::Fixed(k) => Amount::Fixed(bits - k),
            Amount::Dynamic(t) => {
                let k = self.konst(u64::from(bits));
                Amount::Dynamic(self.b.binary(Opcode::SUB, Type::I64, k, t))
            }
        }
    }

    /// Bit `pos` of `v`, but only when the shift amount is exactly one.
    fn only_at_one(&mut self, n: Amount, v: Temp, pos: u32) -> Temp {
        match n {
            Amount::Fixed(1) => self.bit(v, pos),
            Amount::Fixed(_) => self.kbit(false),
            Amount::Dynamic(t) => {
                let bit = self.bit(v, pos);
                let one = self.konst(1);
                let is_one = self.b.setcond(Cond::Eq, Type::I64, t, one);
                self.b.binary(Opcode::AND, Type::I1, is_one, bit)
            }
        }
    }

    // -- multiplies --------------------------------------------------------

    /// One-operand `MUL` and `IMUL`.
    ///
    /// Both products fit in [`Type::I64`] at every operand size this subset
    /// has, so no widening multiply is needed — see the module docs.
    fn multiply(&mut self, f: &Fields, signed: bool, size: u8) -> bool {
        let bits = u32::from(size) * 8;
        let mask = mask_of(size);
        let Some(src) = self.read_arg(f, f.insn.dst, size) else {
            return false;
        };
        let acc = self.read_reg(0, size);
        let (a, b) = if signed {
            let a = self.sext(acc, bits);
            let b = self.sext(src, bits);
            (a, b)
        } else {
            (acc, src)
        };
        let product = self.b.binary(Opcode::MUL, Type::I64, a, b);
        let low = self.and_const(product, mask);
        let shifted = self.shr_const(product, bits);
        let high = self.and_const(shifted, mask);

        if size == 1 {
            // A byte multiply's whole result is `AX`, not `AH:AL` as two
            // registers.
            let up = self.shl_const(high, 8);
            let word = self.b.binary(Opcode::OR, Type::I64, low, up);
            self.write_reg(0, 2, word);
        } else {
            self.write_reg(0, size, low);
            self.write_reg(2, size, high);
        }

        let overflow = if signed {
            let sign = self.bit(low, bits - 1);
            let all = self.konst(mask);
            let zero = self.zero();
            let fill = self.b.emit(Opcode::MOVCOND, Type::I64, &[sign, all, zero]);
            self.b.setcond(Cond::Ne, Type::I64, high, fill)
        } else {
            let zero = self.zero();
            self.b.setcond(Cond::Ne, Type::I64, high, zero)
        };
        self.mul_flags(high, size, overflow);
        true
    }

    /// The 80186's two-operand and three-operand `IMUL`.
    ///
    /// Only `CF` and `OF` are defined, and they say whether the full product
    /// fits in the destination. `Exec` sets the other four from the high half —
    /// the 8088's rule, kept for the 386 rather than inventing a second one —
    /// so this does too.
    fn imul_short(&mut self, f: &Fields, size: u8) -> bool {
        let insn = f.insn;
        let bits = u32::from(size) * 8;
        let mask = mask_of(size);
        let Some(a_raw) = self.read_arg(f, insn.src, size) else {
            return false;
        };
        let b_raw = if matches!(insn.aux, Arg::None) {
            match self.read_arg(f, insn.dst, size) {
                Some(t) => t,
                None => return false,
            }
        } else {
            let width = if matches!(insn.aux, Arg::Ibs) {
                1
            } else {
                size
            };
            match self.read_arg(f, insn.aux, width) {
                Some(t) => {
                    if width == 1 {
                        let wide = self.sext(t, 8);
                        self.and_const(wide, mask)
                    } else {
                        t
                    }
                }
                None => return false,
            }
        };
        let a = self.sext(a_raw, bits);
        let b = self.sext(b_raw, bits);
        let product = self.b.binary(Opcode::MUL, Type::I64, a, b);
        let truncated = self.and_const(product, mask);
        let back = self.sext(truncated, bits);
        let fits = self.b.setcond(Cond::Eq, Type::I64, back, product);
        let spilled = self.b.unary(Opcode::NOT, Type::I1, fits);
        let shifted = self.shr_const(product, bits);
        let high = self.and_const(shifted, mask);
        self.mul_flags(high, size, spilled);
        self.write_arg(f, insn.dst, size, truncated)
    }

    /// The flags a multiply leaves: `CF` and `OF` say the product overflowed,
    /// and the four documented as undefined come from the **high half**.
    fn mul_flags(&mut self, high: Temp, size: u8, overflow: Temp) {
        let bits = u32::from(size) * 8;
        let zf = self.is_zero(high);
        self.write_flag(F_ZF, zf);
        let sf = self.bit(high, bits - 1);
        self.write_flag(F_SF, sf);
        let pf = self.parity(high);
        self.write_flag(F_PF, pf);
        let off = self.kbit(false);
        self.write_flag(F_AF, off);
        self.write_flag(F_CF, overflow);
        self.write_flag(F_OF, overflow);
    }

    // -- closing ------------------------------------------------------------

    /// Close the block: the exit boundary, then the terminator.
    ///
    /// The exit boundary begins no instruction. It carries the outgoing map and
    /// the [`EIP`] slot, which is the only thing that tells a dispatcher where
    /// to resume.
    fn finish(mut self, program_order_eip: u64) -> Block {
        let pc = match self.pc_out {
            Some(t) => t,
            None => self.konst(program_order_eip),
        };
        let mut live = self.live_state(0);
        live.push((EIP, pc));
        let at = self.static_exit.unwrap_or(program_order_eip);
        self.b.insn_start(InsnStart {
            pc: at,
            next_pc: at,
            ticks: self.ticks,
            live,
        });
        self.b.exit_tb();
        self.b.finish()
    }
}

/// Where each of the five status flags sits in the low byte of `EFLAGS`, which
/// `LAHF` and `SAHF` move to and from `AH`.
const LOW_BYTE_LAYOUT: [(usize, u32); 5] = [(F_CF, 0), (F_PF, 2), (F_AF, 4), (F_ZF, 6), (F_SF, 7)];

/// A shift amount, which is a lift-time constant more often than not.
#[derive(Debug, Clone, Copy)]
enum Amount {
    Fixed(u32),
    Dynamic(Temp),
}

/// The mask of an operand of `size` bytes.
const fn mask_of(size: u8) -> u64 {
    match size {
        1 => 0xff,
        2 => 0xffff,
        4 => 0xffff_ffff,
        _ => u64::MAX,
    }
}

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

/// Decide what an encoding means, or reject it.
///
/// Total, and every rejection is a *decision*: the caller ends the block and
/// the interpreter executes the instruction, which is what keeps the
/// interpreter the oracle rather than a fallback nobody exercises. Nothing here
/// emits, which is what makes a rejected instruction leave no debris.
#[allow(clippy::too_many_lines)]
fn classify(world: &World, f: &Fields, next_eip: u64) -> Option<Plan> {
    let insn = f.insn;
    // A prefix that changes what an instruction *means* is out of the subset:
    // `LOCK` needs an atomic the IR cannot type at eight or sixteen bits, and a
    // repeat prefix is a loop — and in front of `DIV` on an 8088 it is an
    // undocumented sign inversion the hardware corpus exercises deliberately.
    if f.lock || f.rep.is_some() || f.has_rex() {
        return None;
    }
    // 16-bit addressing inside a 32-bit segment is a second effective-address
    // form; 64-bit operands do not exist below long mode.
    if f.addrsize != 4 || f.opsize == 8 {
        return None;
    }
    let size = Lifter::width(f);
    if !matches!(size, 1 | 2 | 4) {
        return None;
    }

    let plan = match insn.op {
        Op::ADD | Op::ADC | Op::SUB | Op::SBB | Op::CMP | Op::AND | Op::OR | Op::XOR => {
            Plan::Alu(insn.op)
        }
        Op::TEST => Plan::Alu(Op::TEST),
        Op::INC => Plan::IncDec(true),
        Op::DEC => Plan::IncDec(false),
        Op::NOT => Plan::Not,
        Op::NEG => Plan::Neg,
        Op::MOV => {
            // `MOV Sreg,r/m` and `MOV r/m,Sreg` load or store a descriptor,
            // which `ir`'s decision 4 makes a mode change and therefore a hard
            // barrier; the control-, debug- and test-register moves are
            // privileged and reach state with no slot here.
            if matches!(insn.dst, Arg::Sw | Arg::Cd | Arg::Dd | Arg::Td | Arg::Rd)
                || matches!(insn.src, Arg::Sw | Arg::Cd | Arg::Dd | Arg::Td | Arg::Rd)
            {
                return None;
            }
            Plan::Mov
        }
        Op::MOVZX | Op::MOVSX => Plan::MovX {
            signed: insn.op == Op::MOVSX,
            src_size: if matches!(insn.src, Arg::Eb) { 1 } else { 2 },
        },
        Op::LEA => Plan::Lea,
        Op::ROL | Op::ROR | Op::RCL | Op::RCR | Op::SHL | Op::SHR | Op::SAR | Op::SETMO => {
            let count = match insn.src {
                Arg::One => Count::Fixed(1),
                Arg::Cl => {
                    // A `CL` count of zero writes nothing at all, memory
                    // included, and a conditional store is not expressible in a
                    // block whose branches all go forward over an exit. Register
                    // destinations only.
                    if !f.rm_is_register() {
                        return None;
                    }
                    Count::Cl
                }
                _ => {
                    let n = (f.imm & 0x1f) as u32;
                    if n == 0 {
                        // A 386 with a zero count does nothing at all — no
                        // flags, and no write-back either, so not even the
                        // operand is read.
                        return Some(Plan::Nop);
                    }
                    Count::Fixed(n)
                }
            };
            // An N-bit rotate through the carry is a loop, and the IR has no op
            // for one at an arbitrary width. See the module docs.
            if matches!(insn.op, Op::RCL | Op::RCR) && !matches!(count, Count::Fixed(1)) {
                return None;
            }
            Plan::Shift { op: insn.op, count }
        }
        Op::MUL => Plan::Mul { signed: false },
        Op::IMUL => {
            // The two- and three-operand forms write one register and leave the
            // accumulator alone; they are a different instruction that happens
            // to share a mnemonic, and `Exec::multiply` tells them apart the
            // same way.
            if matches!(insn.dst, Arg::Gv) {
                Plan::ImulShort
            } else {
                Plan::Mul { signed: true }
            }
        }
        Op::BSF => Plan::BitScan { reverse: false },
        Op::BSR => Plan::BitScan { reverse: true },
        Op::PUSH | Op::POP => {
            if matches!(insn.dst, Arg::Sr | Arg::Sw) {
                return None;
            }
            if insn.op == Op::PUSH {
                Plan::Push
            } else {
                Plan::Pop
            }
        }
        Op::LEAVE => Plan::Leave,
        Op::CLC => Plan::Carry(Some(false)),
        Op::STC => Plan::Carry(Some(true)),
        Op::CMC => Plan::Carry(None),
        Op::CLD => Plan::Direction(false),
        Op::STD => Plan::Direction(true),
        Op::LAHF => Plan::Lahf,
        Op::SAHF => Plan::Sahf,
        Op::CBW => Plan::Cbw,
        Op::CWD => Plan::Cwd,
        Op::BSWAP => Plan::Bswap,
        // `90` is a no-operation only without `REX.B`, which cannot be here.
        Op::NOP => Plan::Nop,
        Op::CALL => {
            // A near transfer masks its target to the operand size, so a `66`
            // prefix would truncate `EIP` to sixteen bits — a real instruction,
            // and one no 32-bit guest means.
            if f.opsize != 4 {
                return None;
            }
            match insn.dst {
                Arg::Jv | Arg::Jb => Plan::CallRel {
                    target: relative(f, next_eip),
                },
                _ => Plan::CallInd,
            }
        }
        Op::JMP => {
            if f.opsize != 4 {
                return None;
            }
            match insn.dst {
                Arg::Jv | Arg::Jb => Plan::JmpRel {
                    target: relative(f, next_eip),
                },
                _ => Plan::JmpInd,
            }
        }
        Op::RET => {
            if f.opsize != 4 {
                return None;
            }
            let extra = if matches!(insn.dst, Arg::Iw | Arg::Iv | Arg::Iz) {
                f.imm & 0xffff
            } else {
                0
            };
            Plan::Ret { extra }
        }
        op if op.is_conditional_jump() => {
            if f.opsize != 4 {
                return None;
            }
            Plan::Jcc {
                cc: op.condition_code().unwrap_or(0),
                target: relative(f, next_eip),
            }
        }
        op if op.is_setcc() => Plan::SetCc(op.condition_code().unwrap_or(0)),
        op if op.is_cmov() => {
            // `Exec` raises `#UD` for a `CMOV` on a part without the feature, so
            // whether it is in the subset is a property of the core.
            if !world.cmov {
                return None;
            }
            Plan::CmovCc(op.condition_code().unwrap_or(0))
        }
        _ => return None,
    };
    Some(plan)
}

/// The target of a relative jump: the address of the *next* instruction plus
/// the displacement, wrapped at the operand size.
///
/// The displacement has already been sign-extended to the operand size by the
/// decoder, so this is one addition — and it wraps in the pointer's own width,
/// which is the rule CLAUDE.md's "Arithmetic" section names.
fn relative(f: &Fields, next: u64) -> u64 {
    next.wrapping_add(f.imm) & mask_of(f.opsize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::x86::prot::{SegReg, ar, cr0};
    use crate::ir::{Liveness, bitfield_parts, verify};
    use alloc::vec;

    /// Where the test programs live: page-aligned, and far enough from zero
    /// that a small negative displacement does not wrap into nothing.
    const AT: u64 = 0x0010_0000;

    fn flat_world() -> World {
        World {
            variant: Variant::I80386,
            cs_base: 0,
            seg_base: [0; seg::COUNT],
            cmov: true,
            generation: 0,
            origin: Origin::Flat,
        }
    }

    /// The same world with `CR0.PG` set, on the one part whose translation
    /// buffers are split.
    fn paged_world(phys: u64) -> World {
        World {
            variant: Variant::X86_64,
            origin: Origin::Paged { phys },
            ..flat_world()
        }
    }

    /// A program in memory, as the lifter reads it. Nothing outside it is
    /// readable, which is what ends a block cleanly at the end of a test case.
    struct Bytes<'a>(&'a [u8], u64);

    impl InsnSource for Bytes<'_> {
        fn byte(&mut self, addr: u64) -> Option<u8> {
            let off = addr.checked_sub(self.1)?;
            self.0.get(usize::try_from(off).ok()?).copied()
        }
    }

    /// Lift a program, asserting that the verifier accepts what came out.
    ///
    /// Every test goes through here, which is how "the verifier accepts every
    /// block this frontend produces" is asserted everywhere rather than once.
    fn lift_at(world: &World, at: u64, bytes: &[u8], shape: Shape, smc: Smc, fl: Flags) -> Lifted {
        // The source reads **linear** addresses, which is where `CS.base` goes.
        let mut src = Bytes(bytes, world.linear(at));
        let lifted = lift(world, at, &mut src, MAX_INSNS, shape, smc, fl).expect("lifts");
        verify(&lifted.block).unwrap_or_else(|e| panic!("{e}\n{}", lifted.block));
        lifted
    }

    fn plain(bytes: &[u8]) -> Lifted {
        lift_at(
            &flat_world(),
            AT,
            bytes,
            Shape::default(),
            Smc::default(),
            Flags::default(),
        )
    }

    fn count(block: &Block, op: Opcode) -> usize {
        block.insts().iter().filter(|i| i.op == op).count()
    }

    fn ops(block: &Block) -> Vec<&'static str> {
        block.insts().iter().map(|i| i.op.name()).collect()
    }

    // -- the world -------------------------------------------------------

    /// A `Sys` in the world this frontend lifts: protected, flat `CS`, no
    /// paging, 32-bit everything.
    fn liftable_sys() -> Sys {
        let mut sys = Sys::reset();
        sys.cr0 |= cr0::PE;
        sys.segs[usize::from(seg::CS)] = SegReg {
            selector: 8,
            base: 0,
            limit: 0xffff_ffff,
            ar: ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::DB,
        };
        for index in [seg::DS, seg::ES, seg::SS, seg::FS, seg::GS] {
            sys.segs[usize::from(index)] = SegReg {
                selector: 0x10,
                base: 0x4000,
                limit: 0xffff,
                ar: ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB,
            };
        }
        sys
    }

    #[test]
    fn a_liftable_world_is_recognised_and_carries_the_segment_bases() {
        let sys = liftable_sys();
        let regs = Regs::new();
        let cfg = Config::I8088.with_variant(Variant::I80386);
        let world = World::of(&regs, &sys, &cfg, true, 7, Origin::Flat)
            .expect("this is a world the frontend lifts");
        assert_eq!(world.cs_base, 0);
        assert_eq!(world.seg_base[usize::from(seg::DS)], 0x4000);
        // `CMOVcc` is a property of the instance rather than of the part
        // number, and a 386's preset does not have it.
        assert!(!world.cmov);
        assert_eq!(world.generation, 7);
    }

    #[test]
    fn every_world_this_frontend_cannot_lift_is_refused() {
        let regs = Regs::new();
        let cfg = Config::I8088.with_variant(Variant::I80386);
        // Each of these is a *correctness* condition rather than a missing
        // feature, so each is checked one at a time from a world that is
        // otherwise fine.
        type Break = fn(&mut Sys);
        let cases: [(&str, Break); 4] = [
            ("real mode", |s| s.cr0 &= !cr0::PE),
            // Paging while the caller claims `Origin::Flat`: the origin has
            // to say what the control registers say, or a block gets keyed as
            // though a linear address were a physical one.
            ("paging claimed as flat", |s| s.cr0 |= cr0::PG),
            ("a 16-bit code segment", |s| {
                s.segs[usize::from(seg::CS)].ar &= !ar::DB;
            }),
            ("a limited code segment", |s| {
                s.segs[usize::from(seg::CS)].limit = 0xffff;
            }),
        ];
        for (what, break_it) in cases {
            let mut sys = liftable_sys();
            break_it(&mut sys);
            assert!(
                World::of(&regs, &sys, &cfg, true, 0, Origin::Flat).is_none(),
                "{what} must not be liftable"
            );
        }
        // A 16-bit stack would make every push a deposit and every pop a
        // partial write.
        let mut sys = liftable_sys();
        sys.segs[usize::from(seg::SS)].ar &= !ar::DB;
        assert!(World::of(&regs, &sys, &cfg, true, 0, Origin::Flat).is_none());
        // And a part with 16-bit registers has a different address path
        // entirely.
        let old = Config::I8088;
        assert!(World::of(&regs, &liftable_sys(), &old, true, 0, Origin::Flat).is_none());
        // And a machine with the A20 gate shut aliases two linear pages onto
        // one physical one, which the self-modifying-code guard compares
        // linearly and would therefore miss.
        assert!(World::of(&regs, &liftable_sys(), &cfg, false, 0, Origin::Flat).is_none());
    }

    /// Paging is in the subset on a part whose instruction and data
    /// translations are separate arrays, and out of it on one where they share
    /// an array — because there a data operand evicts the code page's own
    /// translation and the *next instruction fetch* pays for a walk no static
    /// analysis of the block can place.
    ///
    /// The condition is asked of the buffer arrangement rather than of the
    /// part number, which is the fact it is actually about.
    #[test]
    fn paging_is_liftable_exactly_where_the_translation_buffers_are_split() {
        let regs = Regs::new();
        let mut sys = liftable_sys();
        sys.cr0 |= cr0::PG;
        let origin = Origin::Paged { phys: 0x0020_0000 };
        for variant in [Variant::I80386, Variant::I80486] {
            let cfg = Config::I8088.with_variant(variant);
            assert_eq!(cfg.variant.buffers(), Buffers::Unified);
            assert!(
                World::of(&regs, &sys, &cfg, true, 0, origin).is_none(),
                "{variant:?} has one buffer, so a paged block cannot charge its fetches"
            );
        }
        let cfg = Config::I8088.with_variant(Variant::X86_64);
        assert_eq!(cfg.variant.buffers(), Buffers::Split);
        let world = World::of(&regs, &sys, &cfg, true, 0, origin)
            .expect("a split-buffer part pages inside the subset");
        assert_eq!(world.origin, origin);
        // And the claim has to match the control registers in both
        // directions: an unpaged core described as paged is refused too.
        let flat = liftable_sys();
        assert!(World::of(&regs, &flat, &cfg, true, 0, origin).is_none());
    }

    /// The in-block store guard compares **linear** pages, and under paging
    /// two of them may name one physical page — so a store through the other
    /// mapping walks past a guard that cannot see it. [`lift`] refuses the
    /// combination rather than emitting a check with a hole in it.
    #[test]
    fn the_in_block_store_guard_is_refused_under_paging() {
        let world = paged_world(0x0020_0000);
        let mut src = Bytes(&[0x90], world.linear(AT));
        assert!(
            lift(
                &world,
                AT,
                &mut src,
                MAX_INSNS,
                Shape::Trace,
                Smc::Guard,
                Flags::Elide
            )
            .is_err()
        );
        // `Smc::EndBlock` is the policy that holds there, and it lifts.
        let mut src = Bytes(&[0x90], world.linear(AT));
        assert!(
            lift(
                &world,
                AT,
                &mut src,
                MAX_INSNS,
                Shape::Trace,
                Smc::EndBlock,
                Flags::Elide
            )
            .is_ok()
        );
    }

    /// A paged block reports the **physical** page its bytes came from, and
    /// the linear page it was bounded by, and they are different numbers.
    ///
    /// Both ends of the invalidation have to be physical: `jit::cache` matches
    /// a guest store against `Translation::page`, and a host notes the
    /// physical address its store reached.
    #[test]
    fn a_paged_block_is_named_by_a_physical_page_and_bounded_by_a_linear_one() {
        let phys = 0x0020_0000;
        let world = paged_world(phys);
        let mut src = Bytes(&[0x90, 0x90, 0xf4], world.linear(AT));
        let lifted = lift(
            &world,
            AT,
            &mut src,
            MAX_INSNS,
            Shape::Trace,
            Smc::EndBlock,
            Flags::Elide,
        )
        .expect("a paged world lifts");
        verify(&lifted.block).expect("and verifies");
        assert_eq!(lifted.page, phys);
        assert_eq!(lifted.linear_page, AT);
        assert_ne!(lifted.page, lifted.linear_page);
        // With paging off the two are the same number, which is exactly why
        // the distinction was invisible before.
        let flat = plain(&[0x90, 0x90, 0xf4]);
        assert_eq!(flat.page, flat.linear_page);
    }

    /// Two blocks at the same `EIP` under two different mappings are two
    /// different keys, and a paged key is never a flat one.
    ///
    /// This is the whole of what stops a `CR3` reload serving a translation of
    /// bytes that are no longer there. A translation *generation* would say
    /// the same thing far more often than it needs to — `cpu::riscv::engine`
    /// measured that being unusable on Linux — so the key carries the physical
    /// address the entry resolved to instead.
    ///
    /// The **address**, not the page, and that is not tidiness: with a
    /// page-granular key a `CS` base moved by a page maps the same `EIP` to a
    /// different offset in the same frame, which is different bytes and a
    /// different distance to the end of the page, under one key.
    #[test]
    fn a_paged_key_names_the_entry_address_and_never_collides_with_a_flat_one() {
        let a = paged_world(0x0020_0000);
        let b = paged_world(0x0030_0000);
        let flat = flat_world();
        let of = |w: &World| key(w, Shape::Trace, Smc::EndBlock, Flags::Elide);
        assert_ne!(of(&a), of(&b), "a different mapping is a different block");
        assert_ne!(of(&a), of(&flat));
        // The same frame at a different offset is a different block too.
        assert_ne!(of(&a), of(&paged_world(0x0020_0040)));
        // A world generation moves a flat key and is deliberately *not* in a
        // paged one: under `Smc::EndBlock` no segment base reaches the emitted
        // IR, and which bytes the entry names is what the physical address
        // says.
        let mut moved = flat;
        moved.generation = 1;
        assert_ne!(of(&flat), of(&moved));
        let mut same_entry = a;
        same_entry.generation = 99;
        assert_eq!(of(&a), of(&same_entry));
    }

    #[test]
    fn the_cache_key_separates_every_policy_and_every_world() {
        let w = flat_world();
        let mut keys = alloc::collections::BTreeSet::new();
        for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
            for smc in [Smc::EndBlock, Smc::Guard] {
                for fl in [Flags::Eager, Flags::Elide] {
                    assert!(
                        keys.insert(key(&w, shape, smc, fl)),
                        "{shape:?}/{smc:?}/{fl:?}"
                    );
                }
            }
        }
        // A world change is a different key, which is what stops a cache
        // handing back a block lifted through a mapping that no longer exists.
        let mut moved = w;
        moved.generation = 1;
        assert!(keys.insert(key(&moved, Shape::Trace, Smc::Guard, Flags::Elide)));
        // and `CMOVcc` decides whether an encoding is in the subset at all.
        // And a part is its own world: three of them reach here and each has
        // its own number, so a block lifted for one is never served to
        // another. A 386 and an x86-64 shared a number until paging landed.
        let mut parts = alloc::collections::BTreeSet::new();
        for variant in [Variant::I80386, Variant::I80486, Variant::X86_64] {
            let mut part = w;
            part.variant = variant;
            assert!(
                parts.insert(key(&part, Shape::Trace, Smc::Guard, Flags::Elide)),
                "{variant:?}"
            );
        }
        let mut no_cmov = w.without_cmov();
        no_cmov.generation = 0;
        assert_ne!(
            key(&w, Shape::Trace, Smc::Guard, Flags::Elide),
            key(&no_cmov, Shape::Trace, Smc::Guard, Flags::Elide)
        );
    }

    // -- the subset ------------------------------------------------------

    #[test]
    fn a_register_add_lifts_to_the_arithmetic_and_its_six_flags() {
        // add eax, ecx
        let l = plain(&[0x01, 0xc8]);
        assert_eq!(l.insns, 1);
        // The block covers the add and stops where the bytes run out.
        assert_eq!(l.stop, Stop::Unreadable);
        let names = ops(&l.block);
        assert!(names.contains(&"insn_start"));
        assert!(names.contains(&"charge"));
        assert!(names.contains(&"add"));
        // The parity flag is a popcount, on nearly every ALU instruction —
        // decision 1's whole cost, and the thing elision and dead-code
        // elimination exist to pay for.
        assert_eq!(count(&l.block, Opcode::POPCOUNT), 1, "{}", l.block);
        assert_eq!(l.page, AT);
    }

    #[test]
    fn a_sub_register_operand_is_a_bitfield_and_ah_is_register_four() {
        // mov ah, al — register four written from register zero, both inside
        // `EAX`. A lifter that treated `AH` as register one would produce the
        // same op count and the wrong answer, which is why this asserts the
        // positions rather than the shape.
        let l = plain(&[0x88, 0xc4]);
        let extract = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::EXTRACT)
            .expect("a byte read is an extract");
        assert_eq!(bitfield_parts(extract.aux), (0, 8), "AL is bits 0..8");
        let deposit = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::DEPOSIT)
            .expect("a byte write is a deposit");
        assert_eq!(bitfield_parts(deposit.aux), (8, 8), "AH is bits 8..16");
    }

    #[test]
    fn an_encoding_outside_the_subset_ends_the_block_without_debris() {
        // inc eax ; div ecx ; inc eax — the divide is out of the subset, so the
        // block covers one instruction and its exit `EIP` is the divide's.
        let l = plain(&[0x40, 0xf7, 0xf1, 0x40]);
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Unsupported);
        let exit = l.block.marks().last().expect("a block has boundaries");
        assert_eq!(exit.pc, AT + 1, "the interpreter takes over at the divide");
    }

    #[test]
    fn a_lock_prefix_and_a_repeat_prefix_are_both_refused() {
        // `lock inc [eax]` needs an atomic the IR cannot type at eight bits,
        // and `rep` is a loop.
        assert_eq!(plain(&[0xf0, 0xff, 0x00]).insns, 0);
        assert_eq!(plain(&[0xf3, 0x01, 0xc8]).insns, 0);
    }

    // -- ticks and the page bound ----------------------------------------

    #[test]
    fn the_static_charge_is_the_fetch_plus_the_operations_own_clocks() {
        // add eax, ecx: two bytes fetched at two clocks each, plus the ALU's
        // own three. Exactly what `Exec::instruction` charges before it
        // executes anything.
        let l = plain(&[0x01, 0xc8]);
        let charge = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::CHARGE)
            .and_then(|i| i.imm)
            .expect("a charge");
        assert_eq!(charge.bits(), u128::from(2 * 2 + Op::ADD.clocks()));
    }

    #[test]
    fn the_tick_column_is_monotonic_and_sums_the_charges() {
        let l = plain(&[0x40, 0x41, 0x42, 0x43]);
        assert_eq!(l.insns, 4);
        let mut running = 0u64;
        for (n, mark) in l.block.marks().iter().enumerate() {
            assert_eq!(mark.ticks, running, "boundary {n}");
            running += 2 + u64::from(Op::INC.clocks());
        }
    }

    #[test]
    fn a_block_never_leaves_the_page_it_started_on() {
        // Twelve `inc eax` starting eight bytes below a page boundary: the
        // block covers the eight inside the page and stops.
        let at = AT + PAGE_SIZE - 8;
        let l = lift_at(
            &flat_world(),
            at,
            &[0x40; 12],
            Shape::Trace,
            Smc::Guard,
            Flags::Elide,
        );
        assert_eq!(l.insns, 8);
        assert_eq!(l.stop, Stop::Page);
        assert_eq!(l.page, AT);
    }

    #[test]
    fn a_page_straddling_instruction_is_left_to_the_interpreter() {
        // `mov eax, imm32` is five bytes, started three below the page end, so
        // its last two bytes are on the next page — where the fetch could fault
        // *mid-instruction*. The block ends before it rather than lifting a
        // fetch it would have to be able to fault on.
        let at = AT + PAGE_SIZE - 3;
        let l = lift_at(
            &flat_world(),
            at,
            &[0xb8, 0x11, 0x22, 0x33, 0x44],
            Shape::Trace,
            Smc::Guard,
            Flags::Elide,
        );
        assert_eq!(l.insns, 0, "nothing may be lifted:\n{}", l.block);
        assert_eq!(l.stop, Stop::Page);
        // The block is still well formed, and its exit `EIP` is the straddling
        // instruction's own address — which is what a dispatcher hands back.
        let exit = l.block.marks().last().expect("a boundary");
        assert_eq!(exit.pc, at);
    }

    #[test]
    fn a_segment_base_moves_the_page_bound_without_moving_the_program_counter() {
        // The block is bounded by the *linear* page, not the `EIP` page: `CS`
        // need not be page-aligned, and it is the linear page a guest store is
        // matched against.
        let mut world = flat_world();
        world.cs_base = 0x800;
        let l = lift_at(
            &world,
            AT,
            &[0x40; 8],
            Shape::Trace,
            Smc::Guard,
            Flags::Elide,
        );
        assert_eq!(l.page, (AT + 0x800) & !PAGE_MASK);
        assert_eq!(l.page, AT);
        // Eight bytes from `AT + 0x800` is still inside the page.
        assert_eq!(l.insns, 8);
    }

    // -- flags -----------------------------------------------------------

    /// The number of popcounts a program lifts to under each flag policy.
    ///
    /// The measurement decision 1 promised: `PF` is computed on nearly every
    /// ALU instruction and read almost never, and this is what removing it is
    /// worth.
    fn parities(bytes: &[u8], policy: Flags) -> usize {
        let l = lift_at(&flat_world(), AT, bytes, Shape::Trace, Smc::Guard, policy);
        count(&l.block, Opcode::POPCOUNT)
    }

    #[test]
    fn eliding_a_dead_flag_removes_the_arithmetic_behind_it() {
        // Eight adds in a row: each one's flags are overwritten by the next,
        // none of them can fault, and only the last one's survive to the exit.
        let program = [0x01, 0xc8].repeat(8);
        assert_eq!(parities(&program, Flags::Eager), 8);
        assert_eq!(
            parities(&program, Flags::Elide),
            1,
            "only the flags that reach the exit are live"
        );
    }

    #[test]
    fn a_flag_an_instruction_can_fault_at_is_never_elided() {
        // The same eight adds, with a memory operand on the last but one. An
        // instruction with a memory operand can fault *before* it writes any
        // flag, so the boundary must leave the previous instruction's flags
        // recoverable — and the arithmetic behind them therefore stays.
        let mut program = [0x01, 0xc8].repeat(6);
        program.extend_from_slice(&[0x03, 0x03]); // add eax, [ebx]
        program.extend_from_slice(&[0x01, 0xc8]);
        assert_eq!(parities(&program, Flags::Eager), 8);
        // Six adds ahead of the faulting one, and the one before it keeps its
        // flags; the last add's reach the exit.
        // The add before the faulting one keeps its flags, and the last add's
        // reach the exit. Everything between is dead.
        assert_eq!(parities(&program, Flags::Elide), 2, "{program:02x?}");
    }

    #[test]
    fn inc_elides_five_flags_and_never_the_carry() {
        // `INC` preserves `CF`, so it is not a flag the boundary before it may
        // drop — and a lifter that dropped all six would revert the carry to
        // whatever the host last held.
        let l = lift_at(
            &flat_world(),
            AT,
            &[0x01, 0xc8, 0x40],
            Shape::Trace,
            Smc::Guard,
            Flags::Elide,
        );
        let second = &l.block.marks()[1];
        let named: Vec<u16> = second.live.iter().map(|(s, _)| s.0).collect();
        assert!(
            named.contains(&CF.0),
            "the carry survives an inc: {named:?}"
        );
        assert!(!named.contains(&ZF.0), "the zero flag does not: {named:?}");
    }

    #[test]
    fn a_shift_by_cl_elides_nothing_because_it_may_write_nothing() {
        // The one instruction in the subset whose whole effect is conditional.
        // A count of zero leaves every flag alone, so none of them is a flag
        // the boundary may drop — the write is not unconditional, which is
        // half of what the elision rule asks.
        //
        // Nothing *observable* depends on this today, because a `CL` shift with
        // a memory destination is not lifted and so the shift cannot fault; the
        // rule is asserted here rather than differentially for exactly that
        // reason, and it becomes load-bearing the moment that form is lifted.
        let l = lift_at(
            &flat_world(),
            AT,
            &[0x01, 0xc8, 0xd3, 0xe0],
            Shape::Trace,
            Smc::Guard,
            Flags::Elide,
        );
        let second = &l.block.marks()[1];
        let named: Vec<u16> = second.live.iter().map(|(s, _)| s.0).collect();
        for slot in FLAG_SLOTS {
            assert!(
                named.contains(&slot.0),
                "slot {} must survive into the shift: {named:?}",
                slot.0
            );
        }
    }

    #[test]
    fn every_temporary_a_boundary_names_is_assigned_and_live() {
        // The invariant `ir::pass` seeds its liveness from, and the one a
        // frontend can break without any test that does not fault noticing.
        let l = plain(&[0x01, 0xc8, 0x29, 0xd9, 0x83, 0xc0, 0x07, 0x40]);
        let live = Liveness::compute(&l.block);
        for (n, mark) in l.block.marks().iter().enumerate() {
            for (slot, temp) in &mark.live {
                assert!(
                    live.is_live(*temp),
                    "boundary {n} names {temp} for slot {} and nothing keeps it",
                    slot.0
                );
                assert!(
                    l.block.type_of(*temp).is_some(),
                    "boundary {n} names an unallocated {temp}"
                );
            }
        }
    }

    #[test]
    fn a_slot_a_boundary_shadows_stays_shadowed_at_every_later_boundary() {
        // The register half of `InsnStart::live`'s invariant, which the flag
        // half deliberately spends (see the module docs) and the register half
        // does not: a register the block has bound must be named everywhere
        // after it, or a fault reverts it to whatever the host last held.
        let l = plain(&[0x01, 0xc8, 0x29, 0xd9, 0x8b, 0x1a, 0x40]);
        let mut seen: Vec<u16> = Vec::new();
        for (n, mark) in l.block.marks().iter().enumerate() {
            let named: Vec<u16> = mark.live.iter().map(|(s, _)| s.0).collect();
            for slot in &seen {
                if *slot < 8 {
                    assert!(
                        named.contains(slot),
                        "boundary {n} dropped register slot {slot}"
                    );
                }
            }
            seen = named;
        }
    }

    // -- self-modifying code ---------------------------------------------

    #[test]
    fn a_store_ends_the_block_under_one_policy_and_is_guarded_under_the_other() {
        let bytes = [0x89, 0x03, 0x40]; // mov [ebx], eax ; inc eax
        let ends = lift_at(
            &flat_world(),
            AT,
            &bytes,
            Shape::Trace,
            Smc::EndBlock,
            Flags::Elide,
        );
        assert_eq!(ends.insns, 1);
        assert_eq!(ends.stop, Stop::Access);
        assert_eq!(count(&ends.block, Opcode::BRCOND), 0);

        let guarded = lift_at(
            &flat_world(),
            AT,
            &bytes,
            Shape::Trace,
            Smc::Guard,
            Flags::Elide,
        );
        assert_eq!(guarded.insns, 2, "the block goes on past the store");
        // One forward branch over one inline exit: the whole cost of making
        // x86's coherent instruction cache architectural.
        assert_eq!(count(&guarded.block, Opcode::BRCOND), 1);
        assert_eq!(count(&guarded.block, Opcode::EXIT_TB), 2);
    }

    #[test]
    fn the_guard_compares_the_stores_linear_page_against_the_blocks_own() {
        // A data segment with a base of its own: the store's address is a
        // segment offset and the block's page is linear, so the comparison is
        // only right if the base is folded in. That is what puts the segment
        // bases in `World` and the world's generation in the key.
        let mut world = flat_world();
        world.seg_base[usize::from(seg::DS)] = 0x2_0000;
        let l = lift_at(
            &world,
            AT,
            &[0x89, 0x03, 0x40],
            Shape::Trace,
            Smc::Guard,
            Flags::Elide,
        );
        let constants: Vec<u128> = l
            .block
            .insts()
            .iter()
            .filter(|i| i.op == Opcode::MOV)
            .filter_map(|i| i.imm)
            .map(|c| c.bits())
            .collect();
        assert!(
            constants.contains(&u128::from(0x2_0000u64)),
            "the data segment's base is folded in: {constants:x?}"
        );
        assert!(
            constants.contains(&u128::from(AT)),
            "and compared against this block's own page: {constants:x?}"
        );
    }

    // -- control flow ------------------------------------------------------

    #[test]
    fn a_backward_branch_inlines_the_taken_side_and_a_forward_one_the_fall_through() {
        // jnz -3 after a dec: the back edge of a loop, so the taken side is the
        // trace and the fall-through is the side exit.
        let back = plain(&[0x49, 0x75, 0xfd]);
        assert!(
            back.insns > 2,
            "the loop unrolled: {} guest instructions",
            back.insns
        );
        assert!(count(&back.block, Opcode::BRCOND) > 1);

        // jz +2 over two bytes: an `if`, so the fall-through is the trace.
        let forward = plain(&[0x74, 0x02, 0x40, 0x41, 0x42]);
        assert_eq!(forward.insns, 4);
        assert_eq!(count(&forward.block, Opcode::BRCOND), 1);
    }

    #[test]
    fn a_branch_becomes_a_select_where_the_shape_does_not_merge() {
        let l = lift_at(
            &flat_world(),
            AT,
            &[0x74, 0x02, 0x40, 0x41],
            Shape::BasicBlock,
            Smc::Guard,
            Flags::Elide,
        );
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Transfer);
        assert_eq!(count(&l.block, Opcode::BRCOND), 0);
        assert_eq!(count(&l.block, Opcode::MOVCOND), 1);
    }

    #[test]
    fn a_computed_transfer_always_ends_the_block() {
        for bytes in [
            vec![0xff, 0xe0, 0x40], // jmp eax
            vec![0xc3, 0x40],       // ret
            vec![0xff, 0xd0, 0x40], // call eax
        ] {
            let l = plain(&bytes);
            assert_eq!(l.insns, 1, "{bytes:02x?}");
            assert_eq!(l.stop, Stop::Transfer, "{bytes:02x?}");
        }
    }

    #[test]
    fn dead_code_elimination_has_already_run_and_the_block_still_verifies() {
        // `lift` runs the pass itself, because decision 1's cost is incurred
        // here and settling it anywhere else would leave every consumer to
        // remember. Running it again must therefore change nothing.
        let l = plain(&[0x01, 0xc8, 0x29, 0xd9, 0x83, 0xc0, 0x07]);
        let again = crate::ir::eliminate_dead_code(&l.block);
        verify(&again).expect("still well formed");
        assert_eq!(again.insts(), l.block.insts());
    }

    #[test]
    fn a_zero_count_shift_is_lifted_as_doing_nothing_at_all() {
        // A 386 with a zero count does nothing: no flags, no write-back, and
        // not even a read of the operand — so a memory form makes no bus cycle
        // either.
        let l = plain(&[0xc1, 0x20, 0x00]); // shl [eax], 0
        assert_eq!(l.insns, 1);
        assert_eq!(count(&l.block, Opcode::LD), 0, "{}", l.block);
        assert_eq!(count(&l.block, Opcode::ST), 0, "{}", l.block);
        // The fetch and the operation still cost what they cost.
        assert_eq!(count(&l.block, Opcode::CHARGE), 1);
    }

    #[test]
    fn a_rotate_through_carry_by_more_than_one_is_out_of_the_subset() {
        // An N-bit rotate through the carry is a loop, and `Opcode::ROTLC`
        // rotates at the temporary's type width rather than at the operand's —
        // so the IR cannot say it at eight or sixteen bits at all. By one it is
        // a shift, an or and a bit test.
        assert_eq!(plain(&[0xd1, 0xd0]).insns, 1, "rcl eax, 1 is lifted");
        assert_eq!(plain(&[0xc1, 0xd0, 0x03]).insns, 0, "rcl eax, 3 is not");
        assert_eq!(plain(&[0xd3, 0xd0]).insns, 0, "rcl eax, cl is not");
    }

    #[test]
    fn a_shift_by_cl_reaching_memory_is_out_of_the_subset() {
        // A count of zero writes nothing at all, memory included, and a
        // conditional store is not expressible in a block whose branches all
        // jump forward over an exit.
        assert_eq!(plain(&[0xd3, 0x20]).insns, 0, "shl [eax], cl");
        assert_eq!(plain(&[0xd3, 0xe0]).insns, 1, "shl eax, cl is fine");
    }
}
