# Memory consistency models

Consumed by: `ir/` (atomic ops and barrier lowering), `cpu/*` frontends, phase
9. This is where parallel emulation goes wrong, and the failures are
load-dependent, host-specific, and nearly impossible to debug after the fact —
which is why the rules are fixed in advance.

## The problem

When guest CPUs run on parallel host threads, guest memory ordering must be
preserved on a host whose ordering rules are different. Three cases:

- **Guest weaker than host** (e.g. RISC-V or ARM guest on x86 host): nothing to
  emit *for the guest's ordinary loads and stores*. The host is already
  stricter than the guest's baseline requires.
- **Guest stronger than host** (e.g. **x86-TSO guest on AArch64 or wasm**): the
  frontend lifter **must** insert barriers. Miss one and the guest sees
  reorderings its ISA promises cannot happen.
- **A guest barrier instruction is neither case.** It is the guest asking for
  something stronger than *its own* baseline, so "weaker than or equal to the
  host" does not answer it: an `MFENCE` on an x86 host still has to be a host
  fence, because the host's baseline is not stronger than what `MFENCE` asks
  for. That row used to read "nothing to emit" and it cost the tree a year of
  no-op barriers; see below.

`ROADMAP.md` §4.7 assigns this responsibility explicitly to the frontend lifter:
the core provides atomic primitives, the lifter owns the ordering.

## Sources

| Topic | Source |
| --- | --- |
| Rigorous models for x86, ARM, POWER, RISC-V | [Peter Sewell's group, Cambridge — relaxed memory concurrency](https://www.cl.cam.ac.uk/~pes20/weakmemory/) — the `x86-TSO` paper and the ARM/POWER tutorials are the standard references, with formal models and litmus tests |
| x86 ordering rules | Intel SDM Volume 3, "Memory Ordering" **[browser]** |
| ARM ordering rules | Arm ARM (DDI 0487), the memory model chapter **[browser]** |
| RISC-V RVWMO | [riscv-isa-manual](https://github.com/riscv/riscv-isa-manual) Volume 1, Chapter 14 — **CC-BY-4.0** |
| WebAssembly threads | [WebAssembly threads proposal](https://github.com/WebAssembly/threads) — the wasm memory model, relevant to the threaded browser target |
| Rust's model | The `core::sync::atomic` documentation; Rust follows the C++20 model |

## Litmus testing

The Cambridge group publishes **litmus tests** — small multi-threaded programs
with a defined set of allowed outcomes. Running them as guest programs under
parallel translated execution is the only practical way to gain confidence that
barrier lowering is right, and it belongs in the SMP-emulation gate alongside the
atomics stress suite.

## Implementation notes

- Guest atomic instructions lower to host atomics through the IR's atomic ops
  (`cmpxchg`, `fetch_*`, `xchg`, fences).
- Load-linked/store-conditional guests (ARM, RISC-V, PowerPC) do not map
  directly onto compare-and-swap hosts; the standard approaches (address
  monitors, or CAS with an ABA-tolerant scheme) each have documented failure
  cases. Decide deliberately and write the reasoning down.
- Each frontend gets its own memory-model conformance suite (`ROADMAP.md` §12).

## Where this stands today, measured

Three things are kept and one is not. Everything below is reachable **only**
under `ThreadingMode::Parallel`, which is opt-in (`--threading parallel`); no
machine file selects it, `Deterministic` is the default, and `usermode`'s
`ThreadSet` runs every guest thread on one host thread by design. Under one host
thread the finest interleaving there is is one whole instruction, so none of it
can be observed — which is why the tree has got this far without it mattering.

**Kept.** The load-reserved pair, by `core::space::ExclusiveMonitor`: a store by
any observer to the reservation granule clears the slot, so a store-conditional
that raced anything fails and the guest retries. The x86 locked
read-modify-write, by `core::space::BusLock`, against *another* locked one.

**Not kept: single-copy atomicity.** `RamStore` is a `Vec<AtomicU8>` and every
access to it is a byte loop, so a naturally aligned four-byte load racing a
naturally aligned four-byte store can return a mixture of the old and the new
word — a value all three architectures forbid (*Intel SDM* volume 3 §9.1.1, ARM
DDI 0487 B2.2.1, RISC-V Unprivileged ISA §1.4).
`tests/smp_single_copy_atomicity.rs` catches it 117–361 times in sixty thousand
loads, and catches the read half of a `LOCK XADD` torn the same way 90–138 times
with the bus lock held throughout. That second form is x86's; the tearing itself
belongs to every architecture here, because an ordinary aligned load has no
backstop on any of them.

It follows the **store**, and it is narrower than "engine-dependent". An earlier
version of this paragraph said "a store the JIT inlines is one host instruction
of the guest's width and does not tear", which is true and misleads twice. A
*load* the JIT inlines still comes back torn if the racing store went through
the byte loop, because no reader can un-tear a store made in four pieces — so
the guarantee follows whichever core is storing. And `jit::x86` is the *host*
backend: inlining happens only where a core publishes `FastMem::store_plan`,
which AArch64 and RISC-V do and **x86 does not**. An x86 guest inlines no memory
access at all, so its stores tear in both engines, which is precisely the
configuration `tests/smp_single_copy_atomicity.rs` measures.

`core::space::store`'s "What per-byte atomicity is not" has three candidate
shapes, all re-derivable from `tests/memory_model_costs.rs`, and why none was
taken. The one correction worth repeating here is the price: "+12% of the store
path" divided by `SpaceView::write_span` (≈25 ns), which is a fifth of an
interpreted store instruction (≈117 ns) rather than a path a guest executes. The
byte loop is ~1% of that instruction and full conformance costs +2–3% of it.
Both fixes are affordable at that denominator; the question is the memory model,
not the nanoseconds.

**Kept, as of this round: barriers.** Every data barrier now executes one host
`fence(SeqCst)` — `DSB`/`DMB` in `cpu::arm::a64::exec`, `FENCE` in
`cpu::riscv::exec`, `LFENCE`/`MFENCE`/`SFENCE` in `cpu::x86::fpexec`, and
`IrHost::fence`'s default, which used to be an empty body under the comment
*"a no-op on a host with one thread of guest execution"*. What stays a no-op
stays one on an architectural reason rather than a convenient one: A64's `ISB`
and RISC-V's `FENCE.I` order their own PE's instruction *fetch* and no data
access another observer can see (Arm DDI 0487, `ISB`; RISC-V Zifencei), and
`cpu::arm::v7m`'s barriers are on a uniprocessor with no second observer to
order against.

`tests/memory_model_litmus.rs` is the reproducer, and what it found is not what
the argument above predicted.

- Over two bare relaxed `AtomicU8`s — the primitive the previous round measured
  — the store-buffer outcome appears tens to hundreds of times in 200 000
  rounds, and a `fence(SeqCst)` removes it. That much held.
- **Over `RamStore` it was already zero, before any of this.** Every write to
  the store ends in `mark_dirty`, which sets a bit with
  `AtomicU64::fetch_or` — a *relaxed* read-modify-write, which on x86-64 is a
  `lock or`, and a locked instruction is a full barrier (*Intel SDM* volume 3
  §9.2.5). So every guest store to RAM has been draining the host's store
  buffer all along, on the interpreter and inside a translated block alike:
  `jit::Tlb::note_fast_store` marks the same bitmap after an inlined store.
  `tests/memory_model_costs.rs` prices the two identically — 3.96 ns for a
  store plus a `SeqCst` fence, 3.97 ns for a store plus the `fetch_or`.
- Over guest instructions, two `cpu::x86` cores on two host threads: zero
  either way, for a second and independent reason — a whole interpreted
  instruction separates the guest's store from the guest's load, and the host's
  window closes at about forty nanoseconds.

None of that is a reason to leave the barrier a no-op. The accident is x86-only
(`fetch_or(Relaxed)` on AArch64 orders nothing), it covers only what follows a
store, it does nothing between two loads, and it would evaporate the day
somebody batched the dirty bitmap or wrote it with a plain `store`. What it does
mean is that the *exposure* today was smaller than the previous round's
measurement implied, and that the honest reason to fence is portability rather
than a live defect on this host.

**How often a real guest pays.** An arm64 Linux boot executes 479 000 `DSB`/`DMB`
in 876 million instructions — one in 1 800 — plus another 213 000 `ISB`, which
is why `ISB` is excluded rather than lumped in. A RISC-V Linux boot executes
`FENCE` about once in 100 000 instructions. An x86-64 Linux 6.6 guest executes
**`LFENCE` tens of thousands of times and `MFENCE` once**: `smp_mb()` on x86-64
is `lock addl $0,-4(%rsp)`, not `MFENCE`, so the barrier an x86 guest really
leans on is the `LOCK` prefix — which reaches `AddressSpace::bus_lock`, a mutex,
whose acquire/release does not forbid store-then-load. That is the next gap, and
it is `core::space`'s rather than `cpu/`'s.

**What the lifters do, and why one of them does not emit `Opcode::FENCE`.** The
RISC-V and x86 frontends do not lift a fence at all — the block ends at one and
the interpreter runs it. `a64::lift` could emit `Opcode::FENCE`, and does not,
because `jit::x86::compiles` does not lower it and `jit::dispatch` does not
*remember* a refusal — it re-attempts the compilation every time the block is
reached. Measured over the same 120 s of an arm64 Linux boot on `jit-host`:
`FENCE` emitted into the trace is **+4.9%**, `FENCE` alone in a block of its own
is **+2.8%**, and leaving the barrier outside the lifted subset — one dispatcher
round trip and one interpreted instruction per barrier, 0.05% of the stream — is
**+1.3%**, inside the run-to-run noise. The cost is the count of refusals rather
than their size. `a64::lift`'s own
`nothing_this_frontend_emits_is_an_op_the_host_backend_refuses` is the standing
invariant that says so. **Teaching `jit::x86` to emit an `mfence` for
`Opcode::FENCE` is the whole fix**, and the day it lands, `classify`'s
`Op::Dsb | Op::Dmb` arm becomes an emitted `FENCE` and nothing else changes.
