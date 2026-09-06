# Memory consistency models

Consumed by: `ir/` (atomic ops and barrier lowering), `cpu/*` frontends, phase
9. This is where parallel emulation goes wrong, and the failures are
load-dependent, host-specific, and nearly impossible to debug after the fact —
which is why the rules are fixed in advance.

## The problem

When guest CPUs run on parallel host threads, guest memory ordering must be
preserved on a host whose ordering rules are different. Two cases:

- **Guest weaker than host** (e.g. RISC-V or ARM guest on x86 host): nothing to
  emit. The host is already stricter than the guest requires.
- **Guest stronger than host** (e.g. **x86-TSO guest on AArch64 or wasm**): the
  frontend lifter **must** insert barriers. Miss one and the guest sees
  reorderings its ISA promises cannot happen.

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

Two things are kept and two are not. Both of the ones that are not are reachable
**only** under `ThreadingMode::Parallel`, which is opt-in (`--threading
parallel`); no machine file selects it, `Deterministic` is the default, and
`usermode`'s `ThreadSet` runs every guest thread on one host thread by design.
Under one host thread the finest interleaving there is is one whole instruction,
so none of this can be observed — which is why the tree has got this far without
it mattering.

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
with the bus lock held throughout. It is also **engine-dependent**: a store the
JIT inlines is one host instruction of the guest's width and does not tear.
`core::space::store`'s "What per-byte atomicity is not" has the cost of the two
ways to remove it and why neither was taken.

**Not kept: barriers.** Every data barrier in the tree retires as a no-op —
`DMB`/`DSB`, `FENCE`, `MFENCE`/`LFENCE`/`SFENCE`. `core::sync` has the analysis;
the correction it makes to the table above is worth repeating here, because the
table as written is what would stop someone looking:

> **"Guest weaker than or equal to the host, nothing to emit" is wrong for a
> barrier instruction.** It is right for the guest's *baseline* ordering — an
> x86 guest's ordinary loads and stores need nothing on an x86 host. But a
> barrier is the guest asking for something stronger than its own baseline, and
> the host's baseline is not stronger than that. `MFENCE` exists to defeat
> store-then-load reordering, and an x86 host does exactly that reordering to
> the emulator's own accesses. Dropping the guest's `MFENCE` therefore hands the
> guest back the relaxation it just paid to remove.

`tests/memory_model_costs.rs` puts a number on the window: a store-buffer litmus
over the same relaxed `AtomicU8` primitive `RamStore` uses produces the outcome
`MFENCE` forbids tens to hundreds of times in 200 000 rounds when nothing
separates the store from the load, and never once about forty nanoseconds do.
That is the same order as the interpreter's cost per guest instruction and much
longer than the JIT's — so the exposure is small for one engine and real for the
other. On an AArch64 host, store-store and load-load go as well and every
barrier matters.

The fix is one host fence per guest barrier instruction and costs nothing on any
other path; `core::sync` re-exports `fence` and `compiler_fence` for exactly
that, and the IR already carries `Opcode::FENCE` with an `IrHost::fence` hook
whose default body is empty "on a host with one thread of guest execution". What
is missing is the three interpreter arms (`a64`, `riscv`, `x86`), that default,
and `a64::lift` emitting a `FENCE` where it currently emits `Plan::Nop`. All of
those sites are in `cpu/` and `ir/`.
