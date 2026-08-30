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
barrier lowering is right, and it belongs in the phase-9 gate alongside the
atomics stress suite.

## Implementation notes

- Guest atomic instructions lower to host atomics through the IR's atomic ops
  (`cmpxchg`, `fetch_*`, `xchg`, fences).
- Load-linked/store-conditional guests (ARM, RISC-V, PowerPC) do not map
  directly onto compare-and-swap hosts; the standard approaches (address
  monitors, or CAS with an ABA-tolerant scheme) each have documented failure
  cases. Decide deliberately and write the reasoning down.
- Each frontend gets its own memory-model conformance suite (`ROADMAP.md` §12).
