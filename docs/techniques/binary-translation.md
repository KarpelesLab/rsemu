# Dynamic binary translation and JIT compilation

Consumed by: `ir/`, `jit/`. This is the one area where the
primary sources are **academic papers rather than datasheets** — and where the
temptation to read a GPL implementation is strongest. The literature is
extensive, public, and enough.

## Foundational literature

Search by title; most are freely available from the authors or via ACM/USENIX
open archives.

| Topic | Representative work |
| --- | --- |
| DBT fundamentals | Smith & Nair, *Virtual Machines: Versatile Platforms for Systems and Processes* — the standard textbook, covering interpretation, translation, chaining, and precise exceptions |
| Translation caching and chaining | The Shade and Embra papers; the DynamoRIO and Pin literature on code caches |
| Register allocation | Poletto & Sarkar, *Linear Scan Register Allocation*; Wimmer & Franz on linear scan with SSA |
| Trace/superblock formation | Bala et al., *Dynamo: A Transparent Dynamic Optimization System* |
| Tiered execution and feedback | The HotSpot and V8 literature on tier-up, inline caches, and deoptimisation |
| Software MMU design | The literature on address-translation caching in system emulators |

## In-house prior art

[`kataan`](https://github.com/KarpelesLab/kataan) is ours and MIT: a JS engine
with a register bytecode VM, a machine-code JIT emitting into W^X memory via raw
syscalls, tiered execution, and a WebAssembly engine. For the questions "how do
we emit and protect code without libc" and "how should tiering be structured",
it is both a legitimate source and a tested one. Read it.

## Design decisions already made

Recorded in `ROADMAP.md` §9, from first principles rather than by imitation:

- ~60 architecture-neutral typed ops, SSA within a block, helper calls for
  anything rare. The op set is derived from what the target ISA manuals actually
  require, and it is small because breadth belongs in helpers.
- A **software TLB** in front of the address-space dispatch, inlined into
  generated code. This is the single largest determinant of performance;
  everything else is secondary.
- Translation-block cache keyed by `(guest PC, relevant flags)`, with block
  chaining, page-level dirty tracking for self-modifying code, and superblock
  formation at tier 2.
- Compilation runs on the task pool while the interpreter executes, published
  with one atomic store.

## ⚠ Do not consult

QEMU's translator is the obvious reference and is permanently off limits, as is
Unicorn Engine (derived from it) and every GPL emulator's JIT. The papers above
describe the same techniques — they are where those implementations got them
from too.
