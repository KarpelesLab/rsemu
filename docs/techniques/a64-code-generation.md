# Generating A64: the second host backend

Consumed by: `jit/arm64/`. A companion to
[`binary-translation.md`](binary-translation.md), which is about dynamic binary
translation in general; this is about the parts that are *specific to emitting
AArch64*, written while `src/jit/arm64` was being built and kept as the record
of which decisions came from the manual and which from measurement.

The one-line summary: **an A64 backend is easier than an x86-64 backend to
encode and harder to make visible.** Fixed-width instructions remove most of
what `jit::x86::emit` spends its complexity on; a non-coherent instruction
cache adds a requirement x86-64 does not have at all.

## Sources

Everything below is derived from the architecture manual. No emulator's code
generator was consulted, and the ones that exist for this architecture are
mostly GPL (CLAUDE.md, "Provenance").

| Topic | Source |
| --- | --- |
| Instruction encodings | *Arm Architecture Reference Manual for A-profile architecture*, **DDI 0487** — §C4.1's encoding tables and §C6.2's alphabetical instruction descriptions |
| Barriers and the memory model | DDI 0487 §B2.3, *Memory barriers*; and [`memory-models.md`](memory-models.md) for what rsemu's IR asks of them |
| Cache maintenance | DDI 0487, *Synchronization and coherency issues between data and instruction accesses*; the `DC CVAU`, `IC IVAU` and `CTR_EL0` descriptions |
| Calling convention | *Procedure Call Standard for the Arm 64-bit Architecture* (**AAPCS64**), Arm IHI 0055 §6.1.1 |
| Syscall numbers | Linux's **generic** table, `include/uapi/asm-generic/unistd.h` — arm64 has no table of its own |

A useful cross-check that is not a source: `llvm-mc -arch=aarch64
-show-encoding` assembles a mnemonic to a word, and `-disassemble` reads a word
back. Running an assembler as a black box is ordinary use of a tool, and it
caught two hand-derived constants in this backend's prologue before they
reached a runner. LLVM is Apache-2.0-with-LLVM-exception in any case.

## What fixed width buys

Every A64 instruction is exactly four bytes, little-endian in memory whatever
the data endianness, with no prefix, no ModRM byte, no SIB byte and no
variable-length immediate (DDI 0487 §A1.3). Three consequences show up
immediately in the encoder:

- **An instruction is a base word OR-ed with shifted register numbers.**
  `src/jit/arm64/emit.rs` is a third the size of its x86 counterpart and has no
  equivalent of `rex()`, `modrm_rr()` or `modrm_mem()`.
- **A branch fixup is a bitfield, not a hole.** `Asm::bind_to` patches 26, 19 or
  14 bits *in instructions* rather than writing four bytes of `rel32`, so a
  `Fixup` has to remember which form it is — that is the only place the encoder
  is more complicated than x86's.
- **A displacement that does not fit is a real case.** x86-64's `rel32` and
  `disp32` reach further than any block, so `jit::x86` treats an overflow as
  unreachable. A `TBZ` reaches ±32 KiB and a scaled load offset reaches
  4095 units, so `jit::arm64` *refuses the block* — `Refusal::Shape` — and the
  interpreter runs it. Refusing is the whole answer, and it is the same seam
  that already handles an unlowerable opcode.

## What fixed width costs

There is no arithmetic against a 64-bit immediate, and no load-immediate at
all. A constant is built a halfword at a time with `MOVZ`/`MOVK`, or its
complement with `MOVN` (DDI 0487 C6.2). `Asm::mov_imm` picks whichever needs
fewer instructions, which matters more than it sounds: every mask with high
ones — `!PAGE_MASK`, a sign-extension mask, `-1` — is **one** instruction
through `MOVN` and four through `MOVZ`/`MOVK`.

Where a constant would still be needed, the fix is usually to not need one:

| x86-64 | A64 |
| --- | --- |
| `and rax, !PAGE_MASK` (a 10-byte `mov` plus an `and`) | `lsr`/`lsl` by 12 — two instructions, no register |
| `and rax, 1` to canonicalise an `i1` | `ubfx x, x, #0, #1` |
| `mov eax, eax` to canonicalise an `i32` | `mov w, w` — every write to a `W` register clears the top half (DDI 0487 §C1.2.5) |
| `shl`/`sar` pair to sign-extend `n` bits | `sbfx x, x, #0, #n` |
| six instructions for `deposit` | `bfi` |
| a swap cascade for a narrow-lane `bswap` | `rev16` / `rev32` / `rev` |

The last two are the clearest illustration of why the IR's op set is shaped the
way it is. `Opcode::DEPOSIT` and `Opcode::BSWAP`-with-a-lane exist because
*some* architecture has them as one instruction; on this host they are one
instruction, and on x86-64 they are the six and the cascade `jit::x86::compile`
open-codes.

## Registers

AAPCS64 §6.1.1 gives ten callee-saved registers (`x19`–`x28`) where System V
gives six. This backend spends three of them — the context, the temporary
frame, the thunk table — exactly as the x86 backend spends three of six, so the
register allocator gets **seven** saved registers instead of three, plus seven
volatile ones instead of four.

That is the one place where this host is plainly better, and it is the number
to watch when the throughput question is answered: `jit::x86::compile`'s own
docs identify "how many values can be held across a call" as the ceiling on
what its allocator can do, and a block that spilled there may not spill here.
`x18` is never touched — AAPCS64 reserves it as the platform register.

## The barrier

`Opcode::FENCE` lowers to `DMB ISH`. The reasoning is in
[`memory-models.md`](memory-models.md), and the short form is that
`IrHost::fence`'s contract is a `SeqCst` host fence, which on this architecture
is exactly the full inner-shareable `DMB`.

Two near misses are worth naming, because both would pass every test on a
machine that happens not to reorder:

- **`DMB ISHST` and `DMB ISHLD`** differ from `DMB ISH` in one nibble of the
  encoding and are both weaker. `the_barrier_is_the_full_inner_shareable_one`
  asserts the `CRm` field rather than the whole word, so the failure names the
  mistake.
- **`DSB`** is correct and strictly stronger: it waits for completion rather
  than ordering (DDI 0487 §B2.3). A guest barrier does not need that, and
  paying for it on every guest `DMB` would be a real cost. `DSB` appears in
  this backend only in the code buffer, where completion is the requirement.

Note what changed relative to x86-64: there, the barrier was buying back *one*
reordering (store-then-load), because x86-TSO gives the other three for free.
Here it is buying all of them. The lowering is the same one line of code and a
different amount of work.

## Cache maintenance, which x86-64 does not need

**A64's instruction cache is not architecturally coherent with its data
cache.** Bytes written through an ordinary store may sit in the data cache
while an instruction fetch reads stale memory. `mprotect` does not fix this;
neither does anything else the kernel is required to do for you.

DDI 0487 gives the sequence, and `src/jit/arm64/buf.rs` runs it over the range
that was written, once, at the moment the write window is sealed:

```
DC CVAU   for every data-cache line in the range     (clean to the point of unification)
DSB ISH                                              (wait for it, inner-shareable)
IC IVAU   for every instruction-cache line           (invalidate)
DSB ISH                                              (wait for it)
ISB                                                  (this PE re-fetches)
```

Three details are easy to get wrong and each is in the manual:

1. **The line sizes are not fixed.** `CTR_EL0` carries `DminLine` and
   `IminLine`, each the log2 of a line size *in words*. A loop that assumed 64
   bytes would skip lines on a core with 32-byte ones. Read the register.
2. **`CTR_EL0.IDC` and `CTR_EL0.DIC`** say the clean and the invalidate
   respectively are *not required* on this implementation. Honour them; on a
   core that sets them the loops are pure cost.
3. **`ISB` is per-PE.** `IC IVAU` is broadcast across the inner-shareable
   domain, but the pipeline flush is not. Every PE that will execute the new
   code needs a context synchronization event after the invalidation. In rsemu
   a `CodeBuf` belongs to one engine and one CPU model, and a thread migrating
   between PEs does so through an exception entry and return, which is one —
   but a design that compiled on one thread and ran on another would owe an
   explicit `ISB` on the running side.

Linux permits all of it from EL0: `SCTLR_EL1.UCT` enables the `CTR_EL0` read
and `SCTLR_EL1.UCI` enables `DC CVAU` and `IC IVAU`, and where a kernel does
not set them it traps and emulates rather than faulting.

## Page size, and why the buffer rounds to 64 KiB

x86-64 Linux has one base page size and `jit::x86::buf` hard-codes 4 KiB. A64
has **three** translation granules — 4 KiB, 16 KiB and 64 KiB (DDI 0487 §D8) —
and a kernel is built for one of them. A hard-coded 4 KiB would produce
`mprotect` calls that fail with `EINVAL` on a 64 KiB kernel; not a fault (the
buffer answers `None` and the backend degrades to the interpreter) but a whole
class of hosts running slow for a constant.

`jit::arm64::buf` rounds every mapping and every write window to **64 KiB**,
the largest of the three, and asks the kernel nothing. Every smaller granule
divides it, so a 64 KiB-aligned address is page-aligned on all three. The cost
is a coarser W^X window, which changes neither the property (still per-page,
still never both) nor the asymptotics of a flip.

The alternative — reading `AT_PAGESZ` out of `/proc/self/auxv` — is available
and was not taken: it is a file read and a parser on a path that currently has
neither, to learn a number the conservative constant already satisfies.

## macOS is a different problem

Apple silicon is the largest population of aarch64 machines this project will
meet, and this backend **does not run there**. It is `cfg`-gated to aarch64
Linux, and the gap is not a missing `cfg`:

- `mmap` needs `MAP_JIT`, and the binary needs the corresponding entitlement.
- Write protection is per **thread**, not per page:
  `pthread_jit_write_protect_np` flips the calling thread between writing and
  executing the JIT region. That is a different state machine from
  `mprotect`-a-window, and `CodeBuf`'s `open`/`seal` bookkeeping does not
  express it.
- It is reached through a libc function. rsemu's dependency policy is raw
  syscalls, and there is no stable syscall behind that call.

So it is a separate piece of work with its own design review — including
whether the `unsafe` and dependency questions it raises are answerable at all
under the current rules — rather than an extension of this one. Until then an
Apple-silicon build gets what it has always had: the IR interpreter.

## What is not known

**Throughput.** There is no number, and there will not be one until it is taken
on an aarch64 host. This backend was written on an x86-64 machine, which cannot
execute a single instruction it emits.

What would answer it, in order:

1. Make the backend reachable from a guest. `jit::dispatch` holds
   `Option<jit::x86::Engine>` behind a `cfg` naming the concrete module, and
   every `cpu::*::engine` that attaches a backend repeats that `cfg` — so on an
   aarch64 host `engine = "jit-host"` still runs the interpreter even in a
   build with this backend compiled. `jit::host` is the alias those `cfg`s
   should be rewritten against; the modules already offer identical names, so
   the change is mechanical and touches no code generation.
2. Widen `benches/jit_dispatch.rs`, `benches/a64_dispatch.rs` and
   `benches/x86_dispatch.rs`'s `with_backend` to that alias. Their `+compiled`
   and `+allocated` columns are then the measurement, unchanged, on a second
   host.
3. Run them. The interesting rows are the ones where the two hosts should
   *differ*: `alu-loop` (does the extra callee-saved register keep more guest
   registers live?), `load-heavy` (the inlined probe is shorter here — three
   `CBZ`s instead of three compare-and-branch pairs), and anything with a fence
   in it (`DMB ISH` does real work where `MFENCE` was partly free).

Until then the honest claim is the one the module makes: it emits the
instructions the manual says it does, and whether that is fast is unmeasured.
