# Debugging protocols

Consumed by: `host/gdbstub`. Debugging a guest kernel is a headline
feature, not a nicety — and it is what makes rsemu useful to people doing
bring-up work rather than just running old games.

## GDB remote serial protocol

| Source | Covers |
| --- | --- |
| [GDB manual — Remote Protocol appendix](https://sourceware.org/gdb/current/onlinedocs/gdb.html/Remote-Protocol.html) | The complete packet-level protocol: register access, memory read/write, breakpoints and watchpoints, `vCont`, thread/multiprocess extensions, and the `qXfer` object transfers used for target descriptions |

The GDB manual is GFDL-licensed documentation of a **wire protocol**.
Implementing a protocol from its published specification is exactly what the
specification is for; the restriction that matters is on copying the
documentation text, not on speaking the protocol.

## Target descriptions

`qXfer:features:read` returns an XML register-layout description, which is how
GDB learns a target's register set without being recompiled. rsemu generates
each one from the same table its `g` packet is built from
(`src/host/gdb/arch.rs`), so the two cannot disagree — which is the classic
gdbstub bug. There is no `RegView` on `Device` to generate them from: the core
exposes no register accessor and `Any` is deliberately kept out of `Device`'s
supertrait chain, so a map is a table of byte offsets into the device's
**snapshot chunk** instead. That is the standing item in `src/host/gdb/mod.rs`'s
"what is not here", and the one thing that would collapse every map in that file
into a call.

## DWARF

| Source | Covers |
| --- | --- |
| [dwarfstd.org](https://dwarfstd.org/) | The DWARF debugging format specification |

Not needed for the gdbstub (GDB does its own symbol handling), but relevant if
rsemu ever resolves guest symbols itself for tracing or profiling output.

## Proving it against a real client

Two ends of the same protocol written by the same people agree with each other
by construction, so `tests/gdb_session.rs` — rsemu's own client against rsemu's
own stub — cannot be the phase-9 gate on its own.
`tests/gdb_real_client.rs` runs the **distribution's `gdb` binary** in batch
mode against a running guest and asserts on what GDB printed. Running a GPL
program as a client is black-box use, which `ROADMAP.md` §1 permits explicitly;
nothing in that test reads GDB's source.

It drives **three** guests — a 16-bit x86 one, an x86-64 one and an AArch64 one
— and each skips on its own if the `gdb` to hand has no gdbarch for it. On the
common x86-64 developer machine the two x86 sessions run and the AArch64 one
skips; with `gdb-multiarch`, or a cross `gdb` named in `$RSEMU_GDB`, all three
run. Several tests in the same file need no `gdb` at all — the x86 fixture board
resets into its ROM, both x86 register maps are compared register by register
against the core they describe, and an AArch64 breakpoint stops on the
instruction it names — so those surfaces are still covered where a session
skips. `tests/gdb_cores.rs` does the same for the three cores that had no map at
all.

It skips cleanly when there is no `gdb`, and when the `gdb` there is has no
gdbarch for the guest. That second condition is the interesting one, and it is
a property of GDB rather than of the stub:

- A target description gives GDB the **registers**. GDB still insists on a
  **gdbarch** for the machine, and a distribution's `gdb` usually carries one
  architecture — the host's. So the guest a stock `gdb` can debug on the common
  developer machine is an x86 one, which is why `machines/tests/x86-mini.machine`
  exists.
- GDB **accepts** our AArch64 description, and that is not luck: the
  `org.gnu.gdb.aarch64.core` feature has a fixed meaning — `x0`-`x30`, `sp`,
  `pc`, `cpsr`, in that order and at those widths — and the map supplies
  exactly it, so it is entitled to the name and to
  `<architecture>aarch64</architecture>` with it. A session does not even need
  `set architecture`. That is the shape every future map should aim at.
- GDB used to **reject** our x86 description (`warning: Architecture rejected
  target-supplied description`) because its `i386` gdbarch will only accept a
  feature named `org.gnu.gdb.i386.core` carrying the x87 register block as well
  as the integer sixteen. Having rejected it, GDB fell back to its built-in
  i386 layout — whose first sixteen registers happen to be exactly the order
  `cpu.x86` saves them in, which is why the fallback worked rather than
  producing garbage. Agreeing by luck is not the same as being right, and it was
  no use at all in 64-bit mode, where GDB's fallback layout is not that chunk.
  Both x86 maps now supply the x87 block, so both descriptions are accepted.
- GDB has **no notion of x86 segmentation**: `$pc` is `eip` and every address in
  an `m`/`M`/`Z` packet is that flat number. On a real-mode guest whose `CS`
  base is not zero, `eip` and the linear address the guest fetches from are
  different numbers. That is the protocol's shape, not a defect, and it is why
  the fixture's ROM far-jumps to `0000:0500` before anything interesting runs.

`RSEMU_GDB_DEBUG_REMOTE=1` makes that test run GDB with `set debug remote 1`,
which prints every packet it sends. That listing is how the set of packets this
stub has to answer was established, and it is the first thing to reach for when
a real session misbehaves.

## The monitor

`qRcmd` — GDB's `monitor` command — carries the commands that have machinery
behind them but no packet of their own: `devices`, `spaces`, `map`, `x`, `xp`,
`translate`, `time`, `hash`. The pair that justifies the surface is `x` and
`xp`: GDB's own `x` is always **virtual**, because the protocol has no
physical-address packet, so a bus address — a boot ROM under a page table, a BAR
before the guest has mapped it — is not reachable from a GDB session by any
other route. Every one of those reads sets `MemAttrs::debug`.

### Why the monitor is not a TUI

`ROADMAP.md` phase 9 names a **`noroi` monitor TUI**. It is not built, and the
reasons are recorded here so the line is not implemented by inertia later.

- **The command set already exists, and on a surface that reaches further.**
  Everything a monitor would be for — the address spaces, the region map, a
  hexdump by virtual *and* physical address, the device tree, the clock, the
  state hash — is the list above, reachable from any GDB session and from any
  program that can open a TCP socket and write a packet. A TUI would be a second
  parser and a second set of eight commands over the same `DebugTarget`.
- **`noroi` cannot be in `--all-features`.** Its README's "zero external crates"
  is a claim about the dependency graph, not about linkage: with `std` enabled
  it declares `tcgetattr`, `tcsetattr` and `ioctl` in an `unsafe extern "C"`
  block and hard-codes the **Linux** `termios` layout, so `src/sys/mod.rs`
  raises `compile_error!` on Windows and `src/sys/unix.rs` raises one on every
  unix that is not Linux or Android. rsemu's test job runs on ubuntu, macos and
  windows; a feature that fails to compile on two of the three cannot be in the
  `--all-features` build those jobs use. That is before the design argument, and
  it is enough on its own.
- **It contradicts §0's raw-syscall rule.** Linking libc symbols is exactly what
  `host::terminal` goes out of its way to avoid — it shells out to `stty`
  instead, which is why the emulator's own console works on a `fullrust` target.
  A debugger UI that is less portable than the console it debugs through is the
  wrong way round.
- **It would want stdin, and the guest already has it.** `rsemu run apple1`
  gives the terminal to the machine. A full-screen monitor wants the same
  terminal, so a TUI is either mutually exclusive with the console — which is
  the only frontend most boards in this tree have — or it needs a multiplexer
  nobody has asked for.
- **It reads input on a thread it spawns itself** (`EventStream::spawn`), which
  is not how work is started here (`CLAUDE.md`: submit jobs, never spawn
  threads).

If a monitor is ever wanted, the thing to build is not a second command set: it
is a way to reach *this* one without a GDB client — the commands are
`MachineTarget::monitor` in `src/host/gdb/target.rs` and they take a string and
return a string, which is the whole interface a REPL, a socket or a wasm export
would need.

## Implementation notes

- Multi-CPU machines present as **threads** to GDB, which maps cleanly onto our
  CPU list.
- Attaching a debugger must **stop the world** through the safe-point protocol
  (`ROADMAP.md` §4.7), not by racing the scheduler. In practice the stub gets
  that for free in both threading modes: virtual time only advances inside
  `DebugTarget::resume` and `DebugTarget::step`, and the scheduling round those
  drive joins every job it submitted before it returns — under `parallel` that
  join *is* §4.7's rendezvous. `tests/gdb_multicpu.rs` debugs a two-core machine
  in that mode rather than leaving it a claim.
- A watchpoint, and every memory access, belongs to **a CPU**: the address is
  virtual and the space is that core's. Polling a watchpoint through CPU 0 on a
  machine with two of them reads a different byte, and on a board where nothing
  is mapped at that number it reads a constant — so the watchpoint never fires
  and never says why.
- Every debugger memory access sets `MemAttrs::debug` — reading a device
  register from GDB must not acknowledge an interrupt or pop a FIFO. This is
  invariant 5 in `ROADMAP.md` §15 and this is the code path that violates it
  first.

## What a debugger can do, per platform

Established by attaching to each board rather than by reading the code. A CPU
appears to GDB as a thread when this build has a register map for its class
(`src/host/gdb/arch.rs`) *and* the machine gave it a clock domain. A board
whose only core has no map presents **zero threads**, and `target remote` on it
is a session with nothing in it — no registers, no `$pc`, no breakpoints.

| Board | Threads | Register view |
| --- | --- | --- |
| `arm64-virt`, `a64-mini` | 1 | `x0`-`x30`, `sp`, `pc`, `cpsr`; a 268-byte `g` packet. The description is `org.gnu.gdb.aarch64.core` and GDB **accepts** it rather than rejecting it. Virtual addresses go through the VMSAv8-64 walk. |
| `riscv-virt` | 1 | `x0`-`x31` and `pc`; a 264-byte `g` packet, claiming `riscv:rv64`. On an RV32 machine the widths are 64-bit and the values sign-extended — `arch.rs` says why. |
| `q35`, `q35-uefi`, `q35-linux`, `pc64` | 1 | `rax`-`r15`, `rip`, `eflags`, the six selectors and the x87 block; a 276-byte `g` packet claiming `org.gnu.gdb.i386.core` and `<architecture>i386:x86-64</architecture>`, which GDB **accepts**. |
| `q35-linux-smp` | **2** | as above, one thread per processor |
| `pc-at` | 1 | the i386 sixteen plus the same x87 block; a 144-byte `g` packet. `org.gnu.gdb.i386.core` with **no** `<architecture>`, so GDB resolves it to `i386` on its own — measured, on a GDB built for x86-64 — and `set architecture i8086` still validates against it. |
| `pc-at-smp`, `pc-apic` | **2** | as above, one thread per processor |
| `gameboy` | 1 | `a f b c d e h l sp pc`; a 12-byte `g` packet. No `<architecture>`: upstream GDB has no SM83. |
| `stm32f407` | 1 | `r0`-`r12`, `sp`, `lr`, `pc`, `xpsr`; a 68-byte `g` packet claiming `org.gnu.gdb.arm.m-profile` and `arm`, which GDB **accepts**. |
| `m68k-mini` | 1 | `d0`-`d7`, `a0`-`a5`, `fp`, `sp`, `ps`, `pc`; a 72-byte `g` packet claiming `org.gnu.gdb.m68k.core` and `m68k`, which GDB **accepts**. |
| `apple1`, `nes-*`, `sms-*`, `arm926`, `beneater-6502` | 1 | per core. The 6502, Z80 and ARMv5 maps are complete; the first two carry no `<architecture>`, so a stock GDB refuses the description — see above. |
| `mips-mini` | **0** | no map for `cpu.mips`, deliberately — see below. |

Three more things that were checked rather than assumed:

- **SMP really is threads.** `machines/pc-at-smp.machine`'s two processors come
  out of `qfThreadInfo` as two, `Hg`/`Hc` select between them, a stop reply
  names the one that stopped, and `info threads` lists both. Each has its own
  register file, its own address space and its own watchpoint polling.
- **`cpu.i8086` is debuggable now.** It is the same core as `cpu.x86` under an
  older class name, and no shipped machine file uses it, but one that did used
  to get a debugger with nothing in it. It shares `cpu.x86`'s 32-bit register
  table. The two classes are still drifted apart in `version` — 6 against 8, for
  byte-identical `save` code — which is a defect in the core rather than in the
  stub: a `cpu.i8086` snapshot claims a version its bytes do not match. The map
  records the drift and `tests/gdb_real_client.rs` fails when it is fixed, so
  the note gets removed rather than rotting.
- **`cpu.mips` is left out on purpose.** GDB's `mips_gdbarch_init` requires
  `org.gnu.gdb.mips.cpu`, `org.gnu.gdb.mips.cp0` *and* `org.gnu.gdb.mips.fpu`,
  and yields no gdbarch when any is absent; the FPU feature is not optional and
  this core models no FPU, so no honest description could be accepted. Its cycle
  counter also has no fixed offset — `save` writes the two caches with a length
  prefix and the counter comes after them — and `RetireCounter` has no computed
  hook, so single-stepping would fall back to comparing the program counter.
  `tests/gdb_cores.rs` asserts the omission so that fixing it is noticed.

### Which of `cpu.x86`'s two views a board gets

`cpu.x86` is one class covering an 8088 and an x86-64 part, and those are
different register files. The register map is per class and the register file is
per instance, so something has to choose, and the instance's `variant` property
is not reachable: a `Machine`'s `DeviceEntry` carries the class, the clock
domain, the requester and the address space, and not the properties the class
was constructed from.

What it does carry is the **address space**, and its width is the same decision
written down by the same person. Every board in `machines/` that says
`variant = "x86-64"` gives its core a 64-bit space; every board that does not
gives it thirty-two bits or fewer. `host::gdb::arch::for_cpu` therefore serves
the 64-bit view to a core on a bus wider than thirty-two bits and the 32-bit
view otherwise, and `tests/gdb_real_client.rs` pins both halves.

It is a proxy and it is named as one. What removes it is a register view — or
merely a variant accessor — on `Device`, which is the standing item in
`src/host/gdb/mod.rs`'s "what is not here".

### What the 64-bit view cost

Two things were in the way, and only one of them was the one that had been
guessed at.

The **read** side needed `Arch::computed`. The long-mode block is in the chunk
— appended at version 4, deliberately, so the 32-bit map's offsets stayed valid
— but it sits behind a **variable-length field**: the prefetch queue is written
with a length prefix, and the bus interface unit's queue fills and empties as
the guest runs. So `RAX` is not at an offset, and a table of constants cannot
find it. That is what `Arch::computed` exists for, and the AArch64 map was its
first user.

The **write** side needed something else. The register file is in the chunk
twice — the 32-bit prefix and the wide block — and `cpu.x86`'s `load` does not
let the second copy win: for everything but `R8`-`R15` it takes the upper
thirty-two bits from the wide block and the lower thirty-two from the prefix,
so that a 32-bit debugger's `P` packet into the prefix is not discarded. A
64-bit register write is therefore genuinely two writes, and the map does both.

Neither obliged a change to the core.

## Debugging a translating engine

`engine = "jit"` and `engine = "jit-host"` put a cache of compiled blocks
between guest memory and what executes, and a debugger has to be right about
both directions through it.

**A breakpoint inside a compiled block fires**, and by construction rather than
by luck. With a breakpoint or a watchpoint armed, `resume` advances one clock
tick at a time; a tick is a budget of about one bus access, and a core declines
to run a block whose worst case does not fit what is left of the budget. An
armed breakpoint therefore degrades a translating core to one interpreted
instruction per tick, and the program counter is compared after each. It works
on a block's entry, in its middle and on its last instruction alike. What it
costs is the engine: a checking slice takes about the same wall time on `jit`
as on `interp`, because while it is checking there is no JIT. That is the price
of not patching trap instructions into guest memory, and it is the same price
the interpreted boards were already paying.

**A debugger's write into guest code has to be announced.** A *guest* store
into a page a block was lifted from invalidates it, through `jit::cache`'s
`note_write`, driven from the core's own execution path. A write from the
gdbstub does not go through that path — it goes straight at the address space —
so nothing noticed it, and the cached block outlived the bytes it was lifted
from. Measured: a patch one instruction into a hot RISC-V loop was executed
forty times in twenty-five thousand iterations, which is the interpreter's
share of the run, and ignored the rest. Every way a user has of patching a
guest ran into this — `restore`, `set *(int *) …`, `load`.

`MachineTarget::invalidate_translations` closes it by doing what a snapshot
restore does — `save` then `load`, which every core defines as discarding
derived state — after any debugger write. `cpu.riscv` and `cpu.x86` flush their
block cache on `load` and are fixed by it. **`cpu.arm.a64` does not**: its
`Device::load` flushes its TLB and leaves its translations, and its
`Device::reset` does the same. That is a defect in the core rather than in the
debugger, and the debugger is not its only victim — a snapshot restored into a
warm AArch64 core keeps blocks lifted from the RAM the restore just replaced.
`tests/gdb_engines.rs` covers the two cores that honour the contract and says
why it stops there.
