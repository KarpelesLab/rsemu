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
GDB learns a target's register set without being recompiled. Generating these
from each CPU core's `RegView` means new architectures get debugger support for
free — worth doing at the same time as the core rather than later.

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

It skips cleanly when there is no `gdb`, and when the `gdb` there is has no x86
gdbarch. That second condition is the interesting one, and it is a property of
GDB rather than of the stub:

- A target description gives GDB the **registers**. GDB still insists on a
  **gdbarch** for the machine, and a distribution's `gdb` usually carries one
  architecture — the host's. So the guest a stock `gdb` can debug on the common
  developer machine is an x86 one, which is why `machines/tests/x86-mini.machine`
  exists.
- GDB **rejects** our x86 description (`warning: Architecture rejected
  target-supplied description`) because its `i386` gdbarch will only accept a
  feature named `org.gnu.gdb.i386.core` carrying the x87 register block as well
  as the integer sixteen. Having rejected it, GDB falls back to its built-in
  i386 layout — whose first sixteen registers are exactly the order `cpu.x86`
  saves them in, which is why the fallback works rather than producing garbage.
  The session that follows the warning is a complete one.
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
