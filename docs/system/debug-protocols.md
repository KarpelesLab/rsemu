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

## Implementation notes

- Multi-CPU machines present as **threads** to GDB, which maps cleanly onto our
  CPU list.
- Attaching a debugger must **stop the world** through the safe-point protocol
  (`ROADMAP.md` §4.7), not by racing the scheduler.
- Every debugger memory access sets `MemAttrs::debug` — reading a device
  register from GDB must not acknowledge an interrupt or pop a FIFO. This is
  invariant 5 in `ROADMAP.md` §15 and this is the code path that violates it
  first.
