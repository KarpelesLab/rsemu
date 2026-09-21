# The monitor console

`ROADMAP.md` phase 9 names a "`noroi` monitor TUI" in one clause and §8 calls it
"Console/monitor". This is that console: an interactive prompt on a **stopped**
machine, which answers the questions this emulator can answer and a debugger has
no packet for, and which can steer the machine without taking time away from the
scheduler.

```console
$ rsemu monitor apple1
monitor attached to "apple1" — the machine is stopped at 0 ns
1 processor; `help` lists what this console can answer

(rsemu) clocks
domain           rate                          ticks       lead  gated
master           157500000/11 Hz                   0          0  no
video            60 Hz                             0          0  no
cpu              11250000/11 Hz                    0          0  no
pia              60 Hz                             0          0  no
(rsemu) run 500ms
stopped at 500000000 ns (+500000000 ns)
(rsemu) regs
a        0x00                x        0x00                y        0x00
sp       0xff                p        0x30                pc       0xff1f
(rsemu) x ff00 16
0000ff00  d8 a2 ff 9a a9 7f 8d 12 d0 a9 a7 8d 11 d0 8d 13  |................|
(rsemu) rewind 200ms
rewound to 300000000 ns (asked for 300000000 ns)
(rsemu) quit
```

Two spellings, one session:

```console
$ rsemu monitor <machine> [run options…]     # the subcommand
$ rsemu run <machine> --mon [run options…]   # the flag, on any run
```

`rsemu monitor` is `run --mon`, exactly as `rsemu debug` is `run --gdb :1234`.
The flag is `--mon` and not `--monitor` because **`--monitor <name>` was already
taken**, by the built-in ROM images a 6502 board can boot (`rsmon`, `wozmon`).
Two meanings of one word on one command line is a bug waiting for somebody to
type it.

Needs a build with the `monitor` feature. Without it the flag is *refused* with
the feature's name rather than answered "unknown option" — the same rule
`--trace` and the media schemes follow.

---

## What it is for

A gdbstub answers what a debugger asks, because that is the vocabulary the
protocol has: registers, memory, breakpoints, threads. rsemu knows a great deal
more about a machine than that, and none of it is reachable over GDB's wire:

| the console answers | the protocol has no packet for it |
| --- | --- |
| `devices`, `device <path>` | a device's class, its declared properties, and its whole current state |
| `clocks` | every domain's exact rate, tick count, lead and gating |
| `wires` | the net graph, what drives each net and what it settles at |
| `sched` | the threading mode, the quantum, and the event queue by target |
| `trace` | the `--trace` counters *now*, rather than at the end of a run |
| `timeline`, `rewind` | the keyframes a session holds, and going back through them |
| `save`, `load` | a machine snapshot, by hand, at an instant you chose |

The half a debugger *does* have — memory, registers, stepping — is here too,
because a console that could describe a board but not read a byte of it would
send you to a second program for every other question.

---

## The commands

```
session
  help [command]        the set, or one command in detail
  status                where the machine is, and whether its hash still means anything
  quit                  leave the session; end of input does the same

execution
  run <span>            advance virtual time by <span>, then stop
  cont [span]           the same; with no span, until interrupted or --for's deadline
  step [n]              one instruction (or n) on the selected processor
  cpu [n]               show, or select, the processor the per-CPU commands use
  cpus                  every processor: path, class and where it is
  regs [name]           the selected processor's registers, or one of them
  sched                 the scheduler: mode, quantum, and what is queued

the machine
  devices               the device tree, with class and instance path
  device <path>         one device: class, clock, space, properties, current state
  spaces                the machine's address spaces
  map [space]           what is mapped where
  clocks                every clock domain: rate, ticks, lead, gating
  wires                 every net, what drives it and what it settles at

memory
  x <addr> [len]        read at a VIRTUAL address, through the selected processor
  xp <addr> [len]       read at a PHYSICAL address, no translation
  write <addr> <hex>    write bytes at a virtual address
  writep <addr> <hex>   write bytes at a physical address
  translate <addr>      where the selected processor's MMU maps an address

state
  time                  the current virtual instant
  hash                  the machine state hash (ROADMAP.md §0)
  save <file>           write a snapshot
  load <file>           restore one
  timeline              the rewind history: cadence, keyframes, bytes held
  rewind <span>         go back <span>: restore the nearest keyframe and replay

counters
  trace [channel…]      what --trace reports, now, for sched/cpu/clock/mmio
```

Addresses are hex with an optional `0x`; lengths are decimal and bounded at 1024
bytes; spans carry their unit (`1s`, `500ms`, `20us`) because a bare number would
mean whichever unit the code happened to pick, which is how a one-second step
becomes a one-millisecond one.

A blank line is nothing and a line starting with `#` is a comment, so a script
piped into a session can explain itself.

---

## Eight commands are answered somewhere else, on purpose

`devices`, `spaces`, `map`, `x`, `xp`, `translate`, `time` and `hash` were
already implemented — for GDB's `qRcmd` packet, which is what `monitor <command>`
inside a gdb session sends. They are answered against `host::gdb::target::MachineTarget`,
and the console **forwards** to the same code rather than reimplementing them.

So `monitor map` typed into gdb and `map` typed at this prompt are one answer and
not two. A second copy would be a second thing to keep honest, and the drift
between them would be invisible until somebody compared the output. The test
`every_command_the_gdbstub_already_answers_is_reachable_here` is the door that
holds that open.

The same reasoning is why the `monitor` feature implies `gdb`. A console needs
the three things a debugger needs — a register map per CPU class, an MMU-aware
read that sets `MemAttrs::debug`, and a stepper — and `MachineTarget` has all
three. Duplicating the register tables would mean keeping two sets of byte
offsets honest against every core's snapshot layout.

---

## How it reaches a running machine

It does not reach a *running* machine. It owns when the machine runs, exactly as
`rsemu debug` does, and for the same reason: a console that had to race a
free-running guest to look at it would be answering questions about a machine
that had already moved on.

The session loop is the one in `src/bin/rsemu.rs`: read a line, execute it, print
the text, and — for `run` and `cont` — advance. **Advancing is the only thing
that touches time**, and it is `Machine::run_until` in ten-millisecond slices:
the same call, and the same slice, as `run_headless`, `interact` and the
gdbstub's free-running slice. Between slices the session drains the character
ports and checks whether a signal has arrived, which is where every other loop in
the CLI does the same work and for the same reason (an undrained `CharPort` fills
at 64 KiB and stalls the guest).

Nothing here sleeps, nothing reads the wall clock, and nothing spawns a thread.
The scheduler still owns time.

### Slicing is safe because `run_until` is additive

`Machine::run_until` stops on the machine's own scheduling boundaries and never
splits a round (`ROADMAP.md` §11.6). That is what makes it *additive*: a span
taken in pieces and the same span taken whole reach the same state. So the
console's ten-millisecond slices cannot move the answer, and `run 500ms` four
times is `run 2s`.

This is asserted, not assumed. `tests/cli_monitor.rs` runs
`rsemu run apple1 --for 2s --headless` and a session that types `run 2s`, and
compares the state hash — and then does it again in four quarters. All three are
`0xe343f99814306fdb`.

That test covers more than slicing. A monitor session attaches a recorder, a
timeline that takes a keyframe every virtual second, the trace counters on every
channel, and the debug-halt level broadcast to every device. **None of those may
move the number**, and the test is what says so.

### `step` is the exception, and it says so

`step` uses the debugger's stepper, which *does* split a scheduling round — it
has to, or stepping an instruction at a time would mean running to the end of the
round and past every breakpoint in it. That is `Machine::step_until`, it is not
additive, and a session that has used it is no longer comparable with a headless
run of the same span.

`status` says so, once the session has actually stepped:

```
note       this session has used `step`, which splits a scheduling round;
           the hash above is no longer comparable with a headless run
```

Said only after the fact rather than as a standing warning, because until then
the hash *is* comparable and a permanent caveat would train people to ignore it.

---

## Looking never changes anything

Every read goes through `MachineTarget`, whose single attribute constructor
starts from `MemAttrs::DEBUG`. That attribute exists for exactly this: a debugger
or monitor read must not pop a FIFO, clear a status bit or advance a pointer
(`ROADMAP.md` §15, invariant 5), and every MMIO device in the tree honours it.

The Apple 1's PIA is the worked example. A guest read of `$D010` takes the key
out and clears the key-waiting flag in `$D011` — that is the 6821's rule and it
is what makes the flag self-clearing. A console read of the same byte does not.
Two tests hold it:

* `an_inspection_leaves_a_waiting_keystroke_exactly_where_it_was`
  (`src/host/monitor/tests.rs`) types a key into the port, lets the ROM latch it,
  reads `$D010` four times through the console and asserts `$D011` still has its
  flag — then reads `$D010` once with ordinary attributes and asserts the flag
  clears, so the first half is about a bit that really would have moved.
* `looking_at_a_machine_does_not_change_it` (`tests/cli_monitor.rs`) runs sixteen
  commands' worth of inspection between two `hash` commands and asserts the two
  numbers are identical — with `a_write_through_the_console_does_change_it`
  beside it, so the assertion is not about a number that never moves.

A debug **write** is a different matter: `write` and `writep` are asked for
explicitly and do change the machine. Some devices refuse a debug write outright
rather than guess at it — the Apple 1's PIA does, because a debug write to `$D012`
would put a character on the screen — and the console reports the refusal instead
of a cheerful "wrote 1 byte".

---

## Rewind

`rewind <span>` goes back. It is not a separate mechanism: `machine::Timeline` is
periodic snapshot plus replay (`ROADMAP.md` §4.5), and the session attaches one at
startup so there is history to reach back through by the time anybody asks. The
default cadence is one virtual second.

```
(rsemu) run 2s
stopped at 2000000000 ns (+2000000000 ns)
(rsemu) timeline
cadence   1000000000 ns
keyframes 2 (9740 bytes held)
  0 ns
  1000000000 ns
(rsemu) rewind 1s
rewound to 1000000000 ns (asked for 1000000000 ns)
```

Three things are worth knowing before leaning on it, and all three are the
timeline's own documented behaviour rather than anything the console adds:

* **Only under deterministic threading.** `Machine::set_recorder` refuses any
  other mode, and it is right to: a parallel or accelerated run cannot be
  replayed, so a keyframe would restore to a machine that then diverged. A
  session without a timeline says so when `rewind` is typed.
* **Output already emitted has left.** Characters the guest printed are on your
  terminal and in your capture file; a rewound machine emits them again and the
  host sees them twice. That is the correct behaviour for a debugger.
* **A writable disk image is outside the snapshot** and does not move. See
  `src/machine/timeline.rs` for the policy table — `--drive …,snapshot=capture`
  is the position that makes a rewind sound, at the price of the capacity per
  keyframe.
* **Keyframes are held in memory and are unbounded.** A long session on a machine
  with gigabytes of RAM will notice; `timeline` says how many bytes are held.

---

## Counters, live

`trace` renders exactly what `rsemu run --trace` writes, in the same two columns
with the same `#` header and the same absence of any wall clock — but now, rather
than at the end of a run.

For that to mean anything the counters have to be *counting* from the first
instant, so a monitor session turns every channel on before the machine is built
(including the per-CPU capture the `cpu` channel needs) and gives them no
destination, so nothing is written at the end. That costs the run what
`--trace all` costs it and changes nothing the guest does — which
`tests/cli_trace.rs` asserts for the flag and
`a_session_reaches_the_state_a_headless_run_reaches` asserts again through this
path.

```
(rsemu) trace sched
# rsemu-trace 1
# machine         apple1
# guest-ns        500000000
# threading       deterministic
# channels        sched
# state-hash      0x…
sched.budgets                              1000
sched.quanta                                501
…
```

---

## One keyboard, one reader

A monitor session takes this terminal's keyboard, so the guest's own console is
**not** attached to it. A `CharPort` hands each byte to whoever asks first, and a
prompt and a guest reading one stdin would each get half of what was typed. The
guest's port is drained and discarded like any other unwatched one, or captured
with `--capture console=boot.log`.

That is also why there is no escape key — no `Ctrl-A c` that drops out of a
running guest into the prompt. It would have to steal a byte out of the guest's
input stream, and an input stream this tree can record and replay bit for bit is
not one to start taking bytes out of. If a session wants both, `rsemu debug`
attaches gdb *and* a console, and `monitor <command>` inside gdb reaches the eight
forwarded commands. Making the rest reachable the same way is a small change to
`DebugTarget::monitor` and is the obvious next step.

Because the console reads lines rather than keystrokes, stdin stays in cooked
mode and a pipe is a session:

```console
$ printf 'run 1s\nhash\nquit\n' | rsemu monitor apple1 -q
stopped at 999999999 ns (+999999999 ns)
0x…
```

which is how `tests/cli_monitor.rs` drives one, with no TTY anywhere.

---

## What is deliberately not here

**Media: what is in each slot, and swapping it.** Listed for a first version and
left out, because there is no machine-wide seam to build it on and the commands
that do exist are per-device. `dev::medium::MediumSlot` is a *hand-off*: a drive
takes the medium out of the slot as it is constructed, so after realize every
slot reads empty and a `media` command over it would list nothing for every
machine in the catalog. The live doors that exist —
`dev::pc::fdc::drives::Drive::insert`, the Amiga floppy's `insert(MfmDisk)`, the
SD card's `insert`/`eject` — are host objects of three different types with no
common trait, so a `media` command would work on one board and not the rest,
which is the half-working row a first version is meant to avoid. The fix is a
`Device`-level media seam: a way to ask any device what it is holding and to hand
it something else. That is a change in `dev/`, not in `host/`, and it is the
first thing to add here once it exists.

**Breakpoints and watchpoints.** The gdbstub has them, implemented with a
program-counter comparison after every tick and a polled shadow copy — and, more
to the point, with a *stepper* behind them. A monitor `break` would have to
advance the machine through `step_until` rather than `run_until` while one was
armed, so `run` would stop being additive whenever a breakpoint existed and a
session's hash would depend on whether one happened to be set. One advancing path
per front end is the property worth keeping; `rsemu debug` is a command away.

**Disassembly.** `CLAUDE.md` says the instruction-table generator emits the
disassembler and that "gdb and the monitor both need it". It is not wired up yet
in either, and doing it here first would put the interface in the wrong place.

**A full-screen `noroi` TUI.** `ROADMAP.md` phase 9 names one, and `noroi` is
MIT, has zero external crate dependencies and would fit the dependency policy —
but it is **not published on crates.io**, and rsemu is (`rsemu 0.0.5`). A `git`
dependency makes a crate unpublishable, so the console is line-oriented and the
command engine is deliberately separate from the front end: `Monitor::execute`
takes a `&str` and returns text plus a `Flow`, so a panelled full-screen frontend
over the same engine is additive work behind a second feature the day `noroi`
ships to the registry. The engine is also what makes the whole command set
testable without a TTY, which is worth having either way.

---

## Where the code is

| | |
| --- | --- |
| `src/host/monitor/mod.rs` | the command engine: parse a line, answer it, say what to do next |
| `src/host/monitor/tests.rs` | every command, driven by calling the engine |
| `src/bin/rsemu.rs` | `monitor_command`, `monitor_session` — the loop, the prompt, the pipe |
| `tests/cli_monitor.rs` | the binary, with a script on stdin: the determinism and debug-attribute gates |
| `src/host/gdb/target.rs` | `MachineTarget`, and the eight commands both front ends share |
