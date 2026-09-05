# The level-3 process ABI, and the sandbox policy

Consumed by: `src/usermode/`, and by any crate that builds a syscall kernel on
it. This is the layer where a *program* — not a machine, not a kernel — becomes
something rsemu can start.

`ROADMAP.md` §2.1 splits the work: rsemu supplies the machine (a core that
exits at `ecall`, a memory map with no devices in it, a scheduling contract,
and the record/replay funnel), and the consumer supplies the operating system
(the ELF loader, the syscall table, descriptors, errno, the process model). The
sources below are the ones the *consumer* half is written from. They are listed
here rather than in the consumer's own repository because the split is a design
decision of this project's and the reader of `src/usermode/` needs to know what
is on the other side of it.

## Specifications

| Source | Covers | Access |
| --- | --- | --- |
| [System V ABI, generic part (gABI)](https://www.sco.com/developers/gabi/) | `Elf64_Ehdr`, `Elf64_Phdr`, segment types and flags, and "Process Initialization" — the initial stack | Free |
| [RISC-V psABI](https://github.com/riscv-non-isa/riscv-elf-psabi-doc) | The RISC-V processor supplement: register roles, the 16-byte stack alignment, `EM_RISCV = 243` | Free (Creative Commons) |
| [ARM 64-bit ELF ABI](https://github.com/ARM-software/abi-aa) | The AArch64 supplement: `EM_AARCH64 = 183`, `x8` for the syscall number, `TPIDR_EL0` for the thread pointer | Free |
| [Arm ARM (DDI 0487)](https://developer.arm.com/documentation/ddi0487/latest/) | What state a core has to be in for user code: `PSTATE.EL`, `CPACR_EL1.FPEN`, `SCTLR_EL1.M` | Free (registration) |
| `clone(2)`, `futex(2)`, `set_tid_address(2)`, `sigaltstack(2)`, `rt_sigaction(2)` | The threading calls, and which of their arguments are *queries* | Free — `man-pages` |
| `elf(5)`, `getauxval(3)`, `mmap(2)`, `brk(2)`, `getrandom(2)` | The Linux manual pages: the auxiliary vector's `AT_*` values and what the kernel actually puts in them | Free — `man-pages`, GPL-compatible documentation, quoted rather than copied |
| [Linux `include/uapi/asm-generic/unistd.h`](https://www.kernel.org/) | The syscall numbers RISC-V, AArch64 and every architecture added since 2012 share | The header is a UAPI interface definition; **numbers are facts** (`ROADMAP.md` §1, "facts versus expression") |
| [`Documentation/arch/riscv/hwprobe.rst`](https://www.kernel.org/) | What `AT_HWCAP` means on RISC-V: one bit per single-letter extension | Free |

**On the kernel headers.** rsemu does not read Linux's *implementation* — it is
GPLv2 and §1 is unambiguous. Syscall numbers, structure layouts and `AT_*`
constants are the published interface a program is compiled against, and are
facts about the ABI in exactly the way a cycle count from a datasheet is a fact
about a chip. Where behaviour was needed rather than a number, it was obtained
by **running** a program and reading its trace, which §1 explicitly permits.

## Two architectures, and what the second one cost

RISC-V was first. AArch64 is second, and it is there because §2.1's claim is
that a syscall exit is **a property of a core** rather than a property of
RISC-V — a claim only a second core can test.

The answer is short, which is the result. `src/usermode/proof.rs` names
everything an architecture contributes in one struct:

| | RISC-V | AArch64 |
| --- | --- | --- |
| `e_machine` | 243 | 183 |
| syscall number | `a7` (`x17`) | `x8` |
| arguments | `a0`..`a5` (`x10`..`x15`) | `x0`..`x5` |
| result | `a0` | `x0` |
| thread pointer | `tp` (`x4`) | `TPIDR_EL0` |
| the call | `ecall` | `svc #0` |
| unprivileged state | `priv = User` | `PSTATE.EL = EL0` |
| `AT_HWCAP` | one bit per single-letter extension | `HWCAP_FP`, `HWCAP_ASIMD` |

Plus two decisions a Linux kernel makes for a process it is about to enter and
that a level-3 consumer therefore has to make itself, both AArch64-specific:
**`CPACR_EL1.FPEN = 0b11`**, because the architecture resets it to *trap* and
the first `stp q0, q1` inside a `memcpy` would otherwise take an `UNDEFINED`
that no guest kernel is there to handle; and **`SCTLR_EL1.M = 0`**, because
level 3's memory model is "there is no page table" and the map `UserMemory`
builds is the address space the guest sees.

Nothing else moved. **The ELF loader, the initial stack, the auxiliary vector,
every syscall, the errno values, the host-filesystem policy and the journal are
byte-identical between the two**, and the first AArch64 run of the same
`hello` binary made the same twenty-five calls in the same order as the RISC-V
one and refused none of them. The claim that the seam is not RISC-V-shaped is
now measured rather than asserted.

`AT_HWCAP` is worth one more line, because it is a promise rather than a
description. `HWCAP_ATOMICS` is deliberately **absent**: `Config::cortex_a53`
has no `FEAT_LSE`, compiler-rt's out-of-line atomics read that bit to choose
between `casal` and an `ldxr`/`stxr` loop, and a part that claimed the bit
would take an `UNDEFINED` on the first atomic a threaded guest executed.

## The differential oracle, which is the point of level 3

A level-3 guest is the cheapest correctness experiment this project has: the
same statically linked binary runs on the host and under the emulator, and the
two must agree. Not only on output — on the **syscall trace**.

That is how `src/usermode/proof.rs` was written. Rather than implementing a
list of syscalls, the guest was run until it stopped, and whatever it asked for
next was implemented. The same program was then built for `x86_64-unknown-linux-musl`
and run under `strace` on the host, and the two traces compared:

```text
host (x86-64 musl)                     rsemu (rv64gc and aarch64 musl)
  set_tid_address                        set_tid_address
  poll                                   ppoll
  rt_sigaction × 2                       rt_sigaction × 2
  sigaltstack(NULL, &old)                sigaltstack(NULL, &old)
  mmap(12288, RW)                        mmap(12288, RW)
  mprotect(PROT_NONE)                    mprotect(PROT_NONE)
  sigaltstack(&new, NULL)                sigaltstack(&new, NULL)
  rt_sigprocmask                         rt_sigprocmask
  rt_sigaction × 3                       rt_sigaction × 3
  brk(NULL); brk(+8K)                    brk(NULL); brk(+8K)
  mmap(FIXED, PROT_NONE, at brk)         mmap(FIXED, PROT_NONE, at brk)
  mmap(4096, RW)                         mmap(4096, RW)
  write(1, …)                            write(1, …)
  munmap; sigaltstack(disable); munmap   munmap; sigaltstack(disable); munmap
  exit_group(0)                          exit_group(0)
```

That second group used to read `× 2` on both sides, and it was wrong on both
counts: the host makes three calls there and rsemu made two. See below.

This found a defect a passing test would not have, and then found the same
defect twice more when the method was repeated. `sigaltstack` had been
stubbed to return `0` and write nothing — which looks harmless and is worse than
wrong: the caller queries the current alternate stack first, and a query that
leaves the guest's own buffer untouched reads back as `ss_flags == 0`, meaning
*"one is already installed"*. Rust's standard library then skipped installing
its own, and a stack-overflow handler that exists on Linux silently did not
exist here. `SS_DISABLE` has to be **said**. Nothing about the program's output
changed; only the trace did.

The mirror-image lesson: the `mmap(MAP_FIXED, PROT_NONE)` over the first heap
page *looks* like a bug in the emulated `brk`, and the host trace shows `musl`
doing exactly the same thing on real Linux. Without the second trace that would
have been a day spent fixing something that was not broken.

**The same shape, twice more.** `rt_sigaction` was stubbed to return `0`, and
its third argument is a *query* too: a runtime asks "is a handler already
installed for this signal?" before installing its own. The emulated trace made
**four** `rt_sigaction` calls where the native one made **five**, with
byte-identical output — the missing one being the `SIGBUS` handler Rust's
standard library skipped after reading its own untouched buffer. `sigaltstack`
one signal along, found the same way, a month later. `rt_sigprocmask` is the
third instance and was fixed pre-emptively rather than after the fact: its
`oldset` is a query, `pthread_create` saves the mask through it and the child
restores what it saved.

**And once with a count instead of a call.** `sigaltstack` was stored *per
process*. Nothing crashed and the output was byte-identical; the trace was
short by two `sigaltstack`s, one `mmap`, one `mprotect` and one `munmap` **per
thread**, because every thread after the first queried, read the first
thread's alternate stack, concluded one was already installed, and silently ran
with no stack-overflow handler. 130 calls against the native 148. An alternate
signal stack is per thread; a signal *disposition* is per process; a signal
*mask* is per thread again. The trace is what tells you which.

With all three fixed, `hello` makes **25** calls under rsemu on both
architectures and 25 on the host (its 27 less `execve` and `arch_prctl`, which
have no level-3 counterpart), with every per-call count equal. The threaded
guest makes **166** against the host's 168; the difference is three `futex`
calls, and it is real rather than a defect — the host runs its threads in
parallel, so its locks are contended more often.

## Threads

`usermode::ThreadSet` is `ROADMAP.md` phase 5b's third deliverable — *"a
scheduling contract for guest threads"* — and until a threaded guest ran on it,
it was a round robin with a snapshot and no user. `tests/usermode/threads.rs`
is that user: four workers hammering one atomic, then three threads on a
condition variable released by one notify. Ordinary `std` Rust, written with no
knowledge of the emulator.

What the consumer had to add, and none of it is rsemu's half:

- **`clone(flags, stack, ptid, tls, ctid)`**, insisting on
  `CLONE_VM|CLONE_THREAD|CLONE_SIGHAND` — a `clone` without them is a
  *process*, and this stand-in has one. The argument order is the one an
  architecture selecting `CONFIG_CLONE_BACKWARDS` gets (RISC-V, AArch64 and
  x86-64 all do), which is **not** the order `clone(2)` documents for the libc
  wrapper; `strace` prints the fields by name, which is how it was settled
  rather than guessed. The child is the parent's registers on a new stack with
  **zero** where the call's result goes, and that zero is the entirety of how
  every libc's `__clone` tells the two apart.
- **`futex` `WAIT` and `WAKE`**, which is all a threaded musl uses. `WAIT`
  compares and blocks in one step because only one guest thread executes at a
  time, so the classic lost-wakeup race is impossible here rather than merely
  unlikely. Waiters are woken in **arrival order**: the kernel promises only
  "some waiter", but a consumer that picked by hash order would have a replay
  that diverges the first time two threads contend.
- **`set_tid_address` and `CLONE_CHILD_CLEARTID`**, which together are the
  whole of `pthread_join`: the joiner blocks on the child's tid word, and the
  exiting thread zeroes it and wakes whoever is there. Forget the wake and
  every thread ends up blocked with no deadline — which `ThreadSet::run_next`
  reports as "nothing is runnable" rather than spinning on, and which is how
  the omission announced itself.
- **`exit` versus `exit_group`.** One thread stops; the process stops when its
  *last* thread does. The main thread is not special — it is the one that has
  nobody left to outlive it.

Two things rsemu's half supplied unchanged, and they are why the above was
short: a thread is an `ExitingCore` and nothing more, and a `Stop` names which
thread produced the exit. Blocking with a deadline is `ThreadSet::block(id,
Some(instant))`, and when nothing is runnable virtual time **jumps** to the
earliest deadline — so `nanosleep` costs no host time and lands on the same
instruction every run.

One design note, because it fell out well. A `FUTEX_WAIT` with a timeout has
two possible answers and they become true at different moments, so they are
written at those moments: `-ETIMEDOUT` goes into the result register when the
thread blocks, and a `FUTEX_WAKE` overwrites it with `0`. A deadline that fires
leaves what was already there. That is what lets `ThreadSet` stay out of it —
the framework never has to say *why* a thread became runnable.

## A ledger line that closed: the exclusive monitor used to be per core

The threaded guest is where this surfaced, and it was the one thing level-3
threading found that was **not** on the consumer's side of §2.1's line.

Both cores kept their reservation in their own execution state — RISC-V's
`reservation`, AArch64's `State::exclusive` — and broke it only on a store
*that core* made. Level-3 threads are one core each over one `UserMemory`, so a
sibling's store did not break this core's reservation: a `sc.d`/`stxr` that the
architecture requires to fail succeeded instead, and the sibling's update was
lost.

Whether a guest *hit* it depended on its compiler rather than on its
architecture, which is why it took both:

| | `fetch_add` compiles to | the threaded guest's counter, before |
| --- | --- | --- |
| `riscv64gc` (has `A`) | one `amoadd.d` | 40000 of 40000 |
| `aarch64` baseline (no `FEAT_LSE`) | an `ldxr`/`stxr` loop | 32038 of 40000 |

`core::space::ExclusiveMonitor` closed it: a reservation table on the
`AddressSpace`, one slot per core, keyed on the guest-**physical** granule and
consulted by every store that reaches `SpaceView::write_span` — which a
`UserMemory` write is, so a syscall that writes into a reserved granule breaks
it too. The core's own field stayed as the *local* monitor and a
store-conditional now needs both to agree. Both columns of that table are
40000 of 40000, and `proof.rs`'s
`a_siblings_store_breaks_this_cores_reservation` — two hand-assembled threads,
one word, one preemption in the wrong place, **on both architectures**, with no
toolchain — asserts the inverse of what it used to.

The knob that used to be the only mitigation is worth remembering rather than
reaching for: lengthening `ThreadSet::set_quantum` made the race arbitrarily
rare and could not make it impossible, and a knob that turns a wrong answer
into a rare wrong answer is not a fix.

What is **not** fixed by this is memory *ordering*. A core here executes one
instruction at a time and completes every access before the next, so a fence is
a no-op and a guest that depends on a weak memory model being weak still has
nothing to disagree with. Atomicity and ordering are different promises.

## The initial stack

A static binary's entry point is handed one thing: a stack pointer. Everything
else it knows it reads from there.

```text
  sp -> argc
        argv[0] .. argv[argc-1], NULL
        envp[0] .. envp[n-1],    NULL
        auxv: (AT_*, value) pairs .., (AT_NULL, 0)
        (gap)
        the argv and envp strings, and AT_RANDOM's sixteen bytes
  top ->
```

`sp` is 16-byte aligned, and `_start` does not re-establish that — the psABI
requires the *caller* to have done it, and the caller here is us.

The auxiliary vector is where the mistakes are. A static binary has no dynamic
loader to tell it anything, so `AT_PHDR`, `AT_PHENT` and `AT_PHNUM` are how it
finds its own program headers, and it needs them to locate `PT_TLS` and set up
thread-local storage before `main`. **`AT_PHDR` is a guest address, not a file
offset**: it is derived from the `PT_LOAD` segment whose file range covers
`e_phoff`, which is why a linker puts the program header table inside the first
loadable segment. Get it wrong and the guest starts and immediately faults,
which is the single most common symptom of a malformed auxv.

The others that matter: `AT_PAGESZ` (a libc that gets this wrong will `mmap`
wrongly), `AT_ENTRY`, `AT_RANDOM` (sixteen bytes for stack-protector and hash
seeding — see below), `AT_HWCAP`, and `AT_SECURE`.

## Loading

Three things are easy to get wrong and are worth stating:

- **`p_memsz` beyond `p_filesz` is `.bss` and must be zeroed.** A static
  binary's uninitialised globals live there and it never writes them first.
- **Segments are page-granular and may share a page.** A linker is entitled to
  end a read-only segment and begin a writable one inside one page. Mapping
  each segment separately makes the second erase the first, so the map is built
  from the *union* of the segments' page ranges, filled, and only then given
  the *union* of their permissions.
- **A position-independent executable is a different problem.** `ET_DYN` needs
  relocation processing, and `PT_INTERP` needs a dynamic loader run first. Both
  are an operating system's job (§2.1) and both are refused with a message that
  says so rather than half-loaded.

## The host-filesystem policy

Decided **before** `openat` was written, which is the only time this decision
can be made honestly.

> **A level-3 guest may be told about itself. It may not be told about the
> host.**

Concretely, in `src/usermode/proof.rs`:

- Every path-taking call — `openat`, `faccessat`, `readlinkat`, `newfstatat` —
  answers `-ENOENT` **without looking at the path**. The guest sees an empty
  namespace, which is a coherent thing for a filesystem to be, rather than a
  permission error that invites a retry.
- The one exception is not an exception to the rule above: `/proc/self/maps` is
  served from `UserMemory::mappings()`, which is the guest's own address space
  and consults no host. (`usermode::mem`'s own documentation names
  `/proc/self/maps` as the first reason that bookkeeping is an ordered list.)
- Descriptors 0, 1 and 2 exist and are backed by memory the harness owns.
  `read` from 0 is a clean end of file; `write` to 1 or 2 appends to a buffer;
  every other descriptor number is `-EBADF`.
- `mmap` is anonymous-only. A file-backed mapping is `-ENODEV`, which follows
  from the above rather than being a second rule: there is no descriptor for a
  host file to map.
- There is **no flag to widen this**. The moment there is one, the module stops
  being a proof that the seam works and becomes a sandbox with a policy to get
  wrong.

Why so strict: the appeal of level 3 (§2, *"run this program somewhere it
cannot hurt me"*) is gone the instant `openat("/etc/shadow")` can be answered,
and "which paths are safe" is not a question with a checkable answer.
"Everything on this answer came from the guest's own memory" is.

A real consumer will need passthrough — `npm install` reads files — and will
have to design it. §2.1 already says that design is nixvm's, and nothing in
this repository should pre-empt it with a half-policy.

**Neither of the two things that landed after this policy was written needed it
widened.** A second architecture reads its own `AT_HWCAP` and its own `uname`,
both of which describe the emulated core rather than the host. A threaded guest
maps its stacks anonymously, joins through a futex word in its own memory, and
never opens anything. The policy has cost nothing so far, which is the argument
for leaving it where it is.

## Determinism: where non-determinism actually enters

`ROADMAP.md` §0 requires every non-deterministic input crossing into the machine
to go through the record/replay seam. At level 3, almost nothing qualifies, and
that is by construction rather than by luck:

| | why it is already deterministic |
| --- | --- |
| `clock_gettime`, `nanosleep` | `usermode::GuestClock` advances by executed ticks, and a sleep is a jump to a virtual deadline |
| thread interleaving | `usermode::ThreadSet` preempts on a tick quantum, not a wall-clock one, and scans in id order out of a `BTreeMap` |
| `clone` | the thread id is the scheduler's, and `ThreadSet` never reuses one |
| `futex` wakes | waiters are queued and released in arrival order, not by hash order |
| `mmap` placement | `UserMemory`'s top-down search is a pure function of the map |
| `brk` | the break starts at the image's own end |
| `getpid`, `gettid`, `uname`, `getuid` | constants, or the thread id above |

Threading added the middle three rows, and each was a decision rather than a
discovery: a `BTreeMap` because a hash would iterate in a different order, a
`Vec` per futex word because "wake some waiter" has to mean the *same* waiter
every run, and ids that are never reused because a replay that reuses one
cannot tell two threads apart. All three were checked the cheap way — the
threaded guest replays with its trace, its tick count and its output all
identical, on both architectures, with the entropy source replaced by one that
panics.

What is left is **entropy**, and there are exactly two doors:

1. the sixteen bytes `AT_RANDOM` points at, asked for while the initial stack is
   built — before the guest has executed an instruction, so at virtual instant
   zero; and
2. `getrandom(2)`, asked for at the virtual instant of the `ecall`.

Both go through `Journal::ask`, and both are therefore recorded and replayed. A
replayed run is handed an entropy source that **panics if called**, so "the
journal is the only door" is a property the test suite checks rather than a
claim in a comment.

## Running the milestone

The guest source is `tests/usermode/hello.rs` — ordinary `std` Rust, written
with no knowledge of the emulator. It is *built*, never committed (CLAUDE.md,
Testing: a compiler's output does not belong in the repository any more than a
downloaded ROM does):

```console
$ rustup target add riscv64gc-unknown-linux-musl aarch64-unknown-linux-musl
$ scripts/fetch-testdata.sh usermode-guests
$ cargo test --all-features usermode::proof -- --nocapture
usermode/hello on riscv64: 25 syscall(s), 1 thread(s), 24326 tick(s); refused []
usermode/hello on riscv64: stdout "hello from level 3\nargv = [\"hello\"]\nRSEMU = Some(\"1\")\n"
usermode/hello on aarch64: 25 syscall(s), 1 thread(s), 16592 tick(s); refused []
usermode/hello on aarch64: stdout "hello from level 3\nargv = [\"hello\"]\nRSEMU = Some(\"1\")\n"
usermode/threads on riscv64: 166 syscall(s), 8 thread(s), 486722 tick(s); refused []
usermode/threads on riscv64: stdout "joined [0, 1, 2, 3]\ncounter = 40000\nrendezvous ok\n"
usermode/threads on aarch64: 166 syscall(s), 8 thread(s), 851156 tick(s); refused []
usermode/threads on aarch64: stdout "joined [0, 1, 2, 3]\ncounter = 32038\nrendezvous ok\n"
```

`RSEMU_USERMODE_TRACE=1` adds the whole `(number, result)` list, which is what
the comparison above is made from. `RSEMU_USERMODE_GUEST` overrides the path if
you want to point it at some other static binary. An architecture whose target
is not installed is skipped with a note, and with no fixture at all the test
says how to build one and passes; every other test in the module — the ELF
loader, the auxiliary vector, the policy, the journal, the reservation ledger —
runs unconditionally on synthetic images the test assembles itself, for
**every** architecture in the build, so `cargo test` stays hermetic and offline.

RISC-V was the first architecture because it is the most measured core in the
tree (RV64GC, `riscv-tests` 409/409, and the one that boots Linux), because
`asm-generic` gives it the cleanest syscall ABI of the three 64-bit candidates,
and because the Rust toolchain can produce a static musl binary for it with
nothing vendored. AArch64 is the second for that last reason and because it is
the core that boots Linux on `arm64-virt`. x86-64 is the obvious third and is
the one that would test the seam hardest, because its syscall ABI is *not*
`asm-generic`: different numbers, `rcx` and `r11` clobbered by the instruction
itself, and `arch_prctl` where the other two have a thread-pointer register
user code can write.
