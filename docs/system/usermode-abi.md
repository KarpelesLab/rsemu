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
| the FP unit | `mstatus.FS = Initial` | `CPACR_EL1.FPEN = 0b11` |

Plus the decisions a Linux kernel makes for a process it is about to enter and
that a level-3 consumer therefore has to make itself.

**Turning the floating-point unit on** is the first, and it is *not*
architecture-specific, though this document said it was for a year. Both
architectures reset the unit to trap and both leave enabling it to the kernel:
AArch64 spells it `CPACR_EL1.FPEN = 0b11`, RISC-V spells it `mstatus.FS =
Initial`, and Volume II is explicit that with `FS` off *every* FP instruction
is illegal — including `fsd`, which does no arithmetic at all. Only the
AArch64 half was written, because it was needed the day that core landed: the
first `stp q0, q1` inside a `memcpy` took an `UNDEFINED`. The RISC-V half was
missing for as long as there was nothing to notice it with. `hello` and
`threads` never execute a floating-point instruction; the first third-party C
program did, six syscalls into its startup, saving `fs0` across a call.
`a_guest_may_use_the_floating_point_unit_from_its_first_instruction` is the
test, and it is one program text assembled for both architectures precisely so
that a decision like this one cannot be made for only one of them again.

**`SCTLR_EL1.M = 0`** is the second and is genuinely AArch64's: level 3's
memory model is "there is no page table" and the map `UserMemory` builds is the
address space the guest sees, so there is nothing for an MMU to translate.

Nothing else moved. **The ELF loader, the initial stack, the auxiliary vector,
the errno values, the host-filesystem policy and the journal are byte-identical
between the two**, and the first AArch64 run of the same `hello` binary made
the same twenty-five calls in the same order as the RISC-V one and refused none
of them. The claim that the seam is not RISC-V-shaped is now measured rather
than asserted.

**One thing that was on that list has come off it, and it took a real C library
to find:** the syscall *numbers* are not entirely shared. `asm-generic`'s header
reserves 244..259 "for architecture specific syscalls", and RISC-V spends two of
them where AArch64 spends none — `__NR_riscv_hwprobe` is 258. glibc's RISC-V
`ld.so` calls it before it picks an ifunc for `memcpy`, so a consumer that
implements "the `asm-generic` table" and stops has a hole on exactly one of its
two architectures. It is answered `-ENOSYS`, which is what a kernel older than
6.4 says, and glibc falls back to `AT_HWCAP` — the description this consumer
already gives honestly, so the fallback is the *accurate* path rather than a
degraded one. The number is matched with a guard on `e_machine` rather than
unconditionally, because on AArch64 258 is unassigned and a program asking for
it should be told so.

### `AT_HWCAP` is a promise, and now a measurable one

`AT_HWCAP` is a promise rather than a description, and until recently that was
all it was: a number a test asserted. `HWCAP_ATOMICS` was deliberately
**absent** on AArch64 because `Config::cortex_a53` has no `FEAT_LSE`,
compiler-rt's out-of-line atomics read that bit to choose between `casal` and
an `ldxr`/`stxr` loop, and a part that claimed the bit would take an
`UNDEFINED` on the first atomic a threaded guest executed.

That is still true of `ARCH`. What is new is `ARCH_LSE`, the same architecture
on a `Config::neoverse_n1` — Armv8.2-A, where `FEAT_LSE` is mandatory — which
sets bit 8 and is the first place a level-3 guest is told something that
changes **what it executes** rather than what it prints.

The measurement is one binary on two parts. `threads-aarch64` is built for
`aarch64-unknown-linux-musl`, whose baseline is Armv8.0, so it cannot contain
an inline `LDADD` — but it contains fourteen LSE words anyway, in compiler-rt's
out-of-line atomics, behind a runtime branch on `__aarch64_have_lse_atomics`
that is initialised from bit 8 of `AT_HWCAP`:

```
usermode/threads on aarch64:     166 syscall(s), 8 thread(s), 851702 tick(s)
usermode/threads on aarch64+lse: 166 syscall(s), 8 thread(s), 690851 tick(s)
    stdout "joined [0, 1, 2, 3]\ncounter = 40000\nrendezvous ok\n"  (both)
```

Same file, same syscalls, same answer, **19% fewer instructions** — because on
the second run the branch went the other way and forty thousand `fetch_add`s
were one `LDADD` each instead of an `ldxr`/`stxr` loop that retries under
contention. `an_lse_part_says_so_in_at_hwcap_and_the_guest_changes_what_it_executes`
asserts the outputs are equal and the tick counts are *not*, which is how it
knows the guest actually read the bit rather than the test measuring nothing.

The second guest is `threads-lse-aarch64`: the same `tests/usermode/threads.rs`
built `-C target-feature=+lse`, so the atomics are inline in the guest's own
text rather than behind compiler-rt's dispatch — ninety-eight LSE words
including `casb` and `swpb`, the byte forms an Armv8.0 `.text` never contains
at all. It runs on the Neoverse part and gives the same answer in fewer ticks
still (165 syscalls, 410199 ticks), and on the Cortex-A53 it must **not** run:

```
usermode/threads-lse on aarch64 without FEAT_LSE: thread 1 at pc 0x228654
  executed an instruction this core does not implement, encoded 0xf8280008
```

That negative is the half that keeps the feature lattice honest. `FEAT_LSE`'s
encodings are `UNDEFINED` on a part without it — that is *how* a guest probes
for the feature — so a core that decoded them anyway would make
`Config::cortex_a53` a claim no other test in this file could catch.

A **whole glibc** gets the same bit, which is the version of this experiment
worth the most: glibc's AArch64 `init_cpu_features` keys its ifunc resolvers
off `AT_HWCAP`, so setting bit 8 is not a local change to one dispatch variable
but an input to an entire library's idea of what part it is on. Four of the six
defects this module has found came from handing glibc something it had not been
handed before. This one broke nothing:

```
usermode/glibc-threads on aarch64:     205 syscall(s), 8 thread(s), 1150910 tick(s)
usermode/glibc-threads on aarch64+lse: 205 syscall(s), 8 thread(s),  988478 tick(s)
```

The **same 205 calls in the same order**, the same output, 14% fewer
instructions. That the syscall trace is unchanged is the interesting half: an
ifunc resolver picking a different `memcpy` is not supposed to be visible to a
kernel, and here it demonstrably is not.

RISC-V has no counterpart and needs none: `A` is not optional in RV64GC, so
there is no second encoding for a guest to select between.

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
- **A position-independent executable is loaded at a base the loader chose**,
  and every `p_vaddr` in it is an offset from that base rather than an address.
  That is the whole difference between `ET_EXEC` and `ET_DYN`, and it is worth
  applying *once*, on the way in, so that the mapping, the fill, the
  permissions, `AT_PHDR` and the break are all written in guest addresses and
  cannot forget it. Both additions are checked: a `p_vaddr` near the top of the
  address space plus a base is a wrap the static case could not produce.

## Dynamic linking

`ET_DYN` and `PT_INTERP` used to be refused with a message saying an
operating system's loader is an operating system's job. They are supported
now, and nothing about §2.1's line moved: the operating system is still the
consumer's, and what follows is that consumer doing three things.

**One.** The executable is placed at a fixed base, and so is the interpreter.
Linux picks both with ASLR and a level-3 run must not, so they are constants —
a base that is a function of *nothing at all* is the only kind that replays.
The interpreter goes below the executable so that the break, which starts
above the executable and grows up, cannot walk into it.

**Two.** `PT_INTERP` is a *name*, and the named file is loaded **as well**,
at its own base, and the process is entered at the interpreter's entry point
rather than the executable's. The interpreter is entered on the executable's
stack, not one of its own.

**Three.** The auxiliary vector describes each to the other, and the
asymmetry is the entire mechanism:

| | describes | why the other one cannot supply it |
| --- | --- | --- |
| `AT_BASE` | the **interpreter** | it is entered with no relocations applied, so this is the only address it knows about itself |
| `AT_PHDR`, `AT_PHENT`, `AT_PHNUM` | the **executable** | the loader has to find `DT_DYNAMIC`, and it gets there through the program headers |
| `AT_ENTRY` | the **executable** | where the interpreter jumps when it has finished |

**What does not happen here is relocation processing**, and that is the point
rather than an omission. rsemu's consumer does not need a relocation
processor; it needs the auxiliary vector to be right. A dynamic loader that
starts and immediately faults is almost always a malformed auxv — which was
already true of the static case, one layer down.

`AT_BASE` is emitted whether or not there is an interpreter, and is zero when
there is not, which is what Linux does and what `getauxval(AT_BASE)` therefore
returns for a static binary.

### The hostile cases multiply, and they are the interesting half

A loader that takes an interpreter is a loader that follows a pointer out of a
file into another file. Each of these is refused rather than half-done, and
each has a test:

| | |
| --- | --- |
| `PT_INTERP` with `p_filesz` of zero, or above `PATH_MAX` | a loader that reads as much as it is told to is a loader an image can make read anything |
| `PT_INTERP` with no NUL in its payload | there is no path there |
| `PT_INTERP` whose payload runs off the end of the file | the ordinary truncation case, one indirection along |
| two `PT_INTERP` segments | a process has one interpreter |
| an interpreter with a `PT_INTERP` of its own | that recursion has no end |
| an interpreter for another architecture, or that is not an ELF file | the same checks the executable gets, because it is a program too |
| `p_vaddr` plus the load bias wrapping the address space | the new arithmetic, and the new place to get it wrong |
| an interpreter that was not staged | the *policy* answering, not the loader — see below |

`ET_DYN` was previously in that list and has left it; `ET_CORE` and everything
else is still refused.

### What it runs

`tests/usermode/dynamic/` is an `ET_DYN` executable with a `PT_INTERP`, one
`DT_NEEDED`, one data relocation (`R_AARCH64_GLOB_DAT`) and one function
relocation (`R_AARCH64_JUMP_SLOT`), linked against a shared object built from
`lib.rs` beside it. Both halves are `#![no_std]` and link no libc, which is
deliberate: the loader is what is under test, and a libc between it and
`_start` only adds four hundred instructions to whatever goes wrong. The
string it prints lives in the *shared object* and the length comes from a
function there, so nothing appears on standard output unless both relocations
were resolved by somebody — and that somebody is a real `ld-linux-*.so.1`,
copied into the git-ignored corpus by the fetch script.

It runs on **both** architectures:

```console
usermode/dynamic on aarch64: 22 syscall(s), 1 thread(s), 16733 tick(s); refused []
usermode/dynamic on aarch64: stdout "hello from a shared obj\n"
usermode/dynamic on riscv64: 18 syscall(s), 1 thread(s), 24150 tick(s); refused []
usermode/dynamic on riscv64: stdout "hello from a shared obj\n"
```

RISC-V went untested for a while for a reason that had nothing to do with the
loader: **no compiler can produce an `ld.so`**, and while a Linux host usually
has an `aarch64` cross sysroot lying about, it rarely has a RISC-V one. The
fetch script now falls back to a pinned Debian `libc6-riscv64-cross` package
— about a megabyte, architecture-independent, fetched into the corpus with a
checksum and never committed — so the answer to "is the loader AArch64-shaped?"
stopped depending on what the host happened to have installed. It is not: the
same code placed both, and **the RISC-V run makes four fewer calls**, which is
the one difference and is `p_align`. AArch64 objects align to 64 KiB, so glibc
over-allocates the span and trims it with two `munmap`s; RISC-V's align to the
page size and there is nothing to trim.

The same program built for `x86_64-unknown-linux-gnu` and run under `strace`
makes 26, and every difference is accounted for:

```text
host (x86-64 glibc)                    rsemu (aarch64 glibc ld.so)
  execve                                 —          (no level-3 counterpart)
  brk                                    brk
  mmap                                   —
  access(/etc/ld.so.preload)             faccessat(/etc/ld.so.preload) = -ENOENT
  openat ×3, newfstatat ×3               —          (the hwcaps search; see below)
  openat(libgreet.so); read; fstat       openat(libgreet.so); read; fstat
  mmap ×3                                mmap ×2; munmap ×2; mprotect; mmap ×2
  close                                  close; mmap
  arch_prctl                             —          (no level-3 counterpart)
  set_tid_address; set_robust_list; rseq  set_tid_address; set_robust_list; rseq
  mprotect ×3                            mprotect ×3
  write(1, …); exit_group                write(1, …); exit_group
```

The two extra `mmap`s and the two `munmap`s are the AArch64 objects' `p_align`
of 64 KiB: glibc over-allocates and trims to the alignment it needs, and an
x86-64 object aligned to the page size needs no trimming. **The missing hwcaps
search is a choice**: glibc probes `glibc-hwcaps/` subdirectories only when
`AT_HWCAP2` and the platform strings give it something to probe *with*, and
this auxiliary vector supplies neither — RISC-V's Linux does not set
`AT_HWCAP2` and arm64's does, and one auxv shared by both architectures is
worth more here than four probe syscalls. There is no vDSO either
(`AT_SYSINFO_EHDR` is absent), because a vDSO is *guest code* and level 3 has
no kernel to have supplied it.

### `PROT_EXEC` is enforced, and here is where that lands

`Perms::EXEC` is carried through `mmap`, `mprotect` and `/proc/self/maps`, and
**it is now enforced**: an instruction fetch from a page that does not permit
execution is refused by the mapping, and the guest takes an architectural
fault at the instruction that could not be fetched.

This section used to be a request rather than a description — it recorded a
gap, argued that closing it belonged in `core::space` and `cpu/` rather than
here, and wrote out the shape of the change. The change landed close to that
shape, with one deliberate difference, and the record of what it cost is worth
more than the request was.

#### The mechanism

Three pieces, and only the first is new machinery:

1. **`MemAttrs::purpose`**, an `AccessPurpose` — a `#[repr(transparent)]`
   newtype with `DATA` and `FETCH` constants, not the `fetch: bool` this
   section originally asked for. Direction is not in it: an access reaches a
   region through `read` or `write`, so a `DataRead`/`DataWrite`/`Fetch`
   enumeration would restate the direction and make a write carrying
   `DataRead` expressible. A purpose grows where a bool cannot — a hardware
   page-table walk and a cache-maintenance operation are separate rows in
   AArch64's `ESR` and x86's page-fault error code, and both are reads a
   region may legitimately answer differently. `DATA` is zero, so
   `MemAttrs::DEFAULT` and `MemAttrs::DEBUG` mean what they always did.
2. **Each core's fetch path setting it**, in the one place a core already
   knows which it is doing.
3. **`FlatLeaf::read` asking for the right permission**, through
   `MemAttrs::read_perm`: `Perms::EXEC` for a fetch, `Perms::READ` otherwise.

Two departures from the sketch above, both deliberate:

- **`EXEC`, not `RX`.** An execute-only mapping is a real thing — AArch64
  permits `--x` at EL0, and `mprotect(PROT_EXEC)` asks for exactly it — so a
  fetch is checked for `EXEC` *instead of* `READ` rather than for both. A text
  segment is `r-x`, which contains `EXEC`, so the ordinary case is identical
  either way and only the exotic one differs.
- **A debug access is always a data read.** `read_perm` will not ask for
  `EXEC` when `MemAttrs::debug` is set, so a monitor disassembling a
  non-executable range still gets the bytes. That is the opposite of the write
  side, where a refused write *prevents* a side effect and is enforced against
  a debugger too (`ROADMAP.md` §15, invariant 5).

#### Which cores mark a fetch

A refusal is only worth raising by a master that can deliver it to the guest,
so the boundary is drawn at the fault path rather than at convenience:

| Core | Marks a fetch | Where a refusal lands |
| --- | --- | --- |
| `cpu-arm-a64` | yes | instruction abort |
| `cpu-arm` (A-profile) | yes | prefetch abort, external fault |
| `cpu-arm-v7m` | yes | `BusFault`, escalating to `HardFault` |
| `cpu-mips` | yes | `IBE`, bus error on an instruction fetch |
| `cpu-riscv` | yes | instruction access fault, cause 1 |
| `cpu-x86` | **no** | nowhere. An x86 has no bus-error input, so `Exec::phys_read` turns a refused access into open bus and a counter; a refused fetch would become `0xff` bytes in the instruction stream rather than a fault. Execute permission on x86 is `NX` in the page tables, which `src/cpu/x86/paging.rs` already consults on a fetch and only on a fetch. |
| 6502, Z80, SM83, m68k | **no** | nowhere, and nothing asks. No MMU, no execute permission, open bus on a refusal, and boards that use `Perms` for ROM write protection only. |

Both architectures this consumer runs are in the first group, which is why the
level-3 story is complete even though the table is not.

#### What it is measured by

`usermode::proof::a_fetch_from_a_mapping_that_forbids_execution_is_refused`
runs both shapes, on both architectures, and both are now refused:

- an ELF image whose only `PT_LOAD` is `rw-` — the same `hello` file every
  other loader test uses with one `p_flags` word changed — loads, is recorded
  in `mappings()` as `rw-`, and faults on its **first fetch**, before a single
  syscall retires. That is what Linux does with that image;
- a guest that calls `mprotect(text, PAGE, PROT_READ)` on the page it is being
  fetched out of faults on the **next instruction**, with the `mprotect`
  itself having returned 0. That is `ld.so`'s RELRO step aimed at the wrong
  range, caught in eleven instructions.

The test asserts the syscall count at the fault in both halves, so "it stopped"
and "it stopped in the right place" are separate claims.

`core::space`'s own
`a_fetch_from_a_mapping_that_forbids_execution_is_refused_and_a_load_is_not`
is the other half, and the second clause in its name is the load-bearing one: a
check on the read path is one line away from refusing *every* read, so a test
that a plain load of the same bytes still succeeds is what distinguishes
enforced from broken.

#### Two things that were already in place

Neither had to be built, which is most of why the change was small:

- **Stale translations are already dropped.** `UserMemory::protect` reaches
  `AddressSpace::topology().reprotect`, and `core::space`'s
  `reprotect_changes_the_terms_and_bumps_the_generation` asserts a permission
  change is a retopology. Every cache keyed on the generation — the flat view,
  a JIT's block cache — is invalidated by an `mprotect`, so a block lifted out
  of a page that then stops being executable cannot survive it. The JIT's
  software TLB was updated in the same change to ask for `EXEC` on a fetch
  fill, so its fast path cannot disagree with the slow one.
- **The error is already the right one.** `BusError::Protected` is what a
  copy-on-write fault raises, and this consumer's fault handler already tells
  a `Protected` it can resolve from one it cannot
  (`UserMemory::resolve_write_fault` returning `Ok(false)`).

#### Three limitations, recorded rather than discovered later

- **A shared page is the union of its segments' permissions.** A mapping is
  `rwx` when two segments share a page and one asked for `w` and the other for
  `x` (`two_segments_sharing_a_page_get_the_union_of_their_permissions` is that
  case, and a real linker produces it). A union is the right answer for a
  page-granular map — it is what Linux's own `load_elf_binary` does with the
  same congruent segments — but it means enforcement does **not** catch a
  `W^X` mistake inside a shared page.
- **An execute-only mapping under a readable one loses.** The flattener picks
  a read winner with `Perms::READ`, so an `--x` mapping stacked beneath a
  higher-priority readable mapping does not answer the fetch, and the fetch
  then fails on the readable one. Resolving fetches separately would need a
  third winner scan and a third leaf per flat entry. Measured rather than
  inherited: +2.2% of a four-byte read, +0.62% of an `nes-ntsc` run, +1.65% of
  a `riscv-virt` one, and +5.8% if resolved lazily after a refusal. It was
  declined on the modelling rather than the number — there is no `/FETCH` pin,
  and a fetch that fell past a PCI BAR landed over RAM would execute the RAM
  underneath. `core::space::flat`'s *Two winners, and why there is no third*
  has the argument, pinned by a test.
- **Both the interpreters and the translating engines enforce it.** A `jit`
  build once admitted a block on its MMU translation alone and lifted it
  through a `MemAttrs::DEBUG` read, so a translated block executed out of a
  mapping the interpreter's fetch would refuse. `jit::executable_run` closes
  that at the lift, per instruction word rather than per page — a flat entry is
  not a page, and the interpreter aborts at the boundary between an executable
  mapping and a non-executable one. This consumer is unaffected — level 3
  runs `Cpu`, the interpreter — but it is an interpreter/engine divergence, and
  the interpreter is the oracle. The cheap place to close it is the lift: a
  permission change is a retopology, so the generation bump already drops every
  block lifted under the old terms.


### A whole C library, on both architectures

The same experiment with a real glibc in it. `tests/usermode/hello.rs` — the
static milestone guest, unchanged — linked against a C library instead of
statically against musl, and run under that library's own `ld.so`. Everything
the shared-object guest above does happens here too and then keeps going: the
loader opens each library by path out of the stage, maps its segments from a
descriptor, trims them to their alignment, applies every relocation, resolves
the ifuncs glibc picks its `memcpy` and `strlen` with, transfers control, and
runs a C library's entire startup — TLS, the stack guard, the standard streams
— before the program's own first line.

```console
usermode/glibc ["glibc"] on riscv64: 62 syscall(s), 506038 tick(s); refused []
usermode/glibc on riscv64: stdout "hello from level 3\nargv = [\"glibc\"]\nRSEMU = Some(\"1\")\n"
usermode/glibc ["glibc"] on aarch64: 59 syscall(s), 286409 tick(s); refused []
usermode/glibc on aarch64: stdout "hello from level 3\nargv = [\"glibc\"]\nRSEMU = Some(\"1\")\n"
```

The two runs are not the same shape, which is why both are run:

| | AArch64 | RISC-V |
| --- | --- | --- |
| objects the loader places | 2 | 4 |
| syscalls | 59 | 62 |
| segment trimming (`p_align`) | 64 KiB, so `munmap` ×4 | page-sized, so none |

The RISC-V guest is linked against a **pre-2.34** glibc, so `libpthread`,
`libdl` and `librt` are still `DT_NEEDED`s of their own rather than having been
merged into `libc.so.6`. Four images to open, place, relocate and order instead
of two — a harder exercise for the loader, and what a great deal of shipped
software still looks like.

**This section used to describe a stop**, and the history is worth keeping
because it is what the arrangement is for. Until the A64 core grew the
halving-narrow three-different group, this guest got forty-two syscalls in —
refusing nothing, every library opened, every segment mapped, every relocation
applied — and then stopped at `strlen+0x68` on `ADDHN v2.8b, v1.8h, v1.8h`,
which `src/cpu/arm/a64/simd.rs` listed under *"what is deliberately absent"*.
The test asserted the half that was this layer's and ledgered the half that was
not, and said in as many words that the day the core gained the group it would
start asserting the program's output instead. It does.

That ledger has **not** been removed, only emptied, and the reason is the one
thing that makes this guest different from every other one here: nobody chose
its contents. It is whichever glibc the host had. So a stop inside an object
the loader placed, on an instruction `src/cpu/` does not implement, is still
reported and skipped rather than failed — through the same runner the
third-party corpus uses, for the same reason. Every other failure is this
module's and fails.

#### With four threads in it

`tests/usermode/threads.rs` against the same C library, in the same namespace,
because a libc's threading is the part of it least like any other libc's:

```console
usermode/glibc-threads ["glibc-threads"] on riscv64: 204 syscall(s), 1005849 tick(s); refused []
usermode/glibc-threads on riscv64: stdout "joined [0, 1, 2, 3]\ncounter = 40000\nrendezvous ok\n"
usermode/glibc-threads ["glibc-threads"] on aarch64: 205 syscall(s), 1150910 tick(s); refused []
usermode/glibc-threads on aarch64: stdout "joined [0, 1, 2, 3]\ncounter = 40000\nrendezvous ok\n"
```

Against the same program built for `x86_64-unknown-linux-gnu` and run under
`strace`, **every thread-related count is equal**:

| | host | rsemu |
| --- | --- | --- |
| `clone3` | 7 | 7 |
| `exit` | 7 | 7 |
| `set_robust_list` | 8 | 8 |
| `sched_getaffinity` | 8 | 8 |
| `gettid` | 8 | 8 |
| `futex` | 8 | 8 |
| `rt_sigprocmask` | 29 | 29 |
| `sigaltstack` | 24 | 24 |
| `rt_sigaction` | 6 | 6 |

That is a stronger result than the musl guest's 166-against-168, and the
difference is the point: musl's threads contend a lock the host contends more
often because it really is parallel, and this program's do not. Everything left
over is in the loader — the host consults `/etc/ld.so.cache` and this run does
not, because `LD_LIBRARY_PATH` finds the library first — plus `execve` and
`arch_prctl`, which have no level-3 counterpart.

#### `clone3`, which the trace found and the output did not

The first threaded glibc run printed `counter = 40000` and refused syscall
**435**. glibc's `pthread_create` asks for `clone3` *first* and falls back to
the five-register `clone` on `-ENOSYS`; musl never asks at all. So the answer
was already right, every thread ran, and the only thing that said anything was
the refusal list — the same shape as `sigaltstack` and `st_ino` before it,
one layer up.

It is implemented rather than answered `-ENOSYS`, which is a choice worth
stating. A fallback that always fires is a path never tested, and `clone3` is
the only call in this table whose **arguments are in guest memory** rather than
in registers: a level-3 kernel has to read a `struct clone_args` out of the
address space it is servicing before it can act. Two things move in that
translation and both are places to be wrong — `clone` is handed the stack's
*top* and `clone3` its bottom plus a length, and `CLONE_ARGS_SIZE_VER0` is a
version rather than a length, so a shorter struct is `-EINVAL` and a longer one
is read as far as is understood. The legacy form stays exercised by every musl
guest, so implementing this covers a path rather than replacing one.

## Third-party software, which is the only witness that counts

Every guest above is ours. `tests/usermode/hello.rs`, `threads.rs` and
`dynamic/` were written in the same repository as the harness that runs them,
and a guest written here can only ask for what somebody here thought to
implement. §2.1's stated purpose for this exercise is to *"find every place
that surface is not actually usable"*, and the only reliable way to do that is
to run software aimed at Linux rather than at rsemu.

Three programs, chosen for what they ask of the ABI rather than for fame, all
permissive, all fetched at a pinned version, cross-built unmodified by
`scripts/fetch-testdata.sh usermode-guests` and never committed:

| | | what it asks for |
| --- | --- | --- |
| **SQLite 3.45** | public domain | opens files, seeks, locks them, reads pages at absolute offsets |
| **Lua 5.4.7** | MIT | floating point, a garbage collector, string patterns, coroutines |
| **sbase** | MIT | coreutils, so the answers can be diffed against the host's own |

```console
usermode/sqlite ["sqlite3", "/work/demo.db", ".read /work/query.sql"] on riscv64: 93 syscall(s), 755325 tick(s); refused []
usermode/sqlite on riscv64: stdout "osaka\nkyoto\nnara\n4509538\n2870\n"
usermode/sqlite ["sqlite3", "/work/demo.db", ".read /work/query.sql"] on aarch64: 93 syscall(s), 491827 tick(s); refused []
usermode/lua ["lua", "/work/bench.lua"] on riscv64: 71 syscall(s), 22964816 tick(s); refused []
usermode/lua on riscv64: stdout "primes below 20000: 2262\nsquares: 1,4,9,16,25\n…\nfloat: 4.442883\nversion: Lua 5.4\n"
usermode/sbase ["sha256sum", "/work/poem.txt"] on riscv64: 10 syscall(s), 76918 tick(s); refused []
usermode/sbase on riscv64: stdout "5247febdfa80a88b0bbad97e0b370c8e97d954f8faa0eb75459e25cfad0febbb  /work/poem.txt\n"
```

SQLite's two architectures make the **same ninety-three calls in the same
order** and print the same rows. sbase's digest, `wc` counts and `cksum` are
compared against the *host's* `sha256sum`, `wc` and `cksum` over the same
bytes, which is what makes that test different in kind from the other two: a
core that computed something plausible would still fail it.

### What they found

Four holes, all of them in the *consumer's* half — none needed anything from
rsemu and none needed the sandbox widened.

| | how it announced itself |
| --- | --- |
| **`mstatus.FS` was Off** | an illegal instruction on a `fsd` six syscalls in. See "what the second architecture cost" above: the decision was made for AArch64 and not for RISC-V, and no guest of ours had ever touched a floating-point register |
| **no `readv`** | musl's `__stdio_read` fills the `FILE`'s own buffer and the caller's in **one** call, so every C program that reads a file through stdio needs it — and no Rust guest did, because `File::read` is a plain `read`. The native trace and the emulated one now both read `readv(3, [15, 1024], 2) = 1039` |
| **no `pread64`** | SQLite reads the hundred-byte database header at offset zero while a `read` cursor is elsewhere in the same descriptor. Implementing it as a seek and a read would pass every test but corrupt the other cursor |
| **no `fcntl`** | SQLite takes a shared lock before reading and turns `-ENOSYS` there into `disk I/O error`. The locks are *granted*, and that is honest rather than lax: one process, no host file underneath, so nothing can conflict and `F_GETLK` genuinely has `F_UNLCK` to report |

Two more came from the C library above rather than from these three, and both
are written up there: **`riscv_hwprobe`**, which is the one syscall number the
two architectures do not share, and **`clone3`**, which glibc's
`pthread_create` reaches for before it falls back to `clone`.

And one that the differential trace found rather than a crash — the same shape
this document keeps returning to, a field nobody was reading until somebody
did. `openat` **ignored its flags**. A guest asking for `O_RDWR | O_CREAT` got a read-only
descriptor and was told nothing, and would find out at its first `write`, from
an `-EBADF` that says the descriptor is invalid rather than that the file
cannot be written. The native trace opens the database `O_RDWR|O_CREAT` and
falls back to read-only; the emulated one did not, because it was never
refused. It is `-EROFS` now — the namespace describing itself — and the
emulated trace has the same `open`-refused-`open` pair the host's does.

### The aarch64 ledger, which is now empty

For a while this section was a list that grew. Three third-party programs
reached three different Advanced SIMD groups that `src/cpu/arm/a64/simd.rs`
listed under *"what is deliberately absent"*, each stopping a program that ran
to completion on RISC-V:

| | stopped at | encoding | closed |
| --- | --- | --- | --- |
| glibc's `strlen` | `ADDHN v2.8b, v1.8h, v1.8h` | `0x0e214022` | yes |
| Lua's number conversion | `SCVTF d0, d0` (the scalar **SIMD** form, not `SCVTF Dd, Xn`) | `0x5e61d800` | yes |
| sbase's `sha256sum` | `SHLL v18.4s, v4.4h, #16` | `0x2e613892` | yes |

**All three are implemented and every guest here runs on both architectures.**
No run in this document is ledgered any more; the ledger branches are still in
the tests because the next program to be tried is the one that finds the fourth
group, not because any of them fires today.

The shape is worth keeping even so. This was never "aarch64 does not work" —
every other applet of the same sbase binary ran, and so did the whole of
SQLite. It is that a compiler auto-vectorising an ordinary loop reaches one of
these groups often enough that the third program tried hit a third one, and
that a program nobody here wrote is the only thing that samples the instruction
set the way real code does. `SCVTF` remains the instructive entry: it was not
on that list at all, because the scalar-SIMD register-to-register conversion is
a different encoding from the scalar floating-point one the core already had.

`Kernel::encoding_at` is why those words are in this table. A `FAULT` whose
`Access` is `None` is not a memory fault — the core reached an instruction and
refused it — so the diagnostic says which of the two it was and prints the
word. That is the difference between a report a CPU maintainer can act on and
an address three people then disassemble by hand, which is what the first two
of these turned into.

### What a stage cannot do, and why that is the right answer

`ls` does not run. A [stage](#the-host-filesystem-policy-and-the-one-time-it-moved)
is a map from guest path to bytes, so `/work/poem.txt` exists and `/work` does
not:

```console
ls: lstat /work: No such file or directory
```

Making it run needs two things that are the same thing: `getdents64`, and a
notion of a directory — which means a prefix relation over the keys, which is
the *search rule* the policy exists for not having. A program that is **told**
a path runs here; a program that **discovers** paths does not. That line is
almost exactly §2.1's, and the discovery half is what genuine passthrough is
for: `npm install` reads directories nobody told it about, and that design is
nixvm's.

So this is written down rather than fixed. The stage is not a filesystem and
gets no closer to being one by growing the two calls that would make it look
like a small one.

## The host-filesystem policy, and the one time it moved

Decided **before** `openat` was written, which is the only time this decision
can be made honestly:

> **A level-3 guest may be told about itself. It may not be told about the
> host.**

Three things landed after that without needing it widened. A second
architecture reads its own `AT_HWCAP` and its own `uname`, both of which
describe the emulated core rather than the host. A threaded guest maps its
stacks anonymously, joins through a futex word in its own memory, and never
opens anything. A position-independent executable is placed by the loader and
asks nobody.

**Dynamic linking is the thing that could not be done under it.** An
interpreter opens `libc.so.6` *by path*; "there is no such file" is a coherent
namespace and it is one in which no ordinary program on any real system runs
at all. So the rule now reads:

> **A level-3 guest may be told about itself, and about what it was handed. It
> may not be told about the host.**

### What "what it was handed" means

A **stage**: a map from guest path to bytes, fixed before the guest executes
its first instruction and never added to while it runs. It is an argument to
the run, as reviewable as `argv` is.

- `openat`, `faccessat` and `newfstatat` all resolve through **one map and one
  function**, so there is a single place in the module where a name becomes
  content. A miss is `-ENOENT`.
- There is no prefix, no root, no search rule and no normalisation: a path is
  a key. `/lib//libgreet.so` and `/lib/../lib/libgreet.so` do not exist, and
  neither does `/`. A test asserts each of those, because the *absence* of a
  resolution algorithm is the property, and an algorithm is what would have to
  be got right.
- The generated name, `/proc/self/maps`, is unchanged and is the same shape:
  rendered from `UserMemory::mappings()`, consulting no host.
- `mmap` of a descriptor is served by copying out of the same bytes `read`
  would have returned — that is what `MAP_PRIVATE` means, and every mapping
  here is private, so a copy-on-write nobody shares is a copy. `MAP_SHARED` of
  a file is `-ENODEV`: a store has to go somewhere.
- **No descriptor can be written**, so a guest cannot change what the next
  thing to open a name will see. The stage is immutable from inside, and
  `openat` **says so**: `O_WRONLY`, `O_RDWR`, `O_CREAT` and `O_TRUNC` are
  refused with `-EROFS` rather than quietly handed a read-only descriptor. The
  flags used to be ignored, which was the same defect shape as `sigaltstack`
  and `st_ino` one layer along — a real program branches on which answer it
  got, and SQLite's entire read-only mode hangs off exactly this errno.
- `st_ino` is real, and it is the third instance of the defect shape this
  document keeps returning to: a field stubbed to zero, harmless until
  something reads it.
  A dynamic loader identifies an object by `(st_dev, st_ino)` so that a
  library reached under two names is loaded once. A `struct stat` full of
  zeros makes every file in the process the same file: glibc's `ld.so` loaded
  `libgcc_s.so.1`, decided `libc.so.6` was the object it already had, and
  reported `undefined symbol: memcpy` with ten lines about version information
  first. The inode is the path's index in the stage — a function of the stage
  and of nothing on the host.

### What the widening did *not* cost

The property that made the original rule worth holding was never "the guest
cannot open files". It was that the answer is **checkable**, and the checkable
form is now mechanical rather than argued:

> **Nothing that services a syscall links `std`.**

`Kernel` and every function it calls compiles in a build where `std` does not
exist. There is no `open`, no path type, no filesystem, and therefore no code
path from a guest pointer to a host path. CI's **feature-combination** job
builds exactly that on every commit — `cargo test --no-default-features
--features ...,usermode`, derived from the tree by
`scripts/feature-matrix.py` rather than from a list somebody maintains — which
is more than a paragraph can do.
The two places a host file is read are `guest_binary` and `guest_root`, both
`#[cfg(feature = "std")]`, both in the harness, and both finished before a
guest exists — by the time anything is running there is no host path left to
reach.

**There is still no `--allow` flag**, and that is the same decision as before
rather than a survivor of it. A flag makes the *guest's* question decide which
host file is opened, which is precisely the code path this design does not
have. Staging is the opposite shape: the harness decides, up front, in one
place, and what it decided is a value you can print.

Three alternatives were weighed and this is why they lost. *A read-only
directory the harness stages* would put `std::fs` inside the syscall kernel and
give up the mechanical check for nothing — the harness can walk the directory
itself, and does. *A preloaded set of libraries mapped before the guest starts*
would mean rsemu deciding what a `DT_NEEDED` resolves to, which is the
interpreter's job and the reason there is an interpreter. *A path allow-list*
is "which paths are safe" wearing a different hat.

Two later additions were weighed against that sentence and neither moved it.
The **`FEAT_LSE` part** is a construction property of a core: `ARCH_LSE`
differs from `ARCH` by a `Config` and one `AT_HWCAP` bit, and an auxiliary
vector entry is a number the harness computed before the guest existed, not an
answer to a question the guest asked. The **`PROT_EXEC` ledger test** adds no
syscall and no descriptor; both of its guests are images assembled in memory.
The kernel's list of host-reachable functions is still empty, and the
feature-combination job still proves it.

A real consumer will still need genuine passthrough — `npm install` writes
files, and reads directories it was not told about — and §2.1 says that design
is nixvm's. A stage is not that and does not pretend to be; it is the smallest
thing that lets a dynamically linked program run without inventing a filesystem
to get wrong.

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
| where a PIE and an interpreter are loaded | two constants; Linux picks them with ASLR and a level-3 run must not |
| which libraries are opened, and in what order | the interpreter's own logic over a stage that was fixed before it ran |
| a staged file's `st_ino` | its index in the stage, which is a function of the stage |
| `AT_SYSINFO_EHDR` | absent, because there is no vDSO — see the dynamic-linking section |
| `getpid`, `gettid`, `uname`, `getuid` | constants, or the thread id above |

Threading added the `clone`, `futex` and interleaving rows, and each was a
decision rather than a discovery: a `BTreeMap` because a hash would iterate in
a different order, a `Vec` per futex word because "wake some waiter" has to
mean the *same* waiter every run, and ids that are never reused because a
replay that reuses one cannot tell two threads apart. All three were checked
the cheap way — the threaded guest replays with its trace, its tick count and
its output all identical, on both architectures, with the entropy source
replaced by one that panics.

**Dynamic linking added four rows and no new door.** Each was a place a real
kernel is non-deterministic on purpose and this one must not be: Linux chooses
a PIE's base and an interpreter's with ASLR, and its inode numbers come off a
filesystem. Making each a function of the program rather than recording it is
the better answer wherever it is available, because a journal entry is a thing
that can go stale and a constant is not — and it was available for all four.
The dynamically linked guest replays with the entropy source replaced by one
that panics, exactly as the static and threaded ones do, and the replay
consumes its recording exactly — `Journal::remaining() == 0` is asserted for
every built guest, which is *"the journal is the only door"* stated in the
other direction: nothing reached the host, and nothing the host said went
unused.

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
usermode/hello on riscv64: 25 syscall(s), 1 thread(s), 24446 tick(s); refused []
usermode/hello on riscv64: stdout "hello from level 3\nargv = [\"hello\"]\nRSEMU = Some(\"1\")\n"
usermode/hello on aarch64: 25 syscall(s), 1 thread(s), 16734 tick(s); refused []
usermode/hello on aarch64: stdout "hello from level 3\nargv = [\"hello\"]\nRSEMU = Some(\"1\")\n"
usermode/threads on riscv64: 166 syscall(s), 8 thread(s), 487136 tick(s); refused []
usermode/threads on riscv64: stdout "joined [0, 1, 2, 3]\ncounter = 40000\nrendezvous ok\n"
usermode/threads on aarch64: 166 syscall(s), 8 thread(s), 851702 tick(s); refused []
usermode/threads on aarch64: stdout "joined [0, 1, 2, 3]\ncounter = 40000\nrendezvous ok\n"
usermode/dynamic on aarch64: 22 syscall(s), 1 thread(s), 16733 tick(s); refused []
usermode/dynamic on aarch64: stdout "hello from a shared obj\n"
usermode/dynamic on riscv64: 18 syscall(s), 1 thread(s), 24150 tick(s); refused []
usermode/glibc ["glibc"] on aarch64: 59 syscall(s), 286409 tick(s); refused []
usermode/glibc ["glibc"] on riscv64: 62 syscall(s), 506038 tick(s); refused []
usermode/glibc-threads ["glibc-threads"] on aarch64: 205 syscall(s), 1150910 tick(s); refused []
usermode/glibc-threads on aarch64: stdout "joined [0, 1, 2, 3]\ncounter = 40000\nrendezvous ok\n"
usermode/sqlite [...] on riscv64: 93 syscall(s), 755325 tick(s); refused []
usermode/sqlite on riscv64: stdout "osaka\nkyoto\nnara\n4509538\n2870\n"
usermode/lua ["lua", "/work/bench.lua"] on riscv64: 71 syscall(s), 22964816 tick(s); refused []
usermode/sbase ["wc", "/work/poem.txt"] on aarch64: 15 syscall(s), 36723 tick(s); refused []
usermode/threads on aarch64+lse: 166 syscall(s), 8 thread(s), 690851 tick(s); refused []
usermode/threads-lse on aarch64+lse: 165 syscall(s), 8 thread(s), 410199 tick(s); refused []
usermode/glibc-threads on aarch64+lse: 205 syscall(s), 8 thread(s), 988478 tick(s); refused []
```

The tick counts moved by a few hundred against the numbers this document used
to quote, and the reason is the auxiliary vector: it now carries `AT_BASE`,
`AT_FLAGS` and `AT_EXECFN`, which a Linux kernel emits for every process and
this one did not. Nothing about the syscall traces changed.

**The dynamic guests need one thing a compiler cannot produce**, which is a
real dynamic loader. `scripts/fetch-testdata.sh usermode-guests` looks for an
`ld-linux-<arch>.so.1` in the usual cross sysroots, honours
`RSEMU_USERMODE_LDSO`, and failing both fetches a pinned Debian
`libc6-<arch>-cross` package — about a megabyte, architecture-independent,
checksummed — and takes the loader out of that. That fallback is what makes
the RISC-V half reproducible: an `aarch64` cross sysroot is common on a Linux
host and a RISC-V one is not, which is the whole reason dynamic linking went
untested on one of the two architectures rather than both.

The loader, and the `libc.so.6` beside it, are copied into the git-ignored
corpus and run. glibc is LGPL-2.1: running a program is ordinary use
(`CLAUDE.md`, Provenance), shipping one here would not be, and
`PROVENANCE.txt` beside the corpus says which build it was.

**The whole-glibc guests need a link driver** on top of that, because a glibc
executable pulls in `Scrt1.o`, `crti.o` and a `libgcc` and rustc will not
assemble those itself. A cross `gcc` is one; `zig cc` — which the third-party
guests already need — is the other, and it deliberately links against glibc
**2.28**, which predates the 2.34 merge, so the executable names `libpthread`,
`libdl` and `librt` separately and the loader has four objects to place instead
of two. With neither driver they are skipped and everything else still builds.

**The third-party guests need one thing a Rust toolchain cannot produce**: a
**C** cross compiler for `<arch>-linux-musl`. rustc ships musl's `libc.a` for
both targets and none of its headers, and a distribution's
`aarch64-linux-gnu-gcc` is a glibc toolchain with no static libc at all, so the
script probes for `zig cc` — one download that carries musl's sources and
headers for every target it knows — and honours `RSEMU_ZIG`. With none it says
so and the corpus keeps the guests it already has. Sources are fetched at
pinned versions with checksums, built unmodified, and left in the git-ignored
corpus with a `PROVENANCE.txt` beside them.

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
