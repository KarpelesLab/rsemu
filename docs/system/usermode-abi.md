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
| [ARM 64-bit ELF ABI](https://github.com/ARM-software/abi-aa) | The AArch64 supplement, for when a second architecture lands here | Free |
| `elf(5)`, `getauxval(3)`, `mmap(2)`, `brk(2)`, `getrandom(2)` | The Linux manual pages: the auxiliary vector's `AT_*` values and what the kernel actually puts in them | Free — `man-pages`, GPL-compatible documentation, quoted rather than copied |
| [Linux `include/uapi/asm-generic/unistd.h`](https://www.kernel.org/) | The syscall numbers RISC-V, AArch64 and every architecture added since 2012 share | The header is a UAPI interface definition; **numbers are facts** (`ROADMAP.md` §1, "facts versus expression") |
| [`Documentation/arch/riscv/hwprobe.rst`](https://www.kernel.org/) | What `AT_HWCAP` means on RISC-V: one bit per single-letter extension | Free |

**On the kernel headers.** rsemu does not read Linux's *implementation* — it is
GPLv2 and §1 is unambiguous. Syscall numbers, structure layouts and `AT_*`
constants are the published interface a program is compiled against, and are
facts about the ABI in exactly the way a cycle count from a datasheet is a fact
about a chip. Where behaviour was needed rather than a number, it was obtained
by **running** a program and reading its trace, which §1 explicitly permits.

## The differential oracle, which is the point of level 3

A level-3 guest is the cheapest correctness experiment this project has: the
same statically linked binary runs on the host and under the emulator, and the
two must agree. Not only on output — on the **syscall trace**.

That is how `src/usermode/proof.rs` was written. Rather than implementing a
list of syscalls, the guest was run until it stopped, and whatever it asked for
next was implemented. The same program was then built for `x86_64-unknown-linux-musl`
and run under `strace` on the host, and the two traces compared:

```text
host (x86-64 musl)                     rsemu (rv64gc musl)
  set_tid_address                        set_tid_address
  poll                                   ppoll
  rt_sigaction × 2                       rt_sigaction × 2
  sigaltstack(NULL, &old)                sigaltstack(NULL, &old)
  mmap(12288, RW)                        mmap(12288, RW)
  mprotect(PROT_NONE)                    mprotect(PROT_NONE)
  sigaltstack(&new, NULL)                sigaltstack(&new, NULL)
  rt_sigprocmask                         rt_sigprocmask
  rt_sigaction × 2                       rt_sigaction × 2
  brk(NULL); brk(+8K)                    brk(NULL); brk(+8K)
  mmap(FIXED, PROT_NONE, at brk)         mmap(FIXED, PROT_NONE, at brk)
  mmap(4096, RW)                         mmap(4096, RW)
  write(1, …)                            write(1, …)
  munmap; sigaltstack(disable); munmap   munmap; sigaltstack(disable); munmap
  exit_group(0)                          exit_group(0)
```

This found a defect a passing test would not have. `sigaltstack` had been
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

## Determinism: where non-determinism actually enters

`ROADMAP.md` §0 requires every non-deterministic input crossing into the machine
to go through the record/replay seam. At level 3, almost nothing qualifies, and
that is by construction rather than by luck:

| | why it is already deterministic |
| --- | --- |
| `clock_gettime` | `usermode::GuestClock` advances by executed ticks |
| thread interleaving | `usermode::ThreadSet` preempts on a tick quantum, not a wall-clock one |
| `mmap` placement | `UserMemory`'s top-down search is a pure function of the map |
| `brk` | the break starts at the image's own end |
| `getpid`, `uname`, `getuid` | constants |

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
$ rustup target add riscv64gc-unknown-linux-musl
$ scripts/fetch-testdata.sh usermode-guests
$ cargo test --all-features usermode::proof -- --nocapture
usermode: 24 syscall(s), 24077 tick(s); refused []
usermode: stdout "hello from level 3\nargv = [\"hello\"]\nRSEMU = Some(\"1\")\n"
```

`RSEMU_USERMODE_GUEST` overrides the path if you want to point it at some other
static `riscv64` binary. Without the fixture the test says how to build one and
passes; every other test in the module — the ELF loader, the auxiliary vector,
the policy, the journal — runs unconditionally on a synthetic image the test
assembles itself, so `cargo test` stays hermetic and offline.

RISC-V is the architecture because it is the most measured core in the tree
(RV64GC, `riscv-tests` 409/409, and the one that boots Linux), because
`asm-generic` gives it the cleanest syscall ABI of the three 64-bit candidates,
and because the Rust toolchain can produce a static musl binary for it on this
machine with nothing vendored. AArch64 is the obvious second, and needs only
the register mapping and `EM_AARCH64` — the loader, the stack, the policy and
the journal are architecture-neutral.
