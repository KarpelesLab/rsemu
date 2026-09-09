# `arm64-virt` — an AArch64 board a Linux kernel can actually be pointed at

Consumed by [`machines/arm64-virt.machine`](../../machines/arm64-virt.machine),
[`src/dev/arm`](../../src/dev/arm) and
[`tests/a64_linux.rs`](../../tests/a64_linux.rs). The core is
[`cpu.arm.a64`](../../src/cpu/arm/a64); this page is about the board around it.

## Why the board exists

`cpu.arm.a64` was, before this, a strong core with nowhere to run. 666
declarative instruction rows, scalar floating point and Advanced SIMD with no
host `f32` on the guest path, a four-level MMU a guest builds itself,
`LDXP`/`STXP`, the generic timer, and a nine-guest conformance suite at 9/9 with
an empty ledger — and one board,
[`a64-mini`](../../machines/a64-mini.machine), which is RAM, a core and a
peripheral aperture. That board's own comments are honest that it has no
interrupt controller and models no product.

So an operating system had no console to print on, no controller to take an
interrupt from, and no way to switch the machine off. This board is the four
things that were missing:

| | |
| --- | --- |
| an interrupt controller | a **GICv2**: a distributor and a CPU interface, [`src/dev/arm/gic.rs`](../../src/dev/arm/gic.rs) |
| a console | an ARM PrimeCell **PL011**, [`src/dev/arm/pl011.rs`](../../src/dev/arm/pl011.rs) |
| a firmware interface | **PSCI** over `SMC`, in the core at [`src/cpu/arm/a64/psci.rs`](../../src/cpu/arm/a64/psci.rs), landing on [`src/dev/arm/power.rs`](../../src/dev/arm/power.rs) |
| a way to describe itself | a **device tree generated from the realized machine**, [`src/dev/arm/dt.rs`](../../src/dev/arm/dt.rs) |

A fifth arrived later and is not an AArch64 device at all: a **disk**. The
board maps two [`virtio`](../buses/virtio.md) MMIO windows —
[`src/dev/virtio`](../../src/dev/virtio), the same transport and the same two
device models `riscv-virt` uses — so that a kernel on this board can find a
root filesystem on a block device rather than only in a ramdisk.

The layout is the conventional AArch64 `virt` one, and conventional is all it
is — every address the guest uses comes out of the generated tree, so nothing
outside this repository fixes any of these numbers.

```text
  0x0000_0000  boot ROM: the reset vector, the exception vectors, then the DTB
  0x0800_0000  GIC distributor        (4 KiB)
  0x0801_0000  GIC CPU interface      (8 KiB)
  0x0900_0000  PL011                  (4 KiB)
  0x0a00_0000  virtio-mmio: the disk  (4 KiB)
  0x0a00_1000  virtio-mmio: entropy   (4 KiB)
  0x4000_0000  DRAM
  0x4020_0000  where the kernel is loaded and entered
```

## GICv2, and why not GICv3

Deliberately, and the reason is one sentence: **GICv2 is entirely
memory-mapped, and GICv3 is not.**

* A GICv2 CPU interface is a register block at an address that the board maps.
  A GICv3 one is a *system register* file — `ICC_IAR1_EL1`, `ICC_EOIR1_EL1`,
  `ICC_PMR_EL1`, `ICC_SRE_EL1` and a dozen more — which would put an interrupt
  controller inside `cpu.arm.a64`'s `MRS`/`MSR` path. The board could not reach
  it, and a board *without* a GIC would carry it anyway. That is the crate-shape
  rule the other way round.
* GICv3 replaces the distributor's per-CPU half with a **redistributor per
  core**, at its own stride, holding the banked SGI and PPI state and the LPI
  configuration and pending tables — a second register file and a per-core
  address calculation before a single interrupt is delivered.
* GICv3 brings LPIs and, in practice, an ITS to route them, which is a command
  queue the controller DMA-walks. There is nothing on this board to signal a
  message-signalled interrupt.
* A `virt` board may legitimately present either. GICv2 does eight CPUs and 1020
  interrupt ids, which is more than this board has any use for, and a kernel
  that finds `arm,cortex-a15-gic` in its tree binds a driver it has had since
  2012.

GICv3 is the right thing to build the day this board wants more than eight cores
or wants MSIs. Until then it is a redistributor-per-core design bought with
nothing.

## The generic timer had to leave the core

`a64-mini` says the generic timer is *inside* the core and reaches `IRQ`
without crossing the board, which is true of a board with no interrupt
controller. Here it is a **private peripheral interrupt**: the core drives
`cntp` and `cntv` out of two new output pins, the distributor decides whether to
forward them, and the CPU interface drives `nIRQ` back.

Connecting either pin is also what takes that timer *out* of the core's internal
OR (`Lines::route_timer`). That is not tidiness. A core that raised the
interrupt both ways hands a kernel an interrupt its controller never saw; the
kernel reads `GICC_IAR`, is told 1023 (spurious), returns, and takes it again
forever. It is a live-lock rather than a crash, and it is the failure mode this
wiring exists to avoid.

`machines/arm64-virt.machine` therefore has two `wire` statements no other board
in this tree has:

```text
  wire cpu.cntp -> gic.ppi14    # the EL1 physical timer, interrupt 30
  wire cpu.cntv -> gic.ppi11    # the EL1 virtual timer, interrupt 27
```

A kernel entered at EL1 uses the **virtual** timer, so `ppi11` is the one that
matters; both are wired because a kernel picks by exception level and the
binding's interrupt list is positional.

## PSCI is an assertion the board makes, not something the core does

`SMC` is architecturally UNDEFINED when EL3 is not implemented, and
`cpu.arm.a64` implements EL0 and EL1 — `ID_AA64PFR0_EL1` says so and a guest can
read it. So `psci = "smc"` on the core object is **the board asserting something
the identification registers do not**: there is firmware behind that
instruction, it is not modelled as an exception level, and it answers these
calls. That is exactly what a machine with firmware in ROM looks like from EL1.

It is a construction property with `none` among its values for that reason.
`-p psci=none` takes the assertion away and `SMC` goes back to being UNDEFINED.

The honest alternative is implementing EL3: a second stack pointer, a second
vector table, `SCR_EL3`, and a whole exception level whose only inhabitant would
be the forty lines of `match` in `psci.rs`. That is the trade, written down.

What is implemented is what a single-processor kernel calls — `PSCI_VERSION`,
`SYSTEM_OFF`, `SYSTEM_RESET`, `CPU_OFF`, `CPU_ON`, `AFFINITY_INFO`,
`MIGRATE_INFO_TYPE`, `PSCI_FEATURES` — and `PSCI_FEATURES` reports exactly that
set, so a kernel discovers the gap rather than falling into it. `CPU_SUSPEND` is
answered `NOT_SUPPORTED`: a kernel told `SUCCESS` would believe it had been
suspended and resumed.

## The boot ROM has an exception vector table, and that is the diagnostic

`VBAR_EL1` resets to zero, and zero is where this board's ROM is mapped — so
until a guest writes its own `VBAR_EL1`, every exception it takes lands in the
ROM's first 2 KiB. A ROM that left those sixteen slots as unprogrammed zeros
gives a guest that takes an `UNDEFINED` at the vector, takes another one at the
same vector, and spins there with `ELR_EL1` and `ESR_EL1` overwritten by each
trip round. That is precisely the state in which nobody can tell what went
wrong, and it is where this board's first three debugging sessions were spent.

So the ROM fills the fifteen slots it does not use as the reset vector with a
default handler, exactly as firmware does:

```text
  movz x0, #0x0008           ; PSCI_SYSTEM_OFF
  movk x0, #0x8400, lsl #16
  smc  #0
  b    .
```

An unhandled early exception now **stops the machine**, with `ESR_EL1` still
naming what happened and `ELR_EL1` still naming the instruction that did it.
That is the difference between "the kernel hangs" and "the kernel executed `MRS
x0, ID_AA64DFR0_EL1` and this core does not implement it".

It only helps before the guest installs its own vectors, which on Linux is the
first thing after the MMU comes on — and the two *core* bugs below both struck
before it, which is why it was worth building.

## How far a kernel gets

Measured, not asserted. The kernel is Debian's `arm64` installer kernel,
**Linux 6.12.94+deb13-arm64**, fetched by `scripts/fetch-testdata.sh
arm64-linux` and never committed (it is GPL-2.0; running one as an emulated
guest is ordinary use, shipping one here would be redistribution).

```
scripts/fetch-testdata.sh arm64-linux arm64-initramfs

RSEMU_ARM64_KERNEL=testdata/arm64/linux \
RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \
    cargo test --release --features machine-arm64-virt \
        --test a64_linux -- --nocapture
```

It boots. The whole of kernel initialisation, then userspace, then a shell.
Quoted from the console the test prints, with the lines that say this board's
four pieces were each found and used:

```text
[    0.000000] Booting Linux on physical CPU 0x0000000000 [0x410fd034]
[    0.000000] Linux version 6.12.94+deb13-arm64 (…) #1 SMP Debian 6.12.94-1
[    0.000000] Machine model: rsemu arm64-virt
[    0.000000] earlycon: pl11 at MMIO 0x0000000009000000 (options '')
[    0.000000] printk: legacy bootconsole [pl11] enabled
[    0.000000] psci: probing for conduit method from DT.
[    0.000000] psci: PSCIv1.0 detected in firmware.
[    0.000000] psci: Using standard PSCI v0.2 function IDs
[    0.000000] psci: Trusted OS migration not required
[    0.000000] psci: SMC Calling Convention v1.0
[    0.000000] Kernel command line: earlycon=pl011,0x9000000 console=ttyAMA0 rdinit=/init
[    0.000000] NR_IRQS: 64, nr_irqs: 64, preallocated irqs: 0
[    0.000000] Root IRQ handler: gic_handle_irq
[    0.000000] arch_timer: cp15 timer(s) running at 62.50MHz (virt).
[    0.000000] clocksource: arch_sys_counter: mask: 0x1ffffffffffffff …
[    0.054838] hw-breakpoint: found 1 breakpoint and 1 watchpoint registers.
[    0.057002] Serial: AMBA PL011 UART driver
[    0.058239] 9000000.pl011: ttyAMA0 at MMIO 0x9000000 (irq = 13, …) is a PL011 rev1
[    0.058263] printk: legacy console [ttyAMA0] enabled
[    0.286118] Trying to unpack rootfs image as initramfs...
[    0.295933] Freeing initrd memory: 1928K
```

Every one of those is something the board had to be right about. `Machine
model` is the generated tree; `earlycon: pl11` is the PL011 being written to
before any driver exists; the four `psci:` lines are the core answering `SMC`;
`Root IRQ handler: gic_handle_irq` is the GICv2 driver binding; `arch_timer …
62.50MHz (virt)` is `CNTFRQ_EL0` as the machine file set it and the *virtual*
timer being the one a kernel at EL1 picks; and `is a PL011 rev1` is the AMBA
bus having read the eight identification registers and believed them.

Then the end:

```text
[    1.002021] clk: Disabling unused clocks
[    1.002034] PM: genpd: Disabling unused power domains
[    1.041397] Freeing unused kernel memory: 3072K
[    1.073299] Checked W+X mappings: passed, no W+X pages found
[    1.121977] Run /init as init process

rsemu initramfs on Linux 6.12.94+deb13-arm64 aarch64


BusyBox v1.37.0 (Debian 1:1.37.0-6+b8) built-in shell (ash)
Enter 'help' for a list of built-in commands.

/bin/sh: can't access tty; job control turned off
rsemu#
```

Typing `poweroff -f` at that prompt stops the machine, which is
`tests/a64_linux.rs`'s second test and is PSCI end to end from userspace:
busybox asks the kernel, the kernel's PSCI driver executes `SMC` with
`SYSTEM_OFF` in `x0`, `cpu.arm.a64` services it, the core pulses its `poweroff`
pin, `arm.power` raises the host signal, and the run loop stops.

**About three minutes of wall time** for 1.12 seconds of guest time, optimised,
on one interpreted core. The guest's own timestamps are guest time: the generic
timer counts this core's bus accesses divided by an integer, so `[ 1.12]` is
what the kernel measured and not what a person waited.


## Booting off the disk rather than out of the ramdisk

The board had no block device until virtio moved out from under `dev/riscv/`
into [`src/dev/virtio`](../../src/dev/virtio) behind `dev-virtio`. It has one
now, and the difference is the difference between "a root filesystem the kernel
unpacked into memory before any driver existed" and "a filesystem on a disk".

Debian's kernel builds every part of that path as a module — `virtio_mmio`,
`virtio_blk`, and `ext4`, which needs `jbd2`, `mbcache`, `crc16` and a `crc32c`
shash of its own — so an initramfs is still what starts. It `insmod`s the
seven, mounts `/dev/vda` and `switch_root`s, which is how a real Debian system
boots. `scripts/fetch-testdata.sh arm64-rootfs` builds both fixtures: an ext4
filesystem in `rootfs.img` and that initramfs.

```
scripts/fetch-testdata.sh arm64-linux arm64-initramfs arm64-rootfs

RSEMU_ARM64_KERNEL=testdata/arm64/linux \
RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \
RSEMU_ARM64_ROOTFS_INITRD=testdata/arm64/initramfs-virtio.cpio \
RSEMU_ARM64_DISK=testdata/arm64/rootfs.img \
    cargo test --release --features machine-arm64-virt \
        --test a64_linux -- --nocapture
```

Quoted from the console the test prints:

```text
[    1.096425] Run /init as init process
[    1.327321] virtio_blk virtio0: 1/0/0 default/read/poll queues
[    1.327848] virtio_blk virtio0: [vda] 131072 512-byte logical blocks (67.1 MB/64.0 MiB)

rsemu initramfs on Linux 6.12.94+deb13-arm64 aarch64: mounting /dev/vda
[    1.977836] EXT4-fs (vda): mounted filesystem 5253454d-5541-524d-3634-726f6f746673 r/w with ordered data mode. Quota mode: none.

rsemu arm64-virt: this shell is running from an ext4 root filesystem on /dev/vda
/dev/vda / ext4 rw,relatime 0 0


BusyBox v1.37.0 (Debian 1:1.37.0-6+b8) built-in shell (ash)
Enter 'help' for a list of built-in commands.

sh: can't access tty; job control turned off
rsemu-disk#
```

`virtio_blk virtio0` is the kernel's own driver binding to
`dev::virtio::mmio`'s register block at `0x0a000000`, found through the
`virtio_mmio@a000000` node the generator emitted; `131072 512-byte logical
blocks` is the `storage` parameter the test set (64 MiB, against the machine
file's default of 16) read back out of the device's configuration space;
`/dev/vda / ext4` is `/proc/mounts` on the far side of `switch_root`, which is
the guest's own statement that its root is the block device and that the
ramdisk has been freed.

**Six and a half minutes of wall time** for 2.0 seconds of guest time, against
three for the ramdisk boot. The difference is not the disk: it is four
megabytes of kernel modules being relocated and linked by a single interpreted
core, and it is silent while it happens — which is what made
`tests/a64_linux.rs`'s idle guard report it as a hang until the guard became a
parameter.

**One thing here is not a real disk and should be**: the test binds the image
to the `disk` **media slot**, so the whole 64 MiB lives in host memory as a
`RamStore`. The other contract — `--drive disk=root.qcow2`, a host file behind
`dev::medium::Medium` — works on this board for the same reason it works on
`riscv-virt`, and `tests/riscv_virtio_blk.rs` is the test that holds it. There
is no AArch64 equivalent of that test yet; the device under it is the same
object, which is the whole point of the move.


## Choosing an execution engine

`cpu.arm.a64` takes three, and they are `interp`, `jit` and `jit-host` with the
same meanings the other two cores give them:

- **`interp`** — the interpreter, and the oracle everything else is measured
  against. The default.
- **`jit`** — the translation runtime in [`src/jit`](../../src/jit): guest
  instructions lifted into IR blocks by
  [`cpu::arm::a64::lift`](../../src/cpu/arm/a64/lift.rs), cached under
  `(pc, physical page)`, executed by the portable IR backend. Runs everywhere
  the crate does.
- **`jit-host`** — the same runtime with the **host code generator** attached,
  so blocks are lowered to machine code by [`jit::x86`](../../src/jit/x86) on
  x86-64 Linux in a build with `jit-x86`. Anywhere else it falls back to
  `jit`'s backend and answers identically.

The board takes `-p engine=jit` and `-p engine=jit-host`; the property is a
`param` in [`machines/arm64-virt.machine`](../../machines/arm64-virt.machine)
precisely so that one board can be run on all three and have the state hashes
compared. `interp` is the default because it is the oracle, and a build without
`cpu-arm-a64-lift` and `jit` refuses the other two with a message rather than
quietly interpreting.

All three are **indistinguishable to the guest**, cycle counts included, and
that is asserted rather than hoped: `tests/a64_engines.rs` runs a minimal
AArch64 board on each and compares `Machine::state_hash` at ten checkpoints,
then moves a snapshot between engines in both directions;
`cpu::arm::a64::engine`'s own tests do it for a bare core, a paged one, and one
that takes an interrupt mid-run.

It holds on a real guest too. The kernel and initramfs above, 512 MiB, over
twenty seconds of virtual time — well past the point where the kernel is doing
real work rather than decompressing:

| `engine` | 20 s of guest time |
| --- | --- |
| `interp` | 18.94 s |
| `jit` | 10.42 s (**1.82×**) |
| `jit-host` | **3.37 s (5.61×)** |

Every cell is the median of five **interleaved** runs — one of each engine, in
turn, round and round, because the interpreter is the control and a control
measured in a different sitting is not one — and all fifteen finished on one
state hash. Trust the ratios ahead of the seconds: the host was building other
things throughout, and the interpreter column moved by 2% between sittings on
identical code, which is exactly why the reps are interleaved.

### Twenty seconds was not far enough, and two defects were hiding past it

That table is where the claim stopped, and stopping there cost it. Run the same
board out to **120 s** of guest time and the engines parted:
`0x7d0edfc4879cda23` interpreted against `0x03b29d3abf721c43` translated, with
`jit` and `jit-host` agreeing with each other and only the interpreter — the
oracle — on the other side. Bisecting the window in guest time found two
independent defects in `cpu::arm::a64::engine`, both of which move *when* a
quantum ends rather than what an instruction computes, which is why twenty
seconds of the same boot never showed either:

1. **A chained boundary the frontend declined charged that instruction's page
   walk and then returned**, leaving `Cpu::run_budget`'s budget test standing
   between an instruction's translation and its fetch — where an interpreted
   core has it in front of both. The two engines then stopped on different
   instructions and carried a different `State::debt`. It self-corrects at the
   next quantum; it came round about twenty times in twenty-five seconds of
   boot, always in front of an `MRS` on a page the TLB had just lost.
   `advance` now interprets that instruction in the same call.
2. **A translated block did not notice its own generic timer.** The comparator
   is reached by ticks the block itself charges, and the block only looked at
   the end of a chain, so a timer that fired while the guest was inside lifted
   code was taken up to a chain late — a different `ELR_EL1`, and after it a
   different scheduling decision. This is what actually broke the hash, at
   **23.46 s**: the first timer to fire while the guest was busy rather than
   parked in `WFI`. `IrHost::spent` now compares the tick counter against
   `Exec::timer_edge` at every guest instruction boundary, so the block leaves
   on the boundary the interpreter would have taken the interrupt after.

All three engines now finish 120 s on `0x7d0edfc4879cda23` — the interpreter's
own hash, unchanged by either fix, which is what says which side was wrong.
`cpu::arm::a64::engine`'s
`a_declined_chained_boundary_charges_its_walk_with_the_instruction_it_belongs_to`
and `the_generic_timer_is_taken_at_the_same_instruction_by_both_engines` are
the regressions, and both fail on the code before them. A third defect of the
same class was found later, by the long-run harness rather than by hand, and it
has its own section below. The fixes are also
slightly *faster* — 20 s of this boot went from 11.19 s to 10.91 s on `jit` and
from 5.11 s to 4.94 s on `jit-host`, because interpreting a declined
instruction in place saves a dispatcher round trip.

What the mechanisms did over that run:

| | |
| --- | --- |
| blocks executed | 23 810 578 |
| of those, compiled to host code | 23 809 916 (**99.997%**) |
| of those, reached by a patched exit | 20 571 853 (86.4%) |
| distinct blocks lifted | 17 638 |
| guest instructions retired **inside** a block | 153 130 249 (**97.96%**) |
| compiled loads served by an inlined TLB probe | 18 712 518 |
| compiled stores served the same way | 13 350 310 |

The blocks the code generator refused are the ones holding a `UDIV` or an
`SDIV`, the only two ops this frontend emits that `jit::x86` does not lower.
The inlined probes — `ROADMAP.md` §9.1's first mechanism, the software TLB's
fast path emitted into generated code rather than called into — are worth a
step of that ratio on their own: before `cpu::arm::a64::mmu`'s `Tlb` had a
`jit::Tlb` shadow to publish, the same sweep put `jit-host` at 6.92 s and
2.54×. [`src/cpu/arm/a64/engine.rs`](../../src/cpu/arm/a64/engine.rs) has the
argument for what a plan may cover on this architecture, and the three things
that looked as though they might forbid one — address tagging, the two `TTBR`s
and granule selection — none of which does.

### And a third one, which needed a `TLBI` on the same instruction

Defect 2's fix carried a claim, written down in `Exec::timer_edge`: the
registers that decide the comparator are `MSR`-only, no `MSR` is inside the
lifted subset, so a block ends at one and the edge can be computed **once per
`advance`**. That claim is sound. What it did not cover is *where in `advance`*
the edge is computed, and there is a window.

`engine::admit` asks `Exec::pending_interrupt` and **then** calls
`Exec::translate_fetch`, which on a TLB miss walks the guest's tables and
charges three or four accesses. `Host::new` computes the edge after both. The
generic timer's count is this core's own tick counter divided by `cntdiv`
(`Exec::counter`), so those charged ticks can carry it across a comparator that
the check a moment earlier found un-crossed — and `Exec::timer_edge` answers
`u64::MAX` for a comparator already crossed, because its question is when the
outputs next *change* and an asserting output cannot rise again. That is the
right answer to its own question and the wrong edge for a run: `IrHost::spent`
wanted "leave at the first boundary", and got "never leave". The interrupt then
waited for the next *chained* boundary's `admit`, which does look.

Opening it takes a **cold instruction-fetch translation on the instruction a
timer fires on**, and on this core only a `TLBI` produces one:
`cpu::arm::a64::mmu`'s `Tlb` keeps fetch, load and store entries in three
separate 256-entry sets, so no amount of data-side pressure evicts a code page
and the alternative is a guest that executes from 257 of them. Which is why a
**forty-second boot of this board never reached it** — Linux issues `TLBI`
constantly, but a timer edge has to land on one — and why the synthetic
workload in `tests/engine_longrun.rs`, which flushes on half its passes round a
loop the timer fires inside, reaches it in **0.417 s of guest time**. The
harness found it on its first run, at quantum 417: `ELR_EL1` `0x1014`
interpreted against `0x4` translated, two guest instructions and one cycle
apart, with `jit` and `jit-host` giving the same wrong answer because the edge
is computed above both code generators.

`engine::leave_at` was the fix and it was four lines: when the run's own
`Exec::pending_interrupt` is already `Some` — after the walk, which is the only
thing that can have changed since `admit` looked — the edge is the current tick
count, so `IrHost::spent` is true at the first boundary it is asked at. Which,
because `ir::Interp` never asks at a block's first boundary and `jit::dispatch`
never asks at a run's first block, is the boundary *after one retired
instruction* — exactly what `Exec::step_once` does, charging the fetch, running
the instruction, and taking the interrupt on its next call.

Asking `pending_interrupt` rather than the timer condition alone is what keeps
that from being a throughput cliff, and it is the reason the fix is at the call
site rather than inside `Exec::timer_edge`. A comparator stays crossed until the
guest re-arms it; on this board the timer is routed out to the GIC and a kernel
holds `PSTATE.I` across critical sections, so "the condition is met" describes
long stretches of code with no interrupt to take. Through all of them
`pending_interrupt` is `None` and blocks run to their natural ends. Measured on
`benches/a64_dispatch.rs`, every row of both tables moved by less than the
interpreter column — a path the change does not touch — moved between sittings
on identical code, which on this host is about 2.5%.

`a_tlbi_in_the_loop_agrees_across_the_engines` in `tests/engine_longrun.rs` was
committed `#[ignore]`d as the reproduction and is now un-`#[ignore]`d as the
regression: it fails on the code before this at quantum 417 and passes for
30 000 quanta after it, on both translated engines.

**`leave_at` has since been replaced by `Admitted::leave`, and the reason is
the half it could not see.** A walk moves the interrupt inputs in two ways: by
its *ticks*, which is the generic timer above, and by its *reads*, because a
descriptor read is an ordinary physical access that the address space answers
however the board decided. A translation table over a lazily-advanced device
raises a line the timer edge knows nothing about, and `leave_at` — computed
once in `Host::new`, from the timer — saw neither that nor a chained block's
entry translation, which happens inside `Frontend::enter` after
`Dispatcher::run` has already asked `IrHost::spent`. So the question moved to
`admit`, where the window is, and is now the same three lines the RISC-V and
x86 engines carry: re-ask `Exec::pending_interrupt` when the translation
charged, and `Host::hand_back` at both call sites.

This board cannot reach the reads half today, and the reason is worth writing
down because it is not a design: an A64 descriptor read is always `Width::U64`,
and `arm.gic`'s distributor accepts `U8`..`U32` while its CPU interface accepts
only `U32`, so a walk over `GICC_IAR` takes an external abort instead of
acknowledging an interrupt. `uart.pl011` and the two `virtio.mmio` transports
are not lazily advanced at all. `riscv-virt`'s CLINT and `pc-at`'s HPET both
take the width their architecture's walk uses, so the two sibling cores are
reachable and this one is protected by an accident. `TTBR0_EL1` is a
guest-written register either way. The regression test is
`a_walk_that_raises_an_interrupt_is_taken_where_the_interpreter_takes_it` in
`src/cpu/arm/a64/engine.rs`: a level-1 table mapped over a device that asserts
`IRQ` on the *n*-th walk that reads it, over the two-page `TLBI` loop, swept
over *n* so the line comes up on the prologue's translation and on a chained
boundary's in turn. It fails with either check removed.

The move was free and slightly better than free: callgrind over
`benches/a64_dispatch --smoke`, with `Exec::step` as the control row, has
`Cpu::advance` inclusive going **3 163 841 599 -> 3 162 906 823, -0.030%**,
against an `Exec::step` that moved by nothing at all — 7 949 792 616 on both
sides, to the instruction. `leave_at` called `Exec::timer_edge` and sometimes
`Exec::pending_interrupt` on every run; `admit` now asks the second only when
the entry translation charged, which on a warm block is never.

`admit` has one exit this does not cover, and on this core the *timer* reaches
it: a known-unliftable PC on a cold page, where the walk crosses the comparator
and `Subset::get` then hands the instruction to `Exec::step_once`, whose own
first act is to take the interrupt — so the instruction never runs, where an
interpreted core would have run it. Closing it needs `Exec` to offer one step
with the interrupt check already discharged, which is a change to the oracle
rather than to the engine.

Neither RISC-V nor x86 can have this shape. Both reach their timers as devices
on the bus — `dev::riscv::clint` on its own clock domain, the APIC — so
`riscv::engine::Host::spent` and `x86::engine::Host::spent` are
`used >= allowance` and nothing else, and `riscv::exec::Exec::pending_interrupt`
reads only a wire. `SFENCE.VMA` opens the identical *window*; there is nothing
inside it that a hart's own charged ticks can change.

### Where the interpreter is reached, and where it used to be

The 97.96% above was measured before `ir::IrHost::spent` landed. Over the boot
— a different and rather harder window — the same figure was 97.54%, and it is
now **99.44%**. What closed the gap was not a wider frontend: it was that seam,
which lets a translated block *leave* at a guest instruction boundary.

A translated block used to run only if its **worst case** fitted what was left
of the scheduler quantum, or the two engines would stop on different
instructions and the state hash they are supposed to share would part. For a PC
nothing had been lifted at, that worst case was the frontend's own limit — 64
instructions of an unaligned pair access, each byte walked four levels, **5 188
ticks** — against a quantum of 10 000. So the last half of every quantum could
admit nothing, and it could not recover inside the quantum either: the PC after
an interpreted instruction is in the middle of a block, and only a lift filled
the cost table, so it was uncosted too. `engine.rs`'s `Probe` closed half of
that by lifting the cold PC instead of guessing at it. The seam closes the rest
and deletes both, because a block that can leave never needs to be refused.

Measured over the **boot**, from reset to `/init`'s banner — a guest-side
window, so both columns run the same 137 984 quanta and the same 1 071 503 716
guest instructions — median of three interleaved reps:

| | before the seam | with it |
| --- | --- | --- |
| wall clock | 23.30 s | **19.54 s** (1.19×) |
| retired **inside** a block | 1 045 147 550 (97.54%) | 1 065 491 373 (**99.44%**) |
| taken by the interpreter | 26 356 169 | **6 012 343** (−77.2%) |

and what is left, by the reason `admit` gave:

| why the interpreter ran it | instructions | share of interpreted |
| --- | --- | --- |
| outside the lifted subset, already known | 6 010 437 | **99.97%** |
| a lift that produced nothing | 1 614 | 0.03% |
| a pending interrupt or a stalled `WFI` | 292 | 0.00% |
| the scheduler-budget guard declined a block | **0** | — |

The exclusions cost **0.56%** of the guest's instructions between them, and
they are the same ones as before: `MRS` (of which `SP_EL0` — Linux's `current`
— is the largest single row), the exclusives and acquire/release accesses,
`MSR`, the `DC`/`IC`/`TLBI` maintenance operations, and `RBIT`. **No SIMD or
floating-point instruction executed at all**, and no `LDTR`/`STTR` either. The
largest documented absence in the frontend is worth nothing on the guest the
board exists to run, which is the sort of thing only a profile says.

`benches/a64_dispatch.rs`'s second table sweeps the quantum so that this cliff
is a column rather than a paragraph.


### A store no longer ends a block, and that was the largest number in the profile

The first profile of this board under callgrind — twenty guest seconds of the
boot, `engine = "jit-host"` — found 350 host instructions per guest
instruction, of which **twenty-five** were the code the JIT generated. Nearly
everything else was per *block*:

| | share |
| --- | --- |
| the dispatch loop (`Cpu::advance`) | 24.3% |
| replaying deferred charges and boundaries | 19.3% |
| the address space and the software TLB | 18.4% |
| admitting a block: the entry fetch, its walk, the interrupt check | 13.7% |
| reading a guest register | 7.8% |
| **the code the JIT generated** | **7.7%** |

Every row but the last is divided by the number at the bottom of
`benches/a64_linux_boot.rs`'s census, and that number was **6.44 guest
instructions per block** against a frontend limit of 64. A block ended at every
store, because a store into the page a block was lifted from makes every
instruction after it a translation of bytes that no longer exist, and the
invalidation that catches it runs at a block boundary. Stores are 8.9% of this
guest's instructions, so more than half of all block boundaries were one.

#### Why x86's answer does not transfer, and what does

`cpu::x86::lift` met this first. x86 makes coherent instruction caches
architectural, so it *must* notice, and it does it with an **in-block guard**:
after each store, three IR instructions compare the store's address page
against the block's own and leave through a precise exit when they match. Then
comes the part that matters here — the IR only has the address the guest
computed, so the comparison is in **linear** space, two linear pages may alias
one physical page, and a store through the other mapping walks straight past
it. `cpu::x86::engine`'s `admit` therefore picks `Smc::Guard` only with paging
**off** and `Smc::EndBlock` under it. On a paged guest — which is every guest
worth measuring — x86 still ends a block at every store. The mechanism proposed
for adoption here is one its author cannot use on the workload that motivated
it, and copying it would have bought this board nothing at all.

What transfers is the *argument*: the check wants to be a comparison against
the physical page the block's bytes came from. And on this core there is
somewhere better than the IR to make it. `engine::Host` sees the
guest-**physical** address of every store there is — `Exec::wrote` already
feeds `jit::DirtyPages`, which is what invalidates a *cached* translation, and
`Admitted::base` is already the physical page the entry translation resolved
to. `lift::Smc::HostGuard` is that same comparison made one step earlier: when
a store's physical page is the running block's own, the host retires the run's
tick allowance and the block leaves at its next guest instruction boundary,
through the seam `ir::IrHost::spent` opened for the quantum. It is strictly
stronger than the x86 guard rather than a weaker stand-in:

- **no aliasing hole**, because both ends are physical — and on this
  architecture that is not hypothetical, since patching kernel text through the
  linear map while it is mapped executable elsewhere is how arm64 Linux does
  modules, `ftrace` and jump labels;
- **no IR at all**, where x86 pays three instructions per store;
- **one implementation**, because a compiled store reaches
  `Host::note_writes` through `FastMem::note_fast_store` and every other store
  through `IrHost::store`.

What it gives up is precision: the exit is per page, so a store to a datum that
merely shares a page with the code leaves the block too. That is the block
cache's own granularity, and the census prices it — 6 874 translations killed
by a block's store in twenty guest seconds against 154 233 958 guest
instructions retired. It is not a rate.

#### The two things a store was the boundary for and nobody had listed

"A store ends the block" was load-bearing for three things, and only one of them
is self-modifying code. The other two are the engine's to pay, and both are
answered in `IrHost::store` — neither can arrive through the backend's inlined
path, because a memory plan covers plain little-endian RAM over a whole page:

1. **An interrupt the store raised.** A write to the GIC, or to a device that
   answers by pulling a wire, now happens between two instructions of a lifted
   block, where `admit` would have seen it at the boundary that no longer
   exists. `cpu::x86::engine` pays this on the identical argument and said so
   first.
2. **A store that remaps the address space.** The backend takes the inlined
   memory path's host pointers out of the shadow TLB **once per block** and
   they are valid until the TLB is flushed — and `jit::x86`'s own comment says
   a flush happens at a block boundary, *never inside one*. A store that
   retopologises was the only way in, and it was safe only because it ended its
   block. The host now samples `AddressSpace::generation` at entry and again
   after each such store.

#### What it bought

`benches/a64_linux_boot.rs`, twenty guest seconds of the boot,
`engine = "jit-host"`, under callgrind with `--cache-sim=no`. The two columns
are the same binary built twice with `engine.rs`'s `SMC` constant as the only
difference, so nothing else moved; the same board under `rsemu run --for 20s`
ends both runs on the same `Machine::state_hash` (`0x9cc4de4dee51678b`) with
the same 154 233 958 guest instructions retired, so this is host cost alone:

| | ends the block | the host guard |
| --- | --- | --- |
| **host instructions** | 51 758 768 516 | **44 220 119 264** (−14.56%) |
| blocks executed | 23 935 454 | **14 283 856** (−40.3%) |
| **guest instructions per block** | **6.44** | **10.80** |
| distinct blocks lifted | 17 256 | 12 173 |
| the dispatch loop (`Cpu::advance`) | 12 837 327 816 (24.80%) | 8 223 732 221 (18.60%) |
| replaying deferred bookkeeping (`flush_thunk`) | 9 890 417 533 (19.11%) | 9 577 607 664 (21.66%) |
| `admit` | 2 708 590 165 (5.23%) | 1 686 283 235 (3.81%) |
| the entry translation (`Exec::translate`) | 2 148 445 889 (4.15%) | 1 511 586 330 (3.42%) |
| reading a guest register (`get_slot_thunk`) | 1 898 506 495 (3.67%) | 1 522 825 771 (3.44%) |
| the interrupt check (`Exec::pending_interrupt`) | 1 211 777 280 (2.34%) | 806 436 078 (1.82%) |
| the per-block TLB resync (`jit::Tlb::sync`) | 1 040 195 675 (2.01%) | 634 823 351 (1.44%) |
| the guard's own row (`Host::note_writes`) | 483 850 939 (0.93%) | 470 049 880 (1.06%) |
| the inlined store path (`jit::Tlb::note_fast_store`) | 1 100 681 040 | 1 100 631 680 |

Read the last three rows together with the second. The interrupt check went
**down** even though this change added a call site to it, because there are 40%
fewer blocks to admit; the inlined store path did not move at all, because the
guest makes the same stores either way; and the guard's own row is where its
cost lands — measured against the tree as it stood *before* this change, which
carries no guard code at all, that row goes from 414 078 569 to 470 049 880, so
the comparison costs about four host instructions per store and 56 M over the
run against the 7.5 **G** the policy saves. And `flush_thunk` is where the
saving stops: `ir::hoist_slot_reads` already found that what a replay costs is
mostly the events in it, and halving the number of blocks moves events between
regions rather than removing them, so that row grew as a *share* while shrinking
by 3.2% in absolute terms.

The same pair measured through `rsemu run … --for 20s --headless`, which also
hashes a gigabyte of guest RAM at exit and so is a laxer denominator, agrees:
63 910 138 422 → 56 758 893 297, or 50 158 333 101 → 43 007 087 976 (−14.26%)
once the reset fill and the final hash — identical in both — are taken out.

`benches/a64_dispatch.rs`'s ladder has a `+guard` column now, so the same claim
has a runnable baseline beside it rather than a number in a commit message.
`lift::Smc::EndBlock` stays, and `tests/a64_lift_differential.rs` runs the
generated corpus through both policies: a policy that got faster by getting
wrong should fail a test rather than win a column.


## Two processors

[`machines/arm64-virt-smp.machine`](../../machines/arm64-virt-smp.machine) is
this board with a second core. It is a **separate file** rather than a `param`
on the first, for the reason `pc-apic` is separate from `pc-at`: the
description language declares objects and cannot be told how many to make, so a
one-core run of a two-core file would be a board with a spare core spinning in
the boot ROM. `arm64-virt` is unchanged, down to the byte in its boot ROM, and
it is still the file the distribution-kernel gate runs on.

Three things had to be true, and they are three different problems.

### 1. The GIC has to know which core is asking

A GICv2's low 32 interrupt ids are **banked**: interrupt 27 is a different
interrupt on each core, with its own enable bit, its own pending bit and its
own priority (IHI 0048 §4.1.3). `GICD_ISENABLER0` is therefore one address and
*N* registers, and a distributor that stored one copy of it would have one core
enabling the other core's timer.

The information was already on the bus and simply never read back.
`MemAttrs::requester` identifies the initiator, `cpu.arm.a64` stamps it on
every access it makes, and `core::space` carries it to the leaf unchanged. What
was missing was the **map** from a requester id to a CPU interface number — and
it could not be written in a machine file, because an object's requester id is
allocated at realize time by declaration order. So the file names the
*processors*:

```text
  object gic "arm.gic" {
    cpus       = 2
    processors = [cpu0, cpu1]
  }
```

and `Gic::bind` resolves each of them through `BindCtx::peer`, which exists for
exactly this and which `pc.lapic` uses for the architectural local-APIC page.
`core::space` needed no change at all.

Two decisions worth stating. An access whose requester is not on the list —
a debugger, a DMA engine, a `MemAttrs::DEFAULT` from a test — reads **interface
0**, which is the same fallback the APIC window makes and is what a person
reading the machine expects. And a block with `cpus > 1` and no `processors`
is a **build error**: every core would read interface 0's banked registers,
which is precisely the defect the property removes, so it fails loudly rather
than answering the wrong core's registers.

Interprocessor interrupts come out of the same fact. `GICD_SGIR`'s target list
filter `0b01` means *everyone but me*, and *me* is only knowable because the
write carried a requester.

### 2. Two cores means two generic timers

The generic timer is a **private peripheral interrupt**: each core has its own
and it arrives in that core's own bank of the distributor. So the wiring is per
core —

```text
  wire cpu0.cntv -> gic.cpu0ppi11
  wire cpu1.cntv -> gic.cpu1ppi11
  wire gic.irq0  -> cpu0.irq
  wire gic.irq1  -> cpu1.irq
```

— and `parse_input` already understood `cpu<C>ppi<N>`, so this cost nothing
beyond writing the four lines. Wiring a core's `cntv` out is also what takes
the timer *out* of that core's internal interrupt OR, which is the live-lock
described at the top of this page, now doubled.

### 3. Both cores come out of reset at the same address

That is what a reset vector is. A stub that unconditionally jumped to the
kernel would enter the kernel once per processor, simultaneously, which is not
a boot. So `arm.boot`'s reset vector now begins:

```text
  mrs  x9, mpidr_el1
  and  x9, x9, #0xff        ; Aff0, which is this board's processor index
  cbnz x9, secondary
```

and everything but affinity 0 goes to a **parking loop** in the same 128-byte
vector slot, waiting on its own 64-bit word of a **release table** — one word
per processor, in RAM below the kernel, zero until something writes an address
into it, at which point that processor clears `x0`-`x3` and branches there.

A one-processor board emits none of it and its ROM is byte-identical to the one
this file generated before secondaries existed; `a_one_processor_rom_is_the_stub_it_always_was`
is the test that says so.

That table is the board's warm-boot entry point, and it is deliberately the
*same* table under both boot methods:

* `secondary = "spin-table"` tells the guest it exists and where it is —
  `enable-method` and `cpu-release-addr` on each `cpu@N` node (Devicetree
  Specification v0.4 §3.8.1: "the physical address of a spin table entry that
  releases a secondary CPU from its spin loop"). The generated tree also
  carries a **memory reservation** over the page, because a table the kernel's
  own allocator can hand out is a table that gets overwritten before it is
  read.
* `secondary = "psci"` says `enable-method = "psci"`, and the secondaries do
  not park at all: they are declared `start = false` and are genuinely off
  until a `CPU_ON` names them. **This is what `arm64-virt-smp` ships now**;
  the section after next is what it took, and the machine file's own header is
  why the two properties travel together.

**What could not be established from a permissive source.** DTSpec §3.8.1
defines `enable-method` and `cpu-release-addr` and is where both come from
here. The AArch64-specific *release procedure* — that the value written is the
entry point, that it is a 64-bit little-endian store, that the released
processor starts with the same requirements as the primary one — is documented
in Linux's own `Documentation/arm64/booting.rst`, which is GPL-2.0 and was not
read. What is implemented was derived from the binding plus **black-box
observation of the guest** (§1 permits exactly that): the board published a
release address and the kernel's writes to it were watched. If that protocol
has a corner this board gets wrong, this is where it will be.

### What a kernel does with it

Debian's `arm64` kernel, same image as the single-core gate, on
`arm64-virt-smp`. Quoted from its own console:

```text
[    0.021881] EFI services will not be available.
[    0.022077] smp: Bringing up secondary CPUs ...
[    0.022568] Detected PIPT I-cache on CPU1
[    0.022594] CPU1: Booted secondary processor 0x0000000001 [0x410fd034]
[    0.022713] smp: Brought up 1 node, 2 CPUs
[    0.022736] SMP: Total of 2 processors activated.
[    0.022747] CPU: All CPU(s) started at EL1
```

`CPU1: Booted secondary processor` is printed **by the secondary**, which is
what makes it evidence rather than a claim about the device tree.

It also runs userspace, and the shell is where the rest of this page's claims
get checked. Typed at the prompt, three and a half minutes into the run:

```text
rsemu# cat /proc/interrupts; head -3 /proc/stat; poweroff -f
           CPU0       CPU1
 11:        184        199 GIC-0  27 Level     arch_timer
 13:          4          0 GIC-0  33 Level     uart-pl011
IPI0:        38         66       Rescheduling interrupts
IPI1:       104        115       Function call interrupts
IPI2:         0          0       CPU stop interrupts
…
cpu  0 0 118 45 0 0 0 0 0 0
cpu0 0 0 67 13 0 0 0 0 0 0
cpu1 0 0 50 31 0 0 0 0 0 0
[    0.853819] reboot: Power down
```

Four separate things in one screen. **Interrupt 27 is banked**: each processor
has its own architected timer and its own count of it, 184 and 199, which a
distributor keeping one copy of `GICD_ISENABLER0` could not produce. The
**PL011's shared interrupt is not** banked — 4 and 0, all on the processor the
distributor targets. **`GICD_SGIR` works both ways**: 181 interprocessor
interrupts reached CPU1 and 142 reached CPU0, which is the path a
requester-blind model cannot even address. `/proc/stat` says **CPU1 ran
tasks** — 50 ticks of system time and 31 idle. And `poweroff -f` still reaches
`PSCI_SYSTEM_OFF` through an `SMC` with two processors running.

[`tests/a64_smp.rs`](../../tests/a64_smp.rs) asserts those lines and types that
command, and carries a hermetic test beside it — a dozen hand-assembled
instructions, no download — in which the boot processor writes the other one's
word of the release table and waits for it to answer.

### What PSCI `CPU_ON` cost, and what it bought

The board would rather say `enable-method = "psci"`, because that is what a
`virt` board conventionally is and because a spin table cannot power a
processor *off* again. It says it now. Six things were needed and all six are
in `cpu.arm.a64`; what follows is what each turned out to be, because the
prediction and the implementation differ in two places and the differences are
the interesting part.

1. **A roster of siblings**, as predicted: [`psci::Cluster`] is a
   `HostKind::rendezvous("arm-cluster")` host object that every `cpu.arm.a64`
   on a board opens by name (a `cluster` property, defaulted, so an ordinary
   board writes nothing), holding `(affinity, Weak<Lines>)` per processor.
   `core::hosts` needed no change.

   Two things the prediction did not say. The roster is keyed on the
   **affinity** and not on `MPIDR_EL1`: bit 31 is RES1 and is set on every
   real `MPIDR`, so a kernel's `CPU_ON(1, …)` would find nothing on a board
   whose `mpidr` is `0x80000001` if the raw value were the key. And joining
   it **refuses a duplicate**, because two processors with one `MPIDR_EL1` is
   a board where `CPU_ON` starts whichever the roster happened to find and
   `AFFINITY_INFO` reports on the other one.

2. **`Lines` grew four cells** — `powered`, `start_entry`, `start_context`,
   `start_pending` — exactly as predicted, and for exactly the stated reason:
   atomics on the sibling's `Lines`, no lock taken across the call, the `BUS`
   rank untouched. It also grew a fifth field the prediction did not have: the
   `Arc<Cluster>` itself. It lives there rather than on `Cpu` because `Lines`
   is what the interpreter already holds when it services an `SMC`, and
   threading a second reference through `Exec::new` would have cost every
   caller an argument for the sake of one instruction. The roster holds `Weak`
   back, so there is no cycle.

   `Lines` also stopped deriving `Default`. `AtomicBool::default()` is false,
   and a derived `Default` would have switched every existing board off.

3. **A `start` construction property**, default true, as predicted. A core
   that is not powered consumes its budget and executes nothing — and
   *consuming* it is the part that matters: returning zero instead looks like
   a core that cannot make progress, the run loops break on that, and the
   machine's clock would stop advancing past a processor that is merely
   switched off.

4. **The pending start is applied at an instruction boundary**, in both
   `run_budget`'s path and `step`'s: the architectural reset state, then `PC =
   start_entry` and `X0 = start_context`, TLB and block cache flushed. The
   same route `pc.lapic`'s INIT and Start-Up take into `cpu.x86`.

5. **`psci::call` takes the roster**, through a `Siblings { cluster, cpus, me }`
   argument. `CPU_ON` is DEN 0022 §5.1.3 — target affinity in `x1`, entry in
   `x2`, context in `x3`, and the started processor enters with `X0 =
   context_id` — with `INVALID_PARAMETERS` for an affinity no processor has,
   `ALREADY_ON` for one that is powered, and `SUCCESS` otherwise.

   The prediction called `call` a pure function and it is not one any more:
   `CPU_ON` writes the target's atomics and `CPU_OFF` clears the caller's own
   `powered`. Everything a **board** must do is still returned in
   `Outcome::effect` rather than done inside; what changed is that a sibling
   processor is not a board.

6. **`AFFINITY_INFO` and `CPU_OFF` stopped being constants.** `AFFINITY_INFO`
   answers `1` (OFF) for a processor nothing has started and `0` (ON) for one
   that is running; `CPU_OFF` on a secondary clears `powered` and the
   processor retires nothing afterwards, while `CPU_OFF` on the **last powered
   processor** is still `DENIED`, because a machine with every processor off
   is a machine nothing can start again.

A board with no cluster keeps every answer it had, which is what
`machines/a64-mini.machine` and a spin-table `arm64-virt-smp` rely on.

**Nothing in `core::` changed**, as predicted, and nothing in `dev/arm/`
changed at all: `arm.boot` already accepted `secondary = "psci"` and already
emitted the parking loop. What the board writes is two words rather than one —
`secondary = "psci"` and `start = false` on the secondary — and they travel
together because a processor `AFFINITY_INFO` reports as `ON` before anything
started it is a processor `CPU_ON` refuses with `ALREADY_ON`.

**What is not here.** `CPU_SUSPEND` is still refused rather than answered.
`MIGRATE` and the `SYSTEM_SUSPEND` family are not implemented and
`PSCI_FEATURES` says so. And the snapshot chunk carries the power state
appended after the interrupt lines, read back only when it is present, rather
than as a version-3 chunk: a version-2 chunk's missing power state has exactly
one correct value — running, nothing pending — which is what version 2 meant,
so the fallback is a migration rather than a guess. That also keeps
`src/host/gdb/arch.rs`'s AArch64 register map where it is; every offset it
reads is before those four fields.

### The one thing `start = false` broke, and it was already broken

A processor that is switched off executes nothing, so its cycle counter stops
— and `CNTPCT_EL0` on this core is that counter divided by `cntdiv`. The
first PSCI boot printed `[ 8795.876863] Detected PIPT I-cache on CPU1` where
the spin-table boot had printed `[ 0.022568]`: the secondary's view of time
was nothing like its sibling's, and a kernel that subtracts two processors'
timestamps was getting an unsigned register's idea of a negative number.

The spin table had hidden it. A parked processor is *executing* — a two-
instruction loop, but executing — so its counter advanced at roughly the same
rate as the boot processor's and the two never drifted far enough to notice.
Switching it genuinely off is what made a modelling choice visible that had
been wrong since the second core landed.

The architecture is unambiguous: the system counter is in the **always-on
power domain** (DDI 0487 D11.1.2) and provides one view of time to every PE.
So two things changed, and neither is about PSCI. A core that is not powered
charges its scheduler budget to its cycle count and retires nothing, and
`CPU_ON` preserves `cycles` and `debt` across the reset it applies. Both are
in `Cpu`, both are one line, and both would have been needed by any mechanism
that stops a core — `CPU_SUSPEND`, or a debugger that halts one.

What is still *not* modelled is a counter that is genuinely shared. Each core
still derives `CNTPCT_EL0` from its own tick count, which agrees with its
siblings' only because the scheduler hands every core on a domain the same
budget. That is true of every board here and it is not a guarantee.

### What it looks like from the guest

`tests/a64_smp.rs`'s `psci_cpu_on_starts_the_second_processor_and_cpu_off_stops_it`
is hermetic — two dozen hand-assembled instructions, nothing downloaded — and
the two things it asserts are the two a spin table cannot do. The word the
boot processor waits on is the **context id** the `CPU_ON` passed, so a start
that did not deliver `X3` into the target's `X0` hangs rather than passes. And
the loop after it waits for `AFFINITY_INFO` to report `OFF`, which happens
only because the secondary called `CPU_OFF`.

## The exclusive monitor is global now, and this board is why

**Was**: `arm64-virt-smp` ran two processors and the reservation each one kept
was private to it. `cpu::arm::a64`'s `State::exclusive` — like `cpu::riscv`'s
`reservation` — was per-core state, so a sibling's store did not break this
core's reservation: an `stxr` the architecture *requires* to fail succeeded,
and the sibling's update was lost. The board still booted, because a kernel's
spinlocks are uncontended almost always and two cores rarely reach the same
lock inside one scheduler quantum. That was luck about timing, not a property
of the model.

It was found from the other direction — `usermode`'s threaded guest puts two
cores on one `UserMemory` and lost increments measurably: an AArch64
`AtomicU32::fetch_add` loop (an `ldxr`/`stxr` pair, no `FEAT_LSE`) landed
**32038 of 40000**, while the same program on RISC-V landed 40000 because LLVM
emits a single `amoadd.d` there. Same source, same defect, different compiler
output — neither architecture alone would have named it.

**Is**: `core::space::ExclusiveMonitor` is DDI 0487 B2.9's *global* monitor,
living on the `AddressSpace` because a space is one coherence domain. Each
core claims a slot when it is given the space; `State::exclusive` stays as the
*local* monitor and a `STXR` passes only if both agree. Every store that
reaches `SpaceView::write_span` — a guest store, a DMA burst, a `LOCK`ed x86
access on a heterogeneous board — breaks any reservation covering the bytes it
touches, keyed on the **physical** address, because two cores contending for
one lock reach it through their own translation regimes. The compiled fast path
does not go through the space at all, so `Exec::note_fast_store` tells the
monitor itself.

The reservation granule is sixteen bytes: `IMPLEMENTATION DEFINED` between 8
and 2048 in the architecture, and 16 is the smallest that holds a 64-bit
`LDXP`/`STXP` pair, which is a single 16-byte access. Larger would be legal and
would cost forward progress.

So `arm64-virt-smp`'s green boot is now evidence about its atomics as well as
about bring-up, banking and IPIs. What it is still not evidence about is
*ordering*: this core executes one instruction at a time and completes every
access before the next, so a guest that depends on a weak memory model being
weak has nothing here to disagree with. (`DMB`, `DSB` and the acquire/release
forms are no longer no-ops — each one executes a host `fence(SeqCst)`,
`docs/techniques/memory-models.md` has that round — but a fence on a host that
was already ordering the accesses changes no outcome this board can produce.)

## The monitor was necessary and not sufficient: three windows inside one instruction

The section above closes the window *between* a `LDXR` and its `STXR`. It says
nothing about the windows *inside* each of those instructions, and there were
three of them. All three are the same shape — this core issues a guest atomic
as several separate accesses through `AddressSpace`, and the only lock held
across them is the core's own `BUS`-ranked session mutex, which is per core and
excludes nothing on a sibling.

`tests/a64_lse_atomicity.rs` is the instrument: two cores, two host threads,
each incrementing one word 60 000 times, once with `STADD` and once with an
`LDXR`/`STXR` loop, beside a second word incremented with a plain load, add and
store as the witness that the two really overlapped. A lost update is not a
reordering — it is a value no interleaving of the two programs could have
produced — so unlike almost everything else in this area it can be *asserted*
on an x86-64 host rather than printed.

| window | lost of 120 000 | runs that lost any |
| --- | --- | --- |
| a `FEAT_LSE` atomic's read and write, with nothing between them | 3 410, 8 497, 8 850 | every one |
| a `STXR`'s monitor check and its store | 318 | most |
| a `LDXR`'s read and the moment it claims the granule | 46 | most |
| a sibling `STXR`'s own `note_store`-to-transfer gap | 1 to 3 | 33 of 60 |
| all four, as the tree stands | 0 | 0 of 126 |

The first two — and, as the fourth showed, the load-exclusive as well — are
closed by `AddressSpace::bus_lock`, taken by
`Exec::lock_bus` before the instruction issues an access and held until it
ends — the span `cpu::x86::exec` already uses for a `LOCK` prefix, and for the
same second reason: a bus lock fences at both ends, so a guard that opened
after the first access would put the barrier inside the instruction it is meant
to bracket. It is the bus lock rather than the monitor because `FEAT_LSE` lands
on x86's side of the line `core::space::BusLock` opens with — a monitor is
licensed to fail spuriously and is paid for by the guest's retry loop, and
`LDADD` has no status register and no retry loop, so a spurious clear would
make it return a wrong answer rather than go round again.

The third is not a lock's problem. `ExclusiveMonitor` walks *live* slots, so a
sibling's store that lands after a `LDXR`'s read but before it registers its
reservation clears nothing, and the `STXR` that follows then commits a value
that was already stale. The fix is to claim the granule before the read issues
rather than after it returns (`Exec::reserve_then_read`), which costs nothing at
all: the same translation, the same read, in the other order. What it changes is
that a load-exclusive which faults leaves the monitor cleared instead of holding
what it held before — DDI 0487 B2.9 licenses a spurious clear explicitly, and
the alternative would be a reservation on a granule the core never loaded.

The fourth is the one that survived the first three fixes, and it is worth the
space because it says something about the whole shape of this. `SpaceView::`
`write_span` broke reservations **before** it transferred and not after, on
purpose: a store that then faults has still broken them, which is a licensed
spurious clear, and clearing afterwards looked like it would only move the
problem. The consequence is that a committing `STXR` had a window *inside
itself*, between telling the monitor and writing the bytes:

```text
core 0: stxr [bus held]              core 1: ldxr [no lock]
  reservation holds
  note_store: walks the live slots,
    and core 1 has none yet
                                       reserve: the slot goes live
                                       read [x0] -> 5, not written yet
  write [x0] <- 6
[bus released]
                                     stxr: nothing broke the reservation,
                                     so it commits 6 — core 0's update is gone
```

Claiming the granule first does not help: the claim lands *after* the sibling
looked. The `STXR` holding the bus does not help either, because the `LDXR` on
the other side was holding nothing. What closes it is the load-exclusive taking
the bus lock too — not to make a write indivisible, since it has none, but so
that claiming the granule and reading it are one transaction against a sibling
in the act of storing. That is what hardware gives for free: the reservation is
taken by the same coherent access that returns the data.

Two lessons rather than one. **A per-instruction fix is not a fix if the other
instruction of the pair is unlocked** — an earlier round of this work argued
that a `LDXR` had nothing to gain from the bus and was wrong, and the failure
rate says how wrong: **33 of 60 runs** lost between one and three updates, on a
loaded host, and none at all on an idle one. And a lost update of *one* in
120 000 is exactly the shape a green run hides, which is why the number that
matters here is a failure rate over dozens of runs rather than a verdict from
one.

There is a second way to close it, in `core::space` rather than here, and it
has since been taken: `SpaceView::write_span` now tells the monitor on **both**
sides of the transfer. That also closes a residual this does not — a *plain*
store racing a `LDXR` had its `note_store` run before the `LDXR` claimed the
granule and its bytes land after the `LDXR` had read, leaving a reservation that
should have been broken and an `STXR` free to commit against it.

The fault trade-off this paragraph declined turned out not to be one. Clearing
on a store that *completes* is required — RISC-V `zalrsc`: "The `sc` must fail
if a store to the reservation set from another hart can be observed to occur
between the `lr` and `sc`"; DDI 0487 B2.9.2: a successful write to the marked
block by another observer "is guaranteed to clear the marking" — while clearing
on a store that *faults* is only permitted (B2.9.5, "cleared at any time without
an application-related cause"). Keeping both calls keeps both properties, and
the second one measured inside the run-to-run spread on the store path: one
extra host instruction per store under callgrind. `core::space::monitor`, "The
transfer is the window", has the rest.

`LDXP`/`STXP` gets the same treatment and one more claim. The 16-byte access is
still two eight-byte bus accesses, because that is what `AddressSpace` offers,
but with the bus lock held across both halves it is indivisible against every
other 16-byte atomic in the machine — the only ones are `CASP` and another
pair, and both now take the lock. What is left is a plain `STR` overlapping half
of it, which is `BusLock`'s documented residual everywhere else as well. There
is no plain 16-byte store to worry about: `STP` of two `X` registers is not
single-copy atomic across its sixteen bytes (DDI 0487 B2.2.1 gives it
per-register atomicity only), so a guest racing one against a pair has no
architectural claim to begin with.

### Which mode, and what it costs

Reachable only under `ThreadingMode::Parallel`, which is opt-in
(`--threading parallel`) and which no board in `machines/` selects — though a
machine file now can, with a `threading` statement; see
[`../techniques/parallel-execution.md`](../techniques/parallel-execution.md).
The argument that `Deterministic` is safe is structural rather than statistical:
one host thread cannot interleave inside an instruction, and the test's
one-instruction-quantum run — finer than any quantum the scheduler hands out —
loses nothing before any of the fixes or after them. That is why this was worth
fixing carefully rather than urgently.

Measured on one core with nothing contending, release build, as nanoseconds per
loop iteration — the instruction under test plus the `subs` and `b.ne` that
drive it, which is the ~105 ns bottom row. Mean of four runs on an idle host:

| | before | after | delta |
| --- | --- | --- | --- |
| `STADD` (`LDADD` with `Rt == XZR`) | 218 ns | 232 ns | +14 |
| `LDXR` + `STXR`, uncontended | 277 ns | 313 ns | +36 |
| `LDAXR` + `STLXR`, uncontended | 287 ns | 314 ns | +27 |
| plain `LDR` | 172 ns | 172 ns | 0 |
| plain `STR` | 173 ns | 174 ns | +1 |
| the loop alone, as the control | 106 ns | 104 ns | −2 |

So about +14 ns for each instruction that now takes the bus, which is one
uncontended mutex with its two fences (`core::space::BusLock` prices its own
acquire at ~13 ns). The `LDXR`/`STXR` row pays it twice, and the acquire/release
row pays less than twice because the suffix no longer emits fences of its own —
a guard that fences at both ends of the instruction is strictly stronger than
any suffix in these encodings asks for.

The ordinary load and store paths are untouched, which is the point of the two
control rows: nothing outside the atomics reads the bus lock, so a board that
executes none pays nothing. What a real guest pays depends entirely on whether
it has `FEAT_LSE` — an Armv8.0 kernel is `LDXR`/`STXR` in every lock, an 8.1
one is `LDADD` and `CASAL` — and in both cases it is one mutex on an
instruction the guest executes thousands of times a second, not millions.

## Where it stops, and what is still in the way

It did not stop, so this section is a list of what the board *has not got*
rather than of what defeated it. In rough order of what the next person will
want:

### `CPU_SUSPEND` is refused rather than implemented

`CPU_ON`, `CPU_OFF` and `AFFINITY_INFO` are real now — see [Two
processors](#two-processors) — and `CPU_SUSPEND` is the one bring-up call that
is not. A kernel told `SUCCESS` would expect to have been suspended and
resumed and this core does neither, so `PSCI_FEATURES` reports it absent and a
kernel discovers the gap rather than falling into it. Implementing it means
deciding what a suspended core does about its generic timer, which is a
scheduler question rather than a PSCI one.

### `GICC_CTLR.EOImode` is not implemented

A write to `GICC_EOIR` drops the priority *and* deactivates. Splitting the two
is what `GICC_DIR` is for, and a kernel that has not asked for the split never
uses it — but a hypervisor does, and this model would deactivate early.

### No RTC, so the guest has no wall clock

A `pl031` is about eighty lines and is the obvious next device. Until then the
kernel says it cannot open an RTC and `date` starts at the epoch.

### The debug registers are storage-free

`DBGBVR0_EL1`, `DBGBCR0_EL1`, `DBGWVR0_EL1` and `DBGWCR0_EL1` read back the zero
a guest did not write, and `ID_AA64DFR0_EL1` reports one breakpoint and one
watchpoint because the fields are "one less than implemented" and zero is the
smallest thing they can say. So `ptrace` hardware watchpoints will not work, and
the guest can tell.

### `AT S1E1R` and its family are not implemented

`PAR_EL1` reads `1` — the fault bit — because that is the only honest answer for
a core that does not perform the translation. A zero would claim a successful
translation to physical address zero.

### The guest sees no caches

`CLIDR_EL1` is zero, so there are no levels to select, `CCSIDR_EL1` describes
nothing and `CSSELR_EL1` is read-as-zero. That is true — there is no cache — and
it means `/sys/devices/system/cpu/cpu0/cache` is empty.

### `psci = "smc"` on a core with no EL3

Stated above and repeated here because it is the one place this board tells a
guest something its identification registers deny. `-p psci=none` takes it back.

### The PL011's DMA and modem control are storage

`UARTILPR` and `UARTDMACR` are stored and read back and drive nothing; the modem
status inputs are the loopback of the control outputs, which is what a driver's
self-test checks and is all any of it is used for.


## Three bugs and one gap this board found

Two were in `cpu.arm.a64` and had been there since it was written; the third
was in the GIC this board brought with it; the fourth was a family of
instructions the core knew it did not have. None of them could have been found
by a test that did not boot an operating system. They are recorded here rather than only
in the commit message because they are the argument for building the board at
all.

### `ID_AA64MMFR0_EL1` claimed `FEAT_LPA2` and a 64 KiB granule

The constant was `0x1000_0025`, with a comment saying it meant "the 4 KiB
granule supported and the 16 KiB and 64 KiB granules not". It did not.

The three granule fields use three different conventions:

| field | bits | "supported" | "not supported" |
| --- | --- | --- | --- |
| `TGran4` | 31:28 | `0b0000` | `0b1111` |
| `TGran64` | 27:24 | `0b0000` | `0b1111` |
| `TGran16` | 23:20 | `0b0001` | `0b0000` |

and `TGran4 == 0b0001` is not "supported", it is **supported with `FEAT_LPA2`**
— 52-bit addressing and a different descriptor format. So the old value told a
guest it could use 52-bit page tables and a 64 KiB granule, and
[`mmu.rs`](../../src/cpu/arm/a64/mmu.rs) faults on both.

The symptom was a **level-0 translation fault on a write**, inside the kernel's
`__pi_early_map_kernel`, with the MMU already on and `VBAR_EL1` not yet set —
that is, a completely silent hang. The value that looks symmetrical is wrong in
two fields at once, which is why the constant is now written out field by field
with its own paragraph.

### The identification register space was not read-as-zero

`MRS x0, ID_AA64DFR0_EL1` raised `UNDEFINED`, because that encoding had no row
in the system-register table. It is one of sixty-four encodings at `op0 == 3,
op1 == 0, CRn == 0` that DDI 0487 reserves for feature identification, and the
architecture makes the unallocated ones **RES0 and read as zero** rather than
UNDEFINED.

That rule is load-bearing rather than lenient. An operating system's feature
detection reads the whole block unconditionally, because a register that reads
zero is how the architecture says "this extension is absent" — so a core that
raised `UNDEFINED` for an encoding it had never heard of would take an exception
during boot for every extension invented after it was written, and would need a
new row in the table each time one was.

`sysreg::is_id_space` is that rule. `ID_AA64DFR0_EL1` itself is a real row
returning `DebugVer == 0b0110`, because zero there means "no debug architecture"
and a kernel reads it before deciding what to do.

### And a third, in the GIC this board brought with it

`GICD_IPRIORITYR` and `GICD_ITARGETSR` are **byte-per-interrupt arrays**, and
the architecture makes them byte-accessible. The first model here accepted
32-bit accesses only, with a comment saying nothing on this board would use a
byte one.

`gic_set_affinity` uses one, on the first interrupt the kernel ever requests —
its own console's — because a read-modify-write of the containing word would
race with the three interrupts either side of it. The result was an external
abort in `gic_set_affinity` called from `pl011_startup` called from
`console_on_rootfs`: a kernel that had completed all of its own initialisation
and died opening the console.

Narrow accesses are now accepted in `0x400..0xc00` and refused everywhere else,
because the rest of the distributor is bit arrays with write-one-to-set
semantics and a read-modify-write of one would set bits the guest never named.

### And one gap rather than a bug: `LDTR`/`STTR`

The unprivileged load/store family was on `cpu.arm.a64`'s own list of what it
deliberately did not have. `__arch_clear_user` is `STTR` in a loop, so the
kernel reached `Run /init as init process`, started `/init`, and died in
`load_elf_binary` clearing the tail of a segment.

It is now implemented, and the interesting half is not the encoding — bits
11:10 are `0b10` where `STUR` has `0b00` — but the *permission check*: executed
at EL1 these are translated with **EL0's** permissions, which is how a kernel
touches a user page on a process's behalf and gets a fault rather than a
privilege escalation. `Exec::unpriv` is that one bit, and
`an_unprivileged_store_at_el1_is_checked_with_el0s_permissions` is the test that
would fail if it were an ordinary store.

## The format is shared; the generator is not

[`src/dev/fdt.rs`](../../src/dev/fdt.rs) is the DTB *encoder*, behind `dev-fdt`,
and both boards depend on it. It used to be two files — `dev/arm/fdt.rs` was a
copy of `dev/riscv/fdt.rs`, written as a strict subset so the move would be a
deletion, which is exactly what it turned out to be. Chapter 5 of the
Devicetree Specification describes a container: a header, a reservation block,
a token stream, an interned strings block, every integer big-endian whatever
the guest is. Not one byte of that knows what a hart or a GIC is.

The *generator* is not shared and should not be.
[`dev/arm/dt.rs`](../../src/dev/arm/dt.rs) and
[`dev/riscv/dt.rs`](../../src/dev/riscv/dt.rs) write two different documents
through the one encoder:

| | `riscv-virt` | `arm64-virt` |
| --- | --- | --- |
| peripherals | under a `/soc` node with `ranges` | at the root |
| `#interrupt-cells` | 1 | 3, and the binding subtracts the base again |
| the controller | a PLIC, with `riscv,ndev` | a GIC, two apertures in one node |
| the processors | `riscv,isa`, `mmu-type` | `MPIDR_EL1` affinity, `enable-method` |
| power | `syscon-poweroff` / `syscon-reboot` nodes | a `psci` node with a conduit |

Merging those would produce a generator with an architecture switch in every
branch, so what moved was the four hundred lines that had no architecture in
them at all.

`power.rs`'s `Signal` **is** still `riscv/syscon.rs`'s twin, and stayed. The two
`Request` enumerations are not the same type: a syscon can report an exit code
(`Fail(u16)`, which is how a headless RISC-V test says *why* it stopped) and
PSCI has no way to express one, so unifying them means either giving every
AArch64 board a variant its firmware interface cannot raise, or giving the
RISC-V board's tests a narrower signal than the device can emit. The `power`
host-object kind is deliberately *not* called `signal` for the same reason: a
[`HostKind`](../../src/core/hosts.rs)'s identity is its name alone, so two
modules sharing a name must agree about the type stored under it, and these two
do not.

## What the tree is generated from, and the three things it is not

Every address, size and interrupt number in the generated device tree is read
back out of the realized machine: a node's `reg` from the mapping the `map`
statement made, its `interrupts` from the net its pin drives looked up in the
GIC's own pin table, the `memory` nodes from every RAM region in the space. The
GIC's two apertures become one node with two `reg` entries, joined from two
mappings.

Three things are declared on the `arm.boot` object instead, and each is a fact
about something that is **not a region in an address space**:

* **the processors** — a core is not a region and there is no route from a `dyn
  Device` to a `Cpu`, so the count and the `MPIDR_EL1` affinities are declared;
* **the PSCI conduit** — `SMC` is an instruction, not an address;
* **the generic timer's four interrupt numbers** — the timer is inside the core,
  so the wire that carries it out starts somewhere the generator cannot see.
  These are duplicated in the machine file's `wire` statements, which is the one
  number on this board written down twice.

The RISC-V generator has the same three-line limitation for the same reason, and
both collapse the day `RealizeCtx` carries the machine graph (`ROADMAP.md`
§4.4).

## The interrupt numbers, and the base the binding subtracts again

The board wires `uart.irq -> gic.spi1`, and that `1` is the only place the
number appears. The generated tree says `interrupts = <0 1 4>`.

This is the detail that makes a hand-written tree wrong. A GIC device tree
specifier is three cells — kind, number, flags — and the *number* is relative to
the kind's base, not the architectural interrupt id:

| what the board wires | architectural id | what the tree says |
| --- | --- | --- |
| `gic.spi1` | 33 | `<0 1 4>` |
| `gic.ppi11` | 27 | `<1 11 0xf08>` |
| `gic.ppi14` | 30 | `<1 14 0xf08>` |

The `0xf00` in a private interrupt's flags is the GICv2 CPU mask, and `8` is
`IRQ_TYPE_LEVEL_LOW`, which is how the generic timer's interrupts are
conventionally described.

## Provenance

Written from the *ARM Generic Interrupt Controller Architecture Specification
v2.0* (IHI 0048), the *PrimeCell UART (PL011) Technical Reference Manual* (DDI
0183), the *Arm Power State Coordination Interface* (DEN 0022), the *SMC Calling
Convention* (DEN 0028), the *Arm Architecture Reference Manual for A-profile*
(DDI 0487) and the *Devicetree Specification* v0.4. Each module names the
sections it used.

Two facts a board like this needs are stated in the Linux kernel's own boot
documentation, which is GPL-2.0 and **was not read**: the AArch64 `Image`
header layout, and the register state a kernel is entered with. Both were taken
instead from permissive implementations that agree with it field for field, and
each is cited where it is used:

| fact | source | licence |
| --- | --- | --- |
| the 64-byte `Image` header, and `text_offset = 0x80000` when `image_size == 0` | ARM boot-wrapper `scripts/AA64Image.pm` | BSD-3-Clause |
| the same header, emitted field by field | EDK II `ArmVirtPkg/ArmVirtQemuKernel.fdf` | BSD-2-Clause-Patent |
| `flags` bit 0 (endianness), and the `0x80000` default again | crosvm `kernel_loader/src/arm64.rs` | BSD-3-Clause |
| `flags` bits 2:1 (page size) | Zephyr `arch/arm64/core/header.S`, Apache NuttX `arm64_head.S` | Apache-2.0 |
| `x0` is the DTB and `x1`-`x3` are zero; MMU and caches off, `DAIF` masked; a 2 MiB-aligned base | ARM boot-wrapper `common/boot.c`, `arch/aarch64/{boot.S,init.c}` | BSD-3-Clause |
| `enable-method = "psci"`, `compatible = "arm,psci-1.0", "arm,psci-0.2"`, `method = "smc"` | Trusted Firmware-A `fdts/`, ARM boot-wrapper `scripts/addpsci.pl` | BSD-3-Clause |
| the PSCI function ids and return codes | DEN 0022D, cross-checked against TF-A `include/lib/psci/psci.h` | Arm spec / BSD-3-Clause |

**What could not be established from a permissive source**, and is therefore not
acted on anywhere in this tree:

* the meaning of `Image` header `flags` bit 3. Its *position and name*
  (`__HEAD_FLAG_PHYS_BASE`) are attested by Apache NuttX; what it means — "the
  kernel may be placed 2 MiB-aligned anywhere in physical memory" — is stated
  only in the Linux documentation. `arm.loader` reads the flags word, reports
  it, and does nothing with that bit.
* that `res5` is a PE/COFF header offset. Every permissive source labels the
  last word `res5` and writes zero. `arm.loader` ignores it.
* that `flags` bits 4 and above are reserved must-be-zero.
* the DTB placement constraints a kernel imposes (8-byte alignment, a size
  bound, no overlap with the kernel or the ramdisk). This board's DTB is 8-byte
  aligned because the *Devicetree Specification* asks for it, not because of
  anything read about Linux.

No emulator source of any licence was consulted, and in particular no Linux
driver — `docs/README.md`'s exclusion table names that as the trap people
actually fall into.
