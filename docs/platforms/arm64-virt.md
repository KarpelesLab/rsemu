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

## Where it stops, and what is still in the way

It did not stop, so this section is a list of what the board *has not got*
rather than of what defeated it. In rough order of what the next person will
want:

### SMP needs a requester-to-CPU-interface map, and it is the GIC that needs it

`Registers::cpu_of` in [`gic.rs`](../../src/dev/arm/gic.rs) returns zero. That
is correct for a one-core board and wrong for any other, and the reason is a
seam rather than laziness: a GIC's banked registers answer differently
depending on **which core is asking**, the bus carries `MemAttrs::requester`,
and there is no map from a requester id to a CPU interface number. Everything
else in the model is already banked per CPU, and `parse_input` already accepts
`cpu<C>ppi<N>`.

PSCI `CPU_ON` is the other half and is harder: servicing it means reaching a
*sibling* core from inside the one executing the `SMC`, and a `dyn Device` has
no route to a `Cpu`. `psci.rs` answers `ALREADY_ON` for a processor the board
declares and `INVALID_PARAMETERS` for one it does not, which is honest for a
board whose every processor is running.

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
