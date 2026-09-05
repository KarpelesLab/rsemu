# RISC-V `virt` board

Consumed by: `boards/riscv-virt` — the first machine that boots a real
operating system.

## Why this board

It is the smallest credible target that boots upstream Linux: a RISC-V hart, a
CLINT, a PLIC, a 16550 UART, and virtio-mmio devices. No PCI, no ACPI, no
legacy. Everything it needs is specified in freely available documents, and the
guest discovers the topology from a device tree we generate — so there is no
hidden convention to reverse-engineer.

## Components and their specifications

| Component | Source |
| --- | --- |
| Hart (RV64GC) | [`../cpu/riscv.md`](../cpu/riscv.md) |
| CLINT (timer + software interrupts) | RISC-V privileged spec; `mtime`/`mtimecmp` semantics |
| PLIC (external interrupts) | [RISC-V PLIC specification](https://github.com/riscv/riscv-plic-spec) |
| SBI (firmware interface) | [riscv-sbi-doc](https://github.com/riscv-non-isa/riscv-sbi-doc) |
| Firmware | [OpenSBI](https://github.com/riscv-software-src/opensbi) — **BSD-2-Clause**, readable and usable |
| 16550 UART | [`../devices/network-input.md`](../devices/network-input.md) and the National Semiconductor PC16550D datasheet |
| virtio-mmio | [`../buses/virtio.md`](../buses/virtio.md) |
| Device tree | [Devicetree Specification](https://www.devicetree.org/specifications/) |

## Implementation notes

- rsemu **generates** the device tree from the realized machine graph and passes
  it to firmware. That is a genuine test of the machine model: if the DTB can be
  produced mechanically from the topology, the topology is well-formed.
- Boot chain: our firmware load → OpenSBI (M-mode) → kernel (S-mode). SBI calls
  are the ABI between them.
- Networking comes from `pktkit` behind virtio-net; storage from `fstool`
  behind virtio-blk. Neither needs board-specific code.
- The hart's `time` CSR reads the CLINT's `mtime`, through `timer = clint` in
  the machine file. `time` is architecturally a *view* of the platform timer,
  not a counter the hart owns, and before that line existed `rdtime` read zero
  — which every kernel that takes its clocksource from it turns into an
  immediately-expired deadline and a live-lock. The wiring is
  [`Device::export`](../../src/core/device.rs), and it is named explicitly
  rather than searched for.

## Choosing an execution engine

`cpu.riscv` takes three, and this board makes it a parameter so one file runs
all of them:

```console
$ rsemu run riscv-virt --media firmware=fw_payload.bin -p engine=jit
```

- **`interp`** — the interpreter, and the oracle everything else is measured
  against. The default.
- **`jit`** — the translation runtime in [`src/jit`](../../src/jit): guest
  instructions lifted into IR blocks by
  [`cpu::riscv::lift`](../../src/cpu/riscv/lift.rs), cached under
  `(pc, physical page)`, executed by the portable IR backend. Runs everywhere
  the crate does.
- **`jit-host`** — the same runtime with the **host code generator** attached,
  so blocks are lowered to machine code by
  [`jit::x86`](../../src/jit/x86) on x86-64 Linux in a build with `jit-x86`.
  Anywhere else it falls back to `jit`'s backend and answers identically.

All three are **indistinguishable to the guest**, cycle counts included, and
that is asserted rather than hoped: `tests/riscv_virt_engines.rs` runs this
board on each and compares `Machine::state_hash` at ten checkpoints, then moves
a snapshot between engines in both directions.

It holds on a real guest too. OpenSBI plus a Debian riscv64 kernel and a busybox
initramfs, 512 MiB, one binary, `--headless` so nothing is rate-limited to the
wall clock — over sixty seconds of virtual time, which is the boot, and over
four minutes, which is well past the shell prompt:

| `engine` | 60 s of guest time | 240 s of guest time |
| --- | --- | --- |
| `interp` | 25.1 s | 122.3 s |
| `jit` | 19.8 s (**1.27×**) | 103.7 s (**1.18×**) |
| `jit-host` | 10.3 s (**2.44×**) | 56.8 s (**2.15×**) |

Every cell is the median of three interleaved runs, and in each sweep all three
engines — and the binary from before the last change, run in the same sweep —
finished on one state hash: `0x887e28c90a99e82b` at sixty seconds and
`0xb561134639a875b9` at four minutes. Those numbers belong to *this* invocation,
because a state hash names a stopping point as much as a machine: `--for 240s`
stops at a virtual instant, and the environment harness's `RSEMU_RISCV_QUANTA`
stops after a count of quanta, so the two do not hash alike and neither is more
correct.

The host code generator used to *lose* to the portable one on this guest, at
0.50×, and none of the five things that fixed it was the code it emits.
**Blocks are chained** — 86% of them are reached by following a patched exit,
where the count was previously zero in every run. **A compile stopped costing
144 µs**, which is what two `mprotect` calls over a 256 MiB code buffer had
been costing before `jit::x86::buf` learned to flip a page-sized window
instead. **A guest load stopped costing a call**: the software TLB's fast path
is inlined into generated code now that a hart publishes a `MemPlan`, so 97.3%
of compiled loads are a mask, a compare, an add and a `mov` rather than a trip
through the hart's translation and PMP. **A guest store stopped costing one
too** — 99.8% of them, over a second set whose entries were admitted on write
permission and filled by a walk that set the page's dirty bit, with one thunk
left to pay the tick, the store's dirty bitmap, the reservation and the
self-modifying-code check. And **the PMP scan stopped being asked sixteen
entries at a time**: `pmp_allows` was 265 host instructions a call over 5.5
million calls — a fifth of all emulation — and it is now memoized over the span
its answer is provably constant on.

The last two were measured the way this file insists on, with the old binary
and the new one interleaved in one three-rep sweep over the same guest:

| `engine` | 240 s before | 240 s after | |
| --- | --- | --- | --- |
| `interp` | 155.1 s | 122.3 s | **1.27×** |
| `jit` | 118.6 s | 103.7 s | **1.14×** |
| `jit-host` | 69.1 s | 56.8 s | **1.22×** |

The **interpreter** gains most, and that is the point about where the PMP scan
lived: on the path every engine takes, not on the JIT's. It is also why
`jit-host`'s headline ratio *falls* from 2.28× to 2.15× while the engine itself
got 22% faster — the control moved too, and a ratio against a moving control is
the wrong number to quote on its own. Under callgrind, which is
host-CPU-independent, `Hart::advance` taken inclusively — emulation and nothing
else — went from 7.37 G host instructions to 5.25 G over three seconds of this
boot, **28.7% fewer**.
[`src/cpu/riscv/engine.rs`](../../src/cpu/riscv/engine.rs) has the reasoning and
the measurements behind every one of those claims, including what it costs to
keep the engines identical, and `src/jit/fast.rs` has the argument for why a
*paged* hart may publish a plan at all — and what a **store** plan promises on
top of a load's.

**Measure the interpreter in the same sweep as the engines it is the control
for.** This is a shared machine, and a 150-second run of the same binary
varied by 12% between sweeps run twenty minutes apart — larger than most of the
effects being measured. Each table above is the median of an interleaved
three-rep sweep, and a before-and-after taken from two different sittings is
not evidence. Interleave the *binaries* too when the change is to one engine:
the 13% the inlined fast path is worth was measured with the old and the new
`jit-host` in the same sweep. An earlier sweep of the same two binaries, on the
host CPU this machine had before, read anywhere from 3.6% to 9.4% depending on
what else was running — same code, same guest, same instruction counts. When a
machine is busy, prefer the median of the *per-rep ratios* to the ratio of the
medians: they agree here (13.8% against 12.9%) and the first degrades more
gracefully. They agreed again on the sweep above — 1.268 against 1.268 for the
interpreter, 1.215 against 1.218 for `jit-host` — which is what a quiet machine
looks like, and is worth recording so a sweep where they *disagree* is read as
the warning it is.

A build without `cpu-riscv-lift` and `jit` **refuses** both JIT values with a
message saying which features it wants, rather than interpreting quietly — an
engine that silently is not the one you asked for is how a JIT stays unmeasured
for a year. A build that *has* them but is not x86-64 Linux, or that lacks
`jit-x86`, is a different case and **falls back**: `jit-host` runs the same
blocks from the same cache on the portable backend. The first is a configuration
error and the second is a portability property, so they are treated differently
on purpose.

## Booting something real

Everything below is fetched, never committed
(`scripts/fetch-testdata.sh riscv linux`), and gated behind environment
variables so an ordinary `cargo test` skips it.

```console
$ export TD=testdata/riscv
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_RAM=1G RSEMU_RISCV_QUANTA=8000000 \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

OpenSBI's `fw_jump` runs at `0x80000000` and hands control to `0x80200000` in
S-mode, which is where a RISC-V `Image` expects to be.

## Booting to a shell

A kernel with no root filesystem panics in `prepare_namespace`, correctly. Give
it a ramdisk and it does not:

```console
$ scripts/fetch-testdata.sh linux initramfs
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_INITRD=$TD/initramfs.cpio \
  RSEMU_RISCV_BOOTARGS='console=ttyS0 earlycon=sbi' \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=2000000 \
  RSEMU_RISCV_INPUT='rsemu# =>uname -a\n' \
  RSEMU_RISCV_STOP_AT='GNU/Linux' \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

`initramfs.cpio` is **built** by the fetch script, not downloaded: one
statically linked riscv64 busybox out of Debian's own package, a `/dev/console`
node, and a ten-line `/init`. The `newc` cpio writer is forty lines of shell in
`scripts/fetch-testdata.sh`, so the fixture needs no cross toolchain and no
`cpio(1)`, and every entry is written with mtime 0 so the archive is
reproducible.

The board carries the ramdisk the way a real one does. A `riscv.loader` writes
it into DRAM at `initrd_addr`, and the boot ROM puts
`/chosen/linux,initrd-start` and `linux,initrd-end` in the generated tree — the
same media slot named twice, so the length is read from the bytes in both
places and only the address is written down more than once.

`RSEMU_RISCV_INPUT` types at the guest: one `marker=>text` step per line, fed
when the guest has printed `marker`. That is what makes the console
*bidirectional* rather than write-only, and matching on output rather than on
elapsed time keeps the run deterministic.

## Booting with the virtio disk

The Debian kernel builds `virtio_mmio` and `virtio_blk` as modules, so an
unadorned initramfs never claims the `virtio.blk` this board provides.
`initramfs-virtio` is the same archive with those two modules in `/lib/modules`
and an `insmod` loop at the top of `/init`; it resolves the kernel package from
the fetched image's own version banner, because a module whose vermagic
disagrees is refused at load time.

```console
$ scripts/fetch-testdata.sh linux initramfs-virtio
$ RSEMU_RISCV_INITRD=$TD/initramfs-virtio.cpio RSEMU_RISCV_DISK=$TD/disk.img …
```

`--disk` / `RSEMU_RISCV_DISK` binds the `disk` media slot, which is the front of
the disk; the `storage` parameter pads it out with zeroes. That is the
media-slot path: bytes, copied into a `RamStore`, `no_std`, and what a wasm
build runs on.

## Booting off a qcow2

The other path backs the same slot with a host **file**. `virtio.blk` stores
its bytes behind `dev::medium::Medium`, the seam an ATA drive's platter and an
NVMe namespace already use, so `--drive` works here for the reason it works
there and no image format is parsed in rsemu — sparse raw, qcow2, DMG and LUKS
all come from `fstool`.

```console
$ rsemu run riscv-virt --media firmware=fw.bin --drive disk=root.qcow2,new=64M
```

The medium brings its own capacity, so `storage` is ignored and the guest sees
the image's size; a 16 GiB disk costs 16 GiB of *disk* and nothing in host
memory until the guest touches it. Guest writes go into the file, so the next
run is a reboot of the last one — and a machine snapshot **references** the
image (flushing it first) rather than copying it, which is what
[`storage.md`](../buses/storage.md) argues at length.

`RSEMU_RISCV_DRIVE` is the same thing for the test harness, with
`RSEMU_RISCV_DRIVE_NEW=<size>` to create the image and `RSEMU_RISCV_DRIVE_RO`
to open it read-only (which the device reports as `VIRTIO_BLK_F_RO`, so the
guest finds out before it tries):

```console
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_INITRD=$TD/initramfs-virtio.cpio \
  RSEMU_RISCV_DRIVE=$TD/root.qcow2 RSEMU_RISCV_DRIVE_NEW=64M \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=6000000 \
  RSEMU_RISCV_BOOTARGS='console=ttyS0 earlycon=sbi' \
      cargo test --release --all-features firmware_from_the --lib -- --nocapture
…
[  225.340000] virtio_blk virtio0: [vda] 131072 512-byte logical blocks (67.1 MB/64.0 MiB)
```

64 MiB rather than the board's `storage = 16M`, which is the whole point: the
guest is reading the image's geometry, not the machine file's. Two runs against
that image are a write and a reboot:

```console
rsemu# echo rsemu-qcow2-round-trip | dd of=/dev/vda bs=512 count=1 conv=sync,fsync
WROTE-OK
… second run, same qcow2, no RSEMU_RISCV_DRIVE_NEW …
rsemu# head -c 22 /dev/vda
rsemu-qcow2-round-trip
```

**`fsync`, not `sync`.** A guest write is durable in the image when the guest
asks for it to be — `VIRTIO_BLK_T_FLUSH`, which is what `dd conv=fsync` and
`fsync(2)` on the device produce — and `sync(1)` alone is not that. Linux
writes a bare block device's dirty pages back on `sync(2)` but issues the
device cache flush from `blkdev_fsync`, so with `sync` alone the data cluster
reaches the qcow2 and the L2 entry that finds it does not; the next open sees a
hole. That is the flush contract working as specified rather than a defect, but
it is sharp, and nothing yet flushes an image when a *run* ends.

## Booting UEFI

`edk2-riscv-code.fd` from EDK2's `OvmfPkg/RiscVVirt` (BSD-2-Clause-Patent) is
built for the board's NOR flash at `0x20000000`, with the variable store in a
second bank at `0x22000000`. Both are **real `flash.cfi` devices** in
`machines/riscv-virt.machine` — parallel NOR with the Intel/Sharp command set,
a CFI query structure, per-block erase and the bit-clearing-only program
semantics that fault-tolerant write depends on. Staging RAM under those windows
gets the firmware as far as the DXE dispatcher and no further, because
`VirtNorFlashDxe` will not install `gEfiVariableWriteArchProtocolGuid` against
memory.

The one splice that remains is the trampoline, and it has nothing to do with
the flash: `fw_jump.bin` has its hand-off address compiled in at
`0x80200000`, so eight bytes there — `lui t0, 0x20000` then `jr t0` — bridge
OpenSBI to the flash base, leaving `a0` (hart id) and `a1` (device tree) as
OpenSBI set them. OpenSBI's `fw_dynamic` takes the next stage's address at run
time and would need no trampoline at all.

```console
$ printf '\xb7\x02\x00\x20\x67\x80\x02\x00' > $TD/tramp.bin
$ cp /usr/share/qemu/edk2-riscv-vars.fd $TD/vars.fd     # a writable copy
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/tramp.bin \
  RSEMU_RISCV_FLASH0=/usr/share/qemu/edk2-riscv-code.fd \
  RSEMU_RISCV_FLASH1=$TD/vars.fd \
  RSEMU_RISCV_FLASH1_OUT=$TD/vars.fd \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=6000000 \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

## Two harts

[`machines/riscv-virt-smp.machine`](../../machines/riscv-virt-smp.machine) is
this board with a second hart. It is a **separate file** rather than a `param`
on the first, for the reason `arm64-virt-smp` is separate from `arm64-virt`:
the description language declares objects and cannot be told how many to make,
so a one-hart run of a two-hart file would be a board with a spare hart parked
in firmware. `riscv-virt` is unchanged, down to the byte in its device tree,
and it is still the file `tests/riscv_virt_engines.rs` runs its three-engine
equivalence gate on.

### What had to change, and it was less than on the other two architectures

**No device model changed** — not a line of `clint.rs`, `plic.rs`, `dt.rs` or
`boot.rs`, and nothing in `src/cpu/riscv/` or `src/core/`. The diff under
`src/` is a catalog entry, module documentation and tests. That is not modesty
about the work; it is the architectural fact this board exists to demonstrate,
and it is worth stating next to what the other two boards needed.

`arm64-virt-smp` and `pc-at-smp` both had to teach a controller *who was
asking*. A GICv2 banks its low 32 interrupt ids — one address, N registers
(IHI 0048 §4.1.3) — and an x86 local APIC has one architectural page shared by
every processor. Both are demultiplexed on `MemAttrs::requester`, resolved from
the machine file's `processors = [...]` through `BindCtx::peer`.

RISC-V has no such register. Both of its controllers are indexed by hart id in
the *address*:

| | where hart `h`'s copy lives | source |
| --- | --- | --- |
| software interrupt | `MSIP + 4·h` | ACLINT specification, MSWI register map |
| timer comparator | `MTIMECMP + 8·h`, i.e. `0x4000 + 8·h` | ACLINT specification, MTIMER register map |
| interrupt enables | `0x2000 + 0x80·c` for context `c` | PLIC specification §3 |
| threshold and claim | `0x200000 + 0x1000·c` | PLIC specification §3 |

A hart reaches its own registers because it knows its own `mhartid`, which the
boot ROM has always put in `a0`. `mtime` is the one genuinely shared register,
and it is shared on real hardware too. So `harts = 2` on the CLINT and on the
PLIC is the entire model change, and both blocks were written that way in their
first commit — `Clint::with_harts` allocates a comparator and an `msip` bit per
hart, `Plic::build` allocates `harts × 2` contexts, and both refuse a snapshot
whose hart count disagrees. There is **no `processors` property on this board
and no `BindCtx::peer` call anywhere in it.**

The device tree needed nothing either. [`dt.rs`](../../src/dev/riscv/dt.rs)
takes the hart count from `riscv.boot` and already emitted one `cpu@N` node
with its own `riscv,cpu-intc` phandle per hart, and one phandle-and-cause pair
per hart in the CLINT's and the PLIC's `interrupts-extended` — causes 3 and 7
for machine software and machine timer, 11 and 9 for machine and supervisor
external (Privileged Architecture, the interrupt cause table).

What the file does add is eight wires instead of four: `clint.mtip1`,
`clint.msip1`, `plic.meip1` and `plic.seip1` into the second hart. And an
**IPI needs no mechanism at all** — on this architecture an interprocessor
interrupt *is* a store of 1 to a sibling's `msip` word, delivered as cause 3.
`a_store_to_the_other_harts_msip_is_an_interprocessor_interrupt` in
`src/dev/riscv/tests.rs` is that in twenty instructions: hart 0 writes
`0x02000004`, hart 1 is in `wfi` and lands in `mtvec`.

The CLINT's re-entrancy was already right, and this is where a board like this
usually breaks. A CPU holds a `BUS`-ranked lock across the accesses it issues,
so delivering an IPI — a device reaching *another hart* from inside a guest
access — must not take a lock above it. `Registers::write` mutates `msip` under
the state lock, releases it, and only then calls `drive_msip`, which takes the
output pins at `LockRank::LEAF`. That contract is `CLAUDE.md`'s and it was
being kept before there was a second hart to keep it for.

### How the second hart starts: SBI HSM, not a spin table

This is the one design decision the board file makes, and it is different from
`arm64-virt-smp`'s.

Both harts come out of reset at `0x1000` — that is what a reset vector is — and
the boot ROM's five instructions send both of them to `0x80000000` with
`mhartid` in `a0`. There is no parking loop in the ROM, because on this
architecture the firmware already provides one. OpenSBI picks one hart to
initialise the platform on — its own banner names it, `Boot HART ID : 0` —
and holds the others until something asks the **Hart State Management**
extension to start them: `sbi_hart_start(hartid, start_addr, opaque)`, SBI
specification v2.0 §9. Linux's `smp: Bringing up secondary CPUs` is that call.

What the board has to supply for it is exactly what the CLINT already supplied,
because **HSM's wake-up is an IPI**: a hart waiting to be started is woken by a
write to its own `msip` word. So the interprocessor path above is on the
bring-up path and not only on the reschedule one, and `IPI1: 4620 2007` in the
`/proc/interrupts` further down is the same wire doing the ordinary job
afterwards.

The alternative was a boot ROM that parks secondaries by hart id, the way
`arm.boot`'s reset vector parks everything but affinity 0 on a release table.
It was rejected for two reasons. This board's documented boot chain **is**
OpenSBI, so SBI HSM is present in every guest anybody boots on it, and a kernel
would use it in preference to anything else on offer. And a parking loop would
be a second, rsemu-specific bring-up protocol invented here rather than read
out of a specification — exactly the sort of thing the AArch64 board had to
resort to only because PSCI `CPU_ON` needs a route to a *sibling* core that
`cpu.arm.a64` has not got. `cpu.riscv` needs no such route: hart 0 starts hart 1
by writing a register through the ordinary address space, which is what the
hardware does.

Nothing in `riscv.boot` therefore grew a `secondary` property, and nothing in
`src/cpu/riscv/` was asked for.

### The exclusive monitor is per hart, and Linux's spinlocks are built on it

**Read this before quoting the boot below as evidence that anything atomic
works.** `cpu::riscv`'s `reservation` is private per-hart state in
`src/cpu/riscv/exec.rs`: it is broken by *this* hart's stores, its AMOs and its
traps, and by nothing a sibling does. So an `sc.d` the architecture requires to
fail can succeed, and the sibling's update is lost. It is the identical defect
`docs/platforms/arm64-virt.md` records for `arm64-virt-smp`, from the same
cause — `MemAttrs::exclusive` carries the flag to the leaf and nothing reads it
back, because the **global monitor on the address space** that would is not
built.

Why this board still boots to a shell: a kernel's spinlocks are uncontended
almost always, and two harts rarely reach the same lock inside one scheduler
quantum. That is luck about timing rather than a property of the model. Linux
on RISC-V also gets some of it for free — LLVM emits a single `amoadd.d` for an
atomic add where AArch64 needs an `ldxr`/`stxr` pair, and an AMO here is one
instruction that reads and writes without yielding, so it is atomic by
construction under a scheduler that runs one hart at a time. `cmpxchg` is
`lr`/`sc` and is not.

So: treat the boot below as evidence that bring-up, per-hart timer and external
interrupt delivery, and IPIs work. It is not evidence about `lr`/`sc`.

### What a kernel does with it

Debian's `riscv64` kernel — the same image as the single-hart gate, behind the
same OpenSBI `fw_jump.bin` — on `riscv-virt-smp`:

```console
$ export TD=testdata/riscv
$ RSEMU_RISCV_MACHINE=riscv-virt-smp \
  RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_INITRD=$TD/initramfs.cpio \
  RSEMU_RISCV_BOOTARGS='console=ttyS0 earlycon=sbi' \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=8000000 \
  RSEMU_RISCV_INPUT='rsemu# =>cat /proc/interrupts; nproc; head -3 /proc/stat\n' \
  RSEMU_RISCV_STOP_AT='cpu1 ' \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

`RSEMU_RISCV_MACHINE` is the only difference from the single-hart invocation,
which is the point: same firmware, same kernel, same ramdisk, same script.

OpenSBI sizes the board off the generated tree and finds both harts (its own
banner, with the lines this section is about kept and the rest elided):

```text
Platform Name               : rsemu riscv-virt-smp
Platform HART Count         : 2
Platform IPI Device         : aclint-mswi
Platform Timer Device       : aclint-mtimer @ 10000000Hz
Platform HSM Device         : ---
…
Standard SBI Extensions     : ipi,pmu,srst,hsm,rfnc,time,base,legacy,dbcn
…
Domain0 Boot HART           : 0
Domain0 HARTs               : 0*,1*
```

`Platform HSM Device : ---` is not a gap. That line reports a platform-specific
hart power controller; OpenSBI's *generic* HSM implementation — the one that
parks a hart until `sbi_hart_start` arrives and wakes it with an IPI — needs no
device behind it, and `hsm` is in the extension list below it. `0*,1*` is the
domain saying both harts are assigned to it and both may boot.

And the kernel:

```text
[    0.000000] Machine model: rsemu riscv-virt-smp
[    4.379000] smp: Bringing up secondary CPUs ...
[    4.481000] smp: Brought up 1 node, 2 CPUs
[    6.786000] cpu1: Ratio of byte access time to unaligned word access is 4.00, unaligned accesses are fast
[    6.817998] cpu0: Ratio of byte access time to unaligned word access is 1.99, unaligned accesses are fast
```

The last two lines are what makes this evidence rather than a claim about a
device tree. The unaligned-access probe runs **on each hart**, times a copy
loop there, and prints from that hart — so `cpu1:` is a line hart 1 printed
about work hart 1 did. The two ratios differ because the harts measured at
different points in the same virtual timeline, which is what a real pair of
cores does too.

One more line says the same thing from the other direction, and it is a
complaint rather than a status report:

```text
[   57.142009] rcu: INFO: rcu_sched self-detected stall on CPU
[   57.145000] rcu: 	1-....: (5249 ticks this GP) …
[   57.147000] rcu: 	(t=5250 jiffies g=-1131 q=2 ncpus=2)
[   57.150000] CPU: 1 UID: 0 PID: 1 Comm: swapper/0 Not tainted …
[   57.152000] epc : keccakf_round+0x352/0x4f8
[   57.154000]  ra : crypto_sha3_final+0xf2/0x1c0
```

`ncpus=2`, and PID 1 — the kernel's own init thread — is running on **hart 1**
when it stalls. This is the same artifact the single-hart boot has and the same
cause: virtual time here is derived from bus accesses, a crypto self-test makes
a great many of them, and the kernel decides 5250 jiffies went by without a
grace period. It is not an SMP defect; what is new is only that the scheduler
put the thread on the second hart, which nothing but a running second hart can
do. The kernel warns and carries on to a shell.

And it runs userspace on both. Typed at the busybox prompt, three minutes of
host time into the run:

```text
rsemu# cat /proc/interrupts; nproc; head -3 /proc/stat
           CPU0       CPU1
 10:      53635      58007  RISC-V INTC   5 Edge      riscv-timer
 12:         64          0  SiFive PLIC  10 Edge      ttyS0
IPI0:        64         67  Rescheduling interrupts
IPI1:      4620       2007  Function call interrupts
IPI2:         0          0  CPU stop interrupts
…
2
cpu  38 0 18424 7173 0 0 38 0 0 0
cpu0 25 0 6883 5036 0 0 19 0 0 0
cpu1 13 0 11540 2136 0 0 19 0 0 0
```

Five separate claims, each checked by a different column:

* **`nproc` → 2.** Userspace agrees with the kernel.
* **The timer row is per hart** — 53 635 against 58 007. That is `riscv-timer`
  on `RISC-V INTC 5`, the *supervisor* timer, which arrives because OpenSBI
  answers `sbi_set_timer` by programming that hart's own `mtimecmp` in the
  CLINT and the CLINT drives that hart's own `mtip` wire. Two harts, two
  comparators, two wires, two counts.
* **The IPI rows are non-zero in both directions.** 64/67 rescheduling and
  4620/2007 function-call interrupts are `msip` writes each hart made to the
  other's word, and each one is a device driving a wire into a sibling hart
  from inside a guest store — the re-entrancy case the CLINT's lock discipline
  exists for.
* **`ttyS0` counts 64 on CPU0 and 0 on CPU1**, which is the PLIC doing the
  opposite thing correctly: an external interrupt is offered to the contexts
  that enabled it, and Linux enabled it on one.
* **`/proc/stat`'s `cpu1` line has 11 540 jiffies of system time and only
  2 136 idle** — more system time than `cpu0`. The second hart is not parked
  after bring-up; it is where most of the kernel work went.

178 seconds of host time from reset to that output, under the interpreter, in
a release build — on a machine running six other builds at the time, so read it
as an order of magnitude and not as a measurement. This page's own rule about
interleaving a sweep applies: the 143 s the single-hart boot quotes above was
taken in a different sitting and the two are **not** a before-and-after.

### What the hermetic tests cover

Five of them, in `src/dev/riscv/tests.rs`, none of which needs a download:

* `both_harts_run_and_each_one_knows_which_it_is` — one image, both harts
  enter it, `bne a0, x0` sends them to different halves, and hart 0 cannot
  reach its `poweroff` unless hart 1 wrote the handshake word.
* `the_single_hart_board_runs_only_hart_zero` — the control. The same program
  on `riscv-virt` never finishes, which is what says the two board files are
  genuinely different rather than both being SMP.
* `a_store_to_the_other_harts_msip_is_an_interprocessor_interrupt` — hart 1
  sets `mtvec`, enables `mie.MSIE` and `mstatus.MIE` and waits in `wfi`; hart 0
  stores 1 to `0x02000004`; hart 1 lands in its handler, clears the word (the
  only way `msip` clears — there is no acknowledge bit) and answers.
* `a_two_hart_machine_snapshots_and_restores_to_the_same_state_hash` — two
  comparators, two `msip` bits, four PLIC contexts and two harts' register
  files, out and back.
* `the_generated_tree_describes_both_harts` — `cpu@0`, `cpu@1`, and eight cells
  of `interrupts-extended` on each of the CLINT and the PLIC.

## How far each guest gets

Written down rather than rounded up, because a precisely located stopping point
is the only kind that is useful.

### Linux

**Linux 6.12 (Debian riscv64 installer kernel)** runs its whole boot: every
initcall, the driver model, and the console handover off the SBI earlycon onto
this board's own 16550A —

```
[  110.961106] 10000000.serial: ttyS0 at MMIO 0x10000000 (irq = 12, base_baud = 115200) is a 16550A
[  110.969106] printk: legacy console [ttyS0] enabled
[  110.975106] printk: legacy bootconsole [sbi0] disabled
```

— and, given a ramdisk, **reaches a shell prompt that echoes what is typed at
it**:

```text
[  222.376146] Run /init as init process

rsemu initramfs on Linux 6.12.94+deb13-riscv64 riscv64

BusyBox v1.37.0 (Debian 1:1.37.0-6+b8) built-in shell (ash)
Enter 'help' for a list of built-in commands.

/bin/sh: can't access tty; job control turned off
rsemu# uname -a
Linux (none) 6.12.94+deb13-riscv64 #1 SMP Debian 6.12.94-1 (2026-06-20) riscv64 GNU/Linux
```

`uname -a` on the second-to-last line is the *echo* of the nine bytes the
harness fed to the port; the line under it is the reply. Nothing echoes it but
the guest's own terminal line discipline, so that pair is the console proving
it carries bytes in both directions. **143 seconds of host time** from reset to
that prompt, under the interpreter, on one core.

With `initramfs-virtio` — the same archive plus the kernel's own
`virtio_mmio.ko` and `virtio_blk.ko` — Linux claims the board's virtio disk and
reads and writes it:

```text
[  284.552146] virtio_blk virtio0: 1/0/0 default/read/poll queues
[  284.876146] virtio_blk virtio0: [vda] 32768 512-byte logical blocks (16.8 MB/16.0 MiB)
rsemu# head -c 34 /dev/vda
rsemu virtio-blk fixture, sector 0
rsemu# printf "vda-%s" roundtrip-ok > /w && dd if=/w of=/dev/vda bs=512 seek=1 && sync && dd if=/dev/vda bs=512 skip=1 count=1 | head -c 16
vda-roundtrip-ok
```

The first read is the host's `disk.img` arriving through the virtqueue; the
second command writes a sector and reads it back, so the descriptor ring is
exercised in both directions by Linux's own driver rather than by ours. 205
seconds of host time, the extra minute being module relocation and probe.

Without a ramdisk the kernel still panics in `prepare_namespace`, and that
remains the correct end of a boot nobody gave a root filesystem to.

Two observations from these runs that are ours, not the kernel's:

- `jitterentropy` trips the soft-lockup watchdog during `jent_entropy_init`
  (`BUG: soft lockup - CPU#0 stuck for 22s!`, and again at 44s). Virtual time
  here is derived from bus accesses, and jitterentropy's calibration loop makes
  a great many of them; the kernel warns, taints itself `[L]=SOFTLOCKUP`, and
  carries on. It costs real time as well as virtual: the initcall spans t=59s
  to t=105s of the 212 virtual seconds before `/init`, a little over a fifth of
  the boot — call it **half a minute of the two and a half wall-clock
  minutes**. Worth knowing before this becomes a CI fixture, and worth
  measuring against a kernel built without the module before assuming a
  command-line switch would skip it.
- Time is virtual throughout, so the timestamps above measure emulated seconds,
  not patience.

**Its ASID probe found a real bug in our `satp`.** An early initcall discovers
`ASIDLEN` the way the privileged specification suggests — write all ones to
`satp.ASID`, read back which bits stick:

```asm
csrr  a1, satp          # keep MODE and PPN
lui   a5, 65535
slli  a5, a5, 32        # 0xffff << 44 -- the ASID field
or    a5, a5, a1
csrw  satp, a5
csrr  a4, satp          # this fetch is the one that used to fault
```

`satp.PPN` is 44 bits under Sv39 and `ASID` sits directly on top of it. Masking
`PPN` any wider folds ASID bits into the root page table's address, so the
instant that `csrw` retired the whole address space moved and the *next* fetch
took an instruction access fault — as did the fetch of the trap handler, and so
on forever. The guest stayed live, ping-ponging through OpenSBI's M-mode trap
entry on the timer, which is what made it look like an SBI problem. It was not:
every SBI call in the trace is a well-formed `sbi_set_timer` (EID `0x54494d45`,
FID 0) being answered correctly. With the field masked to its real width the
kernel prints `ASID allocator using 16 bits (65536 entries)` and carries on.

`RSEMU_RISCV_STOP_AT` ends the run at the first line containing it — firmware
that reaches a prompt does not stop by itself — and `RSEMU_RISCV_FLASH1_OUT`
writes the variable bank back out when it does. Pointing `FLASH1_OUT` at the
same file `FLASH1` read is a reboot.

### EDK2/UEFI

To a shell, in about two minutes of host time under the interpreter:

```text
[Bds]Booting EFI Internal Shell
UEFI Interactive Shell v2.2
EDK II
UEFI v2.70 (EDK II, 0x00010000)
Shell>
```

`gEfiVariableWriteArchProtocolGuid` (`6441F818-6362-4E44-B570-7DBA31DD2453`)
installs, and "discovered but not loaded" falls from **47 drivers to 13** — the
thirteen left are the network stack and the PCI drivers, whose depexes this
board genuinely does not satisfy.

**A variable written in one run is there in the next**, and the variable store
is where that is visible rather than inferred. UEFI's store is an append-only
log in flash, because appending is the only thing a part that can merely clear
bits is able to do:

| | bytes programmed in the 256 KiB store | log ends at |
| --- | --- | --- |
| the image as shipped | 97 | `0x000063` |
| after one run | 2086 | `0x000857` |
| after a second run from that image | 2158 | `0x00089f` |

The second run **continued** the log rather than restarting it: it read run
one's `BootOrder`, `Boot0000`-`Boot0002`, `ConIn`, `ConOut`, `PlatformLang` and
`Timeout` out of the flash and appended only what had changed. A store that had
come up blank would have ended at `0x857` again.

The firmware finds both banks the same way it finds everything else here — the
generated device tree carries a `cfi-flash` node per bank, and EDK2's
`FdtNorFlashQemuLib` walks them, skips the one overlapping its own firmware
volume, and makes the other `PcdFlashNvStorageVariableBase`. Nothing is written
down twice: the addresses in the tree come out of the `map` statements.
