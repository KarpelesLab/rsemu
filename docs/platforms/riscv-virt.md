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

| `engine` | 60 s of guest time | 240 s of guest time | state hash (240 s) |
| --- | --- | --- | --- |
| `interp` | 36.6 s | 156.9 s | `0x267fac762c374054` |
| `jit` | 25.2 s (**1.45×**) | 120.4 s (**1.30×**) | `0x267fac762c374054` |
| `jit-host` | 14.9 s (**2.45×**) | 68.7 s (**2.28×**) | `0x267fac762c374054` |

The host code generator used to *lose* to the portable one on this guest, at
0.50×, and none of the three things that fixed it was the code it emits.
**Blocks are chained** — 86% of them are reached by following a patched exit,
where the count was previously zero in every run. **A compile stopped costing
144 µs**, which is what two `mprotect` calls over a 256 MiB code buffer had
been costing before `jit::x86::buf` learned to flip a page-sized window
instead. And **a guest load stopped costing a call**: the software TLB's fast
path is inlined into generated code now that a hart publishes a `LoadPlan`, so
97.3% of compiled loads are a mask, a compare, an add and a `mov` rather than a
trip through the hart's translation and PMP. That last one is worth 13% —
78.9 s to 68.7 s over the four minutes — against 20% fewer host instructions
executed, which is the usual gap between an instruction count and a clock.
[`src/cpu/riscv/engine.rs`](../../src/cpu/riscv/engine.rs) has the reasoning and
the measurements behind every one of those claims, including what it costs to
keep the engines identical, and `src/jit/fast.rs` has the argument for why a
*paged* hart may publish a plan at all.

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
gracefully.

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
