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

— and then panics in `prepare_namespace` because nothing supplied a root
filesystem. That is the correct end of a kernel booted with neither an initrd
nor a disk driver: the Debian installer kernel builds `virtio_mmio` as a
module, so the `virtio.blk` in this machine file is never claimed. Staging a
rootfs is the next step, and it is a fixture problem rather than a machine one.

Two observations from that run that are ours, not the kernel's:

- `jitterentropy` trips the soft-lockup watchdog during `jent_entropy_init`
  (`BUG: soft lockup - CPU#0 stuck for 22s!`, and again at 44s). Virtual time
  here is derived from bus accesses, and jitterentropy's calibration loop makes
  a great many of them; the kernel warns, taints itself `[L]=SOFTLOCKUP`, and
  carries on.
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
