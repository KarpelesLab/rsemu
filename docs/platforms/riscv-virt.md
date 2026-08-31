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

**UEFI** works too, with one splice. `edk2-riscv-code.fd` from EDK2's
`OvmfPkg/RiscVVirt` (BSD-2-Clause-Patent) is built for the board's NOR flash at
`0x20000000`, and `fw_jump`'s hand-off address is fixed at build time, so an
eight-byte trampoline at `0x80200000` bridges the two — `lui t0, 0x20000` then
`jr t0`, which leaves `a0` (hart id) and `a1` (device tree) exactly as OpenSBI
set them. The payload list stages RAM under any address outside DRAM, so the
flash windows come up as plain memory:

```console
$ printf '\xb7\x02\x00\x20\x67\x80\x02\x00' > $TD/tramp.bin
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD="0x80200000:$TD/tramp.bin,0x20000000:$TD/edk2-riscv-code.fd,0x22000000:$TD/edk2-riscv-vars.fd" \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=3000000 \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

The variable store is RAM rather than CFI flash, so UEFI variables are readable
but not durably writable. That is a missing device (`dev/flash/cfi`), not a
missing mechanism.

## How far each guest gets

Written down rather than rounded up, because a precisely located stopping point
is the only kind that is useful.

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

**EDK2/UEFI** reaches the end of the DXE dispatcher.
