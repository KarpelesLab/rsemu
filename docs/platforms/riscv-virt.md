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
