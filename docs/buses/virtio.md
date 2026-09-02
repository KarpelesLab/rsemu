# virtio

Consumed by: `dev/virtio/*`. The fastest path to a working device
for any modern guest, and freely specified.

## Primary

| Source | Covers |
| --- | --- |
| [**Virtual I/O Device (VIRTIO) Version 1.2**](https://docs.oasis-open.org/virtio/virtio/v1.2/virtio-v1.2.html) (OASIS) | The complete standard: virtqueue layout (split and packed), feature negotiation, the device-type specifications (net, block, console, rng, balloon, gpu, fs), and both the PCI and MMIO transports |

This is an open OASIS standard, published in full as HTML. There is no reason to
consult anything else — and specifically no reason to read Linux's virtio
drivers, which is the usual trap.

## Implementation notes

- Build the **transport-agnostic core first** (virtqueues, descriptor chains,
  feature negotiation, config space), then bolt on the PCI and MMIO transports.
  Device types then work identically on both, which is what makes virtio-mmio on
  the RISC-V `virt` board and virtio-pci on the PC one implementation.
- Split virtqueues are enough to boot everything; packed virtqueues are an
  optimisation for later.
- virtio-net is a `pktkit::L2Device`; virtio-blk sits on `fstool::BlockDevice`.
  Neither needs transport-specific code.
- **virtio-blk reaches `fstool` through `dev::ata::Medium`, not directly.**
  `src/dev/riscv/virtio` is `no_std` and `fstool` is a `std` crate, so the
  device holds the narrow `&self` trait and `dev/blk` adapts that to a
  `BlockDevice` behind a lock — the same seam an ATA drive's platter and an
  NVMe namespace use, and the reason `--drive disk=root.qcow2` works on all
  three ([`storage.md`](storage.md)). The alternative, a `Vec<u8>` in the
  device, is what was there before: it cost the whole capacity in host memory,
  had no snapshot policy, and let a guest-supplied descriptor length reach an
  allocator.

## ⚠ Do not consult

Linux `drivers/virtio` and `drivers/net/virtio_net.c` are GPLv2. The OASIS
specification is complete, free, and better organised.
