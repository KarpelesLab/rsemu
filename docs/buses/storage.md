# Storage transports

Consumed by: `dev/blk/*`. The image and filesystem layers below these
controllers come from [`fstool`](https://github.com/KarpelesLab/fstool) — see
`ROADMAP.md` §7.1.

## Specifications

| Transport | Source | Access |
| --- | --- | --- |
| ATA / ATAPI | [T13](https://www.t13.org/) — the ATA/ATAPI Command Set standards | Drafts historically free; final standards via INCITS |
| AHCI | Serial ATA AHCI Specification (Intel) | intel.com **[browser]** |
| NVMe | [NVM Express specifications](https://nvmexpress.org/specifications/) | **Free** |
| SCSI | [T10](https://www.t10.org/) — SPC (primary commands), SBC (block commands) | Drafts free |
| SD / MMC | SD Association simplified specifications | sdcard.org **[browser]** — the *simplified* specs are free |
| STM32 SDMMC host controller | ST **RM0433** §55 (H7 family); RM0090 §31 for the F4's older SDIO | st.com **[browser]** — free downloads |
| virtio-blk | [`virtio.md`](virtio.md) | Free |
| Parallel NOR flash (CFI) | JEDEC **JESD68.01** (Common Flash Interface) and **JEP137B**; the Intel StrataFlash P30 datasheet for the Intel/Sharp command set, the Spansion/Cypress S29GL datasheets for the AMD one | **Free** — jedec.org registration; the datasheets are open downloads |

SD/MMC is split in two, and the split is the point: `dev/sd/card` is the
**card** — the command set, the state machine and the registers — and knows
nothing about any controller, while `dev/stm32/sdmmc` is one host controller
that drives it. An SPI-mode card is the same die behind different framing, so
an SPI controller hangs off the same `SdCard` rather than a second model of it.
Like NOR flash, and for the same reason, the card takes a media slot rather
than an `fstool::BlockDevice`: the contents are a flat image and `fstool` would
drag `std` into a `no_std` device. A large or sparse image is a `dev/blk/sd`
variant under the documented `std` exception, reusing the protocol half whole.

NOR flash is here rather than under devices because it is a *transport* too:
the guest sees a memory window with a command protocol on it, not a controller
with a queue, which is why `dev/flash/cfi` is not on `fstool::BlockDevice` and
takes a plain media slot instead.

NVMe and virtio-blk are both freely specified and much simpler to implement
correctly than ATA. Prefer them for new machines; implement ATA because legacy
guests require it.

## Working references

[OSDev: ATA PIO](https://wiki.osdev.org/ATA_PIO_Mode),
[AHCI](https://wiki.osdev.org/AHCI), [NVMe](https://wiki.osdev.org/NVMe).

## Implementation notes

- Controllers sit on `fstool::BlockDevice` (`std::io::Read + Write + Seek +
  Send`). Do not invent a parallel block abstraction — but note it is `Send`,
  *not* `Sync`, and its methods take `&mut self`, so a controller owns its
  device behind the seam rather than sharing it. This is also why `dev/blk/*`
  is one of the two documented `std` exceptions to the `no_std` rule.
- **The flush contract matters more than throughput.** A guest issuing FLUSH
  expects durability; a snapshot taken mid-write must restore to a consistent
  state. Decide the write-back cache semantics before writing the first
  controller, not after.
- I/O runs on the task pool, with completions delivered through the event queue
  at guest-derived virtual times (`ROADMAP.md` §4.7).
- `fstool`'s `crash_inject` block device gives guest-filesystem robustness
  testing without extra work.
