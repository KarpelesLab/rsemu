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
| virtio-blk | [`virtio.md`](virtio.md) | Free |
| Parallel NOR flash (CFI) | JEDEC **JESD68.01** (Common Flash Interface) and **JEP137B**; the Intel StrataFlash P30 datasheet for the Intel/Sharp command set, the Spansion/Cypress S29GL datasheets for the AMD one | **Free** — jedec.org registration; the datasheets are open downloads |
| Serial NOR flash (SPI) | The **Winbond W25Q** datasheets (`W25Q128JV` rev F/H, and the 64/32/16 Mbit siblings) — instruction set §8.1, status registers §7.1, timing §9.6; JEDEC **JEP106** for the manufacturer byte | **Free** — open downloads from winbond.com |
| OCTOSPI / QUADSPI memory interface | ST **AN5050** *Getting started with Octo-SPI…*; the reference manuals **RM0432** (L4+), **RM0455**/**RM0468** (H7A3/H7B3, H723), **RM0438** (L5), **RM0456** (U5). **RM0433's H7 has a QUADSPI, not an OCTOSPI** | **Free** — st.com; ST's CMSIS/HAL headers are BSD-3-Clause and readable as a cross-check |

NOR flash is here rather than under devices because it is a *transport* too:
the guest sees a memory window with a command protocol on it, not a controller
with a queue, which is why `dev/flash/cfi` is not on `fstool::BlockDevice` and
takes a plain media slot instead. `dev/flash/spinor` is the same part reached
the other way round — a frame on four wires rather than an address window — and
it is an `SpiSlave` on [`low-speed.md`](low-speed.md)'s fabric, so it takes a
media slot for the same reason.

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
