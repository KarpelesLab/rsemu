# Storage transports

Consumed by: `dev/blk/*`. The image and filesystem layers below these
controllers come from [`fstool`](https://github.com/KarpelesLab/fstool) — see
`ROADMAP.md` §7.1.

## Specifications

| Transport | Source | Access |
| --- | --- | --- |
| ATA / ATAPI | [T13](https://www.t13.org/) — the ATA/ATAPI Command Set standards; ATA/ATAPI-6 (T13/1410D) is what `dev/ata/disk` was written from | Drafts historically free; final standards via INCITS |
| AT IDE interface | *IBM Personal Computer AT Technical Reference* (1984), the fixed-disk adapter; Ralf Brown's Interrupt List for the `0x3f6`/`0x3f7` split with the diskette adapter | **Free** |
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

ATA is split the same way, and the split is even less of a judgement call than
SD's: **"IDE" means *integrated drive electronics***, so the controller is
physically on the drive and what is left on the motherboard is a decoder and a
buffer. `dev/ata/disk` is therefore everything on the far side of the cable —
the eight command block registers, the command set, the busy/DRQ handshake, the
CHS and LBA translations and the 256-word `IDENTIFY DEVICE` response — and
`dev/pc/ide` is the AT's contribution, which is two chip selects, master/slave
cable position, and a wire to an 8259A. The falsifiable form: `dev/ata/disk.rs`
contains no I/O port address and no register offset (a register is a name,
`Reg`), and `dev/pc/ide.rs` contains no ATA opcode, no `IDENTIFY` word index and
no status bit. The same drive would hang off a PCI IDE controller, a
CompactFlash socket or a PCMCIA adapter without changing, because none of those
change the cable.

**ATAPI is deliberately absent.** `IDENTIFY PACKET DEVICE` is aborted, which is
the specified behaviour of a device that is not a packet device and is how a
driver finds out. A CD-ROM is a packet command set on top of this transport and
is a separate piece of work, not a flag on this one.

Like SD and NOR flash, the ATA drive takes a media slot rather than an
`fstool::BlockDevice`, and for the same reason plus one more: it is `no_std`, so
`rsemu run pc-at --hd0 disk.img` works on every target the crate builds for. The
cost is real and worth stating — the medium is a flat `RamStore`, so a drive
costs its whole capacity in host memory and a snapshot writes all of it. A large
or sparse image is a `dev/blk/ata` variant under the documented `std` exception,
reusing the protocol half whole and replacing only `AtaDisk::read_media` and
`AtaDisk::write_media`.

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
