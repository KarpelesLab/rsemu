# Storage transports

Consumed by: `dev/blk/*`. The image and filesystem layers below these
controllers come from [`fstool`](https://github.com/KarpelesLab/fstool) — see
`ROADMAP.md` §7.1.

## Specifications

| Transport | Source | Access |
| --- | --- | --- |
| ATA / ATAPI | [T13](https://www.t13.org/) — the ATA/ATAPI Command Set standards; ATA/ATAPI-6 (T13/1410D) is what `dev/ata/disk` was written from | Drafts historically free; final standards via INCITS |
| AT IDE interface | *IBM Personal Computer AT Technical Reference* (1984), the fixed-disk adapter; Ralf Brown's Interrupt List for the `0x3f6`/`0x3f7` split with the diskette adapter | **Free** |
| AHCI | Serial ATA AHCI Specification 1.3.1 (Intel), and *Serial ATA: High Speed Serialized AT Attachment* Rev 1.0 for the FIS layouts it defers to | intel.com **[browser]** — see the note below; the SATA revision is an open download from seagate.com |
| NVMe | [NVM Express specifications](https://nvmexpress.org/specifications/) | **Free** |
| SCSI | [T10](https://www.t10.org/) — SPC (primary commands), SBC (block commands) | Drafts free |
| SD / MMC | SD Association simplified specifications | sdcard.org **[browser]** — the *simplified* specs are free |
| STM32 SDMMC host controller | ST **RM0433** §55 (H7 family); RM0090 §31 for the F4's older SDIO | st.com **[browser]** — free downloads |
| virtio-blk | [`virtio.md`](virtio.md) | Free |
| Parallel NOR flash (CFI) | JEDEC **JESD68.01** (Common Flash Interface) and **JEP137B**; the Intel StrataFlash P30 datasheet for the Intel/Sharp command set, the Spansion/Cypress S29GL datasheets for the AMD one | **Free** — jedec.org registration; the datasheets are open downloads |
| Serial NOR flash (SPI) | The **Winbond W25Q** datasheets (`W25Q128JV` rev F/H, and the 64/32/16 Mbit siblings) — instruction set §8.1, status registers §7.1, timing §9.6; JEDEC **JEP106** for the manufacturer byte | **Free** — open downloads from winbond.com |
| OCTOSPI / QUADSPI memory interface | ST **AN5050** *Getting started with Octo-SPI…*; the reference manuals **RM0432** (L4+), **RM0455**/**RM0468** (H7A3/H7B3, H723), **RM0438** (L5), **RM0456** (U5). **RM0433's H7 has a QUADSPI, not an OCTOSPI** | **Free** — st.com; ST's CMSIS/HAL headers are BSD-3-Clause and readable as a cross-check |

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

### The taskfile seam

A drive was reachable only through eight 8-bit ports written in the right order,
which is the right model of a ribbon cable and the wrong model of a Serial ATA
link — a SATA host adapter receives a *Register - Host to Device FIS*, a
twenty-byte structure carrying the whole command block at once, and there is no
ordering, no chip select and no register offset in it. `dev/ata/disk/taskfile`
is that second door: a `Taskfile` of six named fields, loaded into the very same
command block registers a port write would have left, dispatched by the very
same `AtaDisk::command`.

The falsifiable form, and it is the whole point: **delete `taskfile.rs` and the
AT's IDE channel is unchanged**; delete `AtaDisk::command` and both adapters
stop working. `src/dev/pc/ide.rs` did not change by one line when AHCI landed.
The data phase is shared the same way — `taskfile_read` and `taskfile_write`
copy in bulk out of (or into) the block the drive currently has under `DRQ` and
then run the identical `block_consumed` / `block_filled` path a word-at-a-time
drain would have run, so the busy/DRQ handshake, the per-block interrupt timing
and the completion write-back are shared rather than reimplemented.

One thing the seam had to add rather than reuse: the **DMA data transfer
protocols**. A drive on an AT-class cable has no bus master to move bytes for
it, so `IDENTIFY DEVICE` word 49 bit 8 is clear and `READ DMA` aborts; a drive
on a Serial ATA port has one, and a real driver issues `READ DMA EXT` and
nothing else. That is a property of the drive, so it is a property on the drive
(`dma = true`, default false) rather than something the adapter switches on
behind a machine file's back, and the AT is bit-identical without it. Which
protocol a command uses comes *out of the drive* in `Phase::Data { dma }` rather
than being re-derived from the opcode by the adapter, because a Serial ATA PIO
command is announced by a PIO Setup FIS before every block and a DMA one is not
— getting that backwards is the kind of thing that works with one driver and
hangs another.

Like SD and NOR flash, the ATA drive takes a media slot rather than an
`fstool::BlockDevice`, and for the same reason plus one more: it is `no_std`, so
`rsemu run pc-at --hd0 disk.img` works on every target the crate builds for. The
cost is real and worth stating — the medium is a flat `RamStore`, so a drive
costs its whole capacity in host memory and a snapshot writes all of it.

**That variant now exists**, as `dev/blk` under the documented `std` exception,
and it landed exactly as this paragraph predicted: the drive's storage is a
`dev::medium::Medium` and the two implementations are a `RamStore` and a
`dev::blk::Image`, so the protocol half is reused whole and only
`AtaDisk::read_media` and `AtaDisk::write_media` change hands. Three things are
worth knowing about it.

**No image format is parsed in rsemu.** `fstool` already has sparse raw, qcow2
(v2/v3, read/write, allocate-on-write, compressed clusters, backing files,
encryption), UDIF DMG, DiskCopy 4.2 and LUKS, and §7.1 puts controllers on
`fstool::BlockDevice` "rather than on a parallel rsemu invention". `dev/blk` is
therefore an adapter and nothing else: `&mut self` to `&self` behind a
`core::sync` lock, `std::io::Error` to `BusError`, and a snapshot policy. The
remaining formats §7.1 lists as rsemu work — `vmdk`, `vhdx`, `vdi` — are new
*backends*, and a new backend belongs beside the ones it sits next to, which is
in `fstool`.

**A machine file still names a media slot, never a host path.** The run installs
a `Medium` under the slot's name (`rsemu run pc-at --drive hd0=disk.qcow2`) and
`ata.disk` picks it up as it is constructed, so neither `machines/pc-at.machine`
nor `dev/pc/ide` changed. `--hd0 disk.img` still binds bytes and still copies
them into RAM; the two are different contracts and both are supported.

**A snapshot of a file-backed drive references the image.** The chunk holds the
drive's protocol state and the image's identity, `save` flushes the file first
so what is on disk matches the moment the snapshot was taken, and `load` refuses
a chunk that names a different image. `snapshot=capture` puts the bytes in the
chunk for an image small enough to want that, and `snapshot=refuse` says no.
What is not on offer is silently writing sixteen gigabytes into a snapshot.
Closing the remaining gap — the guest that writes to the image *after* the
snapshot — is a copy-on-write overlay, and per §7.1 that is `fstool` work.

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

**virtio-blk is on the `Medium` seam too**, and so is the USB disk
(`dev/usb/msd`, `docs/buses/usb.md` §6), which makes it several devices rather
than two and settles what "the seam" means in this tree: `dev::medium::Medium` is
where a storage device's bytes come from, and a controller that invents its own
backing store is a controller that has quietly opted out of `--drive`, out of
the snapshot policies, and out of qcow2. `dev/riscv/virtio/blk` held a
`Vec<u8>` until it did not, and the three things that cost are worth recording
because they are what any *next* device would also pay: the whole capacity in
host memory whatever the guest actually touched; no answer at all to what a
snapshot of it means, so a 16 GiB disk went into the chunk; and a
guest-supplied descriptor length reaching an allocator, because the range check
was a slice index on the backing vector and the vector was indexed *after* the
buffer for it had been allocated. The seam does not fix the last one by itself
— the fix is to check the range in `u64` and stage the transfer in bounded
chunks — but it is what made the check expressible.

The seam has its own home: `src/dev/medium.rs`, feature `dev-medium`, which
`dev-ata-disk`, `dev-nvme`, `dev-riscv` and `dev-blk` each depend on. It lived
in `src/dev/ata/` for as long as ATA was the only device that wanted it, and
the cost of leaving it there was a `riscv-virt` build linking an ATA command set
it will never issue in order to name a trait, a slot and a three-valued enum —
a crate-shape problem rather than a correctness one, since the file is `no_std`
and dependency-free, but the shape rule is explicit that a NES build links a
6502 and nothing else. The one thing that stayed behind is
`dev::ata::disk::error_bit`, the translation from a medium's three-way answer to
the ATA Error register, because that half belongs to the command set.

`machines/riscv-virt.machine` did not change to get any of this, which is the
test of whether the media-slot design held: the board still says
`image = "disk"`, `--disk rootfs.img` still binds bytes to that slot, and
`--drive disk=root.qcow2` installs a `Medium` under the same name that wins and
brings its own capacity.

**NVMe now exists** (`dev/nvme`, feature `dev-nvme`), and it is the first
storage device in this tree that is a **bus master**. Everything before it was
either programmed I/O — the guest reads a data port a word at a time — or
third-party DMA through an 8237, and in both of those the *guest* or a *third
chip* moves the bytes. An NVMe controller moves them itself: the driver builds a
submission queue, a completion queue and a list of Physical Region Pages in its
own memory, writes one 32-bit doorbell, and the controller fetches the 64-byte
command out of guest RAM, walks the PRP chain, moves the data, writes a 16-byte
completion back with its phase tag, and holds `INTA#` down until the host's
completion queue head doorbell catches up.

Four things about that shape are worth carrying to the next bus master, and
`dev/nvme/ctrl.rs` argues each where it bites.

**The register block is in the address space the device masters.** So a guest
can aim a PRP entry at the controller's own doorbells, and the write handler is
re-entered from inside itself. The answer is the one `core::wire` already gives
for a re-entrant level change: the work is **iterative, not recursive**. A
re-entrant doorbell records its tail and returns, and the outermost loop
re-reads every tail after each command. Recursion depth is one whatever the
guest builds.

**No lock is held across a guest-memory access, a medium access or a wire
change** — the re-entrancy contract, and here it is load-bearing rather than
decorative for exactly that reason. The state lock ranks at `0x5a00`, below
`DEVICE` (so the PCI function's configuration lock is taken and released before
it, never the other way round) and above `WIRE` (so the interrupt output is
driven after it is released). It cannot be `BUS`: `core::space` states that *"a
CPU holds a `BUS`-ranked lock across the accesses it issues"*, and every access
to this register block arrives from inside one.

**Every walk over a guest-built structure is bounded**, and the bounds are
argued rather than guessed. A PRP List's last entry may point at another list,
so a guest can close a ring; the chain is capped. A doorbell write could
otherwise become unbounded work inside one guest instruction if the controller's
own data transfers kept feeding its doorbells, so one entry into the engine
executes at most as many commands as the queues can hold between them — a bound
a legitimate driver cannot reach, because reaching it means the data was the
doorbell. `fuzz/fuzz_targets/nvme_mmio.rs` drives arbitrary bytes through both.

**A full completion queue is back pressure, not an error.** A command whose
completion has nowhere to go stays on its submission queue, and the *completion
queue head* doorbell is what releases it — which means that doorbell has to
resume the engine as well as lower the interrupt. A model that only ran on a
submission doorbell would strand the command until the driver happened to submit
another one; `a_command_on_a_queue_whose_completion_queue_is_full_waits` is
where that was caught.

The namespace is a `dev::medium::Medium` — the same seam an ATA drive's platter
uses, with the same three snapshot policies — so `--drive nvme0=disk.qcow2`
works here for the reason it works there, and no image format is parsed.
`machines/nvme-mini.machine` is the smallest board a driver can run against, and
`tests/nvme_board.rs` is that driver: it enumerates the bus at `0xcf8`, sizes and
places the base address register, builds queues in the board's RAM, and checks
every transfer **against the medium** rather than against the device's own
buffer.

**AHCI now exists too** (`dev/ahci`, feature `dev-ahci`), and it is the second
bus master. It is also the one that needed the taskfile seam above, which is why
that seam is the first commit of this work rather than the last: an AHCI port
carries an ATA command, the command set was already written, and writing it
twice would have been the failure.

The shape is NVMe's with different structures. A driver builds a 32-entry
**command list**, a **received-FIS area** and a **command table** — the command
FIS followed by a Physical Region Descriptor Table — in its own memory, writes
one bit into `PxCI`, and the adapter fetches the header, fetches the FIS, hands
the taskfile to the drive, walks the PRDT moving data between the medium and the
addresses it names, posts the device's answering FIS into the received-FIS area,
writes the byte count back into the header's `PRDBC` field, and holds `INTA#`
down until the driver has cleared `PxIS` **and then** the global `IS`.

Four things it does differently from NVMe, and each is a specification
requirement rather than a preference.

**The interrupt needs two registers cleared, in order.** `IS.IPS[x]` is
write-1-to-clear, and the adapter re-derives it from every port whose
`PxIS & PxIE` is non-zero — so clearing `IS` before `PxIS` achieves nothing and
the line stays down. AHCI §5.5.3 spells the order out; a model that treated `IS`
as ordinary storage would let a driver get away with the wrong one and then
strand it on real hardware.

**A fatal error clears `PxCMD.CR`, not `PxCMD.ST`.** §6.2.2: a taskfile error,
an interface error or a host bus error stops the command list engine and leaves
`PxCI` holding the failing slot so software can see which command it was.
Software restarts the port by writing `ST` one-to-zero — which is what resets
`PxCI` — and then back to one. `OFS` — a PRD table too short for the transfer —
is *not* fatal on a read: §6.1.5 has the adapter make "a best effort to
continue", so the extra sectors are read and discarded and the command completes
short with `PRDBC` saying so. On a **write** it is fatal, because the only thing
an adapter with no data could hand the drive is data it made up and the drive
would put it on the medium. So nothing is invented, the port stops, `PxTFD`
reports the `DRQ` that §6.2.2.1 says to `COMRESET` on, and the sectors that were
not written keep what they had.

**A software reset is two commands and the device answers only the second.**
§10.4.1's sequence is a control FIS asserting `SRST` and another releasing it.
While `SRST` is asserted the device is silent, so nothing clears that slot's
`PxCI` unless the command header's `C` bit asked the adapter to; a slot the
adapter has sent and cannot complete is *parked* rather than re-sent, which is
what keeps the engine making progress instead of looping on it.

**The bounds are the same three NVMe documents**, against a larger structure: a
`PRDTL` of up to 65,535 entries, a data phase capped at the sectors a 48-bit
command can name, and one entry into the engine capped at every port's whole
command list twice over — a bound a legitimate driver cannot reach, because
reaching it means the data was the doorbell. `fuzz/fuzz_targets/ahci_mmio.rs`
drives arbitrary bytes through all three, and the register block is mapped in
the space the adapter masters so the fuzzer can aim a data block at `PxCI`.

`machines/ahci-mini.machine` is the smallest board a driver can run against and
`tests/ahci_board.rs` is that driver, held to the same standard: it enumerates
the bus, sizes `ABAR` at base address register five (§2.1.11 puts it at
configuration offset `24h`, which is not a choice), builds a command list and a
scatter/gather table in the board's RAM, runs a command in each protocol, takes
the completion interrupt off a real wire into an 8259A, and checks every
transfer **against the medium** with the neighbouring sector asserted untouched.

**On obtaining the specification.** Intel's own PDF of *Serial ATA AHCI 1.3.1*
answers `403` to anything that is not a browser, which is why this table has
said `[browser]` here since it was written and why AHCI waited. It is
nevertheless obtainable and
citable: the Internet Archive has Intel's file, and *Serial ATA Revision 1.0* —
which AHCI §4.2.3.1 defers to for the FIS layouts and does not repeat — is an
open download from seagate.com. Both are hardware documentation from the parties
that wrote the standards. **No emulator source and no operating system's AHCI or
libata driver was opened.**

## Working references

[OSDev: ATA PIO](https://wiki.osdev.org/ATA_PIO_Mode),
[AHCI](https://wiki.osdev.org/AHCI), [NVMe](https://wiki.osdev.org/NVMe).

## Implementation notes

- Controllers sit on `fstool::BlockDevice` (`std::io::Read + Write + Seek +
  Send`). Do not invent a parallel block abstraction — but note it is `Send`,
  *not* `Sync`, and its methods take `&mut self`, so a controller owns its
  device behind the seam rather than sharing it. This is also why `dev/blk/*`
  is one of the two documented `std` exceptions to the `no_std` rule. The lock
  that bridges the two is `core::sync`'s, never `std::sync`'s: nothing under
  `dev/` may name that, `std` gate or no `std` gate.
- **A host read takes zero guest time.** Not an omission — if the duration of a
  `pread` reached the guest's timeline, two runs of the same machine would
  diverge on how warm the host's page cache was. When the drive grows an I/O
  delay it will come from a clock domain and a scheduler event, and the host's
  actual latency will still not be it.
- The bounds check is in `u64` and happens before the offset becomes a host
  `usize`. Disk offsets are where the 64-bit-guest-on-32-bit-host rule bites.
- **The flush contract matters more than throughput.** A guest issuing FLUSH
  expects durability; a snapshot taken mid-write must restore to a consistent
  state. Decide the write-back cache semantics before writing the first
  controller, not after.
- **And the run itself ends with a flush**, whether or not the guest asked for
  one — `Machine::flush`, backed by `Device::flush`, and the CLI runs it on
  every way out of a run including the failing ones. A guest that never issued
  FLUSH CACHE has been promised nothing and the drive is entitled to hold the
  write; that entitlement lasts exactly as long as the machine does, and when
  the process exits nobody is left to write those bytes. Linux makes this easy
  to hit: it writes a bare block device's dirty pages back on `sync(2)` but
  issues the device flush from `blkdev_fsync`, so `dd` without `conv=fsync`
  leaves a qcow2 with the data cluster written and the L2 entry that finds it
  still in the image's cache — a hole, not merely a stale sector.
- **"Every way out" had to be made to include Ctrl-C.** A signal is not a
  return from `main`, so for a while it was the one exit that walked past the
  flush: `SIGINT` on a headless run took the process with it and left exactly
  the hole described above. `host::signal` turns `SIGINT`, `SIGTERM` and
  `SIGHUP` into a flag the four run loops read at a slice boundary, which then
  asks the machine to stop through §4.7's safe point and falls out into
  `finish`. `SIGQUIT` is left alone on purpose — an emulator wedged in a guest
  loop is when somebody wants a core file — and a *second* signal of the same
  kind still kills, so a run that never reaches a slice boundary is escapable.
  `tests/cli_interrupt.rs` proves the data half against a `SIGKILL` control:
  same run, same moment, and the killed one's sector is a hole while the
  interrupted one's is the guest's bytes. What it does **not** cover is a
  panic, which unwinds past `finish` with no `Drop` on `Image` to catch it.
- I/O runs on the task pool, with completions delivered through the event queue
  at guest-derived virtual times (`ROADMAP.md` §4.7).
- `fstool`'s `crash_inject` block device gives guest-filesystem robustness
  testing without extra work.
