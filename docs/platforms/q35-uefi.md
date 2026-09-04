# `q35-uefi` — the q35 chipset with UEFI in flash where the ROM socket was

Consumed by [`machines/q35-uefi.machine`](../../machines/q35-uefi.machine),
[`tests/q35_uefi.rs`](../../tests/q35_uefi.rs),
[`src/dev/flash/cfi.rs`](../../src/dev/flash/cfi.rs) and
[`src/dev/q35`](../../src/dev/q35). The chipset is [`q35`](q35.md)'s and is not
repeated here; what this page is about is the **flash**, and where a real OVMF
gets to on top of it.

rsemu already boots UEFI — on RISC-V, out of two CFI NOR banks
([`riscv-virt.md`](riscv-virt.md), "Booting UEFI"). This is the same shape one
architecture over, and the interesting part is how little of it was
architecture-specific.

## Why a fourth q35 board and not a flag

[`q35`](q35.md) has a `pc.rom` at `0xf0000` with an alias of it at the top of
the address space. That is a 1996 machine: a mask ROM the processor executes and
nothing writes. A UEFI machine has **flash**, and the difference is not
cosmetic — the variable store is the firmware's own non-volatile memory, and a
part that can only *clear* bits is what UEFI's fault-tolerant write is built on
top of. The two cannot be one board for the same reason
[`q35-linux`](q35-linux.md) is its own: something has to decode `0xfffffff0`,
and only one thing can.

So `q35-uefi` is `q35` with the sockets replaced and four things removed. Each
absence is load bearing rather than tidying:

* **`q35.acpi` is gone.** The firmware publishes its own ACPI set and hands the
  operating system an RSDP through the EFI configuration table (UEFI 2.10 §4.6).
  A board that *also* staged a generated RSDP at `0xe0000` would be offering a
  legacy scan a second, different description of itself — the ambiguity
  [`q35.md`](q35.md) warns about for the MP table, with the roles reversed.
* **No video adapter.** EDK II's `QemuVideoDxe` binds three PCI identifications
  and none of them is this board's, so a display adapter here would be a card
  nothing drives. The console is the 16550 at `0x3f8`, which is what
  `PlatformBootManagerLib` puts a terminal on.
* **No IDE, no 8237As.** A UEFI build reaches its disks through PCI.
* **A20 is deliberately unwired**, the same decision
  [`q35-linux`](q35-linux.md) documents. A net with a driver comes up low and a
  low `a20` pin shuts the gate; a UEFI reset vector goes from real mode to long
  mode without ever touching `0x92` or the 8042's output port, because the
  machines it is written for come out of reset with A20 already connected.

## The layout, and why it is not a choice

An x86 processor fetches its first instruction from `0xfffffff0` (*Intel SDM*
Vol. 3A §9.1.4). The code bank therefore **ends at 4 GiB** and the variable bank
sits immediately below it, so the pair is one contiguous run of flash whose top
is the reset vector. That is what every split OVMF build is compiled for, and it
is why the machine file takes **sizes** rather than addresses — the addresses
fall out of them:

| | address | size |
| --- | --- | --- |
| variable store (`flash1`) | `0x100000000 - flash` | `vars` |
| firmware (`flash0`, `readonly`) | `0x100000000 - flash + vars` | `flash - vars` |
| reset vector | `0xfffffff0` | 16 bytes, at the top of the last block |

The defaults are a **2 MiB** split OVMF: `OVMF_CODE.fd` at 1920 KiB and
`OVMF_VARS.fd` at 128 KiB, so the pair starts at `0xffe00000`. A 4 MiB build is
`-p flash=4M -p vars=528K` and starts at `0xffc00000`.

## What was reused from the RISC-V path, and what was x86-specific

Reused entire: `flash.cfi` — the CFI query, the Intel/Sharp command set, the
per-block erase, the bit-clearing-only program, the block lock bits, the
snapshot of a half-issued command. The device did not need one line changed to
serve an x86 firmware.

Three properties are x86-specific, and each of them is a real difference rather
than a preference:

| | `riscv-virt` | `q35-uefi` | why |
| --- | --- | --- | --- |
| `width` / `interleave` | 4 / 2 — two x16 parts on a 32-bit bus | **1 / 1 — one x8 part** | EDK II's `OvmfPkg` flash driver issues every command as a **single byte store**; `VirtNorFlashDeviceLib` writes 32-bit words with the command duplicated into both halves. A two-byte command word rejects the `OvmfPkg` driver's writes as misaligned. |
| `block` | 256K | **4K** | a split OVMF lays its variable store, its fault-tolerant working block and that block's spare out in 4 KiB blocks; an erase that took more would destroy a neighbour. |
| `locked` | default (true) | **false** | an Intel P30 powers up with every block locked and wants a `0x60`/`0xd0` unlock. `VirtNorFlashDxe` issues one; the `OvmfPkg` driver never does, so a board that came up locked would answer every variable write with SR.1 set. |

And one thing that is not the flash at all: **the RISC-V board needs a
trampoline and this one does not.** There, OpenSBI's compiled-in hand-off
address had to be bridged to the flash base with eight bytes of hand-written
code. Here the processor's own reset vector already points at the top of the
flash, so the firmware image is the only thing in the machine.

## A part a guest writes is a storage device

`flash.cfi` now takes a [`Medium`](../../src/dev/medium.rs) bound to its media
slot, and implements `Device::flush`. That is the difference between a variable
that survives a run and one that survives a *reboot*:

```console
rsemu run q35-uefi --flash0 OVMF_CODE.fd --drive flash1=OVMF_VARS.fd
```

`--flash1` copies bytes in and nothing takes them out again; `--drive` binds the
bank to a host file, and the flush at the end of the run writes it back. The
write-back is skipped entirely unless a program or an erase actually changed the
array, so a `readonly` firmware bank and an untouched variable store both cost a
boolean rather than a copy. A snapshot obeys the medium's own `Snapshot` policy,
the way every drive in the tree does: `Capture` puts the bytes in the chunk,
`Reference` flushes first and then records *which* medium, `Refuse` fails
loudly.

## How far OVMF gets

Written down rather than rounded up.

Two images, both taken from the local distribution's firmware packages and
neither committed (`scripts/fetch-testdata.sh ovmf`):

| image | size | build |
| --- | --- | --- |
| `edk2-ovmf`'s `OVMF_CODE.fd` + `OVMF_VARS.fd` | 2 MiB | `RELEASE` |
| the qemu package's `edk2-x86_64-code.fd` + `edk2-i386-vars.fd` | 4 MiB | debug strings present |

**Both end the same way** — the same runaway, at the same address, with the
same `cr2` and the same intact page tables under it. The instruction named below
was probed on the 2 MiB image.

### It reaches the DXE dispatcher

The processor runs for **284 seconds of virtual time** — about 85 seconds of
host time under the interpreter — and it gets there through every phase in
order. Each of these is an observation rather than an inference:

* **SEC.** The reset vector at `0xfffffff0` executes out of the code bank, the
  processor is in 32-bit protected mode and then in **long mode** with paging on
  (`cr0=80000023 cr4=0x660 efer=0x500 cr3=0x800000`, and `x86boot`'s run loop
  records `protected=true long=true` from the sample it took while it happened).
  The page tables `cr3` names are at `0x800000` and the walk through them is
  intact right up to the moment below.
* **PEI.** It sizes memory from CMOS `0x34`/`0x35` — the only route to it on a
  board with no `fw_cfg` — and then decompresses the main firmware volume into
  RAM. What that looks like in the trace is a tight loop at `0xfffcd0xx`, in the
  code bank, pulling a byte at a time out of `0xffe39809` and writing low
  memory: a decompressor reading a compressed volume out of flash.
* **DXE.** Execution moves to drivers loaded near the **top of the 128 MiB of
  RAM** (`0x080c0000`-`0x080e0000`) and to a DXE core around `0x0082xxxx`. The
  local APIC's spurious-interrupt vector register is left at **`0x1ff`** — the
  software-enable bit set, against a reset value of `0xff` — which is
  `CpuDxe` having initialised the local APIC.
* The chipset registers the firmware touched, read back after the run:

  ```text
  q35-uefi:   00:00.0 id      = 0x29c08086
  q35-uefi:   PCIEXBAR        = 0x00000000_e0000001
  q35-uefi:   00:1f.0 id      = 0x29188086
  q35-uefi:   PMBASE/ACPI_CNTL= 0x00000601
  q35-uefi:   local APIC ID/SVR= 0x00000000 0x000001ff
  ```

  `PCIEXBAR` enabled at `0xe0000000` and `PMBASE` at `0x600` are what
  `PlatformInitLib` expects to find or to write; `0x29c0` is the identification
  it switches on to decide it is looking at a q35 rather than an i440fx, which
  is why [`q35.md`](q35.md)'s device-id measurement matters here too.

**Nothing is printed to the serial port**, and that is a property of the images
rather than of the board: a `RELEASE` EDK II says nothing until its console
driver comes up, which is after the point below. See "What would make this
visible".

### And then it dies on `MOV RAX, CR8`

The first interrupt or exception delivered to the processor enters EDK II's
`CommonInterruptEntry` (`UefiCpuPkg/Library/CpuExceptionHandlerLib`, X64), whose
job is to fill an `EFI_SYSTEM_CONTEXT_X64` — and that structure has `Cr8` in it.
Four bytes into the handler:

```text
probe: 284069ms vector 6 at handler 0x80d056a (+0), faulting rip 0x80d07d3 err 0x0
probe:   mov rax, cr8   bytes 44 0f 20 c0 50 0f 20 e0 48 0d 08 02 00 00 0f 22
```

`44 0f 20 c0` is `MOV RAX, CR8`: `REX.R` plus `0F 20 /r`, where `REX.R` is what
turns `CR0` into `CR8`. **rsemu's x86 core raises `#UD` on it** —
[`src/cpu/x86/prot.rs`](../../src/cpu/x86/prot.rs)'s `read_control` and
`write_control` answer indices 0, 2, 3 and 4 and return `Fault::bare(VEC_UD)`
for everything else, and `Fields::reg_num` correctly extends the ModRM `reg`
field by `REX.R` to 8. So the exception handler faults on its own second
instruction, `#UD` re-enters the same handler, and the recursion pushes frames
until the stack walks off the bottom of the identity map:

```text
q35-uefi: cs:rip=0038:00000000080d07d0 cr2=0x801ff8 cr3=0x800000
q35-uefi: the walk for cr2 = 0x801ff8:
q35-uefi:   level 1: table 0x802000 -> 0x0000000000000046  (not present)
```

— by which point the descending stack has scribbled over the handler it was
executing, which is why the post-mortem's disassembly at `RIP` looks like
nonsense. The interesting instruction is the one named above, not the one the
processor died on.

**This is a finding, not a fix.** `src/cpu/x86` belongs to another agent this
round. What it needs:

* `CR8` is the **Task Priority Register**, *Intel SDM* Vol. 3A §2.5 and
  §11.8.6 ("Task Priority in IA-32e Mode"). Only bits 3:0 exist; they are the
  task-priority *class*, and they alias the local APIC's `TPR[7:4]`.
* It exists **only in 64-bit mode**. `MOV` to or from `CR8` outside 64-bit mode
  is `#UD`; inside it with `CPL > 0` is `#GP(0)`; writing a value with any of
  bits 63:4 set is `#GP(0)`.
* The alias to the local APIC's TPR is the part with a seam in it —
  `src/accel/state.rs` already writes `cr8` into a local APIC's task-priority
  register through the address space (`state.rs:458`), so the shape exists.
  A first cut that holds a private four-bit register would unblock this board;
  the alias is what an operating system using CR8 for interrupt masking needs.

Until then, `q35-uefi` reaches the DXE dispatcher and no further.

## What would make this visible

Two things, in order of value, and neither is in this round's scope.

1. **A debug console at I/O port `0x402`.** EDK II's `PlatformDebugLibIoPort` —
   what an OVMF build uses for `DEBUG()` output — writes its whole log there,
   but only after `PlatformDebugPortDetect` reads the port and gets back the
   magic byte **`0xe9`**. This board's I/O space is `read-as-ones`, so the
   detect fails and the log is dropped. A one-byte device that reads `0xe9` and
   forwards writes to a character port would turn "reaches the DXE dispatcher"
   from four inferred observations into the firmware's own account of itself.
   That is a `src/dev/pc` addition, which is another agent's this round.
2. **A `DEBUG` OVMF build.** The 4 MiB image in the qemu package carries debug
   strings; the 2 MiB `edk2-ovmf` one does not. Neither prints anything without
   (1).

## Running it

```console
scripts/fetch-testdata.sh ovmf

RSEMU_OVMF_CODE=testdata/x86/OVMF_CODE.fd \
RSEMU_OVMF_VARS=testdata/x86/OVMF_VARS.fd \
RSEMU_OVMF_MS=400000 \
    cargo test --release --features machine-q35-uefi --test q35_uefi -- --nocapture
```

`tests/q35_uefi.rs` has the whole variable table. The three tests that do *not*
need an image run on every `cargo test`: that the two banks are one contiguous
run up to the reset vector, that the variable bank answers the byte-wide probe
`QemuFlashDetected` opens with, and that a program clears bits while only an
erase puts them back — asked of the board, through its address space, at the
width the driver uses.

## The ledger

| | |
| --- | --- |
| reset vector out of flash at `0xfffffff0` | **works** |
| long mode, paging, the SEC page tables | **works** |
| PEI, memory sized from CMOS `0x34`/`0x35` with no `fw_cfg` | **works** |
| `FVMAIN` decompressed into RAM, DXE core entered | **works** |
| DXE drivers dispatched, local APIC software-enabled | **works** |
| `PCIEXBAR`, `PMBASE` as the firmware left them | **works** |
| the flash probe, program and erase the variable driver needs | **works** (asserted without a firmware; the firmware never reaches it) |
| the first interrupt | **`MOV RAX, CR8` is `#UD`** — `src/cpu/x86/prot.rs` |
| any serial output | **none**, and will be none until a `0x402` debug console exists or a firmware gets far enough to open a terminal |
| a variable written in one run present in the next | **not reached**; the store is byte-identical to the image as shipped after both runs |
| SMRAM / SMM | not modelled, and not needed by a non-`SMM_REQUIRE` OVMF; [`q35.md`](q35.md) records the gap |
| `fw_cfg` | absent, and deliberately: EDK II degrades cleanly when the signature at `0x510` does not read `QEMU`, and everything above happened without it |

## Sources

*Intel SDM* Vol. 3A §2.5, §9.1.4 and §11.8.6; Vol. 2B, `MOV`—*Move to/from
Control Registers*. Intel 3 Series Express Chipset Family Datasheet
(316966-002) and Intel I/O Controller Hub 9 Family Datasheet (316972-004) for
the chipset. JEDEC JESD68.01 and the Intel StrataFlash P30 datasheet for the
flash. The UEFI Specification 2.10 and the PI Specification 1.8 for what a
firmware expects of a platform. EDK II itself — BSD-2-Clause-Patent, and
therefore a permitted *reference* under `CLAUDE.md` — for `OvmfPkg`'s flash
command sequence, `PlatformInitLib`'s chipset detection, and
`CpuExceptionHandlerLib`'s saved context.

**No emulator source of any licence was consulted.** The firmware images were
run and never read; every number above is either a register this repository's
own devices hold or a byte the guest itself executed.
