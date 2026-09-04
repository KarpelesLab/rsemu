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

### It reaches the UEFI Shell, and the shell answers what is typed at it

```text
BdsDxe: loading Boot0001 "EFI Internal Shell" from Fv(...)/FvFile(...)
BdsDxe: starting Boot0001 "EFI Internal Shell" from Fv(...)/FvFile(...)
UEFI Interactive Shell v2.2
EDK II
UEFI v2.70 (EDK II, 0x00010000)
map: No mapping found.
Press ESC in 5 seconds to skip startup.nsh or any other key to continue.
Shell> ver
UEFI Interactive Shell v2.2
EDK II
UEFI v2.70 (EDK II, 0x00010000)
```

Every byte of that came out of the 16550 at `0x3f8` — the board's only console,
and the one `PlatformBootManagerLib` puts a terminal on. `ver` is typed by
`RSEMU_OVMF_INPUT`, echoed by the shell's line editor and then executed, so the
path is round trip: guest output drives the keystroke, and the keystroke's reply
ends the run. The whole thing is **367.5 seconds of virtual time** — 106 seconds
of host time under the interpreter, 26 under `jit-host`.

Each phase, as an observation rather than an inference:

* **SEC.** The reset vector at `0xfffffff0` executes out of the code bank, the
  processor is in 32-bit protected mode and then in **long mode** with paging on
  (`cr0=80000023 cr4=0x660 efer=0x500 cr3=0x800000`).
* **PEI.** It sizes memory from CMOS `0x34`/`0x35` — the only route to it on a
  board with no `fw_cfg` — and decompresses the main firmware volume into RAM.
* **DXE.** Drivers dispatched near the top of the 128 MiB of RAM, `CpuDxe`
  software-enabling the local APIC, and the first exception delivered and
  *returned from*.
* **BDS.** `BdsDxe` enumerates its boot options, picks `Boot0001`, and starts
  the internal shell. The processor is in a different world by then —
  `cr3=0x7c01000`, `cr0` with `WP` set, `efer=0xd00` with `NXE`, and the shell
  running out of memory the DXE core allocated.
* The chipset registers the firmware left behind:

  ```text
  q35-uefi:   00:00.0 id      = 0x29c08086
  q35-uefi:   PCIEXBAR        = 0x00000000_e0000001
  q35-uefi:   00:1f.0 id      = 0x29188086
  q35-uefi:   PMBASE/ACPI_CNTL= 0x00000601
  q35-uefi:   PIRQ[A-D]_ROUT  = 0x0b0b0a0a
  q35-uefi:   local APIC ID/SVR= 0x00000000 0x0000010f
  q35-uefi:   local APIC TPR  = 0x00000000
  ```

  `PIRQ[A-D]_ROUT` moved from the board's reset `0x0a0b0a0b` to `0x0b0b0a0a`:
  the firmware routed the legacy interrupt lines itself, which nothing before
  BDS does. `SVR` is `0x10f` — still software-enabled, now with spurious vector
  `0x0f` — where a run that died in DXE left `0x1ff`; something after `CpuDxe`
  reprogrammed it, and which driver is not established here. `TPR` **is** `CR8`
  on this board, and reading it back after the run is reading the register the
  firmware's exception handler saved and restored.

  `src/dev/q35/` needed no change for any of it. Everything below was the
  processor.

### Three things in the x86 core stood between it and the shell

Each was found the same way: run the firmware, watch the processor, name the
instruction. None of them is specific to UEFI — they are architecture this core
did not have, and a 64-bit operating system would have found all three.

**1. `MOV RAX, CR8` was `#UD`.** The first exception delivered to the processor
enters EDK II's `CommonInterruptEntry`
(`UefiCpuPkg/Library/CpuExceptionHandlerLib`, X64), whose job is to fill an
`EFI_SYSTEM_CONTEXT_X64` — and that structure has `Cr8` in it. Four bytes into
the handler:

```text
mov rax, cr8   bytes 44 0f 20 c0 50 0f 20 e0 48 0d 08 02 00 00 0f 22
```

`44 0f 20 c0` is `MOV RAX, CR8`: `REX.R` plus `0F 20 /r`, where `REX.R` is what
turns `CR0` into `CR8`. `read_control` and `write_control` answered indices 0,
2, 3 and 4 and `#UD`'d on everything else, so the exception handler faulted on
its own second instruction and recursed until the stack walked off the identity
map — scribbling over the handler it was executing on the way down, which is
why the post-mortem's disassembly at `RIP` was nonsense.

`CR8` is the **task-priority register** (*Intel SDM* Vol. 3A §2.5 and §11.8.6),
64-bit mode only, bits 3:0 only, `#GP(0)` on `CPL > 0` or on a value with bits
63:4 set. It has **no storage in this core**: it is the top nibble of the local
APIC's `TPR`, and `read_control`/`write_control` reach it by accessing the APIC's
register page at `IA32_APIC_BASE + 0x80` — the same route
`accel::state::tpr_through_space` takes for an accelerated vCPU. So the alias
goes both ways by construction rather than by being kept in step, a write
re-evaluates what the APIC has pending and drives `INTR` from inside the device,
and `CR8` is **not** in the snapshot: the APIC's chunk already carries the byte.
A core with no local controller wired reads zero.

**2. A long-mode interrupt did not align the stack frame.** With `CR8`
answering, the handler got four instructions further and died on this:

```text
probe: vector 13 at handler 0x80d05d3, faulting 0x38:0x80d0829 err 0x0 rsp 0xc2bef8
probe:   fxsave [ds:rdi]   bytes 0f ae 07 fc ff 75 10 48 8b 4d 08 ...
```

`0f ae 07` is `FXSAVE [RDI]`, which `CommonInterruptEntry` emits as raw bytes to
fill the context's `FxSaveState`, with `RDI` taken straight from `RSP`. `FXSAVE`
raises `#GP(0)` on an operand that is not sixteen-byte aligned — and `RSP` was
`0xc2bef8`, eight bytes off.

It was eight bytes off because **in IA-32e mode the processor aligns `RSP` down
to a sixteen-byte boundary before it pushes the interrupt frame** (*Intel SDM*
Vol. 3A §6.14.2, "Stack Frame"; *AMD64 APM* Vol. 2 §8.9.3 says it in the same
words), and this core did not. The value *pushed* is the unaligned one, so
`IRETQ` still returns to the stack the interrupt found and nothing outside the
handler can tell — which is exactly why nothing had caught it: no 32-bit guest
and no interpreter test depended on it, and the first thing that did was a
64-bit handler saving its floating-point state.

`prot::aligned_frame` is the fix, applied to both long-mode delivery paths and
to neither 32-bit one. It is *not* applied to a call gate: SDM §5.8.5.1 loads
`RSP` from the task state segment and leaves it alone.

**3. `RDMSR` of `IA32_PLATFORM_ID` was `#GP(0)`.** With the handler working, the
firmware printed its own exception dump on COM1 — the first serial output this
board had ever produced — and named the address itself:

```text
!!!! X64 Exception Type - 0D(#GP - General Protection)  CPU Apic ID - 00000000 !!!!
ExceptionData - 0000000000000000
RIP  - 00000000080C655D, CS  - 0000000000000038, RFLAGS - 0000000000000006
RAX  - 0000000000000000, RCX - 0000000000000017, RDX - 0000000049656E69
...
CR4  - 0000000000000668, CR8 - 0000000000000000
```

`RSEMU_OVMF_DISASM=0x80c655d` reads that address back out of the still-resident
image:

```text
q35-uefi: what is at 0x80c655d:
q35-uefi:   rdmsr
q35-uefi:   shl rdx, 0x20
q35-uefi:   mov eax, eax
q35-uefi:   or rax, rdx
q35-uefi:   ret
```

— `BaseLib`'s `AsmReadMsr64`, with the index in `RCX` per the Microsoft x64
calling convention: `RCX = 0x17` is `IA32_PLATFORM_ID`. (`RDX = 0x49656E69` is
`"ineI"`, left in the register by the `CPUID` leaf-0 call before it: the
firmware had just been told `GenuineIntel`.) That register is architectural,
read-only, and defined in SDM Vol. 4 Table 2-2; a core that answers
`GenuineIntel` and then `#GP`s on it is the core being wrong. It now reads
**zero** — platform zero of eight — for the same reason `IA32_BIOS_SIGN_ID`
reads zero: the field exists to pick a microcode update out of a container
holding several, and nothing here loads microcode.

With those three, the firmware reaches its shell.

### What the probe is, and why it is in the test

A `RELEASE` EDK II is silent until its console driver comes up, and an exception
whose handler faults on *itself* destroys its own evidence — the recursion
overwrites the handler on the way down. So `RSEMU_OVMF_PROBE=1` re-runs the
board (the machine is deterministic, which is what makes a second run the same
run) to `RSEMU_OVMF_PROBE_MS` before where the first run stopped, then advances
**one processor clock at a time**, reads the guest's own interrupt descriptor
table, and prints the frame the processor pushed for the first gate it enters.
The frame is authoritative where a sampled `RIP` is not.

It has one sharp edge worth recording: it steps with `Machine::step_until` and
not `Machine::run_for`, because `run_for` **declines to split a scheduler
round** (§11.6's additivity), so a forty-nanosecond span inside a
one-millisecond quantum runs the whole quantum and steps straight over the
thing you are looking for.

`RSEMU_OVMF_DISASM` is the cheaper half: when the firmware *does* reach a
console it names its own faulting address and then dead-loops, so the
instruction is still in memory when the run ends and no replay is needed.

### The three engines agree

`RSEMU_ENGINE` now overrides this board's `engine = "interp"`, the way
`tests/q35_linux.rs` already allowed. All three reach `Shell>`, type `ver`, and
stop at **the same virtual instant** — 367540 ms — with byte-identical output:

| engine | host time | guest instructions retired in blocks |
| --- | --- | --- |
| `interp` | 106 s | — |
| `jit` | 62 s | 532,334,334 of 561,105,521 (94.9%) |
| `jit-host` | 26 s | 532,334,334 of 561,105,521 (94.9%) |

The control-register moves are not lifted — `cpu::x86::lift` returns `None` for
every `MOV` naming `CRn`, `DRn`, `TRn` or a segment register — so `CR8` runs on
the interpreter under all three by construction, and the frame alignment is in
the shared delivery path. The identical instruction counts are the evidence that
neither changed which engine ran what.

## What is not reached yet

**No variable is written to flash.** The store is byte-identical to the image as
shipped after a run that reaches the shell (127 programmed bytes before and
after, the log still ending at `0xf020`), so `RSEMU_OVMF_VARS_OUT` produces a
file identical to its input and a reboot has nothing extra to find. A BDS that
selected and started a boot option would normally have written `Boot0001` and
`BootOrder`, so something between `QemuFlashFvbServicesRuntimeDxe` and the
fault-tolerant write is not binding — which the board's own tests say is not the
flash device refusing the commands, because
`the_variable_bank_answers_the_flash_detection_probe` and
`a_variable_store_program_clears_bits_and_an_erase_puts_them_back` assert
exactly those sequences through the board's address space. That is the next
thing to look at on this board, and it is measured rather than assumed.

**A debug console at I/O port `0x402`** would still be worth having.
`PlatformDebugLibIoPort` writes EDK II's whole `DEBUG()` log there once
`PlatformDebugPortDetect` reads back the magic byte `0xe9`; this board's I/O
space is `read-as-ones`, so the detect fails and the log is dropped. It is much
less urgent now that the firmware reaches a real console, but it is the
difference between the last few lines of BDS and the whole boot. That is a
`src/dev/pc` addition.

## Running it

```console
scripts/fetch-testdata.sh ovmf

RSEMU_OVMF_CODE=testdata/x86/OVMF_CODE.fd \
RSEMU_OVMF_VARS=testdata/x86/OVMF_VARS.fd \
RSEMU_OVMF_MS=600000 \
RSEMU_OVMF_INPUT='Shell> =>ver\r' \
RSEMU_OVMF_STOP_AT='UEFI v2.70' \
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
| an exception delivered, handled and returned from | **works** — and it took `CR8`, the sixteen-byte frame alignment and `IA32_PLATFORM_ID` |
| `PCIEXBAR`, `PMBASE`, `PIRQ[A-D]_ROUT` as the firmware left them | **works** |
| BDS, boot options, `Boot0001` started | **works** |
| the UEFI Shell prompt on COM1 | **works** |
| the shell executing what is typed at it | **works** (`ver`, over the board's 16550) |
| the same run under `interp`, `jit` and `jit-host` | **works** — same virtual instant, same output |
| the flash probe, program and erase the variable driver needs | **works** (asserted without a firmware) |
| a variable written in one run present in the next | **not reached**; nothing writes the store, and why is open — see above |
| SMRAM / SMM | not modelled, and not needed by a non-`SMM_REQUIRE` OVMF; [`q35.md`](q35.md) records the gap |
| `fw_cfg` | absent, and deliberately: EDK II degrades cleanly when the signature at `0x510` does not read `QEMU`, and everything above happened without it |
| a boot device | none: this board has no storage controller, so the shell finds `map: No mapping found.` |

## Sources

*Intel SDM* Vol. 3A §2.5 (`CR8`), §6.14.2 (the long-mode stack frame and its
sixteen-byte alignment), §9.1.4 (the reset state), §11.4.4 and §11.8.6 (the
task-priority register and its two names); Vol. 2B, `MOV`—*Move to/from Control
Registers* and `FXSAVE`; Vol. 4 Table 2-2 (`IA32_PLATFORM_ID`). *AMD64
Architecture Programmer's Manual* Vol. 2 §8.9.3 for the same frame alignment. Intel 3 Series Express Chipset Family Datasheet
(316966-002) and Intel I/O Controller Hub 9 Family Datasheet (316972-004) for
the chipset. JEDEC JESD68.01 and the Intel StrataFlash P30 datasheet for the
flash. The UEFI Specification 2.10 and the PI Specification 1.8 for what a
firmware expects of a platform. EDK II itself — BSD-2-Clause-Patent, and
therefore a permitted *reference* under `CLAUDE.md` — for `OvmfPkg`'s flash
command sequence, `PlatformInitLib`'s chipset detection, and
`CpuExceptionHandlerLib`'s saved context and `BaseLib`'s `AsmReadMsr64`.

**No emulator source of any licence was consulted.** The firmware images were
run and never read; every number above is either a register this repository's
own devices hold or a byte the guest itself executed.
