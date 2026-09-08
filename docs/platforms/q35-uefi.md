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
  [`q35.md`](q35.md) warns about for the MP table, with the roles reversed. The
  board still says all of it, through `q35.fwcfg` instead:
  ["The tables"](#the-tables-and-the-one-channel-a-uefi-board-has-to-hand-them-over)
  is what changes and what does not.
* **No video adapter.** EDK II's `QemuVideoDxe` binds three PCI identifications
  and none of them is this board's, so a display adapter here would be a card
  nothing drives. The console is the 16550 at `0x3f8`, which is what
  `PlatformBootManagerLib` puts a terminal on.
* **No IDE, no 8237As.** A UEFI build reaches its disks through PCI — and the
  one thing this board *adds* to `q35` besides the flash is where it reaches
  them: an NVM Express controller at `00:04.0`. What that got to is
  ["The disk"](#the-disk-and-the-one-thing-between-this-board-and-an-operating-system)
  below.
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

Reused almost entire: `flash.cfi` — the CFI query, the Intel/Sharp command set,
the per-block erase, the bit-clearing-only program, the block lock bits, the
snapshot of a half-issued command. Everything an x86 firmware *executes* out of
it worked unchanged, which is why this board reached a UEFI shell without the
device being touched.

It needed exactly one change, and only to *write*: **the status register read
`0x80` where it should have read `0x00`**. That is
["Why nothing wrote the store"](#why-nothing-wrote-the-store-one-bit-in-a-status-register)
below; it is a defect in the model of the part rather than anything
x86-specific, and fixing it left the RISC-V board booting EDK II exactly as
before.

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

With nothing on the PCI bus — the disk is [further down](#the-disk-and-the-one-thing-between-this-board-and-an-operating-system):

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
ends the run.

**The virtual-time figures in this section were measured without a disk
controller.** With one on the bus the prompt arrives at **372.3 seconds** rather
than 367.2 — the extra five seconds are `PciBusDxe` and `NvmExpressDxe` doing
real work — and the shell that comes up is the one on the disk rather than the
one inside the firmware volume. In between those two numbers there was a run
that took 815.6 seconds, and the 444-second difference was `NvmExpressDxe`
waiting out timeouts on a window that never decoded;
["The disk"](#the-disk-and-the-one-thing-between-this-board-and-an-operating-system)
has that whole story.

They also predate the board having anything to say about itself. Since
["The tables"](#the-tables-and-the-one-channel-a-uefi-board-has-to-hand-them-over),
a run that types `map -b` at the prompt and stops on its output ends at
**373 169 ms** — so `AcpiPlatformDxe` reading three files a byte at a time
through a port, relocating them and installing eight tables costs on the order
of a second of guest time, paid once per boot. That number is the same to the
millisecond under `interp` and under `jit-host`, which is the property this
board exists to keep honest: the engines are a speed knob and never a semantic
one.

The whole thing was **367.2 seconds of virtual time** — a couple of
minutes of host time under the interpreter, and under a minute under `jit-host`
on an idle machine.

Each phase, as an observation rather than an inference:

* **SEC.** The reset vector at `0xfffffff0` executes out of the code bank, the
  processor is in 32-bit protected mode and then in **long mode** with paging on
  (`cr0=80000023 cr4=0x660 efer=0x500 cr3=0x800000`).
* **PEI.** It sizes memory from CMOS `0x34`/`0x35` — still the route now that
  the board has a `fw_cfg`, because that device deliberately serves no
  `etc/e820` and `PlatformGetSystemMemorySizeBelow4gb` falls back to the CMOS
  pair — and decompresses the main firmware volume into RAM.
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
stop at **the same virtual instant** — 367174 ms — with byte-identical output,
compared as a hash of the whole console transcript rather than by eye:

| engine | guest instructions retired in blocks |
| --- | --- |
| `interp` | — |
| `jit` | 528,043,760 of 561,787,038 (94.0%) |
| `jit-host` | 528,043,760 of 561,787,038 (94.0%) |

Host time is not in the table any more: these three were measured with six other
builds running on the machine, and a number that says more about what else was
compiling than about the engine is worse than no number. The ordering has not
changed — `jit-host` is several times the interpreter — and `benches/` is where
that belongs.

**Re-measured after `cpu::x86::lift` gained the reserved-NOP space, the
repeat-prefixed no-operations and the computed near transfers, and
`cpu::x86::engine` stopped guessing an unknown block's cost** — it no longer
costs a block at all, and `docs/platforms/pc64.md` measures what each of those
was worth. A different OVMF build, so the row above is not the control for the
row below: this one is Gentoo's `edk2-ovmf` `OVMF_CODE.fd`, which stops at
**366577 ms** rather than 367540, and it is 560 517 194 guest instructions to
the shell rather than 561 105 521.

| engine | host time | guest instructions retired in blocks |
| --- | --- | --- |
| `interp` | 106 s | — |
| `jit` | 59 s | 547,917,237 of 560,517,194 (97.8%) |
| `jit-host` | 22 s | 547,917,237 of 560,517,194 (97.8%) |

What travels between the two tables is the shape rather than the ratio: a
64-bit firmware was already the best case this board had, and the same three
closures that took `pc64` from 84.5% to 97.3% take it from about 95% to about
98%. All three engines still stop at the same virtual instant with
byte-identical output, which is the property this section is about.

The control-register moves are not lifted — `cpu::x86::lift` returns `None` for
every `MOV` naming `CRn`, `DRn`, `TRn` or a segment register — so `CR8` runs on
the interpreter under all three by construction, and the frame alignment is in
the shared delivery path. The identical instruction counts are the evidence that
neither changed which engine ran what.

### Why nothing wrote the store: one bit in a status register

For a while this board booted to a shell and **kept nothing**. The variable
store was byte-identical to the image as shipped after a run that reached
`Shell>` — 127 programmed bytes before and after, the log still ending at
`0xf020` — so `RSEMU_OVMF_VARS_OUT` reproduced its input and a reboot had
nothing extra to find. A BDS that selects and starts `Boot0001` writes
`BootOrder`, so something between `QemuFlashFvbServicesRuntimeDxe` and the
fault-tolerant write was not binding.

It was not SMM, and it was not a missing platform service. What settled it was
watching the bus: with every command cycle either bank received logged, a whole
boot to the shell issued **four**.

```text
vars +0x10 <= 0x50
vars +0x10 <= 0x70
code +0x10 <= 0x50
code +0x10 <= 0x70
```

Two bytes at one address in each bank, and then silence for the rest of the
boot. That is the opening of `QemuFlashDetected`
(`OvmfPkg/QemuFlashFvbServicesRuntimeDxe/QemuFlash.c` — BSD-2-Clause-Patent, so
readable), which tells flash from RAM and from ROM with single-byte cycles
before it will use a bank at all:

| it writes | it reads back | and concludes |
| --- | --- | --- |
| `0x50`, Clear Status Register | the command | RAM |
| `0x70`, Read Status Register | the original byte | ROM |
| | `0x70` | RAM |
| | **`0x00`** | flash — go on and test whether it is writable |
| `0x10`, the original byte, `0x70` | SR.4 set | flash, write protected |
| | SR.4 clear | **flash, writable** |

The probe address is the first byte of the bank that is none of `0x50`, `0x70`
or `0x00`, which in a split OVMF's variable store is offset `0x10` — the first
byte of the firmware volume's `EFI_SYSTEM_NV_DATA_FV` GUID, `0x8d`. Our part
answered the `0x70` with `0x80`: SR.7, ready. That is not `0x8d`, not `0x70`
and not `0x00`, so the driver fell off the end of every branch, `QemuFlashDetected`
returned false, `QemuFlashInitialize` returned `EFI_WRITE_PROTECTED`, and OVMF
fell back to `EmuVariableFvbRuntimeDxe` — a variable store in RAM. Everything
after that *worked*: variables could be set, read back within the run, and
listed. They simply were not in the flash.

**`0x00` is what the silicon says, and our model was wrong.** The Intel
StrataFlash P30 datasheet, §14.1.1:

> The Clear Status Register command clears the status register. It functions
> independent of V<sub>PP</sub>. The Write State Machine (WSM) sets and clears
> SR[7,6,2], but it sets bits SR[5:3,1] without clearing them. […] A device
> reset also clears the Status Register.

So SR.7 is a **latch the write state machine drives**, not a live "am I busy"
signal: it reads back one because an operation finished, and the Clear Status
Register command — and a reset — clear the whole register. A part that has just
been cleared and asked for nothing since reads `0x00`. `flash.cfi` had `0x50`
clearing only the error bits and `SR_RESET` set to `SR_READY`, which is the
plausible-sounding reading of "SR.7 means ready" and the wrong one.

The fix is three lines of behaviour and no new property:

* a device reset leaves the status register at zero;
* `0x50` clears all of it, SR.7 included, and still leaves the read mode alone;
* every operation the write state machine actually runs — a program, an erase,
  a lock cycle, a write-buffer setup, and a refused command sequence — sets SR.7
  when it finishes, which is where SR.7 was always coming from.

Nothing else changed: no machine-file property, no `src/dev/q35`, no CPU. And
the RISC-V board, which shares the device and drives it with EDK II's
`VirtNorFlashDeviceLib`, still boots the same firmware to the same shell — that
driver only ever reads the status register *after* issuing an operation
(`NorFlashWriteSingleWord`, `NorFlashEraseSingleBlock` and
`NorFlashUnlockSingleBlock` all spin on SR.7 after their command, and
`NorFlashWriteBuffer` reads it right after the `0xe8` setup to ask whether a
buffer is free), and each of those now sets it. Re-run with its own flash banks
bound, that board still reaches `UEFI v2.70` and still leaves 1,989 changed
bytes and a `BootOrder` in `edk2-riscv-vars.fd`.

One recorded number did move with it: `riscv-virt`'s entries in
`tests/goldens/frame-hashes.txt`, because the flash part's power-up status
register is machine state and `Machine::state_hash` covers it. That is the
regression doing its job, and it is the only golden this change touches.

### A variable written in one boot is there in the next

With the probe answered, the same run programs **6,206** bytes of the store
where it programmed nothing before (it was 5,799 before the board grew a
`fw_cfg`; the firmware now has a little more to remember), and the names in it are the ones a BDS
writes: `BootOrder`, `Boot0000` (`UiApp`), `Boot0001` (`EFI Internal Shell`),
`Boot0002`, `Timeout`, `PlatformLang`, `ConIn`/`ConOut`/`ErrOut`,
`MemoryTypeInformation`.

The test that holds it is
`a_variable_written_at_the_shell_is_there_after_a_reboot`, and it is a **second
boot** rather than a second look: two machines built from the machine file, and
the only thing carried from the first to the second is the variable bank's
bytes. The bank is bound as a `dev::medium::Medium` — the `--drive` path, not
the media table — so what crosses is what `Machine::flush` wrote back, which is
the call `rsemu run` makes when a machine stops and the one a no-op
`Device::flush` would have skipped.

The first boot types this at the shell:

```text
Shell> setvar rsemu -guid 8f1d4a52-6b3c-4e19-9d20-72736656d757 -nv -bs -rt =0102030405060708
Shell> setvar rsemu -guid 8f1d4a52-6b3c-4e19-9d20-72736656d757
8F1D4A52-6B3C-4E19-9D20-72736656D757 - rsemu - 0008 Bytes
01 02 03 04 05 06 07 08
```

and the second, on a fresh machine that has never been told about the variable:

```text
Shell> setvar rsemu -guid 8f1d4a52-6b3c-4e19-9d20-72736656d757
8F1D4A52-6B3C-4E19-9D20-72736656D757 - rsemu - 0008 Bytes
01 02 03 04 05 06 07 08
```

The GUID is the test's own and deliberately not `gEfiGlobalVariableGuid`: EDK
II's `VarCheckUefiLib` refuses a name under the global GUID that the UEFI
specification does not define, so `setvar rsemu` with no `-guid` answers
"Unable to set" however well the flash works. That was worth finding out the
first time rather than mistaking it for the bug.

## The disk, and the one thing between this board and an operating system

A firmware exists to start an operating system, and this board could not: it
had no storage controller, so `map` answered `No mapping found.` and there was
nothing else to say. It has one now — an **NVM Express controller at
`00:04.0`**, the same part at the same address `q35-linux` gives a kernel, for
a different consumer. EDK II's `NvmExpressDxe` is a generic `MdeModulePkg`
driver that binds on a class code of `010802h` and **polls** its completion
queues, so a namespace needs nothing wired: the board adds ten lines and no
interrupt.

The firmware finds it. `PciBusDxe` enumerates `00:04.0`, sizes its 8 KiB
window, allocates it out of the 64-bit aperture `PlatformInitLib` published,
and `NvmExpressDxe` binds and enables memory space and bus mastering:

```text
q35-uefi:   nvme 00:04.0 command=0x2000006 bar0=0x800000000 (MSE=1, BME=1)
```

And then it reads `0xffffffff` out of every register in that window, because
**a base address register programmed through the ECAM window never decodes**.

### The firmware said so itself, in a buffer it never printed

A `RELEASE` OVMF simply stalls; the debug build asserts. Neither prints
anything, because EDK II's `DebugAssert` offers its message to
`SerialPortWrite` only if the debug port answered, and this board's `0x402`
does not. But the message is *formatted first*, into a 512-byte buffer on the
asserting driver's own stack, and then `CpuDeadLoop` spins with that stack
frame intact. `RSEMU_OVMF_WHOIS` named the frame out of the loaded image's own
PE/COFF debug directory, and `RSEMU_OVMF_ASCII` read the buffer:

```text
q35-uefi:              rip = 0x0007e03058  DxeCore.dll + 0x11058
q35-uefi:       [rsp+0xf8] = 0x00068ee420  NvmExpressDxe.dll + 0x9420
q35-uefi: the string at 0x7df1500: "ASSERT .../MdeModulePkg/Bus/Pci/
    NvmExpressDxe/NvmExpressHci.c(778): (Private->Cap.Mpsmin + 12) <= 12"
```

`CAP.MPSMIN` is `CAP[51:48]`, the smallest host page size the controller
supports; ours is 0, which is the 4 KiB `NvmExpressDxe` requires. The value it
asserted on is `0xf` — the top nibble of an all-ones read. `RSEMU_OVMF_HEX` at
the driver's private structure (found by its `NVME` signature, which the
register `RDI` was left holding pointed straight at) confirms it: `Cap` is
sixteen `ff` bytes.

### Why the window was dead, and why looking at it made it work

`src/bus/pci/bar.rs`'s module docs describe the hard case exactly, under
"Moving a mapping from inside a configuration write". A BAR write arrives
*inside* an address-space access, so that space's topology lock is already held
for reading; taking the blocking write guard would invert `core::sync`'s
ladder. `Bars::sync` therefore uses the order-exempt `try_topology`, and when
that fails it sets a `stale` flag. What the flag used to wait for was **the
next configuration access**, on an assumption the file stated plainly:

> A configuration cycle **travels through the I/O space** […] the retry at the
> next configuration access fails for the same reason, for ever.

It said that of an *I/O* BAR, and refused to map one at all. But a q35 has a
second route to configuration space — **ECAM, in the memory space** — and
through it every BAR was in precisely that position. The write is a memory
access, so `try_topology` on the memory space cannot succeed; neither could the
retry, or the one after that. A firmware that never touches `0xcf8` never healed
it, and a UEFI firmware on a q35 never touches `0xcf8`.

`tests/q35_uefi.rs` reproduces it in sixty milliseconds with no firmware at all
— `the_disk_controllers_window_decodes_when_ecam_placed_it`, which was committed
`#[ignore]`d as a reproduction the way `tests/kvm_q35_linux_smp.rs` was, and now
runs on every `cargo test`.

Its second half is a control and also a warning about instruments. One
configuration access through the *port* space does not hold the memory space's
topology, and through it the window appeared at once. `report_chipset` and
`report_nvme` reach configuration space that way, so **every register they
printed looked perfect because looking at it fixed it** — which is why the
post-mortem above showed a placed BAR and a readable `CAP` while the guest saw
neither. A probe that paraphrases a defect can repair it before you look.

This was not a UEFI problem and not a q35-uefi problem. Any guest on any board
in this tree that programmed a BAR through MMCONFIG got a function that answered
its configuration space and decoded nothing; `q35-linux` escaped it because
Linux assigns its resources through `0xcf8` before MMCONFIG is up.

### What fixed it: the retry moved off the access path entirely

Two routes were open. The `Deferred` action `bar.rs` already named is not
reachable from here — `PciFunction::config_write` has no deferred queue and
neither does `MemOps`, so spelling it that way means threading one through every
device's MMIO path, which is a `core` change and a large one. An "owed
retopology" the address space drains when its last read guard goes is closer to
the hardware's ordering, but it puts a callback registry and a try-lock on the
hottest path in the emulator to serve an event that happens a few dozen times
per boot — and it would still have to be a *try*-lock, because the CPU's
`BUS`-ranked lock is held there and `TOPOLOGY` sits above `BUS`. It is not more
reliable than the alternative, only more expensive.

The alternative was already in this tree, one file away. `q35.mch` met exactly
this problem for its own `PCIEXBAR` window — which is also moved by a
configuration write that arrives through ECAM — and answered it with a **clock
domain**: `Device::advance_to` runs from the run loop with no access in flight,
which is the moment a topology guard is actually available. So the same
mechanism now covers the whole fabric rather than one bridge's own registers:

* `PciFunction` gained `retopology_owed` and `settle`, defaulting to "nothing
  owed"; the four functions in the tree that carry `Bars` answer them.
* `PciBus` keeps one lock-free flag, raised by any configuration cycle that
  leaves a function owing, and `PciBus::settle` is the sweep that drains it.
* `q35.mch` — the one object on the board that both knows every function and
  holds a clock domain — asks for the next tick while anything is owed and runs
  the sweep from `advance_to`.

The bound is one scheduler round, and a round on this board is capped at
`max_ticks_per_quantum` = 10 000 processor cycles rather than at the quantum's
1 ms. **A window placed through ECAM therefore decodes late** — within a few
thousand guest instructions of the write, where real firmware programs every BAR
in `PciBusDxe` and reads the first device register in a different driver
entirely. That is the honest cost of not having a `Deferred` on the access path,
and it is written down in `bar.rs` rather than left to be discovered.

One thing that fell out of it: `Bars::sync` now knows what it already placed, so
the sizing sweep — all-ones, read the mask, write the base, `COMMAND[1]` clear
throughout — asks nothing of the address space at all. It used to flatten the
space once per configuration write and mark the function stale each time.

Boards with no q35 bridge are unaffected and need to be: a 440FX has one route
to configuration space, it is in the I/O space, and `bar.rs`'s original retry
works there. Every board in this tree that publishes ECAM uses `q35.mch`.

### What it cost while it was unfixed

The shell still came up and still answered `map: No mapping found.` — the
`RELEASE` build's `ASSERT` is compiled out, so `NvmExpressDxe` waited out its
timeouts, failed its `Start`, and BDS carried on. `CAP.TO` read as ones with the
rest of the register, which the driver reads as 128 seconds per wait, so the
prompt arrived at **815,584 ms** of virtual time rather than 367,174. Those 444
seconds were a measurement of the defect and went away with it; `DEFAULT_MS` in
the test is back down, to 480,000.

### And then the firmware boots off it

`scripts/fetch-testdata.sh esp` builds the disk, into the same ignored
directory as everything else and committed no more than the firmware is: a
64 MiB FAT volume with an EFI application at **`\EFI\BOOT\BOOTX64.EFI`**, which
is the file name an x64 boot manager looks for on a device it has no `Boot####`
for (UEFI 2.10 §3.5.1.1, "Removable Media Boot Behavior"), plus a `startup.nsh`
the shell runs on its way up. The application is the local edk2 package's own
`Shell.efi` — BSD-2-Clause-Patent, the same source the firmware comes from.

No partition table, deliberately: EDK II's FAT driver binds a whole-disk
`BlockIo` as readily as a partition's, and a bare FAT volume needs `mtools` and
nothing else where a GPT would need `sgdisk` or `parted` on the path. `fstool`
builds either from a TOML spec — `examples/efi-disk.toml` is exactly this
image — once its CLI is installable here, and that is the better answer the day
a board wants a realistic disk.

With the window decoding, BDS finds the disk, makes a boot option out of it and
starts the application on it. Every line of this is a byte the guest wrote to
COM1:

```text
BdsDxe: loading Boot0001 "UEFI RSEMU NVME CONTROLLER RSEMU000000000000000 1"
    from PciRoot(0x0)/Pci(0x4,0x0)/NVMe(0x1,00-00-00-00-00-00-00-00)
BdsDxe: starting Boot0001 "UEFI RSEMU NVME CONTROLLER RSEMU000000000000000 1"
    from PciRoot(0x0)/Pci(0x4,0x0)/NVMe(0x1,00-00-00-00-00-00-00-00)
UEFI Interactive Shell v2.2
EDK II
UEFI v2.70 (EDK II, 0x00010000)
Mapping table
      FS0: Alias(s):F0:;BLK0:
          PciRoot(0x0)/Pci(0x4,0x0)/NVMe(0x1,00-00-00-00-00-00-00-00)
Shell> echo rsemu: startup.nsh ran off %hostname%FS0
rsemu: startup.nsh ran off FS0
Shell> map -b
Mapping table
      FS0: Alias(s):F0:;BLK0:
          PciRoot(0x0)/Pci(0x4,0x0)/NVMe(0x1,00-00-00-00-00-00-00-00)
Shell> ls fs0:
Directory of: fs0:\
09/06/2026  11:56 <DIR>           512  EFI
09/06/2026  11:56                  55  startup.nsh
          1 File(s)          55 bytes
          1 Dir(s)
```

Three separate claims in that, and the third is the one that matters. `FS0:`
says the **block device** was enumerated; `map -b` and `ls` say a **file
system** was mounted on it; and `startup.nsh` says a *file was read off it and
executed*, which is the only one of the three that could not have come from a
correctly-shaped device that returns nothing. The shell printing the banner is
itself the fourth: it is `\EFI\BOOT\BOOTX64.EFI` off this volume, not the one
in the firmware volume, which is what `BdsDxe: starting Boot0001 … NVMe(0x1…)`
names.

### A kernel entered through its EFI stub

`RSEMU_ESP_KERNEL` and `RSEMU_ESP_INITRD` put a `bzImage` and an initramfs
beside the application. A modern `bzImage` **is** a PE/COFF EFI application, so
the shell launches one with a command line of its own and the firmware — not a
loader — is what placed it:

```console
RSEMU_ESP_KERNEL=/boot/vmlinuz RSEMU_ESP_INITRD=testdata/x86/initramfs-x86.cpio \
    scripts/fetch-testdata.sh esp --force

RSEMU_OVMF_CODE=testdata/x86/OVMF_CODE.fd \
RSEMU_OVMF_VARS=testdata/x86/OVMF_VARS.fd \
RSEMU_OVMF_DISK=testdata/x86/esp.img \
RSEMU_OVMF_EXTMEM=1G \
RSEMU_OVMF_MS=4000000 \
RSEMU_OVMF_INPUT='Shell> =>fs0:\\vmlinuz.efi initrd=\\initrd.img console=ttyS0,115200 nokaslr cryptomgr.notests\r' \
    cargo test --release --features machine-q35-uefi --test q35_uefi -- \
        --nocapture a_uefi_firmware
```

```text
EFI stub: Loaded initrd from command line option
[    0.000000] Linux version 6.6.67-gentoo-x86_64 ... #1 SMP PREEMPT_DYNAMIC
[    0.000000] efi: EFI v2.7 by EDK II
[    0.000000] printk: console [ttyS0] enabled
[    0.039999] tsc: Detected 25.779 MHz processor
[   45.262356] smpboot: Total of 1 processors activated (51.70 BogoMIPS)
[  482.358601] x86/mm: Checked W+X mappings: passed, no W+X pages found.
[  482.392389] Run /init as init process

rsemu initramfs on Linux 6.6.67-gentoo-x86_64 x86_64
rsemu# uname -srm
Linux 6.6.67-gentoo-x86_64 x86_64
```

**That is an operating system booted through UEFI**, and every link in it is
this repository's: the firmware read the `bzImage` and the initramfs off a FAT
volume on an emulated NVMe namespace, `LoadImage`/`StartImage`'d a PE/COFF
kernel, handed it a memory map and a system table, and the kernel came up on the
board's 16550 and ran `/init`. 1,337 seconds of virtual time end to end — 372 of
them the firmware — and about nine minutes of host time under the interpreter.

That boot had **no MADT** — `APIC: ACPI MADT or MP tables are not detected`, and
the kernel fell back to virtual wire mode, because this OVMF builds its ACPI
tables from `fw_cfg` and this board had none. It has one now, and the same run
comes up in symmetric I/O mode:

```text
[    0.000000] ACPI: RSDP 0x000000003FD7E014 000024 (v02 RSEMU )
[    0.000000] ACPI: XSDT 0x000000003FD7D0E8 000044 (v01 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: FACP 0x000000003FD79000 0000F4 (v03 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: DSDT 0x000000003FD78000 000772 (v02 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: FACS 0x000000003FDFC000 000040
[    0.000000] ACPI: APIC 0x000000003FD7C000 00005A (v06 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: MCFG 0x000000003FD7B000 00003C (v01 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: HPET 0x000000003FD7A000 000038 (v01 RSEMU  RSEMUQ35 …)
[    0.000000] IOAPIC[0]: apic_id 1, version 17, address 0xfec00000, GSI 0-23
[    0.000000] ACPI: INT_SRC_OVR (bus 0 bus_irq 0 global_irq 2 dfl dfl)
[    0.000000] ACPI: INT_SRC_OVR (bus 0 bus_irq 9 global_irq 9 high level)
[    0.000000] ACPI: Using ACPI (MADT) for SMP configuration information
[    0.000000] APIC: Switch to symmetric I/O mode setup
[    0.029999] ..TIMER: vector=0x30 apic1=0 pin1=2 apic2=-1 pin2=-1
[  149.284209] ACPI: Using IOAPIC for interrupt routing
[  160.371066] hpet0: at MMIO 0xfed00000, IRQs 2, 8, 0
[  160.522105] clocksource: Switched to clocksource hpet
[ 1289.297467] Run /init as init process

rsemu initramfs on Linux 6.6.67-gentoo-x86_64 x86_64
rsemu# uname -srm
Linux 6.6.67-gentoo-x86_64 x86_64
```

Every one of those addresses is where **the firmware** put the table, out of its
own pool, after relocating a blob whose pointers were offsets; every OEM field
is this board's; and every address *inside* the tables — `0xfec00000`, the
override that puts ISA IRQ0 on global interrupt 2, the ECAM window the MCFG
names — was read out of the realized machine by
[`src/dev/q35/acpi.rs`](../../src/dev/q35/acpi.rs) rather than written down.
2,156,716 ms of virtual time end to end, and 334 s of host time under
`jit-host`; 2,720,192,460 of 2,795,479,885 guest instructions retired in blocks
(97.3%).

The `ftrace` self-check and the `DEBUG_WX` page-table walk still each trip the
soft-lockup watchdog — 45 896 ftrace entries and a whole kernel page table
walked at 51 BogoMIPS really is tens of seconds of guest time — which is the
emulator being slow rather than wrong; there are rather more of them than there
were, which is what a watchdog does when the clock it measures against is
running properly.

## The tables, and the one channel a UEFI board has to hand them over

A board describes itself to an operating system; this one could not, and the
reason is worth stating precisely because it is not "ACPI was unimplemented".
[`q35`](q35.md) has a `q35.acpi` device that **generates the whole set from the
realized machine** — a MADT read out of the APICs' own mappings, an MCFG for the
window `PCIEXBAR` placed, a `_PRT` read out of the bridge's routing registers —
and stages it at `0xe0000`, where ACPI §5.2.5.1's search finds it. Under UEFI
nothing searches. A kernel started by the EFI stub takes its RSDP from the EFI
configuration table (UEFI 2.10 §4.6), which holds what the **firmware**
installed, and an OVMF build has exactly one source for that: `fw_cfg`.

So the tables were never the missing part. The *channel* was.

### What `fw_cfg` is

A selector register and a data register — `0x510` and `0x511` on an I/O-mapped
platform — and behind them numbered items, one of which (`0x19`) is a directory
naming the rest. A firmware writes a selector, then reads bytes; a file lookup
is a walk of the directory followed by a second select. Everything about it that
this board implements is established from EDK II's own consumption of it
(`OvmfPkg/Include/IndustryStandard/QemuFwCfg.h`, `OvmfPkg/Library/QemuFwCfgLib`,
`OvmfPkg/Library/AcpiPlatformLib/QemuFwCfgAcpi.c`) — BSD-2-Clause-Patent, and
the permissive source for what the firmware we serve expects. The interface
originated in QEMU, whose source is off limits to this repository and was not
opened.

Two decisions in `src/dev/q35/fwcfg.rs` are worth defending rather than
recording:

* **The signature reads `QEMU`.** `QemuFwCfgInitialize` refuses the interface
  unless key `0x0000` does, so those four bytes are the identifier that means
  "this register pair is that register pair" — the same kind of fact as the
  `010802h` class code that makes `NvmExpressDxe` bind to this board's disk.
  Answering anything else is answering nothing at all, which is what this board
  did before.
* **The DMA path is deliberately absent.** Key `0x0001` reports revision 1
  without `FW_CFG_F_DMA`, and `0x514` is not decoded. The DMA interface would
  make this device a bus master fetching a descriptor the guest wrote and
  copying into a buffer the guest chose, from inside an I/O write, with the
  processor's `BUS`-ranked lock held — the hazard class the virtio work found
  live. It buys nothing here: EDK II's `InternalQemuFwCfgReadBytes` falls back
  to `IoReadFifo8`, one `rep insb`, and the whole table set is tens of
  kilobytes. A guest that genuinely needs DMA — Linux's `fw_cfg` sysfs driver,
  or an MMIO transport, which has no port path at all — is the reason to add it,
  and the termination argument then has to be made rather than assumed.

### The blob and the linker script, which is the part with content

ACPI tables hold *addresses*, and a board cannot know them: the firmware
allocates the tables itself, from its own pool, long after this device has
answered. The arrangement EDK II implements is a blob whose pointers are
**offsets**, plus a script saying how to turn each offset into an address:

| file | what it is |
| --- | --- |
| `etc/acpi/tables` | FACS, DSDT, MADT, MCFG, HPET, FADT and XSDT back to back, every pointer field holding the offset of its target *in this same blob*, and every `Checksum` byte zero |
| `etc/acpi/rsdp` | a revision 2 RSDP whose `XsdtAddress` is likewise an offset |
| `etc/table-loader` | 128-byte commands: `Allocate`, `AddPointer`, `AddChecksum` |

The firmware allocates each named file, adds the containing blob's base to every
pointer field the script names, and then computes each checksum. Two rules fall
out of reading `QemuFwCfgAcpi.c` rather than out of taste, and both are
load bearing:

* **The checksum bytes have to start at zero.** `ProcessCmdAddChecksum` stores
  `CalculateCheckSum8(range)` — `0x100 - sum` — at `ResultOffset`, over a range
  that *includes* that byte. A table shipped with its checksum already correct
  comes back summing to minus that byte instead of to zero, and a table that
  does not sum to zero is one the second pass declines to install, silently.
* **A table is identified by being pointed at.** The second pass walks the
  `AddPointer` commands and asks whether what each one now points at carries a
  plausible length and a zero sum; if so it installs it, skipping only RSDT and
  XSDT (`EFI_ACPI_TABLE_PROTOCOL` builds its own). So the XSDT's entries are
  what get the MADT, MCFG and HPET installed, and the FADT's `FIRMWARE_CTRL` and
  `DSDT` fields are what get the FACS and the DSDT installed — which is why
  those two are in the blob at all, given that §5.2.8 keeps them out of the
  XSDT.

`tests/q35_uefi.rs`'s `the_loader_script_the_board_hands_over_builds_a_madt`
**runs that procedure** against the board — allocating at two addresses of its
own choosing, relocating, checksumming, and then following the RSDP to the XSDT
to the MADT and reading the local APIC and I/O APIC addresses back out of it. It
needs no firmware and it takes milliseconds. It is written that way on purpose:
a test that only asked "does the device answer" would pass on a blob whose
checksums were pre-filled, which is precisely the blob a firmware throws away
without a word.

### An existing generator was reused, and the other one still cannot be

`src/dev/q35/acpi.rs` builds every byte of every table here. Nothing about a
MADT changes because a firmware rather than an operating system will read it,
and the input is the same `survey` of the same realized machine — so `fwcfg`
takes the table *bodies* from that module and adds only the packaging: a layout
at base zero, the list of fields that are pointers, and the script. The one
change inside `acpi.rs` was to lift the XSDT and RSDT builders out of `generate`
into functions taking a list of addresses, because "address" there is now
sometimes an offset. `TableConfig` is shared too, so a board cannot declare one
processor count to a firmware and another to an operating system.

`src/fw/pcbios/tables.rs` is the third generator and stays separate, for the
three reasons its own module docs already give: it is behind `fw-pcbios`, which
must not imply a chipset; its input is a *description* rather than a realized
machine, and there is no address space to survey when a ROM image is assembled;
and its FADT describes a board with no ACPI register block at all.

### What OVMF actually asked for

Measured, not assumed — `q35.fwcfg` keeps the last sixty-four selectors the
guest wrote and `tests/q35_uefi.rs` prints them with the file each names:

```text
q35-uefi: what the firmware asked fw_cfg for:
q35-uefi:   0x0000  <signature>
q35-uefi:   0x0001  <interface version>
q35-uefi:   0x0019  <the file directory>
q35-uefi:   0x0005  <boot cpu count>
…
q35-uefi:   0x0017  <not served: reads as zeroes>
q35-uefi:   0x0008  <not served: reads as zeroes>
q35-uefi:   0x000b  <not served: reads as zeroes>
q35-uefi:   0x0014  <not served: reads as zeroes>
q35-uefi:   0x0019  <the file directory>
q35-uefi:   0x0022  etc/table-loader
q35-uefi:   0x0019  <the file directory>
q35-uefi:   0x0020  etc/acpi/rsdp
q35-uefi:   0x0019  <the file directory>
q35-uefi:   0x0021  etc/acpi/tables
q35-uefi:   0x000e  <not served: reads as zeroes>
q35-uefi:   it stopped 196 byte(s) into item 0x0019
```

Read from the bottom, that is the whole hand-off. `AcpiPlatformDxe` looks up
**`etc/table-loader`**, reads it, and executes it; its two `Allocate` commands
are the two lookups that follow — `etc/acpi/rsdp` and `etc/acpi/tables` — each
preceded by the directory walk `QemuFwCfgFindFile` makes *every time*, since the
interface has no notion of a cached handle and a lookup by name really is a
re-read of the directory. Above them, `0x0017`, `0x0008`, `0x000b` and `0x0014`
are `QemuKernelLoaderFsDxe` asking whether a kernel was passed on a command line
this board has no way of passing one on; all four read as zeroes and it
concludes there is none. `0x000e` is the boot menu. The pairs of
`<signature>`/`<interface version>` scattered through the middle are
`QemuFwCfgInitialize`, once per module that links the library.

The trace keeps the last sixty-four selectors, so the start of the boot has
rolled off the one above; `0x0005` — the processor count
`PlatformMaxCpuCountInitialization` reads in PEI — is visible in a run that
stops earlier. Answering it matters: zero there means "the platform does not
know", and `MpInitLib` then counts application processors by *timeout* rather
than by number.

Every other lookup a firmware makes is a name that is not in the directory, and
each one is a fall-back EDK II is written to take: no `etc/e820`, so memory
sizing stays on CMOS `0x34`/`0x35`; no `etc/system-states`, so `QemuFwCfgS3Enabled`
is false and S3 stays off; no `etc/smbios/*`, so SMBIOS comes from the firmware
volume's own tables; no `bootorder`, so the boot order is the variable store's.
Serving fewer files than QEMU does is not a shortfall to be closed — each one
would be a second place this board describes itself, and the first place already
works.

### And then the board had to grow the wire the description implies

With the tables installed the kernel found its APICs, took symmetric I/O
mode — and panicked:

```text
[    0.000000] APIC: Switch to symmetric I/O mode setup
[    0.000000] ..TIMER: vector=0x30 apic1=0 pin1=2 apic2=-1 pin2=-1
[    0.000000] ..MP-BIOS bug: 8254 timer not connected to IO-APIC
[    0.000000] Kernel panic - not syncing: IO-APIC + timer doesn't work!
```

which is, word for word, the panic
[`q35-linux`](q35-linux.md)'s machine file warns about — and for the same
reason. The set the firmware now installs includes an **HPET table**, so
`hpet_time_init()` succeeds where it used to fail, sets `LEG_RT_CNF`, stops
loading the 8254's counter 0 and waits for the HPET's comparator 0 on IRQ0.
`LEG_RT_CNF` is not a register inside one chip: it is a board-level multiplexer
between three of them (*IA-PC HPET Specification* §2.3.5), and
`machines/q35-uefi.machine` did not have it. `machines/q35-linux.machine` has
had those five gates all along; this board never needed them, because a guest
that has never been told the machine has an HPET does not use it.

That is the shape of this whole change in one sentence: **describing a board
truthfully makes a guest exercise parts of it that were never reached**, and the
gap was in the wiring rather than in the description. The board now carries the
same `wire.not`/`wire.and` multiplexer, and with it the timer check passes
without comment.

## The firmware narrates its own boot

Everything above was read out of the machine — a register the guest left behind,
a string on its stack, four cycles on a bus. The firmware was writing a running
commentary the whole time, and this board was throwing it away.

EDK II's `PlatformDebugLibIoPort` sends every `DEBUG()` and every `ASSERT()` to
**one byte-wide I/O port**, and it asks first. The whole protocol is two
expressions and one build-time token, all of them in `OvmfPkg`
(BSD-2-Clause-Patent, so readable):

| | |
| --- | --- |
| `DebugIoPortQemu.c` | `return IoRead8 (PcdGet16 (PcdDebugIoPort)) == BOCHS_DEBUG_PORT_MAGIC;` — one byte read, compared against `0xE9` |
| `DebugLib.c` | `IoWriteFifo8 (PcdGet16 (PcdDebugIoPort), Length, Buffer)` in `DebugPrintMarker` and again in `DebugAssert`, each guarded by `PlatformDebugLibIoPortFound ()` |
| `OvmfPkg.dec` | `PcdDebugIoPort\|0x402\|UINT16\|4` — where `0x402` comes from, and that it is the *firmware's* build-time choice rather than a chipset's |

`IoWriteFifo8` is `rep outsb`: byte writes, in order, to one address, with
nothing read back between them. So there is no status register, no busy bit and
no handshake — a read is a constant and a write is a character, and that is the
device in its entirety. It is `src/dev/pc/debugcon.rs`, behind
`dev-pc-debugcon`, mapped by `machines/q35-uefi.machine` at `0x402`.

Two consequences worth stating, because both are the reason it is written the
way it is:

* **`0xff` is not `0xe9`.** This board's I/O space is `read-as-ones`, so before
  the port was mapped the detect read `0xff`, `PlatformDebugLibIoPortFound`
  answered false for the rest of the boot, and every `DEBUG()` in the firmware
  went nowhere. Nothing was broken and nothing said so — which is exactly the
  failure mode this whole page keeps running into.
* **A byte the host will not take is dropped, not retried.** `rep outsb` has
  nowhere to put back pressure, so a device that stalled would stall a guest
  with no way of learning it. The count of dropped bytes is kept, so a truncated
  log looks truncated rather than short.

The log goes to its own character port — `debug`, beside COM1's `console` —
rather than into the serial stream, because the firmware writes both at once and
a `DEBUG()` line spliced into the UEFI shell's line editor would ruin both.

`MemAttrs::debug` is honoured in the narrow way the port allows: a monitor's
read answers the same `0xe9` and does not count as a detect, and a monitor's
*write* is refused outright, because it would put a line into the guest's own
log that the guest never wrote.

What comes out is the boot, from the first instruction. `rsemu run q35-uefi
--capture debug=boot.log` with the 4 MiB `edk2-x86_64-code.fd`, which has debug
strings in it:

```text
SecCoreStartupWithStack(0xFFFCC000, 0x820000)
Register PPI Notify: DCD0BE23-9586-40F4-B643-06522CED4EDE
The 0th FV start address is 0x00000820000, size is 0x000E0000, handle is 0x820000
DiscoverPeimsAndOrderWithApriori(): Found 0xD PEI FFS files in the 0th FV
Loading PEIM at 0x0000082B980 EntryPoint=0x0000082EA27 PcdPeim.efi
…
Loading PEIM at 0x00000834840 EntryPoint=0x0000083C488 PlatformPei.efi
Platform PEIM Loaded
CMOS:
00: 36 00 05 00 00 00 05 01 01 26 26 02 50 80 00 00
10: 40 00 00 00 2D 80 02 00 3C 00 00 00 00 00 00 00
20: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 2B
30: 00 3C 20 00 10 07 00 00 00 00 00 00 00 00 00 00
…
QemuFwCfgProbe: Supported 1, DMA 0
Select Item: 0x19
…
PlatformAddressWidthFromCpuid: Signature: '', PhysBits: 40, GuestPhysBits: 0, QemuQuirk: On, la57: Off, Valid: No
AddressWidthInitialization: Pci64Base=0x800000000 Pci64Size=0x800000000
```

Every line of that is the firmware describing this repository's own devices
back to it, and three of them are worth pointing at. `CMOS:` is `pc.rtc`'s
register file, dumped by `PlatformPei` — `0x34`/`0x35` reading `10 07`, which is
`0x710` blocks of 64 KiB above 16 MiB, and puts the top of low memory at
`0x8100000`: exactly the 128 MiB of extended memory
`machines/q35-uefi.machine` declares, read back through the route that has
always carried it.
`QemuFwCfgProbe: Supported 1, DMA 0` is `q35.fwcfg` answering the signature and
*declining* the DMA feature bit, which is the decision
["What `fw_cfg` is"](#what-fw_cfg-is) argues for, confirmed from the consumer's
side rather than from ours. And `Select Item: 0x19` is the directory walk the
same section reconstructed from a ring of selectors — now printed by the
firmware as it happens.

Four hundred seconds of virtual time gets 1,272 lines of it, through PEI, into
the DXE core and down its driver dispatch (`NvmExpressDxe.efi`, `Fat.efi`,
`Ps2KeyboardDxe.efi`); 1,400 seconds gets 1,648 and ends with `Shell.efi` being
loaded. A debug-strings build is a good deal slower than the `RELEASE` image the
timings further up were taken with — every `DEBUG()` is a `AsciiVSPrint` and a
few hundred `outsb` — so those numbers are not comparable with the 367-second
one, and are not meant to be.

`--capture debug=boot.log` runs the machine **unpaced**, so those 1,400 seconds
of virtual time cost 1 m 48 s of wall clock under `jit-host`. It used to be
twenty-three minutes: `--console debug` attaches a terminal, and a terminal
session sleeps off whatever each slice did not use, because a machine somebody
is typing at should run at the speed the machine ran at. `--headless` was
already unpaced and printed no character port at all, so before `--capture`
there was no fast way to get this log out of the CLI at all. The run is the same
one either way — the same 1,648 lines, the same 95,726 bytes, the same state
hash.

`tests/q35_uefi.rs` asserts the same log from the other side, in
`the_firmwares_debug_log_reaches_the_port_at_0x402`: it asks the harness to
**keep** that port's bytes rather than throw them away
(`x86boot::Drains::keeping("debug")`), checks that `pc.debugcon` dropped
nothing, and — because the counters tell the two cases apart — reports a
`RELEASE` image as one rather than failing on it.

### Which layer is allowed to throw bytes away

This board opens three character ports — `console`, `debug` and the 8042's
`keyboard` — and for a while every test on it read one and left the other two to
fill. That is what made the boot log a person went looking for the *first
64 KiB* of one: the port stops at 64 KiB, `pc.debugcon` has nowhere to put the
overflow, and `DebugConsole::dropped` counted the rest away. The fix is not in
either of those two places, and saying why is worth a paragraph, because the
same question comes back for every device with a host on the other end:

* **The device stays faithful.** `uart.ns16550` holding a byte with `THRE` clear
  is what the part does when its line will not take one, and it is what a guest
  polling that bit depends on. A "drop it when nobody is watching" mode inside
  the device would model a UART that cannot be flow-controlled at all.
* **The port stays bounded.** `CharPort`'s 64 KiB and then back pressure is the
  honest queue; one that buffered forever would tell the device model a lie
  about the hardware and grow the heap for as long as a guest kept printing.
* **The host decides whether a cable is plugged in.** It is the only layer that
  knows, so it is the only one entitled to discard — and it counts what it
  discarded, so a truncated stream is a line in the report rather than a short
  log nobody can explain.

`pc.debugcon` is the same argument seen from the other side: EDK II writes its
log with `rep outsb`, which cannot be held, so the device drops and counts. Both
`rsemu run` and `tests/x86boot/mod.rs` now drain every port every slice, which
is why that counter should read zero on any run of this board.

### The line that would have ended the variable-store hunt

["Why nothing wrote the store"](#why-nothing-wrote-the-store-one-bit-in-a-status-register)
took four bus cycles and a datasheet paragraph to establish that
`QemuFlashDetected` was answering *no*. The firmware says so itself, on this
port, and names the address it probed while it is at it:

```text
Loading driver at 0x00007AE4000 EntryPoint=0x00007AE7054 FvbServicesRuntimeDxe.efi
QEMU Flash: Attempting flash detection at FFC00010
QemuFlashDetected => FD behaves as FLASH, writable
QemuFlashDetected => Yes
Installing QEMU flash FVB
```

`FFC00010` is offset `0x10` of the variable bank on a 4 MiB build — the probe
address that section works out from the firmware volume's GUID — and
`=> Yes` is the answer that used to be `No`. It is the same finding, arrived at
by reading rather than by inferring, and it is the argument for this device in
one screen.

### And it keeps going, all the way to the shell

Given enough virtual time the same stream reaches BDS and describes the boot
this page spent three sections establishing from the outside:

```text
[Bds]=============Begin Load Options Dumping ...=============
  Boot Options:
    Boot0000: UiApp 		 0x0109
    Boot0001: UEFI RSEMU NVME CONTROLLER RSEMU000000000000000 1 		 0x0001
    Boot0002: EFI Internal Shell 		 0x0001
…
VirtHstiQemuFirmwareFlashCheck: FFC84010 behaves as FLASH, write-protected
[Bds]Booting UEFI RSEMU NVME CONTROLLER RSEMU000000000000000 1
 BlockSize : 512 
 LastBlock : 7FFF 
[Bds] Expand PciRoot(0x0)/Pci(0x4,0x0)/NVMe(0x1,00-00-00-00-00-00-00-00) -> <null string>
[Bds]Booting EFI Internal Shell
Loading driver at 0x00006639000 EntryPoint=0x00006658BE0 Shell.efi
```

`FFC84010` is the *code* bank, and `write-protected` is `readonly = true` in the
machine file being enforced and observed — the second bank of the pair, checked
by a different driver, agreeing with the first. `LastBlock : 7FFF` is the 16 MiB
namespace `nvme.controller` reports when no image is bound, and the `<null
string>` expansion is the boot manager finding no file system on it and moving
on to `Boot0002` — which is `map: No mapping found.` said from the firmware's
side rather than the shell's.

## What is not reached yet

**The `fw_cfg` DMA interface** is one of two, and it is a design review
rather than an omission — ["What `fw_cfg` is"](#what-fw_cfg-is) has the
argument. Nothing this board runs needs it: the firmware falls back to the port
path by construction, and a Linux guest's `fw_cfg` sysfs driver is the first
consumer that would not. Adding it means a device that walks guest structures
from inside an I/O write, and the case for that has to be made on the day, with
a termination argument for whatever the guest handed over.

**No `etc/e820` and no SMBIOS through `fw_cfg`.** Both are deliberate: the
memory map already reaches the firmware through CMOS `0x34`/`0x35` and SMBIOS
already comes out of the firmware volume, and a second description of either
would be a second thing to keep in step. A board that wanted more than 4 GiB of
guest memory would need the e820 file, because CMOS `0x5b`-`0x5d` is the only
other route and it is three bytes wide.

## Running it

```console
scripts/fetch-testdata.sh ovmf esp

RSEMU_OVMF_CODE=testdata/x86/OVMF_CODE.fd \
RSEMU_OVMF_VARS=testdata/x86/OVMF_VARS.fd \
RSEMU_OVMF_DISK=testdata/x86/esp.img \
RSEMU_ENGINE=jit-host \
RSEMU_OVMF_INPUT='Shell> =>map -b\r' \
    cargo test --release --features machine-q35-uefi,jit,cpu-x86-lift \
        --test q35_uefi -- --nocapture a_uefi_firmware
```

The firmware's own log, which needs an image with debug strings in it — the
4 MiB pair out of the local qemu firmware package has them, and the 2 MiB
`edk2-ovmf` `OVMF_CODE.fd` `fetch-testdata.sh` copies is a `RELEASE` build and
has none:

```console
rsemu run q35-uefi \
    --flash0 /usr/share/qemu/edk2-x86_64-code.fd \
    --flash1 /usr/share/qemu/edk2-i386-vars.fd \
    -p flash=4M -p vars=528K -p engine=jit-host \
    --capture debug=boot.log --for 1400s
```

**`--capture` rather than `--console`, and the difference is twenty minutes.**
`--console debug` attaches a *terminal* to that port, and a terminal session is
paced to wall clock — a second of guest time per second of yours, so the 1 400
seconds this log takes cost 1 400 of them. `--capture` drains the port to a file
while the machine runs unpaced, at whatever rate the host manages: the same
1 648 lines, the same 95 726 bytes, in **1 m 48 s** under `jit-host` on the
machine this paragraph was written on. `--for` still says how far to run, and
`-q` leaves stdout to the guest if the capture goes there rather than to a file.

`--capture debug` picks the debug port's stream, `--capture console` COM1's, and
naming both twice captures both — one to a file each, since two streams
interleaved on stdout cannot be told apart afterwards. Every character port the
machine opened that nothing is watching is **drained and thrown away**, in this
loop and in the console, `--gdb` and `--vnc` ones: an unattached port is a cable
nobody plugged in, and a port left to fill stops at 64 KiB and holds a 16550's
`THRE` clear, which stalls the guest for as long as nobody looks at it. The
summary says how many bytes went that way — 1 503 of them on COM1 in the run
above, which is the shell banner nobody asked to see.

That behaviour has a board of its own to be tested on, because this one needs a
firmware image and the tree cannot ship one:
`machines/tests/two-uarts.machine` is an 8086 with COM1 and COM2 and nothing
else, and `tests/cli_capture.rs` boots a hand-assembled ROM on it that writes
128 KiB — twice a `CharPort`'s capacity — to whichever port the `--capture` did
not name, then says one more line on the port it did. That last line cannot
appear unless the unwatched port was drained, which is the whole assertion.

And when a boot goes quiet, the three instruments that make a silent firmware
talk — the driver an address belongs to, out of the loaded image's own PE/COFF
debug directory; the string a stack pointer is pointing at; and a hex dump
through the guest's page tables:

```console
RSEMU_OVMF_WHOIS=1 RSEMU_OVMF_ASCII=0x7df1500 RSEMU_OVMF_HEX=0x688ad18:384 …
```

The original, without a disk:

```console
RSEMU_OVMF_CODE=testdata/x86/OVMF_CODE.fd \
RSEMU_OVMF_VARS=testdata/x86/OVMF_VARS.fd \
RSEMU_OVMF_MS=600000 \
RSEMU_OVMF_INPUT='Shell> =>ver\r' \
RSEMU_OVMF_STOP_AT='UEFI v2.70' \
    cargo test --release --features machine-q35-uefi --test q35_uefi -- --nocapture
```

and the reboot, which needs no arguments beyond the two images because it types
its own script and knows what it is waiting for:

```console
RSEMU_OVMF_CODE=testdata/x86/OVMF_CODE.fd \
RSEMU_OVMF_VARS=testdata/x86/OVMF_VARS.fd \
    cargo test --release --features machine-q35-uefi --test q35_uefi -- --nocapture \
        a_variable_written_at_the_shell_is_there_after_a_reboot
```

It costs two boots — about two minutes of host time under `jit-host` on an idle
machine, and rather more than twice that under the interpreter — and neither
image is modified: the bank the second boot starts from is the medium the first
flushed to, in memory. It briefly cost seven; the difference was the 444 seconds
of virtual time per boot that `NvmExpressDxe` spent timing out on a window that
never decoded, and it went away with the defect above.

`tests/q35_uefi.rs` has the whole variable table. The tests that do *not*
need an image run on every `cargo test`: that the two banks are one contiguous
run up to the reset vector, that the variable bank answers the byte-wide probe
`QemuFlashDetected` opens with — the whole sequence, not just its first cycle,
which is the difference between the test that passed while nothing was written
and the one that is there now — that a program clears bits while only an erase
puts them back, asked of the board, through its address space, at the width the
driver uses, and that a base address register decodes wherever it was
programmed from, through **both** windows onto configuration space.

## The ledger

| | |
| --- | --- |
| reset vector out of flash at `0xfffffff0` | **works** |
| long mode, paging, the SEC page tables | **works** |
| PEI, memory sized from CMOS `0x34`/`0x35` | **works** — and still the route with a `fw_cfg` on the board, which deliberately serves no `etc/e820` |
| `FVMAIN` decompressed into RAM, DXE core entered | **works** |
| DXE drivers dispatched, local APIC software-enabled | **works** |
| an exception delivered, handled and returned from | **works** — and it took `CR8`, the sixteen-byte frame alignment and `IA32_PLATFORM_ID` |
| `PCIEXBAR`, `PMBASE`, `PIRQ[A-D]_ROUT` as the firmware left them | **works** |
| BDS, boot options, `Boot0001` started | **works** |
| the UEFI Shell prompt on COM1 | **works** |
| the shell executing what is typed at it | **works** (`ver`, over the board's 16550) |
| the same run under `interp`, `jit` and `jit-host` | **works** — same virtual instant, same output |
| the flash probe, program and erase the variable driver needs | **works** (asserted without a firmware) |
| the variable driver binding the flash rather than falling back to RAM | **works** — and it took the status register reading `0x00` after a Clear Status Register |
| a variable written in one run present in the next | **works** — `setvar` at the shell in one boot, read back at the shell in the next, across two machines sharing only the bank's bytes |
| `BootOrder`, `Boot000n`, `Timeout`, `ConIn`/`ConOut` in the store | **works** — 6,206 programmed bytes where the shipped image had 127 |
| an NVMe controller at `00:04.0`, enumerated and bound | **works** — `PciBusDxe` sizes and places its window and `NvmExpressDxe` enables memory space and bus mastering |
| the driver reading a register out of that window | **works** — and it took the fix above: a BAR programmed through ECAM used to never decode at all |
| a file system on that disk, `map` finding an `FS0:` | **works** — `FS0:` on `PciRoot(0x0)/Pci(0x4,0x0)/NVMe(0x1,…)`, and `startup.nsh` read off it and executed |
| an EFI application started off the disk | **works** — `BdsDxe: starting Boot0001 … from …/NVMe(0x1,…)`, which is `\EFI\BOOT\BOOTX64.EFI` on the volume rather than the shell in the firmware volume |
| a Linux kernel entered through its EFI stub | **works** — a Gentoo 6.6.67 `bzImage` and its initramfs read off the ESP by the firmware, `Run /init as init process`, and `uname -srm` answered at a shell |
| that kernel finding an APIC | **works** — `ACPI: Using ACPI (MADT) for SMP configuration information` and `APIC: Switch to symmetric I/O mode setup`, where it used to read `ACPI MADT or MP tables are not detected` and fall back to virtual wire mode |
| the timer that mode needs | **works** — and it took the HPET legacy replacement route: a kernel that learns the board has an HPET sets `LEG_RT_CNF` and stops loading the 8254, so a board without the multiplexer panics with `IO-APIC + timer doesn'''t work!` |
| the ACPI set the firmware installs | **works** — RSDP, XSDT, FADT, DSDT, FACS, MADT, MCFG and HPET, all generated from the realized machine and all placed by the firmware itself |
| the firmware's own `DEBUG()` log | **works** — `pc.debugcon` at `0x402` answers `PlatformDebugLibIoPortDetect` with `0xe9` and takes the log on its own character port, so a build with debug strings narrates the boot from the reset vector: SEC, every PEIM, the CMOS dump, `QemuFwCfgProbe`, the DXE dispatch, and `QemuFlashDetected => Yes`. A `RELEASE` image writes nothing there, and that is the image rather than the board |
| SMRAM / SMM | not modelled, **and not what was stopping the variable writes**; a non-`SMM_REQUIRE` OVMF never touches it, and [`q35.md`](q35.md) records the gap |
| `fw_cfg` | **works** — the selector/data pair at `0x510`, serving `etc/acpi/tables`, `etc/acpi/rsdp` and `etc/table-loader`. The DMA interface at `0x514` is deliberately absent and the feature bit says so, so the firmware takes the `rep insb` path |
| a boot device | **works** — an NVMe namespace with a FAT volume on it, found by BDS, mounted by the shell, and booted from |

## Sources

*Intel SDM* Vol. 3A §2.5 (`CR8`), §6.14.2 (the long-mode stack frame and its
sixteen-byte alignment), §9.1.4 (the reset state), §11.4.4 and §11.8.6 (the
task-priority register and its two names); Vol. 2B, `MOV`—*Move to/from Control
Registers* and `FXSAVE`; Vol. 4 Table 2-2 (`IA32_PLATFORM_ID`). *AMD64
Architecture Programmer's Manual* Vol. 2 §8.9.3 for the same frame alignment. Intel 3 Series Express Chipset Family Datasheet
(316966-002) and Intel I/O Controller Hub 9 Family Datasheet (316972-004) for
the chipset. JEDEC JESD68.01 and the Intel StrataFlash P30 datasheet for the
flash — **§14.1.1** for what the Clear Status Register command and a device
reset do to SR.7, which is the whole of why this board now keeps a variable. The UEFI Specification 2.10 and the PI Specification 1.8 for what a
firmware expects of a platform. EDK II itself — BSD-2-Clause-Patent, and
therefore a permitted *reference* under `CLAUDE.md` — for `OvmfPkg`'s flash
command sequence, `PlatformInitLib`'s chipset detection, and
`CpuExceptionHandlerLib`'s saved context, `BaseLib`'s `AsmReadMsr64`,
`QemuFlashFvbServicesRuntimeDxe`'s `QemuFlashDetected` for the four cycles that
decide whether the variable store is used at all, `VirtNorFlashDeviceLib` for
the status polling the RISC-V board's driver does with the same part, and
`VarCheckUefiLib` for why a variable under the global GUID has to be one the
specification names. For the debug console:
`OvmfPkg/Library/PlatformDebugLibIoPort/DebugIoPortQemu.c` for the detect and
its magic byte, `DebugLib.c` for the `IoWriteFifo8` that is the whole output
path, `DebugLibDetect.c` and `DebugLibDetectRom.c` for who caches the answer and
who cannot, and `OvmfPkg/OvmfPkg.dec` for `PcdDebugIoPort`'s default of `0x402`.
**That port is a QEMU-originated convention (and a Bochs one before it), and
neither source was opened**; what the device implements is what EDK II is
written to do to it.

For the disk: the **UEFI Specification 2.10** §3.5.1.1 for the default file
name a boot manager looks for on a device it has no `Boot####` for, §13.3 for
the EFI System Partition and its FAT requirements, and §7.4/§7.6 for
`LoadImage`/`StartImage`; the **UEFI Shell Specification 2.2** §3.1 for
`startup.nsh`; **NVM Express 1.4** §2.1.5 for the class code and §3.1.1 for
`CAP` and its `MPSMIN`; **PCI Local Bus Specification** revision 2.1 §6.2.5.1
for a 64-bit memory base address register and §6.2.2 for the Command register
bit that gates it; and the Intel 3 Series datasheet §5.1.16 for where in the
ECAM window a function's configuration space is. EDK II's
`MdeModulePkg/Bus/Pci/NvmExpressDxe` named its own assertion and is quoted
above; `MdePkg`'s `BaseDebugLibSerialPort` is why that assertion was formatted
and not printed.

For the tables and the channel they travel down: the **ACPI Specification**
revision 6.5 for every table (§5.2.5.3 the RSDP, §5.2.6 the description header
and its checksum, §5.2.8 the XSDT and what it does *not* list, §5.2.9 the FADT,
§5.2.10 the FACS and its 64-byte alignment, §5.2.12 the MADT, §6.2.13 `_PRT`);
the **IA-PC HPET Specification** revision 1.0a **§2.3.5** for the legacy
replacement route, which is the multiplexer this board grew in the same commit;
and EDK II for the interface itself —
`OvmfPkg/Include/IndustryStandard/QemuFwCfg.h` for the register addresses, the
numbered keys and the directory's big-endian encoding;
`OvmfPkg/Include/IndustryStandard/QemuLoader.h` for the three loader commands
and their 128-byte envelope;
`OvmfPkg/Library/AcpiPlatformLib/QemuFwCfgAcpi.c` for what the firmware
validates, in what order, and for `CalculateCheckSum8`'s consequence that a
checksum byte starts at zero; `OvmfPkg/Library/QemuFwCfgLib/QemuFwCfgLib.c` and
`QemuFwCfgDxe.c` for the directory walk, the signature probe and the DMA feature
bit; `OvmfPkg/Library/PlatformInitLib` for what each *missing* file falls back
to; and `OvmfPkg/OvmfPkgX64.fdf`, which is where the answer to "does the
firmware program the 8254?" is written down — it does not, it dispatches
`LocalApicTimerDxe`.

**`fw_cfg` is a QEMU-originated interface and QEMU's source is off limits under
`CLAUDE.md`; none of it was opened.** What is implemented is what the firmware
this board serves is written to consume, established entirely from EDK II, which
is permissively licensed and is the consumer whose expectations actually matter
here. One consequence is visible in the code: the four bytes at key `0x0000` are
`QEMU` because that is the identifier the driver binds on.

**No emulator source of any licence was consulted.** The firmware images were
run and never read; every number above is either a register this repository's
own devices hold or a byte the guest itself executed — including the assertion
text, which the guest wrote into its own stack and this repository read back
out of it.
