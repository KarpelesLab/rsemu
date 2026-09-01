# MIPS I / R3000A

Sources for `src/cpu/mips` — an LSI/MIPS R3000A-compatible 32-bit interpreter.
Everything here is hardware documentation or a permissively-licensed test
corpus; the exclusions at the bottom are the part that matters most.

## Primary documentation

| Document | What it settles | Where |
| --- | --- | --- |
| **IDT R3051/R3052/R3081 Family Hardware User's Manual** | The best single description of the R3000A system control coprocessor: the CP0 register set, the three-deep `KU`/`IE` stack, the exception model and its vectors, the 64-entry TLB, and the segment map | [bitsavers](https://bitsavers.org/) and its [Internet Archive mirror](https://archive.org/details/bitsavers) |
| **Kane & Heinrich, *MIPS RISC Architecture*** (Prentice Hall) | The MIPS I instruction set, the encoding tables in the appendix, the delay-slot rules, and the `LWL`/`LWR`/`SWL`/`SWR` byte tables | print; scans on the Internet Archive |
| **LSI Logic LR33300/LR33310 datasheet** | What the PlayStation's part has and has not — no TLB, 1 KiB of data cache, 4 KiB of instruction cache | bitsavers |
| **MIPS32 Architecture for Programmers, Vol I–III** | Current, free, and fine for the MIPS I *instruction* subset | mips.com / Imagination |

## MIPS32 is the wrong manual for coprocessor 0

This is the single most common way to get an R3000 wrong, so it is worth
stating separately: **MIPS32 documents the R4000-style CP0**, which is a
different chip. The differences are not cosmetic.

| | R3000 | R4000 / MIPS32 |
| --- | --- | --- |
| mode and interrupt enable | a three-deep stack of `KU`/`IE` pairs in `Status[5:0]`, pushed on an exception and popped by `RFE` | `Status.EXL` / `Status.ERL`, and `ERET` |
| nesting protection | none — a third exception loses the outermost saved state | `EXL` suppresses re-entry |
| TLB entry | one `EntryLo`, mapping one page | `EntryLo0`/`EntryLo1`, mapping a page *pair* |
| `Wired` | **does not exist**; the boundary `TLBWR` will not cross is hard-wired at 8 | a writable register |
| `Context` | 11-bit `PTEBase`, 19-bit `BadVPN` (`VA[30:12]`) | `BadVPN2`, a different split |
| exception vectors | `0x8000_0000` / `0x8000_0080`, or `0xBFC0_0100` / `0xBFC0_0180` with `BEV` | a third vector at `…0200` |
| page size | fixed at 4 KiB | a `PageMask` register |

Use MIPS32 for what `ADDIU` does. Check an R3000-era source before trusting it
on anything with `Status`, `Cause` or the TLB in it.

## Community documentation

- The **nocash PSX specification** (`psx-spx`, and the community fork at
  <https://psx-spx.consoledev.net/>) is *hardware* documentation rather than an
  emulator's source, so it is usable — but check its licence line yourself
  before relying on it, per CLAUDE.md. It is the best description of the
  LR33300's peripheral map, and of the published PlayStation memory map that
  shows `kuseg` is unmapped on that part (RAM answers at `0x0000_0000`,
  `0x8000_0000` and `0xA000_0000` alike).
- [Ken Shirriff's blog](https://www.righto.com/) for die-level analyses.

## Test corpora

| Suite | Licence | What it is |
| --- | --- | --- |
| [`SingleStepTests/r3000`](https://github.com/SingleStepTests/r3000) | **MIT**, © 2025 SingleStepTests (verified against the upstream `LICENSE`) | 56 files, 1000 single-instruction vectors each, covering the MIPS I user-level integer set. Wired up in `src/cpu/mips/conformance.rs`; fetched by `scripts/fetch-testdata.sh mips-r3000` |
| [`JaCzekanski/ps1-tests`](https://github.com/JaCzekanski/ps1-tests) | **MIT**, © 2020 Jakub Czekański | PS-EXE binaries: memory-map timing, coprocessor probing, GTE. Needs a whole PlayStation, so it belongs to a future `machine-psx` rather than to this core. Prebuilt archives on each release |
| [`lemmy-64/n64-systemtest`](https://github.com/lemmy-64/n64-systemtest) | **MIT** | Self-checking, and unusually good on CP0, exceptions and the TLB — but VR4300 (MIPS III) on N64 hardware. Its MIPS I-subset tests could be *ported* with attribution; it cannot be run as-is |
| [`PeterLemon/N64`](https://github.com/PeterLemon/N64) `CPUTest/` | **Unlicense** (public domain) | Bare-metal MIPS III assembly; reports to a framebuffer rather than machine-readably |
| **amidog** `psxtest_cpu.exe` etc. | site content is **CC BY-NC-SA 3.0** | Closed-source prebuilt PS-EXEs. The non-commercial clause makes them **fetch-only, never vendored, never a hosted CI fixture**. Running one as an emulated guest is ordinary use |

**There is no `riscv-tests` equivalent for MIPS.** MIPS Technologies never
published a conformance suite, and no community analogue exists. Do not spend
time looking for one; the gap — CP0, exceptions, the TLB, and the GTE, none of
which the single-step corpus covers — is ours to fill with directed tests
written from the manual.

Two repositories with **no `LICENSE` file at all** turned up in the same
search: `project64/N64-Tests` and `Dillonb/n64-tests`. Treat them as
all-rights-reserved.

## Building small MIPS programs, with nothing vendored

Verified on a stock machine with no cross toolchain installed:

- **clang + `rust-lld`.** `clang --target=mipsel-unknown-none -O2 -ffreestanding
  -c t.c -o t.o` works out of the box — MIPS is an always-built LLVM backend.
  The clang driver cannot find a linker, so invoke LLD directly: symlink the
  `rust-lld` in a rustup toolchain as `ld.lld` and run
  `ld.lld -flavor gnu -e _start t.o -o t.elf`.
- **Nightly Rust + `build-std`.** A `#![no_std] #![no_main]` crate built with
  `cargo +nightly build -Zbuild-std=core --target mipsel-unknown-none` needs no
  target install and no external linker.
- `rustc --print target-list` also has **`mipsel-sony-psx`** (Tier 3), which
  emits ELF by default and PS-EXE through `cargo-psx`.
- Debian and Ubuntu package `gcc-mipsel-linux-gnu` as a first-class package;
  Arch has only an AUR build. `zig cc -target mipsel-linux-musl` works where
  zig is installed.

## Deliberately excluded

- **QEMU's `target/mips`** — GPLv2, and permanently off limits in its entirety
  (`ROADMAP.md` §1).
- **`arch/mips/` in the Linux kernel** — GPLv2. It is what a web search for
  "R3000 TLB refill" or "MIPS exception vector" surfaces first, so be
  deliberate about what you open.
- **mednafen, PCSX-Redux, pcsxr, DuckStation, MAME** — all copyleft.
- **ares** is ISC and therefore *would* be readable, but nothing about this
  core required it and its source was not opened. Only its output was
  consumed, through the MIT-relicensed corpus above.
