# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.5](https://github.com/KarpelesLab/rsemu/compare/v0.0.4...v0.0.5) - 2026-09-05

### Added

- *(space)* [**breaking**] a global exclusive monitor, so a sibling's store breaks the reservation
- *(machine)* the JIT engines are reachable from the command line
- *(cpu-arm-a64)* [**breaking**] an IR frontend, and the engine that runs its blocks
- *(accel)* [**breaking**] a Linux kernel boots to userspace on host silicon
- *(cpu-x86)* [**breaking**] lift paged code, named by what the entry resolved to
- *(cpu-riscv)* engine = "jit", so a machine can reach the dispatcher
- *(ir)* linear-scan register allocation, and what it costs precise state
- *(jit)* an x86-64 host backend, with the software TLB inlined into it
- *(cpu-x86)* lift 32-bit protected mode into the translation IR

### Fixed

- *(cpu-arm-a64)* gate translate_fetch on the engine that calls it
- *(host)* make Ctrl-C end a run through finish rather than around it
- *(cpu-arm-a64)* the bucket-margin report needs std, which the feature does not imply
- *(space)* host_ptr must skip the alignment slack, like every other accessor

### Other

- Merge branch 'worktree-agent-a624808cd91c98a0a'
- *(deps)* fstool 0.4.26 takes the tree from 23 third-party crates to 10
- Merge branch 'worktree-agent-abe5a7278a36b654f'
- *(cpu-arm-a64)* [**breaking**] the budget guard lifts a cold PC instead of guessing at it
- Merge branch 'worktree-agent-a0127d022851e35b4'
- *(cpu-arm-a64)* the software TLB's fast path, inlined into compiled code
- *(deps)* fstool 0.4.25 drops ten third-party crates, tinyvec among them
- Merge branch 'worktree-agent-a257daf5348ddcd77'
- Merge branch 'worktree-agent-a28a9febe7887be47'
- Merge branch 'worktree-agent-ab6c06f2766f3462a'
- Merge branch 'worktree-agent-ad90767a5f7773fb4'
- Merge branch 'worktree-agent-a6f1add36f0b152b3'
- *(accel)* one page walk per slice, and the measured numbers
- *(accel)* what the two engines agree about, measured line for line
- *(cpu-riscv)* [**breaking**] a paged hart publishes a LoadPlan, so a compiled load needs no call
- Merge branch 'worktree-agent-a7f9c932c88907426'
- the unsafe ceiling is seven, and the seventh is the signal disposition
- Merge branch 'worktree-agent-ac79776f77a85412a'
- Merge branch 'worktree-agent-a176e68c47228bf60'
- Merge branch 'worktree-agent-ad298bafa1bf75018'
- Merge branch 'worktree-agent-a582c461314190650'
- Merge branch 'worktree-agent-a8c31f0557c91d5ec'
- *(machine)* q35-linux reads off its disk now
- Merge branch 'worktree-agent-a6237079568edb1f3'
- Merge branch 'worktree-agent-a5ed51bcf433288db'
- *(jit)* [**breaking**] chain blocks on a real guest, and stop paying 144 µs to compile one
- Merge branch 'worktree-agent-a68879c0e859b6804'
- Merge branch 'worktree-agent-aa79687f9f1821bf3'
- Merge branch 'worktree-agent-af4af447a0627593e'
- *(jit)* the guard's compiled numbers, measured at the table's own settings
- Merge branch 'worktree-agent-af09d4e27f0f0fa04'

## [0.0.4](https://github.com/KarpelesLab/rsemu/compare/v0.0.3...v0.0.4) - 2026-09-02

### Added

- *(cpu-x86)* [**breaking**] INIT, Start-Up, and the registers that name state outside the core
- *(cpu-x86)* [**breaking**] the x87 unit and SSE2, computed in software
- *(dev-blk)* a guest drive that is a host file, through fstool
- *(dev-pc-pmc)* a PCI host bridge with PAM, and the RAM shadowing that boots a PC
- *(dev-ata-disk)* an ATA hard disk and the AT's IDE interface

### Fixed

- *(cpu-x86)* merge only the wide registers the gdb prefix carries
- *(bus-pci)* [**breaking**] rank the routing table below BUS, where a config cycle can reach it
- *(cli)* restore the allow attribute a merge dropped from install_capture
- *(cpu-x86)* walk the page tables for a debug read, without touching them
- *(dev-pc)* [**breaking**] A20 is open at power-on, and pc-at is a 486

### Other

- Merge branch 'worktree-agent-a6b085084f72350dd'
- say what the PC board actually does now
- Merge branch 'worktree-agent-abcfcd255e8dada68'
- Merge branch 'worktree-agent-a40927053483c6041'
- Merge branch 'worktree-agent-acb56bd1a51838819'
- Merge branch 'worktree-agent-ab024847a2545b730'
- Merge branch 'worktree-agent-ab3cfa107e5d0cb21'
- Merge branch 'worktree-agent-aef8641698ffb4954'
- Merge branch 'worktree-agent-ac9c594747acefa7b'
- Merge branch 'worktree-agent-aa184cec409a4fa98'
- Merge branch 'worktree-agent-a55dd47cda3540d97'
- Merge branch 'worktree-agent-ab452dfa9e18957c6'
- *(dev-blk)* take fstool 0.4.24, which moves the CLI's deps behind a feature
- Merge branch 'worktree-agent-a519d1c66177ceb2b'
- Merge branch 'worktree-agent-a35cb94fe239ee459'
- Merge branch 'master' into worktree-agent-abbfbfe4d360b2f08
- Merge branch 'worktree-agent-a82ff4501bdb741e1'

## [0.0.3](https://github.com/KarpelesLab/rsemu/compare/v0.0.2...v0.0.3) - 2026-09-01

### Added

- *(dev-stm32)* SPI, OCTOSPI and I2C, with a SPI-NOR flash and an EEPROM to talk to
- *(dev-spinor)* a W25Q-class SPI-NOR flash on the SPI bus
- *(space)* [**breaking**] permissions on a mapping, and a flatten per guard rather than per map
- *(space)* [**breaking**] a mapping layer — what answers, and on what terms
- *(bus-usb)* a USB fabric, a generic EHCI, and the ChipIdea/ARC variant
- *(dev-sms)* the Sega Master System, and phase 4's second genericity proof
- *(host)* an audio seam, WAV capture, and NES sound in the browser
- *(exec)* the level-3 seam — a core that exits at a syscall

### Fixed

- *(machine)* iterate every workload, and correct a generalisation I got wrong
- *(dev-gb)* mooneye acceptance 22 of 66 to 59 of 66, blargg to 12 of 12
- *(machine)* run_for_additive must not panic in a build with no machines

### Other

- describe the no_std gate as it is actually built
- *(cpu-riscv)* riscv-arch-test at 181 of 181, without RISCOF
- *(machine)* measure whether run_for is additive — it is not
- *(machine)* a headless frame-hash regression and a frame-time benchmark
- smoke-run the frame-time benchmark harness
- *(machine)* a headless frame-hash regression and a frame-time benchmark

## [0.0.2](https://github.com/KarpelesLab/rsemu/compare/v0.0.1...v0.0.2) - 2026-08-31

### Added

- *(dev-ppu)* AccuracyCoin 141 of 141 — the ledger is empty
- *(space)* one address, two devices — `Region::split` and the DSL's `split()`
- *(dev-apu)* the DMC's $4015 enable latch, three CPU cycles wide
- *(dev-ppu)* [**breaking**] the 2C02's own address bus — two-dot accesses and the octal latch
- *(dev-ppu)* AccuracyCoin 85/141 to 130/141, and a cycle-exact /RDY
- *(dev-nes)* the aborted DMC fetch, which costs one cycle and does nothing
- *(dev-ppu)* sprite output units are counters and shifters, not a comparison
- *(space)* a master's own data bus is not the one on its pins
- *(dev-ppu)* the OAM read bus, a delayed $2001, and the shifters' serial input
- *(dev-nes)* the RP2A03 DMA unit drives /RDY, a cycle at a time
- *(space)* [**breaking**] an open-bus policy, and a sub-quantum position for lazy catch-up
- *(cpu-riscv)* let the `time` CSR read an attached platform timer
- *(dev-riscv)* the RISC-V virt board, with a generated device tree
- *(dev-riscv)* the RISC-V `virt` board, with a generated device tree

### Fixed

- *(machine)* let the pc-at and Game Boy media fixtures build without `std`
- *(dev-nes)* put every process-wide table on `core::sync::Global`
- *(sync)* make the `single` backend's locks exclude for real
- *(machine-nes)* imply dev-nes-ppu, without which nes-ntsc never realizes
- *(dev-nes)* the 2A03's register block answers on two buses, and only when the core selects it
- *(dev-nes)* [**breaking**] a DMC fetch's no-op cycles overlap the sprite copy's accesses
- *(dev-apu)* the memory reader settles late, and two fetches cannot be back to back
- *(dev-ppu)* the /NMI output register is two dots deep, not one
- *(dev-ppu)* the sprite counters outlive forced blank, and the shifter load is registered
- *(dev-ppu)* [**breaking**] the OAM read line is registered, and step 3a reads rather than checks
- *(machine)* silence dead-code under narrow feature sets, so the sweep is green
- *(dev-ppu)* $2002's three flags are not sampled at the same instant
- *(dev-nes)* a DMA bus conflict merges floating bits, it does not wire-AND them
- *(dev-ppu)* OAM corruption is a handover, not a special case of OAMADDR
- *(dev-nes)* the controllers' latch line is only driven on put cycles
- *(cpu-6502)* the vector is latched on cycle five, and a held cycle polls once
- *(apu)* the frame IRQ flag is not the IRQ line, and $4015 only arms its clear
- *(cpu-6502)* SH* drops the high-byte AND when /RDY falls over the fix-up
- *(dev-nes)* a DMA cycle drives the data bus, and only a real responder conflicts
- *(dev-ppu)* put /NMI on the dot it happens, and clock the CPU's edge detector

### Other

- *(roadmap)* nixvm depends on rsemu rather than merging into it
- *(roadmap)* three levels of execution, and the nixvm merge
- run each feature's tests, not just its compile
- *(conformance)* bind the 6502 seam and make a vacuous skip impossible
- *(device)* [**breaking**] merge the three device-handle mechanisms into one
- record the AccuracyCoin number, and rewrite the ledger around what is left
- *(machine)* regenerate the AccuracyCoin table from the pinned ROM

## [0.0.1](https://github.com/KarpelesLab/rsemu/compare/v0.0.0...v0.0.1) - 2026-08-31

### Added

- *(dev-wdc)* Ben Eater's 6502 board, running the 1976 Woz Monitor
- *(cpu-m68k)* MC68000 core with the prefetch queue and exception frames
- *(cpu-riscv)* an RV64GC interpreter with M/S/U, Sv39 and software IEEE-754
- *(cpu-x86)* 8086/8088 core passing 98.9% of the hardware-generated suite
- *(cpu-x86)* 8086/8088 core passing the hardware-generated single-step suite

### Fixed

- *(cpu-arm)* map the corpus correctly, and fix the twelve bugs it then found

### Other

- permit oxideav-png for framebuffer capture
- bring the status in line with what exists
- state Eater's CC-BY licence plainly, without the hedge
- Ben Eater's Wozmon port is CC-BY, not unlicensed
- *(apple1)* use the manual as a primary source, and settle the DSP polarity
- *(apple1)* record the Woz Monitor as public domain, with the evidence
- Merge PAL and Dendy region variants for the PPU and APU
- badges, matching purecrypto
