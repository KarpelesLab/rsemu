# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
