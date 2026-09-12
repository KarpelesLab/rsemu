# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.6](https://github.com/KarpelesLab/rsemu/compare/v0.0.5...v0.0.6) - 2026-09-12

### Added

- *(dev-stm32-i2c)* [**breaking**] slave mode on both links, and v2's `SBC`
- *(bus-i2c)* one pin pair carries both roles, so a controller can be addressed
- *(dev-psram-qspi)* an APS6404L-class QSPI pseudo-static RAM
- *(dev-stm32-octospi)* [**breaking**] pass CCR's widths through, and count the chip select low
- *(bus-spi)* carry the line width of a phase with the word
- *(dev-st77xx)* the ST7789/ST7735, whose commands are addressed rather than counted
- *(dev-ssd1306)* the SSD1306/SH1106, and the seam a device-owned framebuffer needs
- *(dev-dfuse)* DfuSe (.dfu, UM0391) images, each element at its own address
- *(dev-sdio)* the SDIO answers as a DMA flow controller
- *(machine)* wire the F407's DMA request matrix from RM0090 Tables 43/44
- *(dev-dmamux)* the STM32 DMA request multiplexer
- *(dev-dma)* [**breaking**] CHSEL gates the request, and PFCTRL ends the transfer
- *(wire)* let a DMA peripheral say which item is its last
- *(dev-keypad)* a GPIO matrix keypad, and the stm32f407 gets one on PE0-PE7
- *(wire)* [**breaking**] a third driver state, and a net that resolves itself
- *(dev-stm32)* the HASH processor, context swap included
- *(dev-stm32-firewall)* [**breaking**] the interrupt case, and the call gate is three words
- *(cli)* `--spin-detect`, so a hung board says why
- *(cpu-riscv)* wire the hart interpreter to the spin detector
- *(cpu-arm)* wire the ARMv7-M interpreter to the spin detector
- *(spin)* catch a processor stuck on a load whose value never changes
- *(dev-stm32-dbgmcu)* st.dbgmcu, with a real output pin per freeze bit
- *(cpu-arm)* [**breaking**] the external PPB's vendor window belongs to the board
- *(device)* a debugger's halt reaches a device through `Device::debug_halt`
- *(dev-stm32-firewall)* the STM32L4 Firewall, and FWDIS becomes load-bearing
- *(dev-stm32-exti)* [**breaking**] SYSCFG.MEMRMP moves what answers at address zero
- *(machine)* chain the F407's timers through the internal trigger matrix
- *(dev-stm32-tim)* [**breaking**] input capture, the slave-mode controller and the DMA burst window
- *(dev-i2c-v2)* st.i2c-v2, the block every STM32 since the F0 carries
- *(machine)* the SDIO on the F407, and the first request line anything drives
- *(dev-sdio)* the F2/F4/F7 SDIO, as a sibling of the H7 SDMMC rather than a variant of it
- *(dev-stm32)* [**breaking**] the F0/F3/F7/L4/G4/WB SPI, as a variant with its FIFO
- *(dev-stm32-rng)* the STM32 RNG, from a stream the machine seeds
- *(dev-rcc)* st.rcc drives sysclk, hclk, pclk1, pclk2 and the timer clocks
- *(clock)* a device can re-rate a clock domain, at a scheduling boundary
- *(machine)* the F407 carries its flash interface and its real-time clock
- *(dev-rtc)* the STM32 real-time clock, its BCD calendar and its backup domain
- *(dev-stm32)* an `st.flash` that owns the array it programs
- *(cpu-arm-v7m)* [**breaking**] the DWT cycle counter, and the bit-band alias windows
- *(gdb)* qXfer:memory-map:read, built from the machine's own address space
- *(gdb)* [**breaking**] a target description is a list of features, and ARMv7-M has two
- *(machine)* the F407's nine peripherals, at their Table 62 vectors and behind a bus matrix

### Fixed

- *(machine)* hand a sink the drivers wired to it, not its whole net
- *(dev-stm32)* OTYPER and PUPDR decide what a pad presents, and IDR reads the pin
- *(dev-stm32)* a feature has to compile its own tests, alone
- *(machine)* the F407's two watchdog resets stop sharing one net
- *(machine)* register st.tim in a build that enables only dev-stm32-tim
- *(cpu-x86)* the inlined-path tests need a host that generates code
- *(cpu-riscv)* gate `has_shadow` on what its callers are gated on
- *(sched)* a share that rounds to nothing still gets one tick
- *(sched)* [**breaking**] a budget is a share of the round, not ten thousand ticks

### Other

- *(dev-flash)* the link text already resolves
- *(dev-stm32-hash)* a board for the hash processor, not a part that lacks one
- *(dev-sdio)* prove PFCTRL against a real card, both ways
- bound fstool below the release that demands a filesystem feature
- *(spin)* the detector on a whole board, and what it costs
- *(machine)* the F407's watchdog reset path, end to end
- ignore the per-agent CARGO_TARGET_DIR
- *(dev-sdio)* a guest moves blocks over DMA2 without ever reading the FIFO
- *(machine)* the F407's flash guarantee is no silent modification, not a fault
- *(dev-stm32)* the two flash bases, and that nothing writes the array back
- *(dev-stm32)* link the flash module header at items rustdoc can reach
- *(dev-stm32)* say which SNB field width the F4 flash decodes
- declare the `dev-stm32-flash` feature
- *(gdb-v7m)* the four M-profile behaviours a debugger gets wrong
- Merge branch 'worktree-agent-a2b16c84daee86067'
- Merge branch 'worktree-agent-a4af39317f2a25f0b'
- *(sched)* what the budget change measured, end to end

## [0.0.5](https://github.com/KarpelesLab/rsemu/compare/v0.0.4...v0.0.5) - 2026-09-11

### Added

- *(jit)* a tick budget a block can leave at an instruction boundary
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

- *(trace)* the interning test's key has to fit a 32-bit `usize`
- *(machine)* the machine-file fuzz target learns `threading`
- *(cpu-riscv)* the hart publishes a live position, so a timer read is answered at the instant of the read
- *(sync)* [**breaking**] threaded wasm gets the threaded backend, and it has to say so
- *(space)* a straddling write takes the write side's byte order too
- *(cpu-riscv,cpu-x86)* a load can raise the interrupt wire inside a block
- *(cpu-arm-a64)* the timer edge is computed after a walk that can cross it
- *(cpu-arm-a64)* gate timer_edge with its only caller
- *(cpu-arm-a64)* a host fence for the acquire/release accesses
- *(cpu-arm-a64)* two engine divergences an arm64 Linux boot found past twenty seconds
- *(cpu-arm-a64)* gate translate_fetch on the engine that calls it
- *(host)* make Ctrl-C end a run through finish rather than around it
- *(cpu-arm-a64)* the bucket-margin report needs std, which the feature does not imply
- *(space)* host_ptr must skip the alignment slack, like every other accessor

### Other

- *(jit)* the chaining helpers need the gate their only callers have
- Merge branch 'worktree-agent-a4371c5913d9b0140'
- *(cpu-x86)* [**breaking**] the host guard, so a store no longer ends a paged block
- *(cpu-x86)* a real x86-64 Linux boot, with the census a profile needs
- spell the coverage distinction as `cfg!`, which cannot be dead code
- the coverage constant needs the gate its users have, and signals do not queue
- coverage fractions are a property of the host backend, named once
- *(cpu-x86)* liveness belongs to the corpus, not to one seeded program
- the aarch64 preflight lost a race with itself, and crosshost needs node 24
- *(cpu-x86)* the compiled-coverage floors belong to the backend, not the corpus
- the four failing jobs, three fixed and one written down
- Merge branch 'worktree-agent-a08f7014a7e304730'
- *(cpu-arm-a64)* [**breaking**] a store stops ending a block, because the host can see the page it landed on
- *(cpu-riscv,cpu-x86)* the slot-read hoist reaches the other two frontends
- Merge branch 'worktree-agent-aff679f8302dc0623'
- Merge branch 'worktree-agent-a3db4279c35f6a110'
- Merge branch 'worktree-agent-ab7c55788acc5978c'
- *(wasm)* run the suite on wasm32-wasip1-threads, and say what it found
- Merge branch 'worktree-agent-aaef38d9f719e1cad'
- the AArch64 leg exists now, and two comments still said it did not
- Merge branch 'worktree-agent-a4967afa4517f22c0'
- Merge branch 'worktree-agent-a04b4f34d80f406fb'
- *(cpu-arm-a64)* the long run stops being something somebody does by hand
- *(cpu-arm-a64)* ISB is 31% of the barrier stream, not 40%
- Merge branch 'worktree-agent-ab6745a7cf3aefcef'
- *(cpu-x86)* MAX_INSNS goes to thirty-two now the upper arm is gone
- Merge branch 'worktree-agent-a33c56adcaaf711b9'
- *(jit-x86)* the temporary frame stops being cleared, and the U loses its right arm
- the tick allowance takes both cores past 99% retired in blocks
- Merge branch 'worktree-agent-a8247fd29b1a326ab'
- *(cpu-x86)* [**breaking**] 97.3% to 99.3% of a Linux boot inside a translated block
- *(cpu-riscv)* a block leaves when the hart's ticks run out
- Merge branch 'worktree-agent-a9195ba74c4fd5d79'
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
