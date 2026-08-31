# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
