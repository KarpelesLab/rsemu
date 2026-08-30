# RISC-V

Consumed by: `cpu/riscv`, the `virt` board, and the phase 6 IR/JIT work.

RISC-V is the best-documented architecture rsemu targets and the whole
specification set is **freely and legitimately downloadable** — which is part of
why it is the first architecture to get the IR/JIT treatment.

## Primary

| Source | Covers | Licence |
| --- | --- | --- |
| [RISC-V specifications](https://riscv.org/technical/specifications/) | Official landing page for the ratified documents | — |
| [riscv-isa-manual](https://github.com/riscv/riscv-isa-manual) | Volume 1 (unprivileged: RV32I/RV64I, M, A, F, D, C, and the rest) and Volume 2 (privileged: machine/supervisor/user modes, CSRs, Sv39/Sv48 paging, PMP) | **CC-BY-4.0** — quotable with attribution |
| [riscv-sbi-doc](https://github.com/riscv-non-isa/riscv-sbi-doc) | Supervisor Binary Interface: the ecall ABI between S-mode and M-mode firmware | |

Volume 2 is the one that matters for booting Linux: privilege transitions, trap
delegation (`medeleg`/`mideleg`), the CSR map, and Sv39 page-table walks.

## Firmware

| Source | Notes |
| --- | --- |
| [OpenSBI](https://github.com/riscv-software-src/opensbi) | **BSD-2-Clause** (verified) — permissively licensed, so it may be read *and* used. The reference M-mode firmware; the `virt` board boots through it |

## Memory model

RVWMO (RISC-V Weak Memory Ordering) is specified in Volume 1, Chapter 14. It
matters for parallel translated execution: see
[`../techniques/memory-models.md`](../techniques/memory-models.md) for how
barrier lowering is assigned to the frontend lifter.

## Validation

`riscv-tests` (BSD-3, UC Regents — verified), `riscv-arch-test`, and RISCOF.
See [`../testing/conformance-suites.md`](../testing/conformance-suites.md).
