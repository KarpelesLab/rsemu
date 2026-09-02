# ARM

Consumed by: `cpu/arm/aprofile` (ARMv5TE today, ARMv6/ARMv7-A later),
`cpu/arm/v7m` (ARMv7E-M, Cortex-M3/M4/M7), `cpu/arm/a64` (ARMv8-A AArch64).

## Primary

| Source | Covers | Access |
| --- | --- | --- |
| Arm Architecture Reference Manual for A-profile (DDI 0487) | AArch64 and AArch32: instruction set, exception model, MMU/translation regimes, the memory model | `developer.arm.com/documentation/ddi0487/latest/` **[browser]** |
| Arm ARM for ARMv7-A/R (DDI 0406) | The ARMv7 architecture, for the 32-bit cores | `developer.arm.com/documentation/ddi0406/latest/` **[browser]** |
| Arm ARM, ARMv5 and ARMv5TE (DDI 0100) | The **v5 architecture**: A32 and Thumb encodings, the seven modes, the exception model, and part B's CP15 register map, VMSAv5 translation table walk, domain model, access permissions and fault-status encodings | developer.arm.com **[browser]** |
| ARM926EJ-S TRM (DDI 0198) | The implementation-defined half of the above: the main ID and cache type register values, the c7 `test and clean` behaviour, the TCM status register, and the instruction cycle timings | developer.arm.com **[browser]** |
| GIC Architecture Specification (IHI 0069) | Generic Interrupt Controller v3/v4 — required by any modern ARM board | `developer.arm.com/documentation/ihi0069/latest/` **[browser]** |
| Arm ARM for ARMv7-M (DDI 0403) | The **M profile**: T32 only, Handler/Thread modes, the exception model and `EXC_RETURN`, the NVIC/SCB/SysTick/MPU register map at `0xE000E000`, PMSAv7 | `developer.arm.com/documentation/ddi0403/latest/` **[browser]** |
| Cortex-M4 TRM (DDI 0439), Cortex-M7 TRM (DDI 0489) | The implementation-defined values a guest can see: `CPUID`, how many priority bits, how many MPU regions, instruction timings | developer.arm.com **[browser]** |
| PrimeCell UART (PL011), TRM DDI 0183 | The UART every ARM virtual board exposes | developer.arm.com **[browser]** |

Arm's site blocks automated fetches but the documents are free to download after
a no-cost account. They are the only authoritative source; there is no good
secondary reference for the exception model.

## Implementation notes

- **The memory model is the reason ARM is hard for us**, not the instruction
  set. AArch64 is weakly ordered; emulating a TSO guest (x86) on it, or emulating
  it on a TSO host, both require deliberate barrier placement. DDI 0487's memory
  model chapter plus the Cambridge material in
  [`../techniques/memory-models.md`](../techniques/memory-models.md) are the
  sources.
- Exception levels, the translation regimes (EL0/1/2/3, stage 1 and stage 2),
  and `TTBR0`/`TTBR1` splitting all need modelling before Linux boots.
- **VMSAv5 is a much smaller machine than VMSAv6, and the difference is worth
  keeping in view.** v5 has one `TTBR`, no ASIDs, no supersections, no TEX
  remap and no execute-never bit, so a context switch is a full TLB flush and
  every permission decision is two `AP` bits against c1's `S` and `R`. v6
  changes the descriptor format itself, which is why `cp15.rs` is written for
  v5 rather than "for ARM" — an `Arch` construction property selects the walk,
  and a second walk goes beside the first rather than inside it.
- **The M profile is a different architecture, not a subset.** There is no ARM
  state, so no A32 decoder; the exception model is hardware register stacking
  with `EXC_RETURN` rather than banked modes; the interrupt controller is *inside
  the core* rather than a GIC beside it; and protection is PMSAv7 regions rather
  than a translation table. `cpu/arm/v7m` shares the `arm` family module with the
  A-profile core and nothing else. Its board is
  [`../platforms/stm32f407.md`](../platforms/stm32f407.md), which also records
  how a peripheral raises an interrupt when the NVIC has no wire of its own.
- The A64 encoding is regular enough that a generated decoder from the manual's
  encoding tables is straightforward — unlike x86. `cpu/arm/a64/isa.rs` is that
  decoder: one `(mask, bits)` row per instruction, bucketed at compile time on
  the top-level `op0` field (bits 28:25) that DDI 0487 C4.1 classifies on.
- **AArch64 is a third core beside `aprofile` and `v7m`, not a variant of
  either**, which is the boundary `ROADMAP.md` §6.1.1 anticipated and declined
  to draw until it was needed. It shares neither the register file (31 plus two
  *encodings* rather than 16 with the PC among them), the status word (`PSTATE`
  is fields with no register; `CPSR` is a register), the system-register space
  (a flat `op0:op1:CRn:CRm:op2` rather than CP15), the exception model
  (`ELR_EL1`/`SPSR_EL1`/`ESR_EL1` and sixteen vector slots rather than banked
  modes and eight vector *instructions*), nor the MMU. The lattice still applies
  *within* it: `FEAT_LSE` and `FEAT_CRC32` are per-instance flags a named part
  selects, and an absent one does not decode.
- **The `SP`/`XZR` distinction is a property of the encoding, not of the
  register number**, and it is the single easiest thing to get wrong: register
  31 is the stack pointer in the base register of every load and store and in
  `ADD`/`SUB` immediate and extended-register forms, and the zero register
  everywhere else — including in `ADDS`, whose destination differs from `ADD`'s
  for exactly this reason (DDI 0487 C1.2.5). In `cpu/arm/a64` it lives on the
  operand *format*, so the interpreter and the disassembler cannot disagree
  about it.
- **`TG0` and `TG1` spell the 4 KiB granule differently** — `0b00` and `0b10`
  respectively — and so do `TGran4` and `TGran16` in `ID_AA64MMFR0_EL1`, where
  `0b0000` means *supported* in one field and *not supported* in the other. A
  walker that works on the low half of the address space and not the high one
  has usually tripped on the first of those.
