# ARM

Consumed by: `cpu/arm/aprofile` (ARMv5TE today, ARMv6/ARMv7-A later),
`cpu/arm/v7m` (ARMv7E-M, Cortex-M3/M4/M7), `cpu/aarch64` (ARMv8-A).

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
  encoding tables is straightforward — unlike x86.
