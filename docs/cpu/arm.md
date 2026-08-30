# ARM

Consumed by: `cpu/arm` (ARMv7-A), `cpu/aarch64` (ARMv8-A).

## Primary

| Source | Covers | Access |
| --- | --- | --- |
| Arm Architecture Reference Manual for A-profile (DDI 0487) | AArch64 and AArch32: instruction set, exception model, MMU/translation regimes, the memory model | `developer.arm.com/documentation/ddi0487/latest/` **[browser]** |
| Arm ARM for ARMv7-A/R (DDI 0406) | The ARMv7 architecture, for the 32-bit cores | `developer.arm.com/documentation/ddi0406/latest/` **[browser]** |
| GIC Architecture Specification (IHI 0069) | Generic Interrupt Controller v3/v4 — required by any modern ARM board | `developer.arm.com/documentation/ihi0069/latest/` **[browser]** |
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
- The A64 encoding is regular enough that a generated decoder from the manual's
  encoding tables is straightforward — unlike x86.
