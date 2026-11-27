# ARM

Consumed by: `cpu/arm/aprofile` (ARMv5TE, and ARMv7-A's A32 and T32 (Thumb-2)
instruction sets with VFPv3; NEON to follow),
`cpu/arm/v7m` (ARMv6-M, ARMv7E-M and Armv8-M Mainline, Cortex-M0/M0+/SC000/M3/SC300/M4/M7/M33/M35P), `cpu/arm/a64` (ARMv8-A AArch64).

## Primary

| Source | Covers | Access |
| --- | --- | --- |
| Arm Architecture Reference Manual for A-profile (DDI 0487) | AArch64 and AArch32: instruction set, exception model, MMU/translation regimes, the memory model | `developer.arm.com/documentation/ddi0487/latest/` **[browser]** |
| Arm ARM for ARMv7-A/R (DDI 0406) | The ARMv7 architecture, for the 32-bit cores; part B3 (VMSA) and B4 (the CP15 registers) for `cp15v7.rs` |
| Cortex-A9 TRM (DDI 0388), Cortex-A9 MPCore TRM (DDI 0407) | The A9's identification values, `SCTLR` reset value, cache geometry, `ACTLR`, the power control register, and `CBAR`/`PERIPHBASE` | `developer.arm.com/documentation/ddi0406/latest/` **[browser]** |
| Arm ARM, ARMv5 and ARMv5TE (DDI 0100) | The **v5 architecture**: A32 and Thumb encodings, the seven modes, the exception model, and part B's CP15 register map, VMSAv5 translation table walk, domain model, access permissions and fault-status encodings | developer.arm.com **[browser]** |
| ARM926EJ-S TRM (DDI 0198) | The implementation-defined half of the above: the main ID and cache type register values, the c7 `test and clean` behaviour, the TCM status register, and the instruction cycle timings | developer.arm.com **[browser]** |
| GIC Architecture Specification (IHI 0069) | Generic Interrupt Controller v3/v4 — required by any modern ARM board | `developer.arm.com/documentation/ihi0069/latest/` **[browser]** |
| Arm ARM for ARMv7-M (DDI 0403) | The **M profile**: T32 only, Handler/Thread modes, the exception model and `EXC_RETURN`, the NVIC/SCB/SysTick/MPU register map at `0xE000E000`, PMSAv7, and the **FPv4-SP/FPv5 extension** — A2.5 for `FPSCR` and the data types, A6.4/A7.5 for the encodings, A7.7 for the per-instruction pseudocode, B1.5.7 for `FPCCR`/`FPCAR`/`FPDSCR` and lazy state preservation | `developer.arm.com/documentation/ddi0403/latest/` **[browser]** |
| Armv8-M ARM (DDI 0553) | The **Cortex-M33's architecture**: the stack-limit checks (`MSPLIM`/`PSPLIM`, `UFSR.STKOF`), the v8-M `EXC_RETURN`, the load-acquire/store-release family, `TT`, PMSAv8 (base/limit regions, `MAIR`), and the Security Extension this repository does not model | `developer.arm.com/documentation/ddi0553/latest/` **[browser]** |
| ARMv6-M ARM (DDI 0419E) | The **Cortex-M0 family's architecture**: the v6-M Thumb subset (A5's encoding tables), the one-fault exception model (B1.5), the v6-M SCB/NVIC subset, SysTick and PMSAv6 (B3), `MRS`/`MSR`/`CPS` (B4) | `developer.arm.com/documentation/ddi0419/latest/` (a free PDF) |
| Cortex-M0 TRM (DDI 0432C), Cortex-M0+ TRM (DDI 0484C) | `CPUID`, the implementation options (VTOR, MPU, unprivileged support, SysTick, multiplier), the instruction cycle counts | developer.arm.com |
| SecurCore SC000 TRM (DDI 0456) | **Not public** (SecurCore NDA). What `Config::SC000` rests on instead is stated in its documentation and in `tests/conformance/ledgers/cpu-arm-v6m.txt` | — |
| SecurCore SC300 TRM | **Not public** (SecurCore NDA). `Config::SC300` is the Cortex-M3's options plus CMSIS `core_sc300.h`'s `VTOR` width; the gaps are in `tests/conformance/ledgers/cpu-arm-v7m.txt` | — |
| CMSIS-Core `core_sc000.h`, `core_sc300.h`, `core_cm3.h` (Apache-2.0) | The SecurCore register layouts the TRMs would otherwise give: which SCB/SCnSCB registers exist, the `VTOR` field widths (`TBLBASE` at bit 29 and `TBLOFF` `[28:7]` on the SC300 and pre-r2p1 Cortex-M3), the IMPLEMENTATION DEFINED `SFCR` at `0xE000EF90` | `github.com/ARM-software/CMSIS_5`, `CMSIS/Core/Include/` |
| Cortex-M4 TRM (DDI 0439), Cortex-M7 TRM (DDI 0489), Cortex-M33 TRM (100230) | The implementation-defined values a guest can see: `CPUID`, `MVFR0`/`MVFR1`/`MVFR2`, how many priority bits, how many MPU regions, instruction timings | developer.arm.com **[browser]** |
| Cortex-M35P TRM (100883) | **Available from Arm on request only.** `Config::CORTEX_M35P` rests on the public Security Target below; the gaps are in `tests/conformance/ledgers/cpu-arm-v8m.txt` | — |
| *Arm Cortex-M35P r1p1 Lite Security Target* (PJDOC-466751330-13922) | The M35P's revision (r1p1) and configuration options: the MPU, Security, FPU and DSP extensions, the optional 2-way 2-16 KiB instruction cache, 1-480 interrupts, 8-256 priority levels (§1.4) | `commoncriteriaportal.org` (a free PDF) |
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
- **That second walk is `cp15v7.rs`**, the Cortex-A9's CP15 (DDI 0406C B3.5
  for the walk, B3.12/B3.13 for the check order and the five-bit fault
  status, B4.1 for the registers; DDI 0388 chapter 4 for the part's values).
  Four decisions in it are worth knowing before touching it. *No ASID tags:*
  the core's TLB is flushed on every `CONTEXTIDR`, `TTBR`, `TTBCR`, `DACR`
  and `SCTLR` write, which makes `nG` correct by construction at the cost of
  a refill per context switch. *Inner-shareable TLB operations broadcast only
  to cores joined with `Arm::join_cluster`:* `ID_MMFR3` advertises hardware
  broadcast, so an OS will not send the IPI that would cover a missing join,
  and every MPCore machine must join its cores pairwise. *The ID registers
  tell the truth about this core, not the silicon,* where the two differ:
  `ID_PFR0` reports no ThumbEE and `ID_DFR0` no CP14 debug, because
  advertising either sends a kernel into CP14 accesses that trap. *User-mode
  CP15 access is decided by the coprocessor* — `CpOp::privileged` carries the
  mode — because only the coprocessor knows that `TPIDRURO` is readable and
  `SCTLR` is not; the ARMv5 CP15 ignores the field, as it always allowed User
  mode through.
- **Thumb-2 on the A profile is a second encoding of instructions the core
  already executes.** `aprofile/thumb2.rs` decodes a 32-bit T32 instruction
  into the same `isa::Insn` an A32 word produces — `ADD.W r0, r1, r2, LSL #3`
  and A32's `ADD r0, r1, r2, LSL #3` are one value, run by one arm of the
  interpreter — and adds variants only where T32 genuinely differs: the
  modified immediate (`Operand::Const`, because an odd rotation or a
  replicated byte has no A32 spelling), `ORN`, `LDRT` with an offset and no
  writeback (`Index::Unprivileged`), `LDRD`/`LDREXD` with independent
  registers, and `TBB`/`TBH`. The coprocessor space is decoded by the A32
  decoder itself, since T32's bits 27..0 are the A32 encoding there (DDI 0406C
  A6.3.18), so CP15 and VFP need nothing Thumb-specific. What *is* Thumb's
  own: `ITSTATE` lives in `CPSR[26:25,15:10]` where the architecture puts it,
  so exception entry and return carry it for free; a 16-bit data-processing
  instruction inside an `IT` block sets no flags; an Undefined Instruction
  in Thumb state links to the instruction plus **two** even when it is 32
  bits wide (B1.9's offset table — Linux's `__und_usr` relies on it); `SVC`
  inside a block saves the *advanced* `ITSTATE` while every other exception
  saves the unadvanced one; and every PC-relative address is taken from
  `Align(PC, 4)`. The profile-independent rules — which halfwords start a
  32-bit instruction, `ThumbExpandImm_C`, the `ITSTATE` walk — are in
  `cpu/arm/t32.rs`, shared with `v7m`.
- **The M profile is a different architecture, not a subset.** There is no ARM
  state, so no A32 decoder; the exception model is hardware register stacking
  with `EXC_RETURN` rather than banked modes; the interrupt controller is *inside
  the core* rather than a GIC beside it; and protection is PMSAv7 regions rather
  than a translation table. `cpu/arm/v7m` shares the `arm` family module with the
  A-profile core and nothing else. Its board is
  [`../platforms/stm32f407.md`](../platforms/stm32f407.md), which also records
  how a peripheral raises an interrupt when the NVIC has no wire of its own.
- **Bit-banding is the core's, not the board's.** Cortex-M3 and Cortex-M4
  alias the low megabyte of SRAM bit-by-bit at `0x22000000` and the low
  megabyte of the peripheral map at `0x42000000`; Cortex-M7 does not. The
  remap happens in the *processor's* bus matrix, so it is a `Config` flag on
  `cpu.arm.v7m` and a step in `exec.rs`'s data path — never a region a
  `.machine` file maps. Three consequences follow from putting it there and
  each is wrong anywhere else: the alias reaches whatever is mapped at the
  target, MMIO with read side effects included; it exists on a board that
  puts its SRAM somewhere other than `0x20000000`, because the alias is an
  address-map fact rather than a memory fact; and it disappears with the part
  rather than with the board. The bus matrix is *downstream of the MPU*, so
  an MPU region covering `0x20000000` does not cover `0x22000000` — real
  firmware has to describe both windows, and so does ours. A bit-band write
  is a read-modify-write of the target word under the space's `BusLock`, so a
  device sees exactly one read and one write and no other master gets in
  between.
- **The M profile's FPU is a different shape too.** FPv4-SP/FPv5 is
  single-precision-only on the parts that matter, `FPSCR` carries the condition
  flags a `VCMP` writes (A-profile puts them in `PSTATE`), and — the part with
  no A-profile analogue at all — exception entry can *reserve* the sixteen
  register slots of the extended frame and write them later, on the first
  floating-point instruction the handler executes. `FPCCR.LSPEN`/`LSPACT` and
  `FPCAR` are that mechanism, and firmware's interrupt latency depends on it, so
  it is modelled rather than collapsed into an eager push. The arithmetic itself
  is `crate::float`, shared with every other core, because
  [`ROADMAP.md`](../../ROADMAP.md) §9.1 makes guest floating point
  bit-reproducible across hosts. `tests/conformance/ledgers/cpu-arm-v7m-fp.txt`
  is the list of what is deliberately absent.
- **Armv8-M Mainline is the same core, not a fourth one.** A Cortex-M33 runs
  the same T32 and the same exception model as a Cortex-M4 with a list of
  changes small enough to key off one `Config::arch` flag: the stack limits,
  the `0xFF`-prefixed `EXC_RETURN` (`0xFFFFFFBC` for Thread/PSP), the
  acquire/release loads and stores, `TT`, and PMSAv8 in place of PMSAv7 —
  base/limit regions with no priority among them, so an overlap is a fault.
  It is behind `cpu-arm-v8m` (`part = "cortex-m33"`/`"cortex-m33f"`) and is
  modelled **without the Security Extension**, which is what an STM32U5 with
  `TZEN = 0` gives its firmware: no Secure state, no SAU, RAZ/WI `_NS`
  aliases, `TTA`/`BXNS`/`BLXNS` UNDEFINED. Every new encoding is UNDEFINED on a
  v7-M part, and the v7-M suite runs unchanged with the feature on.
  `src/cpu/arm/v7m/v8m.rs`'s documentation is the precise statement, and
  `tests/conformance/ledgers/cpu-arm-v8m.txt` the list of what is absent.
- **The Cortex-M35P is a Cortex-M33 preset, not an architecture.**
  `Config::CORTEX_M35P` / `CORTEX_M35PF` (`part = "cortex-m35p"` /
  `"cortex-m35pf"`, under `cpu-arm-v8m`) are the M33's options with a
  `CPUID` of `0x411F0001`: r1p1, the revision Arm certified, with a part
  number of zero because the TRM that gives it is not public. What the M35P
  adds — physical anti-tampering and an optional instruction cache — is
  either invisible to software or modelled absent (the part built without
  the cache); `tests/conformance/ledgers/cpu-arm-v8m.txt` says so.
- **ARMv6-M is the same core cut down, not a fifth one.** A Cortex-M0 keeps
  every structural feature of the M profile and loses instructions and
  registers, so it is `Config::arch = Arch::V6m` behind `cpu-arm-v6m`
  (`part = "cortex-m0"`, `"cortex-m0plus"`/`"cortex-m0+"`, `"sc000"`). Decode
  is gated per encoding in front of the shared table — the 16-bit set bar
  `CBZ`/`CBNZ`/`IT`, plus 32-bit `BL`/`MSR`/`MRS`/`DMB`/`DSB`/`ISB` — so
  everything else is UNDEFINED, and the disassembler uses the same gate. Every
  fault is a HardFault (the configurable faults' enables do not exist, so the
  interpreter's escalation path does it), `CCR` is read-only with
  `UNALIGN_TRP` set, priorities are two bits, and the SCB/NVIC/MPU maps are
  the v6-M subsets. The SC000's TRM is not public, so its preset is partly
  unverified and says so. `src/cpu/arm/v7m/v6m.rs`'s documentation is the
  precise statement, `tests/conformance/ledgers/cpu-arm-v6m.txt` the list of
  what is absent.
- **The SecurCore SC300 is a Cortex-M3 preset, not an architecture.**
  `Config::SC300` (`part = "sc300"`, no feature beyond `cpu-arm-v7m`) is
  ARMv7-M with no DSP and no FPU, the optional PMSAv7 MPU modelled present
  (`mpu = false` for silicon without it), and two differences from the
  Cortex-M3 preset: a `CPUID` carrying only the implementer and architecture
  fields, because the SC300's part number is not published, and `VTOR`
  implementing bits `[29:7]`, the layout CMSIS `core_sc300.h` gives it. Its
  anti-tampering features and `SFCR` are not modelled;
  `tests/conformance/ledgers/cpu-arm-v7m.txt` says so.
- **The boot vector table is the SoC's.** `cpu.arm.v7m`'s `vtor` property is
  `VTOR`'s reset value and where every reset, warm or cold, reads `SP` and
  `PC`. It is zero on a part that boots from zero; an STM32U5 boots from
  `0x08000000` and maps nothing at all at zero.
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
