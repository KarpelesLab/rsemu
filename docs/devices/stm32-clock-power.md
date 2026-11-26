# STM32 clock and power: `st.rcc`, `st.pwr` and `st.icache`

Consumed by: `src/dev/stm32/rcc.rs`, `src/dev/stm32/pwr.rs`,
`src/dev/stm32/icache.rs`. Every other STM32
peripheral consumes the *interfaces* below, so this page is a source register
first and an interface contract second.

## Sources

| Document | Answers | Where |
| --- | --- | --- |
| ST **RM0090** rev 21, §7 "Reset and clock control" | The F4 clock tree (§7.2) and the whole register map with reset values (§7.3) | st.com, `DM00031020` |
| ST **RM0090** rev 21, §5 "Power control" | `PWR_CR`/`PWR_CSR`, `DBP` and the backup domain (§5.1.4), the regulator and the F42x over-drive (§5.1.5) | same |
| ST **RM0351** rev 9, §6 "Reset and clock control" | The L4 tree (§6.2), the MSI, the register map (§6.4) | st.com, `DM00083560` |
| ST **RM0351** rev 9, §5 "Power control" | `CR1`…`CR4`, `SR1`/`SR2`, `SCR`, the `PUCRx`/`PDCRx` pulls | same |
| ST **DS8626** rev 9 (F407VG datasheet) | Which crystal a package expects, and the HSE/LSE limits | st.com |
| ST **RM0456** (STM32U5), §11 "Reset and clock control", §10 "Power control", and the ICACHE/DCACHE chapters | The U5 tree, MSI ranges (§11.4), RCC map (§11.8); `VOSR`/`SVMSR`, EPOD boost, SMPS/LDO; the cache control blocks | st.com |
| ST CMSIS **`stm32u585xx.h`** (cmsis-device-u5, Apache-2.0) | `RCC_TypeDef`, `PWR_TypeDef`, `ICACHE_TypeDef`, `DCACHE_TypeDef` offsets, every `_Pos`/`_Msk` | github.com/STMicroelectronics/cmsis-device-u5 |
| ST **STM32U5 HAL** (stm32u5xx-hal-driver, BSD-3-Clause) | Which flags firmware spins on; `HAL_RCC_DeInit`'s reset values; `SystemCoreClockUpdate`'s PLL1 formula | github.com/STMicroelectronics/stm32u5xx-hal-driver |

Nothing here was written from an emulator of any licence (`ROADMAP.md` §1).
Reset values, bit positions and divider tables are facts from the manuals
above; the register *file* layouts in the two device modules are transcriptions
of §7.3.23 and §6.4.31.

## Why these two exist at all

`SystemInit` and `SystemClock_Config` — the two functions every STM32 project
starts with, vendor HAL or not — are a sequence of **spins on hardware-set
bits**:

```
CR.HSEON = 1;      while (!CR.HSERDY) ;
PLLCFGR  = …;
CR.PLLON = 1;      while (!CR.PLLRDY) ;
PWR.CR.VOS = …;    while (!PWR.CSR.VOSRDY) ;      /* F4  */
                   while ( PWR.SR2.VOSF ) ;       /* L4  */
CFGR.SW  = PLL;    while (CFGR.SWS != PLL) ;
```

A RAM window reads back what was written, so **every one of those loops is
infinite** on a board that maps `rcc` as RAM. That is the whole reason these
two devices are the first STM32 peripherals after the GPIO and the USART:
nothing a vendor HAL calls during startup can work until they exist.

## The interfaces another peripheral codes against

### Clock rates — `ExportId::CLOCK_TREE`

The RCC publishes an `Arc<rcc::Clocks>`. A consumer names its controller in the
machine file as a link-valued property and asks for it at bind time:

```rust
// in the machine file:  object usart2 "st.usart" { rcc = "rcc" }
let clocks = ctx.export_as::<rcc::Clocks>("rcc", ExportId::CLOCK_TREE)?;
```

and then:

| Call | Answers |
| --- | --- |
| `clocks.rate(ClockOutput::PCLK1)` | an exact `core::clock::Rational` in hertz; **zero** when that output is stopped |
| `clocks.generation()` | a counter that increases whenever any rate changes |

`ClockOutput` has `SYSCLK`, `HCLK`, `PCLK1`, `PCLK2`, `TIMCLK1`, `TIMCLK2`,
`RTCCLK` and `PLL48`. `TIMCLKx` is already doubled where the APB prescaler is
not one, so a timer model does not repeat that rule.

Rates are **exact rationals, never floats**: a PLL multiplier and a prescaler
are integers and the ratio between them is exact by construction
(`CLAUDE.md`, *Determinism*). 8 MHz ÷ 8 × 336 ÷ 2 is `168000000/1`.

Cache a derived number — a baud divisor, a prescaler reload — against the
generation you computed it at, and recompute when the generation moves. That is
cheaper than recomputing per access and it cannot go stale.

**A rate of zero means "the guest has not switched this on".** It is
distinguishable from every real frequency, and a peripheral that divides by it
should report a configuration the guest can see rather than panicking.

### Gates and resets — wires

Every `xxENR` and `xxRSTR` bit is an output pin. The pin name is the bank
prefix plus the bit number, so a board writes the number from the reference
manual in the board file, exactly as it already writes `cpu.irq38`:

```
#  RM0090 §7.3.13: APB1ENR bit 17 is USART2EN, APB1RSTR bit 17 is USART2RST.
wire rcc.apb1en17  -> usart2.enable
wire rcc.apb1rst17 -> usart2.reset
```

Banks, by pin prefix:

| Prefix | F4 register | L4 register |
| --- | --- | --- |
| `ahb1en`, `ahb2en`, `ahb3en` | `AHB1ENR`… | `AHB1ENR`… |
| `apb1en` | `APB1ENR` | `APB1ENR1` |
| `apb1enb` | — | `APB1ENR2` |
| `apb2en` | `APB2ENR` | `APB2ENR` |
| `ahb1rst`, `ahb2rst`, `ahb3rst`, `apb1rst`, `apb1rstb`, `apb2rst` | the matching `xxRSTR` | the matching `xxRSTR` |

The L4's second APB1 word is `apb1enb` rather than `apb1en2` because a bank pin
is a prefix followed by decimal digits: `apb1en21` would otherwise be two pins
with one spelling.

**What a peripheral implements**, through `src/dev/stm32/gate.rs` — one
`ClockGate` beside its register state and two input pins, both levels:

| Pin | High means | What the peripheral does |
| --- | --- | --- |
| `enable` | the clock is running | low: every register reads as zero, writes are dropped, and anything the block counts stops where it is |
| `reset` | the reset line is pulled | on the rising edge, the register file goes back to its reset values; the block stays deaf while the level stands |

Both are `Device::sink`, and both are safe to receive with no lock held — the
RCC drives them *after* its own critical section. **A peripheral whose `enable`
nobody wired is clocked**, which is what keeps every board written before the
gate working and lets peripherals adopt the pins one at a time; it is
`st.firewall`'s `clken` rule, generalised.

Not a bus fault: ST's note under each `xxENR` register is that the registers
are not readable — zero comes back — and that accesses are not effective, so a
firmware that forgets one `RCC_AHB1ENR` write sees a port that will not
configure rather than a HardFault that would have said why. Removing a clock is
**not** a reset: the values behind the registers survive it, and `xxRSTR` is
the pin that clears them.

The gate's level is deliberately **not** in any device's snapshot. `Rcc::load`
restores `xxENR`/`xxRSTR` and republishes every connected gate pin, and no
peripheral's `load` touches its gate, so the two cannot disagree whichever
order the chunks load in — and adopting the gate moved no device's encoding.

**Who has the pins**, and who has something else instead:

| Class | `enable` | `reset` | Notes |
| --- | --- | --- | --- |
| `st.gpio`, `st.usart`, `st.tim`, `st.dma`, `st.dmamux`, `st.crc`, `st.rng`, `st.syscfg`, `st.i2c`, `st.i2c-v2`, `stm32.spi`, `st.octospi`, `stm32.sdio`, `stm32.sdmmc`, `st.hash`, `st.pwr`, `st.ipcc`, `st.hsem` | yes | yes | |
| `st.wwdg` | yes | — | its `reset` is the watchdog's **output**, and two opposite things cannot share a pin name; `APB1RSTR.WWDGRST` has no input here |
| `st.firewall` | `clken` | — | predates this module: `FWEN` is write-once and its level is inside that device's snapshot |
| `st.rtc` | `rtcen` | `bdrst` | the RTC's gate is `BDCR.RTCEN`, not an `xxENR` bit |
| `st.iwdg`, `st.exti`, `st.dbgmcu` | — | — | the part gives them no `xxENR` bit: the IWDG runs from the LSI whatever RCC says |
| `st.flash` | — | — | not yet gated |

What a gated peripheral does with its **level outputs** — an `irq` line, a DMA
request — is not uniform yet, and the manuals do not settle it. A block with no
clock cannot change its flip-flops, which argues for the line holding what it
had; a request line stuck high at a controller that *is* clocked drains a
stream into a peripheral that is switched off, which argues for dropping it.
`stm32.sdio`, `stm32.sdmmc` and `st.hash` drop `irq`/`dma` on the first gated
access; everything else holds. Closing that properly wants a `Gated`
notification on the enable pin's edge, which this round did not add.

Two more named outputs: `rtcen` (`BDCR.RTCEN`) and `bdrst` (`BDCR.BDRST`), for
an RTC model to watch.

### The clock interrupt — a wire out

`CIR` (F4) and `CIER`/`CIFR`/`CICR` (L4) are live: a ready bit coming true
while its `xxRDYIE` stands sets the matching flag and raises the `irq` output,
which an F4 board wires to NVIC position 5 (`wire rcc.irq -> cpu.irq5`).

The flag is set only when the enable is — "set by hardware when the … clock
becomes stable **and** `xxRDYIE` is set" (RM0090 §7.3.5, RM0351 §6.4.6) — so a
firmware that polls `CR` never finds one of these appear behind it. `CSSF` is
the one flag nothing sets: a failing HSE is not modelled, and on the part it
takes the NMI rather than this line.

### `PWR_CR.DBP` — a wire, not a handle

```
wire pwr.dbp -> rcc.dbp
```

The backup domain — `BDCR`'s `LSEON`, `LSEBYP`, `RTCSEL`, `RTCEN`, `BDRST` — is
write-protected until firmware sets `PWR_CR.DBP` (RM0090 §7.3.20). On the die
that protection is a signal from the power block, so a level on a net is the
honest model; it is also the model that keeps the two devices from ever nesting
one `LockRank::DEVICE` lock inside another. PWR drives the level after its
critical section; the RCC's `dbp` sink stores it in an atomic, and the `BDCR`
write path samples that atomic without calling into PWR at all.

An RCC whose `dbp` pin is **unwired** treats the backup domain as unprotected.
A board with no `st.pwr` has nothing modelling the protection, and a domain
that could never be opened would be a board bug wearing a device bug's clothes.

### Reset causes — wires in

`CSR`'s `xxRSTF` flags are latched by a rising edge on an input pin, and cleared
by `RMVF`. The pins are `borrst`, `pinrst`, `porrst` (F4 only), `sftrst`,
`iwdgrst`, `wwdgrst`, `lpwrrst`, and on an L4 `fwrst` and `oblrst`. A watchdog
or a reset controller drives one; the RCC does not guess. A U5 has `oblrst`,
`pinrst`, `borrst`, `sftrst`, `iwdgrst`, `wwdgrst`, `lpwrrst` (no `porrst`, no
`fwrst`), with `RMVF` at bit 23.

## The U5 (`variant = "u5"`)

### `st.rcc`

Map: RM0456 §11.8 / `RCC_TypeDef`, decoding `0x118` bytes at `0x4602_0C00`.

- **Boot**: `CR` = `0x35` (MSIS and MSIK on and ready), `SW` = `SWS` = MSIS at
  4 MHz, `CSR` = `0x0C00_4400` (PIN + BOR, both standby MSI ranges 4 MHz),
  `PLLxDIVR` = `0x0101_0280`, `ICSCR1` = `0x4400_0000`.
- **MSIS** comes from `CSR.MSISSRANGE[15:12]` until `ICSCR1.MSIRGSEL` (23)
  selects `ICSCR1.MSISRANGE[31:28]`. Sixteen ranges, four RCs ÷ 1/2/3/4, kept
  as exact rationals (range 6 is 4 MHz ÷ 3; ST's `MSIRangeTable` rounds).
- **Switch**: `CFGR1.SW[1:0]`/`SWS[3:2]`, 00 MSIS, 01 HSI16, 10 HSE, 11 PLL1,
  refused while the source is not ready.
- **Prescalers**: `CFGR2.HPRE[3:0]`, `PPRE1[6:4]`, `PPRE2[10:8]`;
  `CFGR3.PPRE3[6:4]`. `ClockOutput::PCLK3` (property `pclk3`, parent `hclk`)
  is new and zero on the F4/L4.
- **PLL1**: `src / (M+1) × (N+1 + FRACN/8192)`, `R+1` behind `PLL1REN` for
  `SYSCLK`, `Q+1` behind `PLL1QEN` for `pll48`; FRACN only while `PLL1FRACEN`.
  `PLL1MBOOST` divides the EPOD booster's clock, not the VCO input.
- **Ready bits** (twelve): `CR` MSIS 0→2, MSIK 4→5, HSI 8→10, HSI48 12→13,
  SHSI 14→15, HSE 16→17, PLL1 24→25, PLL2 26→27, PLL3 28→29; `BDCR` LSE 0→1,
  LSESYS 7→11, **LSI 26→27** (in the backup domain on a U5, so behind `DBP`).
  `CIER`/`CIFR`/`CICR` at `0x50/0x54/0x58`, flags `0x1DFF`. NVIC 9.
- **Gate pins** (`<n>` = the header's `_Pos`):

  | enable | register | reset | register |
  | --- | --- | --- | --- |
  | `ahb1en<n>` | `AHB1ENR` | `ahb1rst<n>` | `AHB1RSTR` |
  | `ahb2en<n>` | `AHB2ENR1` | `ahb2rst<n>` | `AHB2RSTR1` |
  | `ahb2enb<n>` | `AHB2ENR2` | `ahb2rstb<n>` | `AHB2RSTR2` |
  | `ahb3en<n>` | `AHB3ENR` | `ahb3rst<n>` | `AHB3RSTR` |
  | `apb1en<n>` | `APB1ENR1` | `apb1rst<n>` | `APB1RSTR1` |
  | `apb1enb<n>` | `APB1ENR2` | `apb1rstb<n>` | `APB1RSTR2` |
  | `apb2en<n>` | `APB2ENR` | `apb2rst<n>` | `APB2RSTR` |
  | `apb3en<n>` | `APB3ENR` | `apb3rst<n>` | `APB3RSTR` |

  The second word of a pair takes a letter, as the L4's `apb1enb` did, so
  `ahb2en21` can only mean `AHB2ENR1` bit 21. Examples: PWR `ahb3en2`, GPIOA
  `ahb2en0`, USART1 `apb2en14`, USART2 `apb1en17`, LPUART1 `apb3en6`,
  DCACHE1 `ahb1en30`.

### `st.pwr`

Map: RM0456 §10 / `PWR_TypeDef`, `0x98` bytes at `0x4602_0800`.

| firmware writes | and waits for | model |
| --- | --- | --- |
| `VOSR.VOS[17:16]` | `VOSR.VOSRDY` (15), `SVMSR.ACTVOSRDY` (15) | a change clears both; `ready-delay` ticks later both set and `SVMSR.ACTVOS` copies `VOS` |
| `VOSR.BOOSTEN` (18) | `VOSR.BOOSTRDY` (14) | set after `ready-delay`, cleared with `BOOSTEN` |
| `CR3.REGSEL` (1) | `SVMSR.REGS` (1) | follows after `ready-delay` |
| `DBPR.DBP` (0) | read-back | drives the `dbp` output to `rcc.dbp` |

Reset: range 4 on the LDO, `VOSR` = `SVMSR` = `0x8000`. Supply monitors
(`SVMCR` 24–27) report their supply present at once; `SR.CSSF` and `WUSCR`
clear; `UCPDR`, `SECCFGR`, `PRIVCFGR`, `APCR`, the pulls are storage. The gate
is `rcc.ahb3en2 -> pwr.enable`; there is no `PWRRST`, so no `reset` wire.

### `st.icache`

The ICACHE (`variant = "icache"`, `0x4003_0400`, NVIC 107) and DCACHE1
(`variant = "dcache"`, `0x4003_1400`, NVIC 111) control blocks, `0x30` bytes
each. No caching; the flags firmware polls are live. `CR.CACHEINV` — and the
ICACHE's `EN` falling — sets `SR.BUSYF`, which clears after `busy-delay`
ticks (default 16; 0 = inside the write) with `BSYENDF` set and `CACHEINV`
cleared. The DCACHE's `STARTCMD` with a non-zero `CACHECMD` runs `BUSYCMDF`
to `CMDENDF`. `irq` = `SR & IER`; `FCR` is write-one-to-clear; the monitors
read zero; `WAYSEL` and `CRRx` are frozen while `EN` is set. Lazily advanced
like the RCC, with `enable`/`reset` gate inputs (`rcc.ahb1en30` for DCACHE1;
the ICACHE has no enable bit, so leave it undrawn).

## Implementation notes

- **Ready bits are time, and the scheduler owns time.** Both devices are
  lazily advanced (`Device::is_lazy`): they hold their own tick in their own
  clock domain, and the guest access that polls the flag is what catches them
  up. `ready-delay` is a count of *those* ticks, so a board writes `clock = hse`
  on the object and the delay is in crystal periods. Neither device sleeps and
  neither reads the host clock.
- **The ready-bit delay is deliberately short** (16 ticks for the RCC, 8 for
  the PWR). A real crystal takes milliseconds; nothing observable depends on
  which it is except how much *virtual* time a startup spends, and a board that
  wants the datasheet figure writes the property.
- **`SWS` is what the tree is computed from, not `SW`.** They differ for one
  tick after an accepted switch, and that is the difference between "firmware
  asked" and "the hardware did it".
- **A `SW` write naming a source that is not ready is dropped**, field and all,
  so `SWS` never moves. Firmware that polls `xxxRDY` first — every vendor
  sequence — never notices.
- **`VOSRDY` and `VOSF` have opposite polarity.** An F4 sets `CSR.VOSRDY` when
  the regulator arrives; an L4 sets `SR2.VOSF` while it is *changing* and
  clears it on arrival. A model that averaged the two would hang one family.
- The backup domain survives `ResetKind::Warm` and not `ResetKind::Cold`,
  which is what "battery-backed" means; the `CSR` reset flags go with it.

## Open

- **The RCC is told its crystal twice**: once as the `osc` a machine file
  declares and once as this device's `hse`/`lse` properties. Nothing checks that
  the two agree, and `hse` is now load-bearing twice over — it is also the
  reference every driven clock output's ratio is measured against.
- **No board in `machines/` names a clock output yet**, so on every shipped
  board the rates are still fixed at what the machine file declares and only
  the published [`Clocks`] table moves. Rewriting one is a behavioural change:
  a part comes out of reset on its internal RC, so a board that hands `SYSCLK`
  to `st.rcc` runs at 16 MHz until its firmware configures the PLL.
- **An output of zero does not stop its domain.** `SWS` naming a source that is
  not running leaves the domain at the rate it had; gating belongs with the
  peripheral clock-enable half of the problem.
- **A rating is measured against the domain's parent**, so a controller cannot
  say *which* crystal an output came from: a board with one high-speed
  oscillator models HSI and the PLL as exact ratios of HSE. The rate a guest
  measures is exact; the independence of the two cans is not modelled.
- **U5 enable registers reset to zero**, as do its `*SMENR`; RM0456 has some
  SRAM and flash-interface clocks on out of reset. `CCIPR1`…`CCIPR3`, PLL2/3,
  MSIK, HSI48 and SHSI answer their ready bits and drive nothing, and the U5's
  `SECCFGR`/`PRIVCFGR` are storage (TrustZone off).
- The F4 `*LPENR` and L4 `*SMENR` low-power gate registers reset to zero rather
  than to the manual's "every implemented peripheral enabled" constants, which
  were not to hand to check. Nothing in a startup path reads them.
- **The clock security system raises nothing.** `CSSON` is stored and a failing
  HSE is not modelled, so `CSSF` — and the NMI it takes on the part — is the
  one interrupt source here that no path sets. The eight ready flags are live.
- **No shipped board wires a gate pin.** The classes have `enable` and `reset`;
  `machines/stm32f407.machine` draws neither, so on that board every peripheral
  is clocked from the first instruction as it was before. Wiring it is a
  behavioural change that belongs in its own commit: the part's peripherals
  come out of reset gated, so the board's nine hand-assembled test programs and
  the demo firmware `rsemu run stm32f407` ships would each have to enable their
  own clocks first, exactly as vendor firmware does.
  `machines/tests/stm32f4-gated.machine` is the board that does wire them.
