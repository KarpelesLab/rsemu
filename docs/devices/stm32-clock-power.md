# STM32 clock and power: `st.rcc` and `st.pwr`

Consumed by: `src/dev/stm32/rcc.rs`, `src/dev/stm32/pwr.rs`. Every other STM32
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

**What a peripheral should implement.** Two input pins, both levels:

| Pin | High means | What the peripheral does |
| --- | --- | --- |
| `enable` | the clock is running | low: registers read as zero and writes are dropped — or a bus fault, where the part faults |
| `reset` | the reset line is pulled | on the rising edge, `Device::reset(ResetKind::Bus)` on itself; stay reset while it is high |

Both are `Device::sink`, and both must be safe to receive with no lock held —
the RCC drives them *after* its own critical section. A peripheral with neither
pin wired behaves as it does today, ungated, so the five peripherals being
written alongside this one can adopt the pins one at a time.

Two more named outputs: `rtcen` (`BDCR.RTCEN`) and `bdrst` (`BDCR.BDRST`), for
an RTC model to watch.

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
or a reset controller drives one; the RCC does not guess.

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
- The F4 `*LPENR` and L4 `*SMENR` low-power gate registers reset to zero rather
  than to the manual's "every implemented peripheral enabled" constants, which
  were not to hand to check. Nothing in a startup path reads them.
- RCC interrupts (`CIR`, `CIER`/`CIFR`/`CICR`) are storage: no ready or
  clock-security event is raised.
