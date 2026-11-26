//! The STM32 reset and clock controller.
//!
//! `st.rcc` is the chip that every vendor startup sequence talks to first and
//! the one a board cannot fake with RAM. `SystemInit` sets `CR.HSEON` and spins
//! until `CR.HSERDY` comes back; it programs `PLLCFGR`, sets `CR.PLLON` and
//! spins on `CR.PLLRDY`; it writes `CFGR.SW` and spins until `CFGR.SWS` agrees.
//! A RAM cell reads back what was written, so **every one of those spins is
//! infinite** — which is why this device is the first of the STM32 peripherals
//! to be built rather than the fifth.
//!
//! # What it does
//!
//! | | |
//! | --- | --- |
//! | ready bits | each `xxxON` produces its `xxxRDY` after `ready-delay` ticks of this device's own clock domain, and drops it the moment `xxxON` clears |
//! | the switch | `CFGR.SW` is refused outright when the source it names is not ready; when it is accepted, `SWS` follows one tick later |
//! | the tree | `SYSCLK`, `HCLK`, `PCLK1`, `PCLK2`, the timer clocks and `RTCCLK` are computed as **exact rationals** from the PLL factors and the prescalers, and published through [`Clocks`] |
//! | gating | every `xxENR` and `xxRSTR` bit is an output pin, and [`gate`](super::gate) is the input a peripheral receives it on |
//! | interrupts | a ready bit coming true with its `xxRDYIE` set raises `CIR`/`CIFR`'s flag and the `irq` output, which an F4 or a WB board wires to NVIC position 5 and a U5 board to position 9 (`RCC_IRQn`) |
//! | the backup domain | `BDCR` is write-protected until `PWR_CR.DBP` arrives on the `dbp` input, and `BDRST` clears the domain |
//! | reset causes | `CSR`'s `xxRSTF` flags are set by a pulse on the matching input pin and cleared by `RMVF` |
//!
//! # Time, and why there is no sleeping
//!
//! A crystal takes milliseconds to start and a PLL takes microseconds to lock.
//! Neither is a host-clock reading: the device is **lazily advanced**
//! ([`Device::is_lazy`]), it holds its own tick in its own clock domain, and
//! the guest access that polls `CR` is what catches it up. `ready-delay` is
//! therefore a count of *those* ticks — a board with `clock = hse` on its
//! `rcc` object is counting 125 ns ticks of an 8 MHz can — and the default is
//! short on purpose. Nothing here reads the wall clock and nothing sleeps.
//!
//! # The clock outputs
//!
//! The rates live in a [`Clocks`] handle the device publishes as
//! [`ExportId::CLOCK_TREE`], and a consumer names its RCC in the machine file
//! (`rcc = "rcc"`) and asks for it at bind time. Each output is an exact
//! [`Rational`] in hertz: `8 MHz / 8 × 336 / 2` is `168000000/1` and not a
//! rounded `f64`, because a ratio inside one oscillator's tree is exact by
//! construction (`CLAUDE.md`, *Determinism*).
//!
//! **And it re-rates the scheduler's clock domains.** A board that says
//!
//! ```text
//! osc hse = 8000000 Hz
//! object sysclk "clock" { clock = hse * 2 }      # the reset rate: HSI, 16 MHz
//! object hclk   "clock" { clock = sysclk }
//! object pclk1  "clock" { clock = hclk / 4 }
//! object pclk2  "clock" { clock = hclk / 2 }
//! object rcc "st.rcc" {
//!   clock = hse, variant = "f4", hse = 8000000,
//!   sysclk = "sysclk", hclk = "hclk", pclk1 = "pclk1", pclk2 = "pclk2"
//! }   # and `timclk1`/`timclk2` for what the timers count
//! ```
//!
//! gets a `SYSCLK` that really is 168 MHz once the guest's `SystemInit` has
//! run, and peripherals hung off `pclk1` that really do follow `PPRE1`. The
//! request goes through [`crate::core::clock::ClockControl`] and
//! is applied by the scheduler at a round boundary, never mid-round; that type
//! documents the rule and what becomes of the ticks already counted. An output
//! no board named drives nothing, which is every board that has not been
//! rewritten to the shape above — they keep the fixed ratios their machine file
//! declares, and [`Clocks`] is still the way a peripheral reads a rate.
//!
//! `drive_domains` has the two deviations this bakes in: a rating is measured
//! against the domain's parent, so HSI and the PLL are modelled as exact ratios
//! of the HSE crystal rather than as independent cans, and an output of zero
//! leaves its domain where it was rather than stopping it.
//!
//! # Sources
//!
//! * *STM32F405/415, STM32F407/417, STM32F427/437 and STM32F429/439 advanced
//!   Arm-based 32-bit MCUs*, ST **RM0090** rev 21, §7 "Reset and clock control
//!   (RCC)" — §7.2 for the tree, §7.3 for the register map.
//! * *STM32L4x5 and STM32L4x6 advanced Arm-based 32-bit MCUs*, ST **RM0351**
//!   rev 9, §6 "Reset and clock control (RCC)" — §6.2 for the tree, §6.4 for
//!   the register map.
//! * *STM32U5 series Arm-based 32-bit MCUs*, ST **RM0456**, §11 "Reset and
//!   clock control (RCC)" — §11.4 for the tree and the MSI ranges, §11.8 for
//!   the register map — with the offsets and bit positions taken from ST's
//!   CMSIS device header `stm32u585xx.h` (`RCC_TypeDef`, `RCC_*_Pos`;
//!   Apache-2.0). What the U5 variant must answer was read off the polling
//!   loops of ST's BSD-3-Clause U5 HAL (`HAL_RCC_OscConfig`,
//!   `HAL_RCC_ClockConfig`, `HAL_RCCEx_PeriphCLKConfig`) and the PLL formula
//!   off `SystemCoreClockUpdate` in `system_stm32u5xx.c`; none of that code
//!   is reproduced here.
//! * *Multiprotocol wireless 32-bit MCU Arm-based Cortex-M4 with FPU,
//!   Bluetooth Low-Energy and 802.15.4 radio solution*, ST **RM0434** rev 16,
//!   §6 "Reset and clock control (RCC)", for the STM32WB55: §6.2 for the
//!   tree, §6.4.1–§6.4.48 for every register, its bits and its reset value,
//!   and Table 38 for the map. The offsets and bit positions agree with ST's
//!   CMSIS header `stm32wb55xx.h` (`RCC_TypeDef`, `RCC_*_Pos`; Apache-2.0),
//!   and the reset values with ST's SVD `STM32WB55_CM4.svd` (Apache-2.0)
//!   except where the module notes say otherwise; the manual wins every
//!   disagreement. ST **DS11929** rev 18 (the STM32WB55xx/WB35xx datasheet),
//!   §3.10 "Clocks and startup", confirms the 4 MHz MSI start, the twelve
//!   MSI ranges, the 32 MHz HSE, the two ~32 kHz LSIs and the 64 MHz ceiling.
//!   What the WB variant must answer was read off the polling loops of ST's
//!   BSD-3-Clause WB HAL (`HAL_RCC_OscConfig`, `HAL_RCC_ClockConfig` —
//!   which waits on `HPREF`, `C2HPREF`, `SHDHPREF`, `PPRE1F` and `PPRE2F` —
//!   and `HAL_RCC_DeInit`); none of that code is reproduced here.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//!
//! # The U5 variant
//!
//! `variant = "u5"` is RM0456's map, which is the L4's logic redrawn:
//!
//! * It boots on **MSIS at 4 MHz** (`CR` = `0x35`: `MSISON`/`MSISRDY` and
//!   `MSIKON`/`MSIKRDY`), and the MSIS range comes from
//!   `CSR.MSISSRANGE` until `ICSCR1.MSIRGSEL` points at `ICSCR1.MSISRANGE`.
//!   The sixteen ranges are exact rationals — range 6 is 4 MHz ÷ 3 — where
//!   ST's `MSIRangeTable` rounds.
//! * `CFGR1.SW`/`SWS` keep the L4's bits and encoding (MSIS, HSI16, HSE,
//!   PLL1); the prescalers moved to `CFGR2` (`HPRE`, `PPRE1`, `PPRE2`) and
//!   `CFGR3` (`PPRE3`), and [`ClockOutput::PCLK3`] is the APB3 clock.
//! * PLL1 is `src / (M+1) × (N+1 + FRACN/8192) / (R+1)`, with the fraction
//!   only while `PLL1FRACEN` stands and `R`/`Q` only behind `PLL1REN`/`QEN`.
//!   `PLL1MBOOST` divides the EPOD booster's clock and no output.
//! * Twelve ready bits: MSIS, MSIK, HSI16, HSI48, SHSI, HSE, PLL1–3 in `CR`;
//!   LSE, `LSESYS` and **LSI** in `BDCR` — the U5 moved the LSI into the
//!   backup domain, so its enable is behind `DBP` too. `CIER`/`CIFR`/`CICR`
//!   carry `MSIKRDY` at 11 and `SHSIRDY` at 12; `LSESYSRDY` has no flag.
//! * Eight gate banks each way. The second words take a letter, as the L4's
//!   `apb1enb` already does, so no pin name can be read two ways:
//!
//!   | enable pin | register | reset pin | register |
//!   | --- | --- | --- | --- |
//!   | `ahb1en<n>` | `AHB1ENR` `+0x88` | `ahb1rst<n>` | `AHB1RSTR` `+0x60` |
//!   | `ahb2en<n>` | `AHB2ENR1` `+0x8c` | `ahb2rst<n>` | `AHB2RSTR1` `+0x64` |
//!   | `ahb2enb<n>` | `AHB2ENR2` `+0x90` | `ahb2rstb<n>` | `AHB2RSTR2` `+0x68` |
//!   | `ahb3en<n>` | `AHB3ENR` `+0x94` | `ahb3rst<n>` | `AHB3RSTR` `+0x6c` |
//!   | `apb1en<n>` | `APB1ENR1` `+0x9c` | `apb1rst<n>` | `APB1RSTR1` `+0x74` |
//!   | `apb1enb<n>` | `APB1ENR2` `+0xa0` | `apb1rstb<n>` | `APB1RSTR2` `+0x78` |
//!   | `apb2en<n>` | `APB2ENR` `+0xa4` | `apb2rst<n>` | `APB2RSTR` `+0x7c` |
//!   | `apb3en<n>` | `APB3ENR` `+0xa8` | `apb3rst<n>` | `APB3RSTR` `+0x80` |
//!
//!   `<n>` is the bit from the header's `RCC_xxxENR_yyyEN_Pos`: the PWR is
//!   `ahb3en2`/`ahb3rst2`, GPIOA `ahb2en0`, USART1 `apb2en14`, USART2
//!   `apb1en17`, LPUART1 `apb3en6`.
//! * The reset-cause inputs are `oblrst`, `pinrst`, `borrst`, `sftrst`,
//!   `iwdgrst`, `wwdgrst`, `lpwrrst` (`CSR[31:25]`), `RMVF` is bit 23.
//!
//! # The WB variant
//!
//! `variant = "wb"` is RM0434's map for the STM32WB55: the L4's register
//! file with a second CPU bolted on. CPU1 is the Cortex-M4 and reads
//! [`ClockOutput::HCLK`]; CPU2, the Cortex-M0+ that runs the radio stack, has
//! a clock and a set of peripheral gates of its own.
//!
//! * It boots on the **MSI at 4 MHz**, range 6 in `CR.MSIRANGE` — there is
//!   no `MSIRGSEL` and no standby range on a WB, so the range is `CR`'s,
//!   always, and a write of 12…15 is refused ("not allowed (hardware write
//!   protection)", §6.4.1). `CR` comes up `0x63` (on, ready, range 6): the
//!   manual's `0x61` is the POR value before `MSIRDY`, which it says "is read
//!   1 once the MSI is ready", and no access here can land before that.
//! * The **HSE is the 32 MHz radio crystal** (write `hse = 32000000`), and
//!   `CR.HSEPRE` (bit 20) halves it on the way to both `SYSCLK` and the PLL's
//!   M divider — "SYSCLK and PLL M divider input clocks are divided by 2"
//!   (§6.4.1). The RTC's `HSE / 32` takes the undivided crystal.
//! * **`HPRE` has the WB's own table** (§6.4.3): `0001` is ÷3, `0010` ÷5,
//!   `0101` ÷6, `0110` ÷10, `0111` ÷32, `1xxx` the usual ÷2…÷512, others
//!   ÷1. The same encoding is used by `EXTCFGR.C2HPRE[7:4]`, which divides
//!   `SYSCLK` into [`ClockOutput::HCLK2`] (CPU2), and by
//!   `EXTCFGR.SHDHPRE[3:0]`, which divides it into [`ClockOutput::HCLK4`]
//!   (AHB4, the flash and SRAM2) (§6.4.34). `PCLK1`/`PCLK2` hang off `HCLK1`
//!   as on an L4.
//! * **The prescaler handshake.** Each flag is "reset when a new prescaler
//!   value is programmed … set when the programmed value is actually
//!   applied" (§6.4.3, §6.4.34). A write that changes `HPRE`, `PPRE1` or
//!   `PPRE2` clears `CFGR.HPREF`/`PPRE1F`/`PPRE2F` (bits 16–18), and one that
//!   changes `C2HPRE` or `SHDHPRE` clears `EXTCFGR.C2HPREF`/`SHDHPREF` (17,
//!   16); one tick of this device's clock later every cleared flag is set
//!   again. `HAL_RCC_ClockConfig` spins on each, and all five read `1` out of
//!   reset (`CFGR` = `0x0007_0000`, `EXTCFGR` = `0x0003_0000`). A write that
//!   leaves the field as it was leaves its flag alone.
//! * The PLL is `src / (M+1) × N / (R+1)` (§6.4.4) with `PLLM[6:4]`,
//!   `PLLN[14:8]`, `PLLR[31:29]` behind `PLLREN[28]` and `PLLQ[27:25]`
//!   (÷ Q+1, the [`ClockOutput::PLL48`] output) behind `PLLQEN[24]` — not the
//!   L4's `2·(x+1)`. It resets to `0x2204_0100`.
//! * Ten ready bits: MSI, HSI16, `HSIKERDY` (bit 12, set only by `HSIKERON`,
//!   as `HSIRDY` is only by `HSION`), HSE, PLL and PLLSAI1 in `CR`; LSE in
//!   `BDCR`; **LSI1** (0→1) and **LSI2** (2→3) in `CSR`; HSI48 in `CRRCR`.
//!   `LSI2ON` is "enable *and selection*" (§6.4.31): the LSI the RTC sees is
//!   LSI2 while it stands and LSI1 otherwise, both at the `lsi` property's
//!   rate. `CIER`/`CIFR`/`CICR` carry `HSI48RDYF` at 10 and `LSI2RDYF` at 11
//!   (§6.4.6–§6.4.8); `HSIKERDY` has no flag.
//! * `SMPSCR` stores `SMPSSEL`/`SMPSDIV` and resets to `0x301`. `SMPSSWS`
//!   is derived on every read as §6.4.9 states it: `11` (no clock) while
//!   `PWR_CR5.SMPSEN` is clear — it arrives on the **`smpsen`** input, wired
//!   `pwr.smpsen -> rcc.smpsen`, and an undrawn wire reads as clear — `10`
//!   whenever the HSE is running, and otherwise the HSI16 or MSI that
//!   `SMPSSEL` names if it is running. `CSR.RFWKPSEL` (the radio wake-up
//!   clock) and `CSR.LSI2TRIM` are storage. `CSR.RFRSTS` reads back
//!   `APB3RSTR.RFRST`, whose description points there for "the reset status
//!   of the radio system" (§6.4.16).
//! * `HSECR` is keyed (§6.4.33): a write of `0xCAFE_CAFE` sets `UNLOCKED`
//!   (bit 0), the next write lands in `HSES`, `HSEGMC` and `HSETUNE` and
//!   locks it again, and a write while locked is dropped. It resets to `0x30`.
//! * Gate banks — CPU1's at the L4's offsets, with `AHB3ENR` holding the
//!   AHB3 *and* AHB4 clocks and an `APB3RSTR` with nothing in it but the
//!   radio's `RFRST`; CPU2's from `+0x148` (§6.4.35–§6.4.41), each prefixed
//!   `c2`:
//!
//!   | enable pin | register | reset pin | register |
//!   | --- | --- | --- | --- |
//!   | `ahb1en<n>` | `AHB1ENR` `+0x48` | `ahb1rst<n>` | `AHB1RSTR` `+0x28` |
//!   | `ahb2en<n>` | `AHB2ENR` `+0x4c` | `ahb2rst<n>` | `AHB2RSTR` `+0x2c` |
//!   | `ahb3en<n>` | `AHB3ENR` `+0x50` | `ahb3rst<n>` | `AHB3RSTR` `+0x30` |
//!   | `apb1en<n>` | `APB1ENR1` `+0x58` | `apb1rst<n>` | `APB1RSTR1` `+0x38` |
//!   | `apb1enb<n>` | `APB1ENR2` `+0x5c` | `apb1rstb<n>` | `APB1RSTR2` `+0x3c` |
//!   | `apb2en<n>` | `APB2ENR` `+0x60` | `apb2rst<n>` | `APB2RSTR` `+0x40` |
//!   | | | `apb3rst<n>` | `APB3RSTR` `+0x44` |
//!   | `c2ahb1en<n>` | `C2AHB1ENR` `+0x148` | | |
//!   | `c2ahb2en<n>` | `C2AHB2ENR` `+0x14c` | | |
//!   | `c2ahb3en<n>` | `C2AHB3ENR` `+0x150` | | |
//!   | `c2apb1en<n>` | `C2APB1ENR1` `+0x158` | | |
//!   | `c2apb1enb<n>` | `C2APB1ENR2` `+0x15c` | | |
//!   | `c2apb2en<n>` | `C2APB2ENR` `+0x160` | | |
//!   | `c2apb3en<n>` | `C2APB3ENR` `+0x164` | | |
//!
//!   There are no CPU2 reset registers: a peripheral's reset is shared. The
//!   `c2` prefix starts with a letter no CPU1 prefix does, so no pin reads
//!   two ways. `<n>` is the bit in RM0434's register description (and the
//!   header's `RCC_xxxENR_yyyEN_Pos`): HSEM is `ahb3en19`/`c2ahb3en19`, IPCC
//!   `ahb3en20`/`c2ahb3en20` (and `ahb3rst19`/`ahb3rst20` for their
//!   resets), FLASH `ahb3en25`, GPIOA `ahb2en0`, USART1 `apb2en14`, LPUART1
//!   `apb1enb0`, the BLE interface `c2apb3en0` and the radio's reset
//!   `apb3rst0`. HSEM and FLASH (`AHB3ENR` = `C2AHB3ENR` = `0x0208_0000`,
//!   §6.4.19, §6.4.37) and the RTC's APB interface (`APB1ENR1`/`C2APB1ENR1`
//!   = `0x400`) are on out of reset, and every `*SMENR`/`C2*SMENR` word takes
//!   its §6.4.23–§6.4.28 and §6.4.42–§6.4.48 value — the WB is the one
//!   variant whose low-power gates reset to the manual's numbers.
//! * The reset-cause inputs and `RMVF` are the U5's: `oblrst` … `lpwrrst` at
//!   `CSR[31:25]`, `RMVF` at 23; `CSR` comes up `0x0c00_0000` (pin + BOR).
//!
//! # Known deviations
//!
//! * The `*LPENR` (F4) and `*SMENR` (L4) low-power gate registers are storage
//!   that reads back, and they reset to **zero** rather than to the manual's
//!   "every implemented peripheral enabled" constants. Those constants were
//!   not to hand to check, and a number transcribed from memory is worse than
//!   a stated gap. Nothing in a startup path reads them.
//! * **The clock security system raises nothing.** `CSSON` is stored, and a
//!   failing HSE — the event that sets `CSSF` and takes the NMI — is not
//!   modelled, so that one flag of `CIR`/`CIFR` is the only one no path sets.
//!   The eight ready flags and the `irq` output are live.
//! * The interrupt line follows *flag AND enable* rather than the flag alone.
//!   A flag can only be set while its enable stands (ST's own wording, quoted
//!   at `Interrupts::raise`), so the two agree except for firmware that clears
//!   `xxRDYIE` and leaves the flag standing; there the model drops the request
//!   a tick early, which no driver can tell from having serviced it.
//! * `SSCGR`, `PLLI2SCFGR`, `PLLSAICFGR`, `DCKCFGR`, `CCIPR`, `CCIPR2` and
//!   `CRRCR` read back and select nothing: their outputs have no consumer yet.
//!   On a U5 the same is true of `CCIPR1`…`CCIPR3`, `SRDAMR`, `PLL2`/`PLL3`'s
//!   configuration, `MSIK`, `HSI48` and `SHSI` — each answers its ready bit
//!   and drives no output — and of `SECCFGR`/`PRIVCFGR`, which are storage
//!   because the model runs with TrustZone off (`TZEN` = 0).
//! * **U5 reset values.** `CR`, `ICSCR1`…`ICSCR3`, the `PLLxDIVR` words and
//!   `CSR` have their manual/HAL reset values; the `MSICALx`/`HSICAL` factory
//!   trims read zero, and the `xxxENR`/`xxxSMENR` words reset to **zero** for
//!   the reason given for the `*LPENR` above, though RM0456 has some SRAM and
//!   flash interface clocks on out of reset. A board that gates its SRAM on
//!   one of those bits would see it off until firmware sets it.
//! * The U5's `PLL1FRACN` is used live while `PLL1FRACEN` stands; the
//!   hardware latches it on `FRACEN`'s rising edge, which the HAL's
//!   disable–write–enable sequence cannot tell apart. `BDCR.LSIPREDIV`
//!   divides the LSI on its way to `RTCCLK` only, and `MSIPLLEN` (MSI
//!   trimmed by the LSE) changes nothing because the MSI is already exact.
//! * **WB prescalers apply at once; only their flags lag.** The rates follow
//!   `HPRE`, `PPREx`, `C2HPRE` and `SHDHPRE` the moment they are written, and
//!   the `xxxF` flag is what takes the tick. A firmware that read the clock
//!   tree back between the write and the flag could tell; one that waits on
//!   the flag, as the HAL does, cannot.
//! * **WB secondaries.** `PLLSAI1`, `HSI48`, `HSIKERON`, `CCIPR`, `SMPSCR`,
//!   `RFWKPSEL` and `HSECR`'s tuning drive no output — the SMPS step-down
//!   clock and the RF wake-up clock read back and nothing consumes them.
//!   `LSI1` and `LSI2` are one rate. `EXTCFGR.RFCSS` (0: the radio's HCLK5
//!   and APB3 run on HSI16; 1: on HSE ÷ 2) reads zero, because the radio
//!   that would switch it is not modelled. The factory trims (`MSICAL`,
//!   `HSICAL`, `HSI48CAL`) read zero where the manual has `X`.
//! * **The SVD is overruled twice.** `AHB1ENR` resets to `0` (§6.4.17), not
//!   the SVD's `0x100`, and `C2AHB1SMENR` to `0x0001_1007` (§6.4.42), not
//!   `0x0001_1207`.
//! * **CPU2's gates are wires and nothing more.** A peripheral is clocked
//!   when *either* CPU's enable is set, which a board expresses by wiring
//!   both bits to the peripheral's `enable` (the [`gate`](super::gate) fan-in
//!   is an OR). This device does not compute the OR itself.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::clock::{ClockControl, DomainId, Rational};
use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema, port_index};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.rcc";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
///
/// Version 2 widened the register file to the U5's seventy words and the
/// ready-bit table to twelve deadlines; version 3 widened it again to the
/// WB's ninety-eight and added the prescaler-flag deadline. An older chunk is
/// refused rather than guessed at.
const STATE_VERSION: u32 = 3;

/// How many 32-bit words the widest layout occupies. The WB map ends at
/// `C2APB3SMENR` (`+0x184`), so ninety-eight words covers every family and
/// the snapshot encoding is one shape whatever the variant.
const WORDS: usize = 0x62;

/// How many `xxxON`/`xxxRDY` pairs the widest layout has — the U5's twelve.
const MAX_READY: usize = 12;

/// "No deadline": this ready bit is not waiting on anything.
const NO_DEADLINE: u64 = u64::MAX;

/// The default startup delay, in ticks of this device's own clock domain.
///
/// Sixteen ticks is two microseconds of an 8 MHz can — the order of a PLL lock
/// and several orders below a real crystal's millisecond startup. It is
/// deliberately short: a board that wants the crystal's real figure writes
/// `ready-delay` and gets it, and firmware that polls a ready bit cannot tell
/// the difference except in how much virtual time it spends waiting.
const DEFAULT_READY_DELAY: u64 = 16;

/// The name of the `PWR_CR.DBP` input.
pub const DBP_PIN: &str = "dbp";

/// The name of a WB's `PWR_CR5.SMPSEN` input, which `SMPSCR.SMPSSWS`
/// depends on (RM0434 §6.4.9).
pub const SMPSEN_PIN: &str = "smpsen";

/// The name of the RTC clock-enable output (`BDCR.RTCEN`).
pub const RTCEN_PIN: &str = "rtcen";

/// The name of the backup-domain reset output (`BDCR.BDRST`).
pub const BDRST_PIN: &str = "bdrst";

/// The name of the clock-interrupt output — position 5 on an F4's NVIC.
pub const IRQ_PIN: &str = "irq";

/// How wide a gate bank is. Every `xxENR`/`xxRSTR` is one bit per peripheral.
pub const BANK_WIDTH: u32 = 32;

// ---------------------------------------------------------------------------
// Clock outputs
// ---------------------------------------------------------------------------

/// Which output of the clock tree a consumer is asking about.
///
/// An open id space rather than an enum, the `pktkit` `EtherType` pattern
/// (`CLAUDE.md`, *Type conventions*): an H7 has clock outputs an F4 does not,
/// and adding one must not break a `match` somewhere else in the tree.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ClockOutput(pub u16);

impl ClockOutput {
    /// The system clock, after `CFGR.SW`/`SWS` has chosen its source.
    pub const SYSCLK: ClockOutput = ClockOutput(0);
    /// The AHB clock: `SYSCLK` divided by `CFGR.HPRE`.
    pub const HCLK: ClockOutput = ClockOutput(1);
    /// The APB1 (low-speed) peripheral clock: `HCLK / PPRE1`.
    pub const PCLK1: ClockOutput = ClockOutput(2);
    /// The APB2 (high-speed) peripheral clock: `HCLK / PPRE2`.
    pub const PCLK2: ClockOutput = ClockOutput(3);
    /// What an APB1 timer counts: `PCLK1`, doubled unless `PPRE1` is one.
    pub const TIMCLK1: ClockOutput = ClockOutput(4);
    /// What an APB2 timer counts: `PCLK2`, doubled unless `PPRE2` is one.
    pub const TIMCLK2: ClockOutput = ClockOutput(5);
    /// What `BDCR.RTCSEL` selects for the RTC, zero when nothing is selected.
    pub const RTCCLK: ClockOutput = ClockOutput(6);
    /// The PLL's `Q` output — 48 MHz for USB and the SDIO on a configured F4.
    pub const PLL48: ClockOutput = ClockOutput(7);
    /// The APB3 clock: `HCLK / CFGR3.PPRE3`. A U5 has one; on an F4, an L4
    /// or a WB it is always zero.
    pub const PCLK3: ClockOutput = ClockOutput(8);
    /// A WB's CPU2 clock, `HCLK2`: `SYSCLK / EXTCFGR.C2HPRE`. Zero on every
    /// other family.
    pub const HCLK2: ClockOutput = ClockOutput(9);
    /// A WB's shared AHB4 and flash clock, `HCLK4`: `SYSCLK /
    /// EXTCFGR.SHDHPRE`. Zero on every other family.
    pub const HCLK4: ClockOutput = ClockOutput(10);

    /// How many outputs there are, which is the width of a [`Clocks`] table.
    pub const COUNT: usize = 11;

    /// The name this output is known by, for a diagnostic.
    #[must_use]
    pub fn name(self) -> Option<&'static str> {
        Some(match self {
            ClockOutput::SYSCLK => "sysclk",
            ClockOutput::HCLK => "hclk",
            ClockOutput::PCLK1 => "pclk1",
            ClockOutput::PCLK2 => "pclk2",
            ClockOutput::TIMCLK1 => "timclk1",
            ClockOutput::TIMCLK2 => "timclk2",
            ClockOutput::RTCCLK => "rtcclk",
            ClockOutput::PLL48 => "pll48",
            ClockOutput::PCLK3 => "pclk3",
            ClockOutput::HCLK2 => "hclk2",
            ClockOutput::HCLK4 => "hclk4",
            _ => return None,
        })
    }
}

impl fmt::Display for ClockOutput {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "clock output #{}", self.0),
        }
    }
}

/// The clock outputs a machine file may hand `st.rcc` a domain for, each with
/// the output it hangs off — RM0090 §7.2's tree, in the order it is walked.
///
/// A property of this name takes the *object* whose clock domain is that
/// output; `None` for a parent means the crystal itself, which is the `hse`
/// property. Declaration order is top-down, so a single pass over it rates
/// every output against something already settled.
pub const OUTPUT_TREE: &[(ClockOutput, &str)] = &[
    (ClockOutput::SYSCLK, "sysclk"),
    (ClockOutput::HCLK, "hclk"),
    (ClockOutput::PCLK1, "pclk1"),
    (ClockOutput::PCLK2, "pclk2"),
    (ClockOutput::TIMCLK1, "timclk1"),
    (ClockOutput::TIMCLK2, "timclk2"),
    (ClockOutput::PCLK3, "pclk3"),
    (ClockOutput::HCLK2, "hclk2"),
    (ClockOutput::HCLK4, "hclk4"),
];

/// Which output `out` hangs off, or `None` when it hangs off the crystal.
fn parent_output(out: ClockOutput) -> Option<ClockOutput> {
    Some(match out {
        ClockOutput::HCLK | ClockOutput::HCLK2 | ClockOutput::HCLK4 => ClockOutput::SYSCLK,
        ClockOutput::PCLK1 | ClockOutput::PCLK2 | ClockOutput::PCLK3 => ClockOutput::HCLK,
        ClockOutput::TIMCLK1 => ClockOutput::PCLK1,
        ClockOutput::TIMCLK2 => ClockOutput::PCLK2,
        _ => return None,
    })
}

/// The rates an `st.rcc` is driving, as a consumer sees them.
///
/// Published as [`ExportId::CLOCK_TREE`]. A peripheral holds one of these from
/// bind time onwards — it is **wiring, not guest state**, so it is never
/// serialized and it survives reset.
///
/// Rates are exact [`Rational`] hertz. A stopped or unselected output reads as
/// zero, which is the honest answer for "the guest has not switched this on"
/// and is distinguishable from any real frequency.
///
/// [`Clocks::generation`] counts changes. A consumer that caches a derived
/// number — a baud divisor, a prescaler reload — compares against the
/// generation it cached at and recomputes when it differs; that is cheaper
/// than recomputing per access and it cannot go stale.
#[derive(Debug)]
pub struct Clocks {
    rates: Mutex<[Rational; ClockOutput::COUNT]>,
    generation: AtomicU64,
}

impl Clocks {
    /// A tree with every output stopped.
    fn new() -> Clocks {
        Clocks {
            rates: Mutex::with_rank(LockRank::LEAF, [Rational::integer(0); ClockOutput::COUNT]),
            generation: AtomicU64::new(0),
        }
    }

    /// The exact rate of `out`, in hertz. Zero when that output is stopped.
    #[must_use]
    pub fn rate(&self, out: ClockOutput) -> Rational {
        self.rates
            .lock()
            .get(usize::from(out.0))
            .copied()
            .unwrap_or(Rational::integer(0))
    }

    /// How many times any rate has changed since the machine was built.
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// Publish a recomputed table, bumping the generation if anything moved.
    ///
    /// Returns whether anything did, so a caller with more to do about a change
    /// — re-rating the scheduler's domains — does it only when there is one.
    fn publish(&self, next: [Rational; ClockOutput::COUNT]) -> bool {
        {
            let mut rates = self.rates.lock();
            if *rates == next {
                return false;
            }
            *rates = next;
        }
        // Release, so a consumer that reads the generation and then the rates
        // cannot see the new number with the old table.
        self.generation.fetch_add(1, Ordering::Release);
        true
    }
}

// ---------------------------------------------------------------------------
// Variants and layout
// ---------------------------------------------------------------------------

/// Which family's register map this instance has.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variant {
    /// RM0090's F4 map: `PLLCFGR` at `+0x04`, `BDCR` at `+0x70`.
    F4,
    /// RM0351's L4 map: `ICSCR` at `+0x04`, `PLLCFGR` at `+0x0c`, `BDCR` at
    /// `+0x90`, and an MSI the F4 does not have.
    L4,
    /// RM0456's U5 map: `CFGR1`…`CFGR3`, three PLLs each with a `CFGR`, a
    /// `DIVR` and a `FRACR`, split `AHB2`/`APB1` enable words, an APB3, and
    /// the LSI moved into `BDCR` at `+0xf0`.
    U5,
    /// RM0434's STM32WB55 map: the L4's, plus `SMPSCR`, `HSECR`, `EXTCFGR`
    /// (CPU2's and the shared bus's prescalers) and CPU2's gate banks from
    /// `+0x148`.
    WB,
}

impl Variant {
    /// The spelling a machine file writes.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Variant::F4 => "f4",
            Variant::L4 => "l4",
            Variant::U5 => "u5",
            Variant::WB => "wb",
        }
    }

    /// How many bytes of the peripheral's kilobyte actually decode.
    #[must_use]
    pub fn register_bytes(self) -> u64 {
        match self {
            // Through `DCKCFGR` at `+0x8c` (RM0090 §7.3.23).
            Variant::F4 => 0x90,
            // Through `CCIPR2` at `+0x9c` (RM0351 §6.4.31).
            Variant::L4 => 0xa0,
            // Through `PRIVCFGR` at `+0x114` (RM0456 §11.8; ST CMSIS
            // `stm32u585xx.h`, `RCC_TypeDef`).
            Variant::U5 => 0x118,
            // Through `C2APB3SMENR` at `+0x184` (ST CMSIS `stm32wb55xx.h`,
            // `RCC_TypeDef`).
            Variant::WB => 0x188,
        }
    }

    fn layout(self) -> &'static Layout {
        match self {
            Variant::F4 => &F4,
            Variant::L4 => &L4,
            Variant::U5 => &U5,
            Variant::WB => &WB,
        }
    }

    /// The reset-cause pins this family offers.
    fn causes(self) -> &'static [ResetCause] {
        match self {
            Variant::F4 => F4_CAUSES,
            Variant::L4 => L4_CAUSES,
            // The WB's `CSR[31:25]` is the U5's, flag for flag
            // (`RCC_CSR_OBLRSTF_Pos` … `RCC_CSR_LPWRRSTF_Pos`).
            Variant::U5 | Variant::WB => U5_CAUSES,
        }
    }
}

/// Which oscillator or PLL a ready bit belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Osc {
    /// The internal high-speed RC — 16 MHz on every family.
    Hsi,
    /// The external high-speed crystal.
    Hse,
    /// The L4's multi-speed internal RC; the U5's `MSIS`, the one that can
    /// be the system clock.
    Msi,
    /// The main PLL.
    Pll,
    /// A secondary source modelled for its ready bit only — nothing consumes
    /// its output yet: `PLLI2S`/`PLLSAI` on an F4, `PLLSAI1`/`PLLSAI2` on an
    /// L4, and on a U5 `PLL2`, `PLL3`, `MSIK`, `HSI48`, `SHSI` and the
    /// `LSESYS` path; on a WB `PLLSAI1`, `HSI48` and the HSI kernel clock.
    Aux,
    /// The 32.768 kHz backup-domain crystal.
    Lse,
    /// The internal low-speed RC — a WB's `LSI1` and `LSI2` both.
    Lsi,
}

/// An `xxxON`/`xxxRDY` pair and what it switches on.
#[derive(Debug, Clone, Copy)]
struct ReadyBit {
    /// The register both bits live in, as a byte offset.
    reg: u64,
    /// The `xxxON` bit the guest writes.
    on: u32,
    /// The `xxxRDY` bit the hardware answers with.
    rdy: u32,
    /// This source's bit in the interrupt flag register — `CIR` on an F4,
    /// `CIFR` on an L4 or a U5. The enable and the clear live at the same
    /// position in their own words, which is what makes one number enough.
    /// [`NO_FLAG`] for a ready bit with no interrupt (the U5's `LSESYSRDY`).
    flag: u32,
    /// Which source it starts.
    osc: Osc,
}

/// Where a layout keeps its clock-interrupt flags, enables and clears.
///
/// The two families draw the same logic differently: an F4 packs all three
/// into one register at three offsets from the flag's own bit, and an L4 gives
/// each its own word. The *bit positions* are a per-family table either way,
/// which is what [`ReadyBit::flag`] carries.
#[derive(Debug, Clone, Copy)]
enum Interrupts {
    /// `RCC_CIR` (RM0090 §7.3.5): flags in `[7:0]`, enables in `[14:8]`,
    /// write-one-to-clear in `[23:16]`.
    One { cir: u64 },
    /// `RCC_CIER`/`CIFR`/`CICR` (RM0351 §6.4.5–§6.4.7, RM0456 §11.8.18–20):
    /// the same bit in three registers, the flag word read-only and the clear
    /// word write-one. `flags` is which bits of `CIFR` exist.
    Three {
        cier: u64,
        cifr: u64,
        cicr: u64,
        flags: u32,
    },
}

/// A ready bit with no interrupt flag behind it.
const NO_FLAG: u32 = u32::MAX;

/// How far above a flag its enable sits in an F4's `CIR`.
const F4_IE_SHIFT: u32 = 8;

/// How far above a flag its write-one-to-clear bit sits in an F4's `CIR`.
const F4_CLEAR_SHIFT: u32 = 16;

/// The flag bits an F4's `CIR` has: seven ready flags plus `CSSF` at 7.
const F4_FLAGS: u32 = 0xff;

/// The enable bits an F4's `CIR` has: one per ready flag, and none for `CSSF`
/// — a clock security failure is an NMI, not a maskable request.
const F4_ENABLES: u32 = 0x7f << F4_IE_SHIFT;

/// The flag bits an L4's `CIFR` has: eight ready flags, `CSSF` at 8 and
/// `LSECSSF` at 9, plus `HSI48RDYF` at 10 on a part that has an HSI48.
const L4_FLAGS: u32 = 0x7ff;

/// The flag bits a U5's `CIFR` has: `LSIRDYF`…`PLL3RDYF` in `[8:0]`, `CSSF`
/// at 10, `MSIKRDYF` at 11 and `SHSIRDYF` at 12 (ST CMSIS `stm32u585xx.h`,
/// `RCC_CIFR_*`). Bit 9 is reserved.
const U5_FLAGS: u32 = 0x1dff;

impl Interrupts {
    /// Raise `flag`, if the guest asked to be told.
    ///
    /// "Set by hardware when the … clock becomes stable **and** `xxRDYIE` is
    /// set" (RM0090 §7.3.5, RM0351 §6.4.6): the flag is not a record of the
    /// event, it is a record of a *requested* event, which is why a firmware
    /// that polls `CR` never sees one of these appear behind it.
    ///
    /// Returns whether anything moved.
    fn raise(self, state: &mut State, flag: u32) -> bool {
        if flag == NO_FLAG {
            return false;
        }
        let (enables, flags_reg, shift) = match self {
            Interrupts::One { cir } => (state.word(cir), cir, F4_IE_SHIFT),
            Interrupts::Three { cier, cifr, .. } => (state.word(cier), cifr, 0),
        };
        if enables & (1 << (flag + shift)) == 0 {
            return false;
        }
        let word = state.word_mut(flags_reg);
        let before = *word;
        *word |= 1 << flag;
        *word != before
    }

    /// Whether a flag and its enable are both standing.
    fn pending(self, words: &[u32; WORDS]) -> bool {
        match self {
            Interrupts::One { cir } => {
                let value = words[(cir / 4) as usize];
                value & F4_FLAGS & (value >> F4_IE_SHIFT) != 0
            }
            Interrupts::Three {
                cier, cifr, flags, ..
            } => words[(cifr / 4) as usize] & words[(cier / 4) as usize] & flags != 0,
        }
    }

    /// What a read of `offset` answers, or `None` if it is not one of these
    /// registers.
    fn read(self, state: &State, offset: u64) -> Option<u32> {
        match self {
            // The clear bits are write-only and read back as zero.
            Interrupts::One { cir } if offset == cir => {
                Some(state.word(cir) & (F4_FLAGS | F4_ENABLES))
            }
            Interrupts::Three { cicr, .. } if offset == cicr => Some(0),
            _ => None,
        }
    }

    /// Apply a write to `offset`, and say whether it was one of these
    /// registers.
    fn write(self, state: &mut State, offset: u64, value: u32) -> bool {
        match self {
            Interrupts::One { cir } if offset == cir => {
                // Flags are read-only, the enables are ordinary bits, and a
                // one in `[23:16]` clears the flag below it.
                let cleared = (value >> F4_CLEAR_SHIFT) & F4_FLAGS;
                let kept = state.word(cir) & F4_FLAGS & !cleared;
                *state.word_mut(cir) = kept | (value & F4_ENABLES);
                true
            }
            Interrupts::Three { cifr, .. } if offset == cifr => {
                // `CIFR` is read-only: only the hardware and `CICR` move it.
                true
            }
            Interrupts::Three {
                cifr, cicr, flags, ..
            } if offset == cicr => {
                *state.word_mut(cifr) &= !(value & flags);
                // `CICR` holds nothing of its own.
                *state.word_mut(cicr) = 0;
                true
            }
            _ => false,
        }
    }
}

/// One bank of peripheral gate bits: a register and the pin prefix a machine
/// file wires it by.
#[derive(Debug, Clone, Copy)]
struct Bank {
    /// The pin prefix — `ahb1en0` … `ahb1en31`.
    prefix: &'static str,
    /// The register's byte offset.
    reg: u64,
}

/// A `CSR` reset flag and the input pin that latches it.
#[derive(Debug, Clone, Copy)]
struct ResetCause {
    pin: &'static str,
    bit: u32,
}

/// Everything that differs between the two families.
#[derive(Debug)]
struct Layout {
    cr: u64,
    cfgr: u64,
    pllcfgr: u64,
    bdcr: u64,
    csr: u64,
    /// `RMVF`'s bit in `CSR` — 24 on an F4, 23 on an L4, a U5 or a WB.
    rmvf: u32,
    /// The clock-enable banks, in register order.
    enable: &'static [Bank],
    /// The peripheral-reset banks, in register order.
    reset: &'static [Bank],
    /// Every `xxxON`/`xxxRDY` pair.
    ready: &'static [ReadyBit],
    /// Where the clock-interrupt flags, enables and clears live.
    interrupts: Interrupts,
    /// Reset values, as `(offset, value)`. Everything unnamed resets to zero.
    reset_values: &'static [(u64, u32)],
}

/// Every gate-pin prefix either layout uses, so the schema can declare the
/// union.
///
/// `apb1en` is `APB1ENR` on an F4 and `APB1ENR1` on an L4 or a U5; `apb1enb`
/// is the **second** APB1 word and an F4 has none. A U5 splits `AHB2` the same
/// way — `ahb2en` is `AHB2ENR1`, `ahb2enb` is `AHB2ENR2` — and adds an APB3
/// (`apb3en`, `apb3rst`). A `connect` to a bank the variant does not have is
/// refused by name.
///
/// The second word is `apb1enb` rather than `apb1en2` because a bank pin is a
/// prefix followed by decimal digits, and `apb1en21` would then be two pins
/// with one spelling — `APB1ENR1` bit 21 and `APB1ENR2` bit 1. A letter cannot
/// be read as a digit, so the ambiguity cannot arise.
///
/// A WB adds CPU2's enable words as `c2ahb1en` … `c2apb3en`. Every CPU1
/// prefix starts with an `a`, so a `c2…` pin can never be read as one and
/// `c2ahb1en3` and `ahb1en3` cannot be confused.
const ALL_BANKS: &[&str] = &[
    "ahb1en",
    "ahb2en",
    "ahb2enb",
    "ahb3en",
    "apb1en",
    "apb1enb",
    "apb2en",
    "apb3en",
    "ahb1rst",
    "ahb2rst",
    "ahb2rstb",
    "ahb3rst",
    "apb1rst",
    "apb1rstb",
    "apb2rst",
    "apb3rst",
    "c2ahb1en",
    "c2ahb2en",
    "c2ahb3en",
    "c2apb1en",
    "c2apb1enb",
    "c2apb2en",
    "c2apb3en",
];

/// Every reset-cause pin either layout offers, for the schema's union.
const ALL_CAUSES: &[&str] = &[
    "borrst", "pinrst", "porrst", "sftrst", "iwdgrst", "wwdgrst", "lpwrrst", "fwrst", "oblrst",
];

// -- the F4 map (RM0090 §7.3) ------------------------------------------------

static F4_ENABLE: &[Bank] = &[
    Bank {
        prefix: "ahb1en",
        reg: 0x30,
    },
    Bank {
        prefix: "ahb2en",
        reg: 0x34,
    },
    Bank {
        prefix: "ahb3en",
        reg: 0x38,
    },
    Bank {
        prefix: "apb1en",
        reg: 0x40,
    },
    Bank {
        prefix: "apb2en",
        reg: 0x44,
    },
];

static F4_RESET_BANKS: &[Bank] = &[
    Bank {
        prefix: "ahb1rst",
        reg: 0x10,
    },
    Bank {
        prefix: "ahb2rst",
        reg: 0x14,
    },
    Bank {
        prefix: "ahb3rst",
        reg: 0x18,
    },
    Bank {
        prefix: "apb1rst",
        reg: 0x20,
    },
    Bank {
        prefix: "apb2rst",
        reg: 0x24,
    },
];

static F4_READY: &[ReadyBit] = &[
    ReadyBit {
        reg: 0x00,
        on: 0,
        rdy: 1,
        flag: 2,
        osc: Osc::Hsi,
    },
    ReadyBit {
        reg: 0x00,
        on: 16,
        rdy: 17,
        flag: 3,
        osc: Osc::Hse,
    },
    ReadyBit {
        reg: 0x00,
        on: 24,
        rdy: 25,
        flag: 4,
        osc: Osc::Pll,
    },
    // `PLLI2S`, then `PLLSAI` — the second exists only on an F42x/F43x and is
    // harmless on an F407, where the bit is reserved and firmware never sets
    // it.
    ReadyBit {
        reg: 0x00,
        on: 26,
        rdy: 27,
        flag: 5,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x00,
        on: 28,
        rdy: 29,
        flag: 6,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x70,
        on: 0,
        rdy: 1,
        flag: 1,
        osc: Osc::Lse,
    },
    ReadyBit {
        reg: 0x74,
        on: 0,
        rdy: 1,
        flag: 0,
        osc: Osc::Lsi,
    },
];

static F4_RESET_VALUES: &[(u64, u32)] = &[
    // `CR`: HSI on and, once it is ready, HSIRDY — the chip runs from HSI out
    // of reset. HSITRIM defaults to 16, which is the `0x80` (RM0090 §7.3.1,
    // "Reset value: 0x0000 XX83").
    (0x00, 0x0000_0083),
    // `PLLCFGR`: M = 16, N = 192, P = /2, Q = 4 (§7.3.2).
    (0x04, 0x2400_3010),
    // `AHB1ENR`: the CCM data RAM's clock is on out of reset (§7.3.10).
    (0x30, 0x0010_0000),
    // `CSR`: a power-on sets the BOR, pin and POR reset flags (§7.3.21).
    (0x74, 0x0e00_0000),
];

static F4_CAUSES: &[ResetCause] = &[
    ResetCause {
        pin: "borrst",
        bit: 25,
    },
    ResetCause {
        pin: "pinrst",
        bit: 26,
    },
    ResetCause {
        pin: "porrst",
        bit: 27,
    },
    ResetCause {
        pin: "sftrst",
        bit: 28,
    },
    ResetCause {
        pin: "iwdgrst",
        bit: 29,
    },
    ResetCause {
        pin: "wwdgrst",
        bit: 30,
    },
    ResetCause {
        pin: "lpwrrst",
        bit: 31,
    },
];

static F4: Layout = Layout {
    cr: 0x00,
    cfgr: 0x08,
    pllcfgr: 0x04,
    bdcr: 0x70,
    csr: 0x74,
    rmvf: 24,
    enable: F4_ENABLE,
    reset: F4_RESET_BANKS,
    ready: F4_READY,
    interrupts: Interrupts::One { cir: 0x0c },
    reset_values: F4_RESET_VALUES,
};

// -- the L4 map (RM0351 §6.4) ------------------------------------------------

static L4_ENABLE: &[Bank] = &[
    Bank {
        prefix: "ahb1en",
        reg: 0x48,
    },
    Bank {
        prefix: "ahb2en",
        reg: 0x4c,
    },
    Bank {
        prefix: "ahb3en",
        reg: 0x50,
    },
    Bank {
        prefix: "apb1en",
        reg: 0x58,
    },
    Bank {
        prefix: "apb1enb",
        reg: 0x5c,
    },
    Bank {
        prefix: "apb2en",
        reg: 0x60,
    },
];

static L4_RESET_BANKS: &[Bank] = &[
    Bank {
        prefix: "ahb1rst",
        reg: 0x28,
    },
    Bank {
        prefix: "ahb2rst",
        reg: 0x2c,
    },
    Bank {
        prefix: "ahb3rst",
        reg: 0x30,
    },
    Bank {
        prefix: "apb1rst",
        reg: 0x38,
    },
    Bank {
        prefix: "apb1rstb",
        reg: 0x3c,
    },
    Bank {
        prefix: "apb2rst",
        reg: 0x40,
    },
];

static L4_READY: &[ReadyBit] = &[
    ReadyBit {
        reg: 0x00,
        on: 0,
        rdy: 1,
        flag: 2,
        osc: Osc::Msi,
    },
    // The L4's `HSION` is bit 8 and its `HSIRDY` is bit **10**, not 9 — bit 9
    // is `HSIKERON` (RM0351 §6.4.1). The pair is a table for exactly this.
    ReadyBit {
        reg: 0x00,
        on: 8,
        rdy: 10,
        flag: 3,
        osc: Osc::Hsi,
    },
    ReadyBit {
        reg: 0x00,
        on: 16,
        rdy: 17,
        flag: 4,
        osc: Osc::Hse,
    },
    ReadyBit {
        reg: 0x00,
        on: 24,
        rdy: 25,
        flag: 5,
        osc: Osc::Pll,
    },
    ReadyBit {
        reg: 0x00,
        on: 26,
        rdy: 27,
        flag: 6,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x00,
        on: 28,
        rdy: 29,
        flag: 7,
        osc: Osc::Aux,
    },
    ReadyBit {
        reg: 0x90,
        on: 0,
        rdy: 1,
        flag: 1,
        osc: Osc::Lse,
    },
    ReadyBit {
        reg: 0x94,
        on: 0,
        rdy: 1,
        flag: 0,
        osc: Osc::Lsi,
    },
];

static L4_RESET_VALUES: &[(u64, u32)] = &[
    // `CR`: MSI on at range 6, which is 4 MHz — the L4 boots on the MSI, not
    // on an HSI (RM0351 §6.4.1, "Reset value: 0x0000 0063").
    (0x00, 0x0000_0063),
    // `PLLCFGR` and the two SAI PLLs: N = 16, everything else zero (§6.4.4).
    (0x0c, 0x0000_1000),
    (0x10, 0x0000_1000),
    (0x14, 0x0000_1000),
    // `CSR`: MSISRANGE = 4 MHz, and a power-on sets the BOR and pin flags
    // (§6.4.29).
    (0x94, 0x0c00_0600),
];

static L4_CAUSES: &[ResetCause] = &[
    ResetCause {
        pin: "fwrst",
        bit: 24,
    },
    ResetCause {
        pin: "oblrst",
        bit: 25,
    },
    ResetCause {
        pin: "pinrst",
        bit: 26,
    },
    ResetCause {
        pin: "borrst",
        bit: 27,
    },
    ResetCause {
        pin: "sftrst",
        bit: 28,
    },
    ResetCause {
        pin: "iwdgrst",
        bit: 29,
    },
    ResetCause {
        pin: "wwdgrst",
        bit: 30,
    },
    ResetCause {
        pin: "lpwrrst",
        bit: 31,
    },
];

static L4: Layout = Layout {
    cr: 0x00,
    cfgr: 0x08,
    pllcfgr: 0x0c,
    bdcr: 0x90,
    csr: 0x94,
    rmvf: 23,
    enable: L4_ENABLE,
    reset: L4_RESET_BANKS,
    ready: L4_READY,
    interrupts: Interrupts::Three {
        cier: 0x18,
        cifr: 0x1c,
        cicr: 0x20,
        flags: L4_FLAGS,
    },
    reset_values: L4_RESET_VALUES,
};

// -- the U5 map (RM0456 §11.8; ST CMSIS `stm32u585xx.h`, `RCC_TypeDef`) -------

/// A bank, spelled the way the tables below need it.
const fn bank(prefix: &'static str, reg: u64) -> Bank {
    Bank { prefix, reg }
}

/// A ready bit, likewise.
const fn ready(reg: u64, on: u32, rdy: u32, flag: u32, osc: Osc) -> ReadyBit {
    ReadyBit {
        reg,
        on,
        rdy,
        flag,
        osc,
    }
}

/// `RCC_CR` on a U5.
const U5_CR: u64 = 0x00;
/// `RCC_ICSCR1`: `MSIRGSEL` and the two MSI range fields.
const U5_ICSCR1: u64 = 0x08;
/// `RCC_CFGR1`: `SW`/`SWS`, the same two fields at the same bits as an L4's
/// `CFGR`.
const U5_CFGR1: u64 = 0x1c;
/// `RCC_CFGR2`: `HPRE[3:0]`, `PPRE1[6:4]`, `PPRE2[10:8]`.
const U5_CFGR2: u64 = 0x20;
/// `RCC_CFGR3`: `PPRE3[6:4]`.
const U5_CFGR3: u64 = 0x24;
/// `RCC_PLL1CFGR`: `PLL1SRC[1:0]`, `PLL1FRACEN[4]`, `PLL1M[11:8]`,
/// `PLL1MBOOST[15:12]`, `PLL1PEN/QEN/REN[18:16]`.
const U5_PLL1CFGR: u64 = 0x28;
/// `RCC_PLL1DIVR`: `PLL1N[8:0]`, `PLL1P[15:9]`, `PLL1Q[22:16]`,
/// `PLL1R[30:24]`, each "value + 1".
const U5_PLL1DIVR: u64 = 0x34;
/// `RCC_PLL1FRACR`: `PLL1FRACN[15:3]`.
const U5_PLL1FRACR: u64 = 0x38;
/// `RCC_BDCR`, with the LSI's enable in it.
const U5_BDCR: u64 = 0xf0;
/// `RCC_CSR`.
const U5_CSR: u64 = 0xf4;

static U5_ENABLE: &[Bank] = &[
    bank("ahb1en", 0x88),
    bank("ahb2en", 0x8c),
    bank("ahb2enb", 0x90),
    bank("ahb3en", 0x94),
    bank("apb1en", 0x9c),
    bank("apb1enb", 0xa0),
    bank("apb2en", 0xa4),
    bank("apb3en", 0xa8),
];

static U5_RESET_BANKS: &[Bank] = &[
    bank("ahb1rst", 0x60),
    bank("ahb2rst", 0x64),
    bank("ahb2rstb", 0x68),
    bank("ahb3rst", 0x6c),
    bank("apb1rst", 0x74),
    bank("apb1rstb", 0x78),
    bank("apb2rst", 0x7c),
    bank("apb3rst", 0x80),
];

/// Every `xxxON`/`xxxRDY` pair `HAL_RCC_OscConfig` and
/// `HAL_RCCEx_PeriphCLKConfig` poll, with its `CIFR` flag (`RCC_CR_*`,
/// `RCC_BDCR_*`, `RCC_CIFR_*` in `stm32u585xx.h`).
static U5_READY: &[ReadyBit] = &[
    ready(U5_CR, 0, 2, 2, Osc::Msi),
    // `MSIKON`/`MSIKRDY`: the kernel-clock MSI, which nothing here consumes.
    ready(U5_CR, 4, 5, 11, Osc::Aux),
    ready(U5_CR, 8, 10, 3, Osc::Hsi),
    ready(U5_CR, 12, 13, 5, Osc::Aux),
    // `SHSION`/`SHSIRDY`, the secure HSI for SAES (U585 only; reserved and
    // harmless on a U575).
    ready(U5_CR, 14, 15, 12, Osc::Aux),
    ready(U5_CR, 16, 17, 4, Osc::Hse),
    ready(U5_CR, 24, 25, 6, Osc::Pll),
    ready(U5_CR, 26, 27, 7, Osc::Aux),
    ready(U5_CR, 28, 29, 8, Osc::Aux),
    ready(U5_BDCR, 0, 1, 1, Osc::Lse),
    // `LSESYSEN`/`LSESYSRDY`: the LSE propagated to the system, with no
    // interrupt of its own.
    ready(U5_BDCR, 7, 11, NO_FLAG, Osc::Aux),
    // The U5 keeps `LSION`/`LSIRDY` in `BDCR`, not in `CSR`.
    ready(U5_BDCR, 26, 27, 0, Osc::Lsi),
];

static U5_RESET_VALUES: &[(u64, u32)] = &[
    // `CR`: `MSISON`/`MSISRDY` and `MSIKON`/`MSIKRDY` — the U5 boots on the
    // 4 MHz MSIS with the kernel MSI running beside it (RM0456 §11.8,
    // `RCC_CR`, "Reset value: 0x0000 0035").
    (U5_CR, 0x0000_0035),
    // `ICSCR1`: `MSISRANGE` = `MSIKRANGE` = 4, i.e. 4 MHz, and `MSIRGSEL`
    // clear, so the range really comes from `CSR`. The `MSICALx`
    // factory trims read as zero.
    (U5_ICSCR1, 0x4400_0000),
    // `ICSCR2`: every `MSITRIMx` at its mid-scale 16 — the value
    // `HAL_RCC_DeInit` writes back as the default.
    (0x0c, 0x0008_4210),
    // `ICSCR3`: `HSITRIM` = 16, `RCC_HSICALIBRATION_DEFAULT`.
    (0x10, 0x0010_0000),
    // `PLLxDIVR`: N = 129, P = Q = R = 2 — `PLLDIVR_RESET_VALUE`.
    (U5_PLL1DIVR, 0x0101_0280),
    (0x3c, 0x0101_0280),
    (0x44, 0x0101_0280),
    // `CSR`: `MSIKSRANGE` = `MSISSRANGE` = 4 MHz, and a power-on sets the BOR
    // and pin flags.
    (U5_CSR, 0x0c00_4400),
];

static U5_CAUSES: &[ResetCause] = &[
    ResetCause {
        pin: "oblrst",
        bit: 25,
    },
    ResetCause {
        pin: "pinrst",
        bit: 26,
    },
    ResetCause {
        pin: "borrst",
        bit: 27,
    },
    ResetCause {
        pin: "sftrst",
        bit: 28,
    },
    ResetCause {
        pin: "iwdgrst",
        bit: 29,
    },
    ResetCause {
        pin: "wwdgrst",
        bit: 30,
    },
    ResetCause {
        pin: "lpwrrst",
        bit: 31,
    },
];

static U5: Layout = Layout {
    cr: U5_CR,
    cfgr: U5_CFGR1,
    pllcfgr: U5_PLL1CFGR,
    bdcr: U5_BDCR,
    csr: U5_CSR,
    rmvf: 23,
    enable: U5_ENABLE,
    reset: U5_RESET_BANKS,
    ready: U5_READY,
    interrupts: Interrupts::Three {
        cier: 0x50,
        cifr: 0x54,
        cicr: 0x58,
        flags: U5_FLAGS,
    },
    reset_values: U5_RESET_VALUES,
};

// -- the WB map (RM0434; ST CMSIS `stm32wb55xx.h`, `RCC_TypeDef`) ------------

/// `RCC_CR` on a WB: the L4's bits, plus `HSIKERDY` at 12 and `HSEPRE` at 20.
const WB_CR: u64 = 0x00;
/// `RCC_CFGR`: the L4's fields, plus the `HPREF`/`PPRE1F`/`PPRE2F` flags.
const WB_CFGR: u64 = 0x08;
/// `RCC_PLLCFGR`.
const WB_PLLCFGR: u64 = 0x0c;
/// `RCC_SMPSCR`: `SMPSSEL[1:0]`, `SMPSDIV[5:4]`, `SMPSSWS[9:8]`.
const WB_SMPSCR: u64 = 0x24;
/// `RCC_APB3RSTR`, whose only bit is the radio's `RFRST`.
const WB_APB3RSTR: u64 = 0x44;
/// `RCC_BDCR`, at the L4's offset.
const WB_BDCR: u64 = 0x90;
/// `RCC_CSR`: `LSI1`/`LSI2`, `RFWKPSEL`, `RFRSTS` and the reset flags.
const WB_CSR: u64 = 0x94;
/// `RCC_CRRCR`: `HSI48ON`/`HSI48RDY`.
const WB_CRRCR: u64 = 0x98;
/// `RCC_HSECR`, behind its key.
const WB_HSECR: u64 = 0x9c;
/// `RCC_EXTCFGR`: `SHDHPRE[3:0]`, `C2HPRE[7:4]` and their applied flags.
const WB_EXTCFGR: u64 = 0x108;

/// `CR.HSEPRE`: HSE ÷ 2 into `SYSCLK` and the PLL.
const WB_CR_HSEPRE: u32 = 1 << 20;
/// `CFGR.HPREF`, `PPRE1F`, `PPRE2F` — "prescaler value applied".
const WB_CFGR_FLAGS: u32 = 0b111 << 16;
/// `EXTCFGR.SHDHPREF`, `C2HPREF`.
const WB_EXTCFGR_FLAGS: u32 = 0b11 << 16;
/// `EXTCFGR.RFCSS`, the radio's clock source, which the hardware reports.
const WB_EXTCFGR_RFCSS: u32 = 1 << 20;
/// `SMPSCR.SMPSSWS`, the step-down converter's clock switch status.
const WB_SMPSCR_SWS: u32 = 0b11 << 8;
/// `CSR.RFRSTS` — "the radio system is under reset".
const WB_CSR_RFRSTS: u32 = 1 << 16;
/// `HSECR.UNLOCKED`.
const WB_HSECR_UNLOCKED: u32 = 1;
/// `HSECR`'s writable fields once unlocked: `HSES[3]`, `HSEGMC[6:4]`,
/// `HSETUNE[13:8]`.
const WB_HSECR_FIELDS: u32 = 0x3f78;
/// The value that unlocks `HSECR` for one write (`HSE_CONTROL_UNLOCK_KEY` in
/// ST's LL header).
const WB_HSECR_KEY: u32 = 0xcafe_cafe;

/// The flag bits a WB's `CIFR` has: `LSI1RDYF`…`PLLSAI1RDYF` in `[6:0]`,
/// `CSSF` at 8, `LSECSSF` at 9, `HSI48RDYF` at 10 and `LSI2RDYF` at 11
/// (`RCC_CIFR_*` in `stm32wb55xx.h`). Bit 7 is reserved.
const WB_FLAGS: u32 = 0xf7f;

static WB_ENABLE: &[Bank] = &[
    bank("ahb1en", 0x48),
    bank("ahb2en", 0x4c),
    bank("ahb3en", 0x50),
    bank("apb1en", 0x58),
    bank("apb1enb", 0x5c),
    bank("apb2en", 0x60),
    // CPU2's own enables. They are enable banks like any other; a board wires
    // a peripheral's `enable` to both CPUs' bits and the gate ORs them.
    bank("c2ahb1en", 0x148),
    bank("c2ahb2en", 0x14c),
    bank("c2ahb3en", 0x150),
    bank("c2apb1en", 0x158),
    bank("c2apb1enb", 0x15c),
    bank("c2apb2en", 0x160),
    bank("c2apb3en", 0x164),
];

static WB_RESET_BANKS: &[Bank] = &[
    bank("ahb1rst", 0x28),
    bank("ahb2rst", 0x2c),
    bank("ahb3rst", 0x30),
    bank("apb1rst", 0x38),
    bank("apb1rstb", 0x3c),
    bank("apb2rst", 0x40),
    bank("apb3rst", WB_APB3RSTR),
];

/// Every `xxxON`/`xxxRDY` pair `HAL_RCC_OscConfig` polls, with its `CIFR`
/// flag (`RCC_CR_*`, `RCC_BDCR_*`, `RCC_CSR_*`, `RCC_CRRCR_*`,
/// `RCC_CIFR_*` in `stm32wb55xx.h`).
static WB_READY: &[ReadyBit] = &[
    ready(WB_CR, 0, 1, 2, Osc::Msi),
    // `HSION` at 8, `HSIRDY` at 10, exactly as on an L4.
    ready(WB_CR, 8, 10, 3, Osc::Hsi),
    // `HSIKERON`/`HSIKERDY`: the HSI kept running for peripheral kernels.
    ready(WB_CR, 9, 12, NO_FLAG, Osc::Aux),
    ready(WB_CR, 16, 17, 4, Osc::Hse),
    ready(WB_CR, 24, 25, 5, Osc::Pll),
    ready(WB_CR, 26, 27, 6, Osc::Aux),
    ready(WB_BDCR, 0, 1, 1, Osc::Lse),
    ready(WB_CSR, 0, 1, 0, Osc::Lsi),
    ready(WB_CSR, 2, 3, 11, Osc::Lsi),
    ready(WB_CRRCR, 0, 1, 10, Osc::Aux),
];

/// The WB's reset values: RM0434 §6.4.1–§6.4.48 ("Reset value") and its
/// Table 38, cross-checked against ST's `STM32WB55_CM4.svd` and what
/// `HAL_RCC_DeInit` writes back as the default.
static WB_RESET_VALUES: &[(u64, u32)] = &[
    // `CR`: MSI on at range 6 (4 MHz) and ready. RM0434 §6.4.1 gives
    // `0x61` and says `MSIRDY` "is read 1 once the MSI is ready"; this model
    // has no instant at which it is not.
    (WB_CR, 0x0000_0063),
    // `ICSCR`: `HSITRIM` = 0x40, the value `HAL_RCC_DeInit` restores; the
    // `MSICAL`/`HSICAL` factory trims read zero.
    (0x04, 0x4000_0000),
    // `CFGR`: MSI selected, every prescaler ÷1 and *applied*.
    (WB_CFGR, 0x0007_0000),
    // `PLLCFGR` and `PLLSAI1CFGR`: N = 1, P = ÷3, Q = R = ÷2.
    (WB_PLLCFGR, 0x2204_0100),
    (0x10, 0x2204_0100),
    // `SMPSCR`: MSI selected (§6.4.9: "forced by hardware to select MSI" on
    // a POR); `SMPSSWS` is derived on read, so its stored `11` is never seen.
    (WB_SMPSCR, 0x0000_0301),
    // `AHB3ENR`: HSEM (19) and FLASH (25) clocked out of reset.
    (0x50, 0x0208_0000),
    // `APB1ENR1`: `RTCAPBEN`.
    (0x58, 0x0000_0400),
    // CPU1's `*SMENR`: every implemented peripheral clocked in sleep.
    (0x68, 0x0001_1207),
    (0x6c, 0x0001_209f),
    (0x70, 0x0307_0100),
    (0x78, 0x85a0_4e01),
    (0x7c, 0x0000_0021),
    (0x80, 0x0026_5800),
    // `CSR`: a power-on sets the pin and BOR flags.
    (WB_CSR, 0x0c00_0000),
    // `HSECR`: locked, `HSEGMC` = 3.
    (WB_HSECR, 0x0000_0030),
    // `EXTCFGR`: `SHDHPRE` = `C2HPRE` = ÷1, both applied.
    (WB_EXTCFGR, 0x0003_0000),
    // CPU2's enables: HSEM, FLASH and the RTC's APB interface, as CPU1's.
    (0x150, 0x0208_0000),
    (0x158, 0x0000_0400),
    // CPU2's `*SMENR`.
    // RM0434 §6.4.42 gives `0x0001_1007`, `SRAM1SMEN` clear; the SVD's
    // `0x0001_1207` has it set. The manual is the one taken.
    (0x168, 0x0001_1007),
    (0x16c, 0x0001_209f),
    (0x170, 0x0307_0000),
    (0x178, 0x85a0_4601),
    (0x17c, 0x0000_0021),
    (0x180, 0x0026_5800),
    (0x184, 0x0000_0003),
];

static WB: Layout = Layout {
    cr: WB_CR,
    cfgr: WB_CFGR,
    pllcfgr: WB_PLLCFGR,
    bdcr: WB_BDCR,
    csr: WB_CSR,
    rmvf: 23,
    enable: WB_ENABLE,
    reset: WB_RESET_BANKS,
    ready: WB_READY,
    interrupts: Interrupts::Three {
        cier: 0x18,
        cifr: 0x1c,
        cicr: 0x20,
        flags: WB_FLAGS,
    },
    reset_values: WB_RESET_VALUES,
};

// ---------------------------------------------------------------------------
// The declared source frequencies
// ---------------------------------------------------------------------------

/// What the board says its oscillators run at.
///
/// The crystal on `OSC_IN` is a fact about the **board**, not about the part,
/// so it arrives as a property. It has to agree with the `osc` the machine
/// file declares for the same crystal, and nothing checks that today — see the
/// module documentation on the missing clock-control seam.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frequencies {
    /// The external high-speed crystal, in hertz.
    pub hse: u64,
    /// The internal high-speed RC — 16 MHz on both families.
    pub hsi: u64,
    /// The backup-domain crystal.
    pub lse: u64,
    /// The internal low-speed RC.
    pub lsi: u64,
}

impl Default for Frequencies {
    fn default() -> Frequencies {
        Frequencies {
            // The STM32F4 Discovery board's can, and the one
            // `machines/stm32f407.machine` declares.
            hse: 8_000_000,
            hsi: 16_000_000,
            lse: 32_768,
            lsi: 32_000,
        }
    }
}

/// The L4's MSI ranges, `CR.MSIRANGE` = 0…11 (RM0351 §6.4.1).
const MSI_RANGES: [u64; 12] = [
    100_000, 200_000, 400_000, 800_000, 1_000_000, 2_000_000, 4_000_000, 8_000_000, 16_000_000,
    24_000_000, 32_000_000, 48_000_000,
];

/// The U5's MSIS ranges, `MSISRANGE`/`MSISSRANGE` = 0…15, as exact
/// `(numerator, denominator)` hertz (RM0456 §11.4, the MSI clock).
///
/// Four RC oscillators of 48 MHz, 4 MHz, 3.072 MHz and 400 kHz, each divided
/// by 1, 2, 3 and 4. That makes ranges 6 and 14 *thirds* — 1.333… MHz and
/// 133.3… kHz — which `MSIRangeTable` in ST's `system_stm32u5xx.c` rounds to
/// 1 330 000 and 133 000. The hardware divides; it does not round, so neither
/// does this table.
const U5_MSI_RANGES: [(u64, u64); 16] = [
    (48_000_000, 1),
    (24_000_000, 1),
    (16_000_000, 1),
    (12_000_000, 1),
    (4_000_000, 1),
    (2_000_000, 1),
    (4_000_000, 3),
    (1_000_000, 1),
    (3_072_000, 1),
    (1_536_000, 1),
    (1_024_000, 1),
    (768_000, 1),
    (400_000, 1),
    (200_000, 1),
    (400_000, 3),
    (100_000, 1),
];

/// `CFGR.HPRE` as a divisor (RM0090 §7.3.3, RM0351 §6.4.3 — the same table).
fn hpre_div(field: u32) -> u64 {
    match field {
        0..=7 => 1,
        8 => 2,
        9 => 4,
        10 => 8,
        11 => 16,
        12 => 64,
        13 => 128,
        14 => 256,
        _ => 512,
    }
}

/// A WB's `HPRE`, `C2HPRE` and `SHDHPRE` as a divisor (RM0434 §6.4.3, §6.4.34;
/// the same table as `AHBPrescTable` in
/// ST's `system_stm32wbxx.c`; the `LL_RCC_SYSCLK_DIV_x` encodings in
/// `stm32wbxx_ll_rcc.h`).
///
/// The `1xxx` half is everyone's table; the `0xxx` half, which is ÷1 on an
/// F4 or an L4, has ÷3, ÷5, ÷6, ÷10 and ÷32 on a WB, so that a 64 MHz
/// `SYSCLK` can feed CPU2 its 32 MHz ceiling and the flash its own.
const WB_HPRE: [u64; 16] = [1, 3, 5, 1, 1, 6, 10, 32, 2, 4, 8, 16, 64, 128, 256, 512];

/// `CFGR.PPREx` as a divisor. `0xx` is one; otherwise `2^(field - 3)`.
fn ppre_div(field: u32) -> u64 {
    if field < 4 { 1 } else { 1 << (field - 3) }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, plus the device's own position.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The register file, indexed by `offset / 4`.
    words: [u32; WORDS],
    /// When each ready bit comes true, in this device's ticks; [`NO_DEADLINE`]
    /// for one that is not waiting.
    ready_at: [u64; MAX_READY],
    /// When `SWS` catches up with `SW`, and what it will then say.
    switch_at: u64,
    switch_to: u32,
    /// When a WB's cleared prescaler flags (`CFGR.HPREF`…, `EXTCFGR.C2HPREF`,
    /// `SHDHPREF`) come back. Never armed on any other family.
    presc_at: u64,
    /// The tick this device has been advanced to.
    tick: u64,
}

impl State {
    fn reset(layout: &Layout) -> State {
        let mut words = [0u32; WORDS];
        for (offset, value) in layout.reset_values {
            words[(*offset / 4) as usize] = *value;
        }
        State {
            words,
            ready_at: [NO_DEADLINE; MAX_READY],
            switch_at: NO_DEADLINE,
            switch_to: 0,
            presc_at: NO_DEADLINE,
            tick: 0,
        }
    }

    #[inline]
    fn word(&self, offset: u64) -> u32 {
        self.words[(offset / 4) as usize]
    }

    #[inline]
    fn word_mut(&mut self, offset: u64) -> &mut u32 {
        &mut self.words[(offset / 4) as usize]
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The output key of the `rtcen` pin, above every bank.
const KEY_RTCEN: u32 = 0xffff_0000;
/// The output key of the `bdrst` pin.
const KEY_BDRST: u32 = 0xffff_0001;
/// The output key of the `irq` pin.
const KEY_IRQ: u32 = 0xffff_0002;

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    variant: Variant,
    layout: &'static Layout,
    freq: Frequencies,
    ready_delay: u64,
    /// The published rates. Derived state; never serialized.
    clocks: Arc<Clocks>,
    /// The connected gate outputs, keyed by `bank index * 32 + bit`, plus the
    /// two named backup-domain pins at [`KEY_RTCEN`] and [`KEY_BDRST`].
    outputs: Mutex<Vec<(u32, WireSource)>>,
    /// `PWR_CR.DBP`, as it arrives on the `dbp` input. Lock-free because the
    /// `BDCR` write path samples it while holding the state lock, and reaching
    /// into `PWR` from there would be one device-rank lock inside another.
    dbp: AtomicU32,
    /// Whether anything is driving `dbp` at all.
    dbp_wired: AtomicU32,
    /// A WB's `PWR_CR5.SMPSEN`, as it arrives on the `smpsen` input. Low when
    /// nothing drives it, which is the SMPS's reset state.
    smpsen: AtomicU32,
    /// The lock-free half of the lazy contract: `current_tick` and
    /// `next_event_tick` are called at [`LockRank::LEAF`] and must not lock.
    tick: AtomicU64,
    next_event: AtomicU64,
    lazy: Mutex<Option<LazyHandle>>,
    /// Which object the machine file named for each driven clock output, in
    /// the order the chain is measured in. Wiring, fixed at construction.
    named: Vec<(ClockOutput, String)>,
    /// The domains those names resolved to, filled in at bind.
    domains: Mutex<Vec<(ClockOutput, DomainId)>>,
    /// The clock-control seam, or `None` for an instance nobody registered —
    /// a unit test holding the device directly.
    control: Mutex<Option<Arc<ClockControl>>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("variant", &self.variant)
            .field("freq", &self.freq)
            .field("ready-delay", &self.ready_delay);
        match self.state.try_lock() {
            Some(state) => s.field("tick", &state.tick),
            None => s.field("tick", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    // -- the lazy seam -----------------------------------------------------

    /// Catch the device up to the present before answering an access.
    ///
    /// Called with **no lock held**: the handle re-enters this device through
    /// [`Device::advance_to`], which takes the state lock.
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else { return };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        let _ = handle.sync(kind);
    }

    /// Republish what the lock-free lazy surface reads.
    fn republish(&self, state: &State) {
        self.tick.store(state.tick, Ordering::Relaxed);
        let mut next = state.switch_at.min(state.presc_at);
        for at in state.ready_at {
            next = next.min(at);
        }
        // The contract says a reported event must be strictly in the future,
        // or catch-up makes no progress and the device stalls where it stands.
        let next = if next <= state.tick { u64::MAX } else { next };
        self.next_event.store(next, Ordering::Relaxed);
    }

    /// Simulate forward to `tick`: ready bits come true, `SWS` catches up.
    fn advance_to(&self, tick: u64) {
        {
            let mut state = self.state.lock();
            if tick <= state.tick {
                return;
            }
            state.tick = tick;
            for (i, bit) in self.layout.ready.iter().enumerate() {
                if state.ready_at[i] <= tick {
                    state.ready_at[i] = NO_DEADLINE;
                    *state.word_mut(bit.reg) |= 1 << bit.rdy;
                    // The oscillator arriving is what raises the interrupt, so
                    // this is the one place it can happen: a ready bit is set
                    // by time passing and by nothing else.
                    self.layout.interrupts.raise(&mut state, bit.flag);
                }
            }
            if state.switch_at <= tick {
                state.switch_at = NO_DEADLINE;
                let sws = state.switch_to;
                let cfgr = state.word_mut(self.layout.cfgr);
                *cfgr = (*cfgr & !0b1100) | (sws << 2);
            }
            if state.presc_at <= tick {
                // "Set by hardware when the prescaler value is applied": every
                // flag a write cleared comes back together. Setting a flag
                // that was never cleared changes nothing.
                state.presc_at = NO_DEADLINE;
                *state.word_mut(WB_CFGR) |= WB_CFGR_FLAGS;
                *state.word_mut(WB_EXTCFGR) |= WB_EXTCFGR_FLAGS;
            }
            self.republish(&state);
        }
        // Outward, after the critical section: the rates may have moved and a
        // consumer of `Clocks` is a stranger (`CLAUDE.md`, re-entrancy). So may
        // the interrupt line, which a ready bit coming true is what raises.
        self.refresh_outputs();
        self.recompute();
    }

    // -- the clock tree ----------------------------------------------------

    /// Whether the source behind `osc` is ready. Anything else is a rate of
    /// zero, which is what "the guest has not switched it on" looks like.
    fn osc_ready(&self, state: &State, osc: Osc) -> bool {
        if self.variant == Variant::WB && osc == Osc::Lsi {
            // "LSI2ON: LSI2 oscillator enable and selection … 0: LSI2
            // oscillator OFF (LSI1 selected on LSI)" (RM0434 §6.4.31): the
            // LSI is LSI2 while `LSI2ON` stands, ready or not, and LSI1
            // otherwise.
            let csr = state.word(WB_CSR);
            let rdy = if csr & (1 << 2) != 0 { 1 << 3 } else { 1 << 1 };
            return csr & rdy != 0;
        }
        self.layout
            .ready
            .iter()
            .any(|b| b.osc == osc && state.word(b.reg) & (1 << b.rdy) != 0)
    }

    /// The MSI's programmed rate (L4 and U5).
    ///
    /// `CR.MSIRGSEL` chooses whether the range comes from `CR.MSIRANGE` or
    /// from `CSR.MSISRANGE`, which is the range a standby wake-up comes back
    /// at (RM0351 §6.4.1). A U5 moved the select and the run-time range to
    /// `ICSCR1` — `MSIRGSEL` at 23, `MSISRANGE[31:28]` — and keeps the
    /// standby range in `CSR.MSISSRANGE[15:12]` (RM0456 §11.8).
    fn msi_hz(&self, state: &State) -> Rational {
        if self.variant == Variant::WB {
            // A WB has no `MSIRGSEL` and no standby range: `CR.MSIRANGE[7:4]`
            // is the range, always. 12…15 are not ranges, and
            // `MSIRangeTable` answers them with zero, so this does too.
            let range = (state.word(WB_CR) >> 4) & 0xf;
            return Rational::integer(MSI_RANGES.get(range as usize).copied().unwrap_or(0));
        }
        if self.variant == Variant::U5 {
            let icscr1 = state.word(U5_ICSCR1);
            let range = if icscr1 & (1 << 23) != 0 {
                icscr1 >> 28
            } else {
                (state.word(U5_CSR) >> 12) & 0xf
            };
            let (num, den) = U5_MSI_RANGES[(range & 0xf) as usize];
            return Rational::integer(num)
                .checked_scale(1, den)
                .unwrap_or(Rational::integer(0));
        }
        let cr = state.word(self.layout.cr);
        let range = if cr & (1 << 3) != 0 {
            (cr >> 4) & 0xf
        } else {
            (state.word(self.layout.csr) >> 8) & 0xf
        };
        Rational::integer(
            MSI_RANGES
                .get(range as usize)
                .copied()
                .unwrap_or(MSI_RANGES[6]),
        )
    }

    /// The rate of one source, in hertz, or zero when it is not running.
    fn source_hz(&self, state: &State, osc: Osc) -> Rational {
        if !self.osc_ready(state, osc) {
            return Rational::integer(0);
        }
        Rational::integer(match osc {
            Osc::Hsi => self.freq.hsi,
            Osc::Hse => self.freq.hse,
            Osc::Lse => self.freq.lse,
            Osc::Lsi => self.freq.lsi,
            Osc::Msi => return self.msi_hz(state),
            // Neither is a source in its own right.
            Osc::Pll | Osc::Aux => 0,
        })
    }

    /// The HSE as `SYSCLK` and the PLL see it: on a WB, halved while
    /// `CR.HSEPRE` stands (`HAL_RCC_GetSysClockFreq`); everywhere else the
    /// crystal itself.
    fn hse_sys_hz(&self, state: &State) -> Rational {
        let hse = self.source_hz(state, Osc::Hse);
        if self.variant == Variant::WB && state.word(WB_CR) & WB_CR_HSEPRE != 0 {
            return hse.checked_scale(1, 2).unwrap_or(Rational::integer(0));
        }
        hse
    }

    /// The main PLL's system output and its 48 MHz output.
    ///
    /// Exact rational arithmetic throughout: a multiplier and a divider inside
    /// one oscillator's tree are integers and the ratio between them is exact
    /// by construction (`CLAUDE.md`, *Determinism*).
    fn pll(&self, state: &State) -> (Rational, Rational) {
        let zero = Rational::integer(0);
        if !self.osc_ready(state, Osc::Pll) {
            return (zero, zero);
        }
        let cfg = state.word(self.layout.pllcfgr);
        match self.variant {
            Variant::F4 => {
                // RM0090 §7.3.2: M[5:0], N[14:6], P[17:16] (÷ 2·(P+1)),
                // SRC[22], Q[27:24].
                let m = u64::from(cfg & 0x3f);
                let n = u64::from((cfg >> 6) & 0x1ff);
                let p = 2 * (u64::from((cfg >> 16) & 0x3) + 1);
                let q = u64::from((cfg >> 24) & 0xf);
                let src = if cfg & (1 << 22) != 0 {
                    Osc::Hse
                } else {
                    Osc::Hsi
                };
                if m == 0 || n == 0 {
                    return (zero, zero);
                }
                let Some(vco) = self.source_hz(state, src).checked_scale(n, m) else {
                    return (zero, zero);
                };
                let sys = vco.checked_scale(1, p).unwrap_or(zero);
                let out48 = if q == 0 {
                    zero
                } else {
                    vco.checked_scale(1, q).unwrap_or(zero)
                };
                (sys, out48)
            }
            Variant::L4 => {
                // RM0351 §6.4.4: SRC[1:0], M[6:4] (÷ M+1), N[14:8],
                // Q[22:21] (÷ 2·(Q+1)) behind QEN[20], R[26:25] (÷ 2·(R+1))
                // behind REN[24].
                let src = match cfg & 0x3 {
                    1 => Osc::Msi,
                    2 => Osc::Hsi,
                    3 => Osc::Hse,
                    _ => return (zero, zero),
                };
                let m = u64::from((cfg >> 4) & 0x7) + 1;
                let n = u64::from((cfg >> 8) & 0x7f);
                let q = 2 * (u64::from((cfg >> 21) & 0x3) + 1);
                let r = 2 * (u64::from((cfg >> 25) & 0x3) + 1);
                if n == 0 {
                    return (zero, zero);
                }
                let Some(vco) = self.source_hz(state, src).checked_scale(n, m) else {
                    return (zero, zero);
                };
                let sys = if cfg & (1 << 24) != 0 {
                    vco.checked_scale(1, r).unwrap_or(zero)
                } else {
                    zero
                };
                let out48 = if cfg & (1 << 20) != 0 {
                    vco.checked_scale(1, q).unwrap_or(zero)
                } else {
                    zero
                };
                (sys, out48)
            }
            Variant::U5 => {
                // RM0456 §11.8 (`RCC_PLL1CFGR`, `RCC_PLL1DIVR`,
                // `RCC_PLL1FRACR`): SRC[1:0] 01 MSIS, 10 HSI16, 11 HSE;
                // M[11:8] (÷ M+1); N[8:0] (× N+1) plus FRACN[15:3] / 2^13
                // when FRACEN[4] stands; Q[22:16] (÷ Q+1) behind QEN[17];
                // R[30:24] (÷ R+1) behind REN[18].
                //
                // `PLL1MBOOST[15:12]` is deliberately absent: it divides the
                // clock the EPOD *booster* runs on, not the VCO's input, and
                // so changes no frequency a guest can measure.
                let src = match cfg & 0x3 {
                    1 => Osc::Msi,
                    2 => Osc::Hsi,
                    3 => Osc::Hse,
                    _ => return (zero, zero),
                };
                let divr = state.word(U5_PLL1DIVR);
                let m = u64::from((cfg >> 8) & 0xf) + 1;
                let n = u64::from(divr & 0x1ff) + 1;
                let q = u64::from((divr >> 16) & 0x7f) + 1;
                let r = u64::from((divr >> 24) & 0x7f) + 1;
                let frac = if cfg & (1 << 4) != 0 {
                    u64::from((state.word(U5_PLL1FRACR) >> 3) & 0x1fff)
                } else {
                    0
                };
                // VCO = src / M × (N + FRACN / 8192), exactly.
                let Some(vco) = self
                    .source_hz(state, src)
                    .checked_scale(n * 8192 + frac, m * 8192)
                else {
                    return (zero, zero);
                };
                let sys = if cfg & (1 << 18) != 0 {
                    vco.checked_scale(1, r).unwrap_or(zero)
                } else {
                    zero
                };
                let out48 = if cfg & (1 << 17) != 0 {
                    vco.checked_scale(1, q).unwrap_or(zero)
                } else {
                    zero
                };
                (sys, out48)
            }
            Variant::WB => {
                // `stm32wb55xx.h`, `RCC_PLLCFGR_*`: SRC[1:0] 01 MSI,
                // 10 HSI16, 11 HSE (after `HSEPRE`); M[6:4] (÷ M+1);
                // N[14:8]; Q[27:25] (÷ Q+1) behind QEN[24]; R[31:29]
                // (÷ R+1) behind REN[28] — `SystemCoreClockUpdate`'s
                // `pllvco / pllr` with `pllr = R + 1`.
                let src = match cfg & 0x3 {
                    1 => self.source_hz(state, Osc::Msi),
                    2 => self.source_hz(state, Osc::Hsi),
                    3 => self.hse_sys_hz(state),
                    _ => return (zero, zero),
                };
                let m = u64::from((cfg >> 4) & 0x7) + 1;
                let n = u64::from((cfg >> 8) & 0x7f);
                let q = u64::from((cfg >> 25) & 0x7) + 1;
                let r = u64::from((cfg >> 29) & 0x7) + 1;
                if n == 0 {
                    return (zero, zero);
                }
                let Some(vco) = src.checked_scale(n, m) else {
                    return (zero, zero);
                };
                let sys = if cfg & (1 << 28) != 0 {
                    vco.checked_scale(1, r).unwrap_or(zero)
                } else {
                    zero
                };
                let out48 = if cfg & (1 << 24) != 0 {
                    vco.checked_scale(1, q).unwrap_or(zero)
                } else {
                    zero
                };
                (sys, out48)
            }
        }
    }

    /// What `BDCR.RTCSEL` has selected, gated by `RTCEN`.
    fn rtc_hz(&self, state: &State) -> Rational {
        let zero = Rational::integer(0);
        let bdcr = state.word(self.layout.bdcr);
        if bdcr & (1 << 15) == 0 {
            return zero;
        }
        match ((bdcr >> 8) & 0x3, self.variant) {
            (1, _) => self.source_hz(state, Osc::Lse),
            // A U5's `BDCR.LSIPREDIV` (bit 28) divides the LSI by 128 on its
            // way to the RTC (RM0456 §11.8, `RCC_BDCR`).
            (2, Variant::U5) if bdcr & (1 << 28) != 0 => self
                .source_hz(state, Osc::Lsi)
                .checked_scale(1, 128)
                .unwrap_or(zero),
            (2, _) => self.source_hz(state, Osc::Lsi),
            // An F4 divides HSE by `CFGR.RTCPRE`, and a value below 2 means no
            // clock (RM0090 §7.3.3). An L4 divides it by a fixed 32
            // (RM0351 §6.4.3), and so do a U5 (RM0456 §11.8) and a WB
            // (`LL_RCC_RTC_CLKSOURCE_HSE_DIV32`) — the undivided crystal,
            // `HSEPRE` notwithstanding.
            (3, Variant::F4) => {
                let pre = u64::from((state.word(self.layout.cfgr) >> 16) & 0x1f);
                if pre < 2 {
                    zero
                } else {
                    self.source_hz(state, Osc::Hse)
                        .checked_scale(1, pre)
                        .unwrap_or(zero)
                }
            }
            (3, Variant::L4 | Variant::U5 | Variant::WB) => self
                .source_hz(state, Osc::Hse)
                .checked_scale(1, 32)
                .unwrap_or(zero),
            _ => zero,
        }
    }

    /// The whole tree, from the registers as they now stand.
    fn rates(&self, state: &State) -> [Rational; ClockOutput::COUNT] {
        let zero = Rational::integer(0);
        let cfgr = state.word(self.layout.cfgr);
        let (pll_sys, pll48) = self.pll(state);
        // `SWS`, not `SW`: what the system is *running on* is what the
        // hardware answers with, and the two differ for one tick after a
        // switch.
        let sws = (cfgr >> 2) & 0x3;
        let sysclk = match (self.variant, sws) {
            // RM0090 §7.3.3: 00 HSI, 01 HSE, 10 PLL.
            (Variant::F4, 0) => self.source_hz(state, Osc::Hsi),
            (Variant::F4, 1) => self.source_hz(state, Osc::Hse),
            (Variant::F4, 2) => pll_sys,
            // RM0351 §6.4.3, and RM0456 §11.8 for a U5's `CFGR1`: 00 MSI(S),
            // 01 HSI16, 10 HSE, 11 PLL(1).
            // A WB's `CFGR` is the L4's, with `HSEPRE` between the crystal
            // and the switch.
            (Variant::L4 | Variant::U5 | Variant::WB, 0) => self.source_hz(state, Osc::Msi),
            (Variant::L4 | Variant::U5 | Variant::WB, 1) => self.source_hz(state, Osc::Hsi),
            (Variant::L4 | Variant::U5 | Variant::WB, 2) => self.hse_sys_hz(state),
            (Variant::L4 | Variant::U5 | Variant::WB, 3) => pll_sys,
            _ => zero,
        };

        let (hpre, ppre1, ppre2, ppre3) = match self.variant {
            // RM0090 §7.3.3: HPRE[7:4], PPRE1[12:10], PPRE2[15:13].
            Variant::F4 => ((cfgr >> 4) & 0xf, (cfgr >> 10) & 0x7, (cfgr >> 13) & 0x7, 0),
            // RM0351 §6.4.3: HPRE[7:4], PPRE1[10:8], PPRE2[13:11] — and a
            // WB's `CFGR` has them at the same bits.
            Variant::L4 | Variant::WB => {
                ((cfgr >> 4) & 0xf, (cfgr >> 8) & 0x7, (cfgr >> 11) & 0x7, 0)
            }
            // RM0456 §11.8: the prescalers left `CFGR1` for `CFGR2` —
            // HPRE[3:0], PPRE1[6:4], PPRE2[10:8] — and `CFGR3.PPRE3[6:4]`.
            // The encodings are the F4's.
            Variant::U5 => {
                let cfgr2 = state.word(U5_CFGR2);
                (
                    cfgr2 & 0xf,
                    (cfgr2 >> 4) & 0x7,
                    (cfgr2 >> 8) & 0x7,
                    (state.word(U5_CFGR3) >> 4) & 0x7,
                )
            }
        };

        let ahb_div = |field: u32| {
            if self.variant == Variant::WB {
                WB_HPRE[(field & 0xf) as usize]
            } else {
                hpre_div(field)
            }
        };
        let hclk = sysclk.checked_scale(1, ahb_div(hpre)).unwrap_or(zero);
        // A WB's other two AHB clocks, off `SYSCLK` and not off `HCLK1`
        // (`__LL_RCC_CALC_HCLK2_FREQ`/`_HCLK4_FREQ` take `SYSCLK`).
        let (hclk2, hclk4) = if self.variant == Variant::WB {
            let ext = state.word(WB_EXTCFGR);
            (
                sysclk.checked_scale(1, ahb_div(ext >> 4)).unwrap_or(zero),
                sysclk.checked_scale(1, ahb_div(ext)).unwrap_or(zero),
            )
        } else {
            (zero, zero)
        };
        let pclk1 = hclk.checked_scale(1, ppre_div(ppre1)).unwrap_or(zero);
        let pclk2 = hclk.checked_scale(1, ppre_div(ppre2)).unwrap_or(zero);
        let pclk3 = if self.variant == Variant::U5 {
            hclk.checked_scale(1, ppre_div(ppre3)).unwrap_or(zero)
        } else {
            zero
        };
        // "If APBx prescaler is 1, the timer clock is PCLKx, else 2 × PCLKx"
        // (RM0090 §7.2, RM0351 §6.2).
        let tim1 = if ppre_div(ppre1) == 1 {
            pclk1
        } else {
            pclk1.checked_scale(2, 1).unwrap_or(zero)
        };
        let tim2 = if ppre_div(ppre2) == 1 {
            pclk2
        } else {
            pclk2.checked_scale(2, 1).unwrap_or(zero)
        };

        let mut out = [zero; ClockOutput::COUNT];
        out[usize::from(ClockOutput::SYSCLK.0)] = sysclk;
        out[usize::from(ClockOutput::HCLK.0)] = hclk;
        out[usize::from(ClockOutput::PCLK1.0)] = pclk1;
        out[usize::from(ClockOutput::PCLK2.0)] = pclk2;
        out[usize::from(ClockOutput::TIMCLK1.0)] = tim1;
        out[usize::from(ClockOutput::TIMCLK2.0)] = tim2;
        out[usize::from(ClockOutput::RTCCLK.0)] = self.rtc_hz(state);
        out[usize::from(ClockOutput::PLL48.0)] = pll48;
        out[usize::from(ClockOutput::PCLK3.0)] = pclk3;
        out[usize::from(ClockOutput::HCLK2.0)] = hclk2;
        out[usize::from(ClockOutput::HCLK4.0)] = hclk4;
        out
    }

    /// Recompute and publish the tree. Takes the state lock briefly and makes
    /// no outward call while holding it.
    fn recompute(&self) {
        let next = {
            let state = self.state.lock();
            self.rates(&state)
        };
        if self.clocks.publish(next) {
            self.drive_domains(&next);
        }
    }

    /// Ask the scheduler to re-rate whichever clock domains this controller was
    /// told are its outputs.
    ///
    /// # What a rating is measured against
    ///
    /// A [`ClockForest`](crate::core::clock::ClockForest) domain is rated
    /// against its **parent**, so a request is a ratio and never a frequency.
    /// The tree the machine file is expected to have built is the one RM0090
    /// §7.2 draws, and [`OUTPUT_TREE`] is that shape written down: `SYSCLK` off
    /// the crystal, `HCLK` off `SYSCLK`, `PCLK1` and `PCLK2` off `HCLK`, and
    /// each timer clock off its own APB clock. Every output is rated against
    /// the **nearest ancestor this instance was actually given**, falling all
    /// the way back to the `hse` reference: a board that names only `pclk1`
    /// gets `pclk1 / hse`, which is right if that is how it wired the domain
    /// and wrong if it did not.
    ///
    /// `timclk1`/`timclk2` are outputs of their own rather than a `× 2` a board
    /// could write itself, because the doubling is conditional — RM0090 §7.2:
    /// "if the APB prescaler is 1 the timer clock is PCLKx, else 2 × PCLKx" —
    /// and a machine file that froze either answer would be wrong half the
    /// time.
    ///
    /// **`hse` is the reference, and it has to be the truth.** The property
    /// already has to agree with the board's `osc hse` — `stm32f407.machine`
    /// says so at the point it writes the number twice — and this is the second
    /// thing that depends on it.
    ///
    /// # The deviation this bakes in
    ///
    /// An output expressed as a ratio of the domain's parent cannot say *which*
    /// crystal it came from. A board with one high-speed oscillator therefore
    /// models HSI and the PLL as exact ratios of HSE: the rate a guest measures
    /// is exactly right, and the physical independence of the two cans is not
    /// modelled. Saying it properly needs a reparent across trees at the moment
    /// `SWS` changes, which is a bigger seam than this one and is not here.
    ///
    /// An output of zero — its source is stopped, or `SWS` names something that
    /// is not running — leaves its domain at the rate it had rather than
    /// stopping it. Stopping a domain is
    /// [`ClockForest::set_gated`](crate::core::clock::ClockForest::set_gated)'s
    /// business and belongs with the peripheral clock-enable half of the
    /// problem, not here.
    fn drive_domains(&self, rates: &[Rational; ClockOutput::COUNT]) {
        let control = self.control.lock().clone();
        let Some(control) = control else { return };
        let outputs = self.domains.lock().clone();
        if outputs.is_empty() {
            return;
        }
        let reference = Rational::integer(self.freq.hse);
        if reference.is_zero() {
            return;
        }
        for (out, _) in OUTPUT_TREE {
            let Some(domain) = outputs.iter().find(|(o, _)| o == out).map(|(_, d)| *d) else {
                continue;
            };
            let hz = rates[usize::from(out.0)];
            if hz.is_zero() {
                continue;
            }
            // Up the tree until something this instance drives is found: that
            // domain is the parent the machine file must have given it, so that
            // is what the ratio is against.
            let mut against = reference;
            let mut cursor = *out;
            while let Some(parent) = parent_output(cursor) {
                if outputs.iter().any(|(o, _)| *o == parent) {
                    against = rates[usize::from(parent.0)];
                    break;
                }
                cursor = parent;
            }
            if against.is_zero() {
                continue;
            }
            if let Some(ratio) = hz.checked_div(against) {
                control.request_ratio(domain, ratio);
            }
        }
    }

    // -- the gate outputs --------------------------------------------------

    /// The key a bank's bit is filed under.
    fn key(bank: u32, bit: u32) -> u32 {
        bank * BANK_WIDTH + bit
    }

    /// Which register and bit a key names, or `None` for the named pins.
    fn key_source(&self, key: u32) -> Option<(u64, u32)> {
        if key >= KEY_RTCEN {
            return None;
        }
        let bank = (key / BANK_WIDTH) as usize;
        let bit = key % BANK_WIDTH;
        let banks = self.layout.enable.len();
        let reg = if bank < banks {
            self.layout.enable[bank].reg
        } else {
            self.layout.reset.get(bank - banks)?.reg
        };
        Some((reg, bit))
    }

    /// Drive every connected gate pin to whatever the registers now say.
    ///
    /// Called with **no lock held**. The state is copied out first, the wire
    /// list second, and the outward `set` calls last — a sink is free to call
    /// back into this device, and a level a net is already at is not delivered
    /// at all, so refreshing every connected pin is cheaper than diffing.
    fn refresh_outputs(&self) {
        let words = self.state.lock().words;
        let bdcr = words[(self.layout.bdcr / 4) as usize];
        let pending: Vec<(WireSource, Level)> = {
            let outputs = self.outputs.lock();
            outputs
                .iter()
                .map(|(key, source)| {
                    let high = match *key {
                        KEY_RTCEN => bdcr & (1 << 15) != 0,
                        KEY_BDRST => bdcr & (1 << 16) != 0,
                        KEY_IRQ => self.layout.interrupts.pending(&words),
                        key => match self.key_source(key) {
                            Some((reg, bit)) => words[(reg / 4) as usize] & (1 << bit) != 0,
                            None => false,
                        },
                    };
                    (source.clone(), Level::from_bool(high))
                })
                .collect()
        };
        for (source, level) in pending {
            source.set(level);
        }
    }

    // -- the register file -------------------------------------------------

    /// Which bits of `offset` the hardware owns and a guest write cannot set.
    fn read_only(&self, offset: u64) -> u32 {
        let mut mask = 0;
        for bit in self.layout.ready {
            if bit.reg == offset {
                mask |= 1 << bit.rdy;
            }
        }
        if offset == self.layout.cfgr {
            // `SWS` answers `SW`; it is never written.
            mask |= 0b1100;
        }
        if offset == self.layout.csr {
            // The reset flags, which only `RMVF` clears.
            mask |= 0xff00_0000 & !(1 << self.layout.rmvf);
        }
        if self.variant == Variant::WB {
            mask |= match offset {
                WB_CFGR => WB_CFGR_FLAGS,
                WB_EXTCFGR => WB_EXTCFGR_FLAGS | WB_EXTCFGR_RFCSS,
                WB_SMPSCR => WB_SMPSCR_SWS,
                // Derived on read from `APB3RSTR`, never stored.
                WB_CSR => WB_CSR_RFRSTS,
                _ => 0,
            };
        }
        mask
    }

    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        if let Some(value) = self.layout.interrupts.read(&state, offset) {
            return value;
        }
        let value = state.word(offset);
        if self.variant == Variant::WB && offset == WB_SMPSCR {
            return (value & !WB_SMPSCR_SWS) | (self.smps_sws(&state) << 8);
        }
        if offset == self.layout.csr {
            // `RMVF` is "cleared by writing 1" and reads back zero.
            let mut value = value & !(1 << self.layout.rmvf);
            if self.variant == Variant::WB && state.word(WB_APB3RSTR) & 1 != 0 {
                // `RFRSTS`: the radio is under reset while `RFRST` holds it
                // there.
                value |= WB_CSR_RFRSTS;
            }
            return value;
        }
        value
    }

    /// Write one register, then whatever it implies.
    ///
    /// The critical section ends before any wire moves and before the rate
    /// table is published, which is the re-entrancy contract in one function.
    fn write_register(&self, offset: u64, value: u32) {
        {
            let mut state = self.state.lock();
            let before = state.word(offset);

            if offset == self.layout.bdcr {
                self.write_bdcr(&mut state, value);
            } else if self.variant == Variant::WB && offset == WB_HSECR {
                Self::write_hsecr(&mut state, value);
            } else if self.layout.interrupts.write(&mut state, offset, value) {
                // One of `CIR`/`CIER`/`CIFR`/`CICR`, whose rules are neither a
                // plain register's nor `BDCR`'s.
            } else {
                let keep = self.read_only(offset);
                *state.word_mut(offset) = (before & keep) | (value & !keep);
            }

            if offset == self.layout.csr && value & (1 << self.layout.rmvf) != 0 {
                // "RMVF: remove reset flag … cleared by software by writing 1."
                *state.word_mut(offset) &= !0xff00_0000;
            }

            self.arm_ready_bits(&mut state, offset, before);

            if offset == self.layout.cfgr {
                self.arm_switch(&mut state, before);
            }
            if self.variant == Variant::WB {
                self.write_wb(&mut state, offset, before);
            }
            self.republish(&state);
        }
        self.refresh_outputs();
        self.recompute();
    }

    /// What a write to one of a WB's own registers implies, after the plain
    /// read-only-masked store has happened.
    fn write_wb(&self, state: &mut State, offset: u64, before: u32) {
        let after = state.word(offset);
        // Which prescaler fields this register has, and the applied flag
        // each one answers with (`RCC_CFGR_HPREF_Pos` …,
        // `RCC_EXTCFGR_SHDHPREF_Pos` …).
        let fields: &[(u32, u32)] = match offset {
            WB_CFGR => &[
                (0xf << 4, 1 << 16),
                (0x7 << 8, 1 << 17),
                (0x7 << 11, 1 << 18),
            ],
            WB_EXTCFGR => &[(0xf, 1 << 16), (0xf << 4, 1 << 17)],
            WB_CR => {
                // "others: not allowed (hardware write protection)"
                // (RM0434 §6.4.1): a `MSIRANGE` of 12…15 does not land, and
                // the range in use stays.
                if (after >> 4) & 0xf >= 12 {
                    *state.word_mut(WB_CR) = (after & !0xf0) | (before & 0xf0);
                }
                return;
            }
            _ => return,
        };
        let mut cleared = false;
        for (field, flag) in fields {
            if (before ^ after) & field != 0 {
                *state.word_mut(offset) &= !flag;
                cleared = true;
            }
        }
        if cleared {
            // One tick, the same as an accepted `SW`: long enough that a loop
            // which polls the flag sees it clear, short enough never to be
            // the thing a startup sequence waits on.
            state.presc_at = state.tick.saturating_add(1);
        }
    }

    /// `SMPSSWS`, the SMPS step-down converter's clock, as RM0434 §6.4.9
    /// states it: "whenever the SMPS step-down converter is disabled in
    /// `PWR_CR5.SMPSEN`, no clock is used" (`11`); "whenever the HSE is
    /// active the HSE is used regardless of the settings in `SMPSSEL`" (`10`);
    /// otherwise `SMPSSEL`'s HSI16 (`00`) or MSI (`01`) if it is running, and
    /// no clock if it is not or the selection is the reserved `11`.
    fn smps_sws(&self, state: &State) -> u32 {
        if self.smpsen.load(Ordering::Relaxed) == 0 {
            return 3;
        }
        if self.osc_ready(state, Osc::Hse) {
            return 2;
        }
        match state.word(WB_SMPSCR) & 0x3 {
            0 if self.osc_ready(state, Osc::Hsi) => 0,
            1 if self.osc_ready(state, Osc::Msi) => 1,
            _ => 3,
        }
    }

    /// `HSECR`, which takes `0xCAFE_CAFE` to unlock for a single write.
    ///
    /// RM0434 §6.4.33: "Once unlocked a single write access" is allowed, and
    /// `UNLOCKED` clears when "a write access has been performed".
    /// `LL_RCC_HSE_SetCapacitorTuning` writes the key and then does a
    /// read-modify-write of the fields; the second write is the one that
    /// lands, and it locks the register again behind it.
    fn write_hsecr(state: &mut State, value: u32) {
        let word = state.word_mut(WB_HSECR);
        if value == WB_HSECR_KEY {
            *word |= WB_HSECR_UNLOCKED;
        } else if *word & WB_HSECR_UNLOCKED != 0 {
            *word = value & WB_HSECR_FIELDS;
        }
        // Locked, and not the key: dropped.
    }

    /// `BDCR`, which is in the backup domain and write-protected by
    /// `PWR_CR.DBP`.
    ///
    /// RM0090 §7.3.20: "after reset, these bits are write-protected and the
    /// DBP bit in the Power control register has to be set before these can be
    /// modified". The protected set is the whole writable register — `LSEON`,
    /// `LSEBYP`, `RTCSEL`, `RTCEN` and `BDRST` — and not a subset of it.
    fn write_bdcr(&self, state: &mut State, value: u32) {
        if self.dbp_wired.load(Ordering::Relaxed) != 0 && self.dbp.load(Ordering::Relaxed) == 0 {
            // Dropped on the floor, exactly as the hardware drops it. A board
            // with no `st.pwr` wired to `dbp` has nothing modelling the
            // protection, so it gets none rather than a backup domain that can
            // never be opened.
            return;
        }
        let keep = self.read_only(self.layout.bdcr);
        let before = state.word(self.layout.bdcr);
        let next = (before & keep) | (value & !keep);
        *state.word_mut(self.layout.bdcr) = next;
        if next & (1 << 16) != 0 {
            // "BDRST: backup domain software reset" — everything in the domain
            // goes, the LSE and its ready bit included, while the bit stands.
            *state.word_mut(self.layout.bdcr) = 1 << 16;
            for (i, bit) in self.layout.ready.iter().enumerate() {
                if bit.reg == self.layout.bdcr {
                    state.ready_at[i] = NO_DEADLINE;
                }
            }
        }
    }

    /// Start or stop whatever `offset`'s `xxxON` bits just changed.
    fn arm_ready_bits(&self, state: &mut State, offset: u64, before: u32) {
        let now = state.tick;
        let after = state.word(offset);
        for (i, bit) in self.layout.ready.iter().enumerate() {
            if bit.reg != offset {
                continue;
            }
            let was_on = before & (1 << bit.on) != 0;
            let is_on = after & (1 << bit.on) != 0;
            if is_on && !was_on {
                state.ready_at[i] = now.saturating_add(self.ready_delay);
            } else if !is_on && was_on {
                // "cleared by hardware when the oscillator is switched off."
                state.ready_at[i] = NO_DEADLINE;
                *state.word_mut(offset) &= !(1 << bit.rdy);
            }
        }
    }

    /// Accept or refuse a `CFGR.SW` change.
    ///
    /// A source that is not ready is refused outright: the `SW` field keeps
    /// the value it had and `SWS` never moves. Firmware that polls `xxxRDY`
    /// before switching — which is every vendor sequence — never notices.
    fn arm_switch(&self, state: &mut State, before: u32) {
        let cfgr = state.word(self.layout.cfgr);
        let requested = cfgr & 0x3;
        let previous = before & 0x3;
        if requested == previous {
            return;
        }
        let osc = match (self.variant, requested) {
            (Variant::F4, 0) => Osc::Hsi,
            (Variant::F4, 1) => Osc::Hse,
            (Variant::F4, 2) => Osc::Pll,
            (Variant::L4 | Variant::U5 | Variant::WB, 0) => Osc::Msi,
            (Variant::L4 | Variant::U5 | Variant::WB, 1) => Osc::Hsi,
            (Variant::L4 | Variant::U5 | Variant::WB, 2) => Osc::Hse,
            (Variant::L4 | Variant::U5 | Variant::WB, 3) => Osc::Pll,
            // An F4's `SW = 11` is reserved.
            _ => {
                *state.word_mut(self.layout.cfgr) = (cfgr & !0x3) | previous;
                return;
            }
        };
        if !self.osc_ready(state, osc) {
            *state.word_mut(self.layout.cfgr) = (cfgr & !0x3) | previous;
            return;
        }
        state.switch_at = state.tick.saturating_add(1);
        state.switch_to = requested;
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        // Before the lock, and harmless for a debug access: catching the
        // device up is what makes a ready bit come true under a polling loop,
        // and a debug read advances nothing (`ROADMAP.md` §15, invariant 5).
        self.sync(attrs);
        let value = self.read_register(offset & !3);
        let bytes = value.to_le_bytes();
        (*a, *b, *c, *d) = (bytes[0], bytes[1], bytes[2], bytes[3]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `CR` would start an oscillator and one to an
            // `ENR` would move a peripheral's gate wire. Neither can be made
            // harmless, so it is refused rather than guessed at.
            return Err(BusError::BadAccess);
        }
        self.sync(attrs);
        self.write_register(offset & !3, u32::from_le_bytes([*a, *b, *c, *d]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // "The peripheral registers have to be accessed by words (32 bits)."
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// Input pins
// ---------------------------------------------------------------------------

/// What an input pin of the RCC does when it changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InputKind {
    /// `PWR_CR.DBP`, which unlocks `BDCR`.
    Dbp,
    /// A WB's `PWR_CR5.SMPSEN`, which `SMPSCR.SMPSSWS` reports through.
    Smpsen,
    /// A reset source, which latches a `CSR` flag on its rising edge.
    Cause(u32),
}

/// One of the RCC's input pins, as something a wire can drive.
///
/// Holds the register block rather than the [`Rcc`]: the device owns the pin,
/// and a pin that owned the device back would be a cycle nothing could drop.
#[derive(Debug)]
pub struct InputPin {
    regs: Arc<Registers>,
    kind: InputKind,
    inputs: FanIn,
}

impl InputPin {
    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        match self.kind {
            InputKind::Dbp => {
                // One atomic and nothing else: the `BDCR` write path samples
                // it and there is no outward call to make.
                self.regs.dbp.store(u32::from(high), Ordering::Relaxed);
            }
            InputKind::Smpsen => {
                // Read back through `SMPSSWS` on the next access; nothing to
                // republish.
                self.regs.smpsen.store(u32::from(high), Ordering::Relaxed);
            }
            InputKind::Cause(bit) => {
                if !high {
                    return;
                }
                // The flag is latched on the rising edge and stays until
                // `RMVF`; the pulse's width is not modelled and does not
                // matter.
                let mut state = self.regs.state.lock();
                let csr = self.regs.layout.csr;
                *state.word_mut(csr) |= 1 << bit;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32 reset and clock controller.
#[derive(Debug)]
pub struct Rcc {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The input pins this device owns. A net holds only a weak reference.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Rcc {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Rcc> {
        let mut r = props.reader();
        let variant = match r.or_enum("variant", "f4", &["f4", "l4", "u5", "wb"])? {
            "l4" => Variant::L4,
            "u5" => Variant::U5,
            "wb" => Variant::WB,
            _ => Variant::F4,
        };
        let defaults = Frequencies::default();
        let freq = Frequencies {
            hse: r.or("hse", defaults.hse)?,
            hsi: r.or("hsi", defaults.hsi)?,
            lse: r.or("lse", defaults.lse)?,
            lsi: r.or("lsi", defaults.lsi)?,
        };
        let ready_delay = r.or("ready-delay", DEFAULT_READY_DELAY)?;
        let mut named: Vec<(ClockOutput, String)> = Vec::new();
        for (out, prop) in OUTPUT_TREE {
            if let Some(name) = r.optional_str(prop)? {
                named.push((*out, String::from(name)));
            }
        }
        r.finish()?;
        Ok(Rcc::with_outputs(variant, freq, ready_delay, named))
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn with_config(variant: Variant, freq: Frequencies, ready_delay: u64) -> Rcc {
        Rcc::with_outputs(variant, freq, ready_delay, Vec::new())
    }

    /// Build one that drives clock domains, naming the object behind each
    /// output.
    ///
    /// The names are resolved against the machine's objects at bind time; an
    /// output nobody named drives nothing. See `drive_domains` for what a
    /// rating is measured against.
    #[must_use]
    pub fn with_outputs(
        variant: Variant,
        freq: Frequencies,
        ready_delay: u64,
        named: Vec<(ClockOutput, String)>,
    ) -> Rcc {
        let layout = variant.layout();
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(layout)),
            variant,
            layout,
            freq,
            ready_delay,
            clocks: Arc::new(Clocks::new()),
            outputs: Mutex::with_rank(LockRank::WIRE, Vec::new()),
            dbp: AtomicU32::new(0),
            dbp_wired: AtomicU32::new(0),
            smpsen: AtomicU32::new(0),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(u64::MAX),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            named,
            domains: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            control: Mutex::with_rank(LockRank::LEAF, None),
        });
        regs.recompute();
        let region = Arc::new(Region::io(
            "rcc",
            variant.register_bytes(),
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Rcc {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        }
    }

    /// Which register layout this instance has.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.regs.variant
    }

    /// The rates this controller is driving.
    ///
    /// The same handle [`ExportId::CLOCK_TREE`] publishes, for a caller that
    /// already has the device in hand.
    #[must_use]
    pub fn clocks(&self) -> Arc<Clocks> {
        Arc::clone(&self.regs.clocks)
    }

    /// Whether the gate bit `bank`'s bit `bit` is set.
    ///
    /// `bank` is the pin prefix — `"apb1en"`, `"ahb1rst"` — which is what a
    /// machine file names and what a test asks about. `None` when this variant
    /// has no such bank.
    #[must_use]
    pub fn gate(&self, bank: &str, bit: u32) -> Option<bool> {
        let key = bank_key(self.regs.layout, bank, bit)?;
        let (reg, _) = self.regs.key_source(key)?;
        Some(self.regs.state.lock().word(reg) & (1 << bit) != 0)
    }

    /// Set `PWR_CR.DBP` directly, for a test with no wire in hand.
    ///
    /// Also marks the input as driven, so the backup domain is protected from
    /// this moment on.
    pub fn set_dbp(&self, high: bool) {
        self.regs.dbp_wired.store(1, Ordering::Relaxed);
        self.regs.dbp.store(u32::from(high), Ordering::Relaxed);
    }
}

/// The output key for `bank`'s bit `bit`, or `None` if this layout has no such
/// bank.
fn bank_key(layout: &Layout, bank: &str, bit: u32) -> Option<u32> {
    if bit >= BANK_WIDTH {
        return None;
    }
    if let Some(i) = layout.enable.iter().position(|b| b.prefix == bank) {
        return Some(Registers::key(i as u32, bit));
    }
    let i = layout.reset.iter().position(|b| b.prefix == bank)?;
    Some(Registers::key((layout.enable.len() + i) as u32, bit))
}

/// Split `port` into a declared bank prefix and a bit number.
fn parse_bank_pin(layout: &Layout, port: &str) -> Option<u32> {
    // Order does not matter: every prefix is followed by digits, and no
    // prefix is another prefix followed by digits — see [`ALL_BANKS`].
    for name in ALL_BANKS {
        if let Some(bit) = port_index(port, name, BANK_WIDTH) {
            return bank_key(layout, name, bit);
        }
    }
    None
}

/// The banks this layout offers, for a diagnostic.
fn banks_of(layout: &Layout) -> String {
    let mut names: Vec<&str> = layout.enable.iter().map(|b| b.prefix).collect();
    names.extend(layout.reset.iter().map(|b| b.prefix));
    names.join(", ")
}

impl Device for Rcc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and `wire`
        // statements bring the pins.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        {
            let mut state = self.regs.state.lock();
            let backup = state.word(self.regs.layout.bdcr);
            let flags = state.word(self.regs.layout.csr) & 0xff00_0000;
            let tick = state.tick;
            *state = State::reset(self.regs.layout);
            state.tick = tick;
            if kind != ResetKind::Cold {
                // "Battery-backed and always-on state survives": the backup
                // domain is exactly that, and so is the record of why the part
                // reset.
                *state.word_mut(self.regs.layout.bdcr) = backup;
                let csr = state.word_mut(self.regs.layout.csr);
                *csr = (*csr & !0xff00_0000) | flags;
            }
            self.regs.republish(&state);
        }
        self.regs.refresh_outputs();
        self.regs.recompute();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for word in state.words {
            w.write_u32(word)?;
        }
        for at in state.ready_at {
            w.write_u64(at)?;
        }
        w.write_u64(state.switch_at)?;
        w.write_u32(state.switch_to)?;
        w.write_u64(state.presc_at)?;
        // The controller's own position in its domain: the scheduler restores
        // the domain, and without this the two would disagree and every
        // pending ready bit would come true at the wrong instant.
        w.write_u64(state.tick)
        // The wire handles, the `dbp` level and the published rate table are
        // the machine's wiring and derived state, not the chip's (invariant 3).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::reset(self.regs.layout);
        for word in &mut state.words {
            *word = r.read_u32()?;
        }
        for at in &mut state.ready_at {
            *at = r.read_u64()?;
        }
        state.switch_at = r.read_u64()?;
        state.switch_to = r.read_u32()?;
        state.presc_at = r.read_u64()?;
        state.tick = r.read_u64()?;
        {
            let mut held = self.regs.state.lock();
            *held = state;
            self.regs.republish(&held);
        }
        self.regs.refresh_outputs();
        self.regs.recompute();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::CLOCK_TREE)
            .then(|| Export::Opaque(Arc::clone(&self.regs.clocks) as Arc<_>))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let key = match port {
            RTCEN_PIN => KEY_RTCEN,
            BDRST_PIN => KEY_BDRST,
            IRQ_PIN => KEY_IRQ,
            other => parse_bank_pin(self.regs.layout, other).ok_or_else(|| Error::Config {
                at: port.to_string(),
                message: format!(
                    "an `{}` RCC drives `{RTCEN_PIN}`, `{BDRST_PIN}`, `{IRQ_PIN}` and bits \
                     0…{} of each of {}",
                    self.regs.variant.as_str(),
                    BANK_WIDTH - 1,
                    banks_of(self.regs.layout),
                ),
            })?,
        };
        self.regs.outputs.lock().push((key, source));
        self.regs.refresh_outputs();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.regs.refresh_outputs();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let kind = if port == DBP_PIN {
            self.regs.dbp_wired.store(1, Ordering::Relaxed);
            InputKind::Dbp
        } else if port == SMPSEN_PIN && self.regs.variant == Variant::WB {
            InputKind::Smpsen
        } else {
            InputKind::Cause(
                self.regs
                    .variant
                    .causes()
                    .iter()
                    .find(|c| c.pin == port)?
                    .bit,
            )
        };
        let line = match kind {
            InputKind::Dbp | InputKind::Smpsen => 0,
            InputKind::Cause(bit) => bit,
        };
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            kind,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    fn is_lazy(&self) -> bool {
        // A ready bit comes true some number of ticks after the guest asked
        // for it, and the guest's poll of `CR` is what has to see it. The
        // scheduler owns that time; this device never sleeps and never reads
        // the host clock.
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.regs.next_event.load(Ordering::Relaxed) {
            u64::MAX => None,
            at => Some(at),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.regs.lazy.lock() = Some(handle);
    }

    fn attach_clock_control(&self, control: Arc<ClockControl>) {
        *self.regs.control.lock() = Some(control);
        // The rates were computed before this arrived — at construction, and
        // again at every reset — so the first request has to be made from here
        // rather than from the next one. Without it a part that comes out of
        // reset on its internal RC would leave its domains at whatever the
        // machine file declared until the guest happened to touch `CFGR`.
        let rates = *self.regs.clocks.rates.lock();
        self.regs.drive_domains(&rates);
    }
}

impl Instance for Rcc {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let mut resolved = Vec::with_capacity(self.regs.named.len());
        for (out, name) in &self.regs.named {
            let peer = ctx.peer(name)?;
            let domain = peer.domain().ok_or_else(|| Error::Config {
                at: String::from(ctx.path()),
                message: format!(
                    "`{out}` names `{name}`, which has no clock domain of its own; give it a \
                     rate with `clock = …` — an `object {name} \"clock\"` is the usual way to \
                     say that this is a node of the tree and nothing else"
                ),
            })?;
            resolved.push((*out, domain));
        }
        *self.regs.domains.lock() = resolved;
        Ok(())
    }
}

/// The `st.rcc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 reset and clock control: the ready bits, the PLL and prescaler tree, \
              the peripheral gates and the backup domain",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "which register layout: \"f4\" (RM0090), \"l4\" (RM0351), \"u5\" (RM0456) \
                      or \"wb\" (RM0434)",
        },
        PropertySpec {
            name: "hse",
            kind: ValueKind::Uint,
            required: false,
            summary: "the external high-speed crystal, in Hz (default 8000000)",
        },
        PropertySpec {
            name: "hsi",
            kind: ValueKind::Uint,
            required: false,
            summary: "the internal high-speed RC, in Hz (default 16000000)",
        },
        PropertySpec {
            name: "lse",
            kind: ValueKind::Uint,
            required: false,
            summary: "the backup-domain crystal, in Hz (default 32768)",
        },
        PropertySpec {
            name: "lsi",
            kind: ValueKind::Uint,
            required: false,
            summary: "the internal low-speed RC, in Hz (default 32000)",
        },
        PropertySpec {
            name: "ready-delay",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many ticks of this device's clock domain an oscillator takes to \
                      become ready (default 16)",
        },
        PropertySpec {
            name: "sysclk",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is SYSCLK; it must hang off the `hse` \
                      crystal, and this controller re-rates it as the registers move",
        },
        PropertySpec {
            name: "hclk",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is HCLK; it must hang off `sysclk`",
        },
        PropertySpec {
            name: "pclk1",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is PCLK1; it must hang off `hclk`",
        },
        PropertySpec {
            name: "pclk2",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is PCLK2; it must hang off `hclk`",
        },
        PropertySpec {
            name: "timclk1",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is what an APB1 timer counts; it must \
                      hang off `pclk1`, and is PCLK1 doubled unless PPRE1 is one",
        },
        PropertySpec {
            name: "timclk2",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is what an APB2 timer counts; it must \
                      hang off `pclk2`, and is PCLK2 doubled unless PPRE2 is one",
        },
        PropertySpec {
            name: "pclk3",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is PCLK3 (a U5's APB3); it must hang off \
                      `hclk`",
        },
        PropertySpec {
            name: "hclk2",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is HCLK2 (a WB's CPU2, SYSCLK / C2HPRE); it \
                      must hang off `sysclk`",
        },
        PropertySpec {
            name: "hclk4",
            kind: ValueKind::Str,
            required: false,
            summary: "the object whose clock domain is HCLK4 (a WB's shared AHB4 and flash, \
                      SYSCLK / SHDHPRE); it must hang off `sysclk`",
        },
    ],
    construct: |props| Ok(Box::new(Rcc::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rcc::new(props)?)))
}

/// What the validator should know about `st.rcc`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(&["f4", "l4", "u5", "wb"]))
        .prop(PropSchema::new("hse", ValueKind::Uint))
        .prop(PropSchema::new("hsi", ValueKind::Uint))
        .prop(PropSchema::new("lse", ValueKind::Uint))
        .prop(PropSchema::new("lsi", ValueKind::Uint))
        .prop(PropSchema::new("ready-delay", ValueKind::Uint))
        .region("")
        .region("regs")
        .port(RTCEN_PIN, PortDir::Out)
        .port(BDRST_PIN, PortDir::Out)
        .port(IRQ_PIN, PortDir::Out)
        .port(DBP_PIN, PortDir::In)
        // A WB's `pwr.smpsen -> rcc.smpsen`; another variant refuses it.
        .port(SMPSEN_PIN, PortDir::In);
    // The driven clock outputs, from the one table that says what they are.
    for (_, prop) in OUTPUT_TREE {
        schema = schema.prop(PropSchema::new(*prop, ValueKind::Str));
    }
    // The union of both layouts' pins: a `wire` to one this variant does not
    // have is refused by the device, which is where the variant is known.
    for bank in ALL_BANKS {
        schema = schema.port_bank(*bank, PortDir::Out, BANK_WIDTH);
    }
    for cause in ALL_CAUSES {
        schema = schema.port(*cause, PortDir::In);
    }
    schema
}

#[cfg(test)]
mod tests;
