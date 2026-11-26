# STM32WB55 / STM32WB35 peripheral inventory

Scratch input for `machines/stm32wb55.machine`, `machines/stm32wb35.machine`
and the script that generates both. Every row is what a board file needs:
where the block is, how much to map, which class and properties model it,
its CPU1 interrupt number(s), and the RCC bits that gate it. It may be folded
into a platform document once the boards exist.

## Sources

| Source | What it gave |
| --- | --- |
| ST **RM0434** rev 16 (06-Mar-2026), STM32WB55xx/WB35xx reference manual | behaviour, reset values, Table 1 (memory map and register boundaries, §2.2.2), Tables 72/73 (CPU1/CPU2 vectors, §15.4), Table 74 (wakeup/EXTI lines, §15.5), Table 75 (EXTI implementation), Tables 63/64 (DMAMUX), §10.5 (GPIO reset values), §42 (electronic signature) |
| ST CMSIS `stm32wb55xx.h` / `stm32wb35xx.h`, CMSIS device WB **v1.12.4** (Apache-2.0) | every `*_BASE`, `IRQn_Type`, `*_TypeDef` offset and `RCC_*ENR_*EN_Pos` below |
| ST **DS11929** rev 18 (STM32WB55xx/WB35xx datasheet) | Table 2: per-part flash/SRAM/peripheral counts and packages |
| STM32CubeWB `Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x` and `STM32WB3x` `Release_Notes.html` (v1.25.0) | where ST's wireless stacks install, hence the default `SFSA` |

No emulator source was consulted.

## Parts

| | WB55Cx | WB55Rx | WB55Vx | WB35Cx |
| --- | --- | --- | --- | --- |
| Flash | 256K / 512K / 1M | 256K / 512K / 1M | 256K / 512K / 1M (and 640K, `xY`) | 256K (CC) / 512K (CE) |
| SRAM1 at `0x2000_0000` | 64K on 256K-flash parts, else 192K | same | same | **32K** |
| SRAM2a at `0x2003_0000` | 32K | 32K | 32K | 32K |
| SRAM2b at `0x2003_8000` | 32K | 32K | 32K | 32K |
| SRAM2 CPU1 mirror | `0x1000_0000`, 64K (RM0434 Table 1, "SRAM2a/b CPU1 mirror") | | | |
| Package / `PKG[4:0]` | UFQFPN48, `0x0A` | VFQFPN68, `0x13` | WLCSP100 / UFBGA129, `0x11` | UFQFPN48, `0x0A` |
| GPIOs (DS11929 Table 2) | 30 | 49 | 72 | 30 |
| SPI | 1 | 2 | 2 | **1** (no SPI2 at all) |
| LCD / TSC | yes / no | yes / 6 ch | yes / 18 ch | **no / no** |
| Tamper / wakeup pins | 1 / 2 | 3 / 5 | 3 / 5 | 1 / 2 |
| `DBGMCU.IDCODE` | `0x2001_0495` (rev Y) / `0x2003_0495` (rev X) | | | `0x2001_0495` (rev A) / `0x2003_0495` (rev X) |
| UID64 device byte | `0x26` rev Y, `0x27` rev X | | | `0x26` rev A, `0x27` rev X |
| Default `SFSA` (full BLE stack v1.25.0) | 1M: `0xD0`; 640K: `0x7C`; 512K: `0x5C`; 256K: `0x1C` | | | 512K: `0x5C`; 256K: `0x1C` |

`SRAM1_SIZE` is `0x30000` in `stm32wb55xx.h` and `0x8000` in
`stm32wb35xx.h`; `SRAM2A_SIZE`/`SRAM2B_SIZE` are `0x8000` in both. RM0434
Table 1 sizes the flash and SRAM1 windows "depends upon device"; the 64K
SRAM1 of the 256K-flash WB55 parts is DS11929 Table 2.

### What the WB35 lacks (header diff)

* `*_BASE` present in `stm32wb55xx.h` and absent from `stm32wb35xx.h`:
  `GPIOD_BASE`, `LCD_BASE`, `SPI2_BASE`, `TSC_BASE`. (GPIOE and GPIOH remain.)
* `IRQn_Type` entries absent from the WB35: **`SPI2_IRQn` = 35, `TSC_IRQn` =
  39, `LCD_IRQn` = 49** (RM0434 Table 71 says the same). Every other number is
  identical, and nothing exists on the WB35 that the WB55 lacks.
* RCC bits absent from the WB35: `AHB1ENR/RSTR.TSC` (16), `AHB2ENR/RSTR.GPIOD`
  (3), `APB1ENR1/RSTR1.LCD` (9), `APB1ENR1/RSTR1.SPI2` (14), and the CPU2
  (`C2*`) and sleep-mode (`*SMENR`) copies of the same.
* EXTI line 43 (LCD wakeup) does not exist (RM0434 Table 75: "0 to 42, 44 to
  48"; `EXTI_IMR2_IM43` absent): `st.exti` takes `direct-lines =
  0x174f073c00000` (`WB35_DIRECT_LINES`).
* SYSCFG: `EXTICR` ports C/D/E/H codes are WB55-only for most lines (RM0434
  §11.2.3–6 "(STM32WB55xx only)"); `C2IMR2` bits 22:21 (LCD, TSC) reserved.
  Nothing in the model needs a property for either: an unwired port reads low.
* DMAMUX request lines 8/9 (SPI2_RX/TX) have no source (RM0434 Table 62).
* FLASH: 128 pages; `PCROPxy` bit 8 and `WRP1x` bits 7/23 reserved — the
  model derives nothing from them; `size = 512K` or `256K` is the whole change.
* The part number codification word at `0x1FFF_77DC` reads `0x0000_3533` on a
  WB35 (RM0434 §42.4) and is factory-variable on a WB55. Not modelled.

## Memory map

| Region | Base | Size | Object |
| --- | --- | --- | --- |
| User flash | `0x0800_0000` | part size | `st.flash` `variant = "wb"`, region `array` |
| Boot alias (CPU1 `0x0000_0000`) | `0x0000_0000` | 1M | `st.syscfg` `boot` region, `boot-sources = [0x08000000, 0x1FFF0000, 0, 0x20000000, 0, 0, 0x90000000]` (MEM_MODE 0 flash, 1 system, 3 SRAM1, 6 QUADSPI) |
| System memory (bootloader) | `0x1FFF_0000` | 28K | `rom`, if an image is supplied |
| OTP | `0x1FFF_7000` | 1K | not modelled |
| Signature | `0x1FFF_7500` | `0xE2` | `st.devinfo` (below) |
| Option bytes | `0x1FFF_8000` | 128 B | not modelled (the controller's option registers are) |
| SRAM1 | `0x2000_0000` | 192K / 64K / 32K | `ram` |
| SRAM2a | `0x2003_0000` | 32K | `ram` |
| SRAM2b | `0x2003_8000` | 32K | `ram` |
| SRAM2 mirror | `0x1000_0000` | 64K | the same two `ram`s, mapped twice |
| QUADSPI memory | `0x9000_0000` | 256M | no class (`stm32.octospi` is not the QUADSPI) |

The secure parts of SRAM2a/SRAM2b (`SRRVR.SBRSA`/`SNBRSA`, 1K pages; DS11929
chapter 5: CPU2-only, no CPU1 or DMA access) are not enforced by any object.

## Peripherals

`Map` is the size of the class's region, which is what a `map` statement
should give; the hardware window is 1K unless RM0434 Table 1 says otherwise.
RCC registers are offsets in `RCC_TypeDef`: `AHB1ENR` `0x48`, `AHB2ENR`
`0x4C`, `AHB3ENR` `0x50` (AHB3 *and* AHB4), `APB1ENR1` `0x58`, `APB1ENR2`
`0x5C`, `APB2ENR` `0x60`; each `*RSTR*` is `0x20` below its `*ENR*` (`AHB1RSTR`
`0x28` … `APB2RSTR` `0x40`) and uses the same bit. CPU2 has its own enables at
`C2AHB1ENR` `0x148` … `C2APB3ENR` `0x164`, same bit positions.

| Peripheral | Base | Map | Class and properties | CPU1 IRQ | RCC enable / reset bit |
| --- | --- | --- | --- | --- | --- |
| FLASH | `0x5800_4000` | `0x88` | `st.flash` `variant = "wb"`, `size = 1M`/`512K`/`256K`; optional `sfr`, `srrvr`, `optr`; regions `regs`, `array` | 4 `FLASH_IRQn` | `AHB3ENR.FLASHEN` 25 / `AHB3RSTR.FLASHRST` 25 |
| RCC | `0x5800_0000` | — | `st.rcc` (WB variant: other workstream) | 5 `RCC_IRQn` | — |
| PWR | `0x5800_0400` | — | `st.pwr` (WB variant: other workstream) | 1 `PVD_PVM_IRQn` (via EXTI 16/31/33), 21 `C2SEV_PWR_C2H_IRQn`, 43 `PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQn` | — |
| EXTI | `0x5800_0800` | `0x400` | `st.exti` `variant = "wb"`; WB35: `direct-lines = 0x174f073c00000` | 6–10 `EXTI0`–`EXTI4`, 23 `EXTI9_5`, 40 `EXTI15_10` | none (always clocked) |
| IPCC | `0x5800_0C00` | `0x400` | `st.ipcc` | 44 `IPCC_C1_RX`, 45 `IPCC_C1_TX` | `AHB3ENR.IPCCEN` 20 / `AHB3RSTR.IPCCRST` 20 |
| RNG | `0x5800_1000` | `0x0C` | `st.rng` `ced = true` (the `f4` file plus `CR.CED`; RM0434 §24.7) | 53 `RNG_IRQn` | `AHB3ENR.RNGEN` 18 / `AHB3RSTR.RNGRST` 18 |
| HSEM | `0x5800_1400` | `0x400` | `st.hsem` | 46 `HSEM_IRQn` | `AHB3ENR.HSEMEN` 19 / `AHB3RSTR.HSEMRST` 19 |
| AES2 | `0x5800_1800` | 1K | **no class** | 52 `AES2_IRQn` | `AHB3ENR.AES2EN` 17 / 17 |
| PKA (+ RAM `0x5800_2400`) | `0x5800_2000` | 1K + 4K | **no class** | 29 `PKA_IRQn` | `AHB3ENR.PKAEN` 16 / 16 |
| QUADSPI regs | `0xA000_1000` | 1K | **no class** | 50 `QUADSPI_IRQn` | `AHB3ENR.QUADSPIEN` 8 / 8 |
| DMA1 | `0x4002_0000` | `0xAC` | `st.dma` `variant = "channel"` (`CSELR` at `+0xA8` decodes but is reserved on a WB — the DMAMUX routes instead) | 11–17 `DMA1_Channel1`–`7` (pins `irq1`–`irq7`) | `AHB1ENR.DMA1EN` 0 / 0 |
| DMA2 | `0x4002_0400` | `0xAC` | `st.dma` `variant = "channel"` | 55–61 `DMA2_Channel1`–`7` | `AHB1ENR.DMA2EN` 1 / 1 |
| DMAMUX1 | `0x4002_0800` | `0x148` | `st.dmamux` `channels = 14`, `requests = 64`; `ch0`–`ch6` → `dma1.req1`–`req7`, `ch7`–`ch13` → `dma2.req1`–`req7` | 62 `DMAMUX1_OVR_IRQn` | `AHB1ENR.DMAMUX1EN` 2 / 2 |
| CRC | `0x4002_3000` | `0x18` | `st.crc` `variant = "u5"` (v2 with a 32-bit `IDR`, RM0434 §17.4.2) | — | `AHB1ENR.CRCEN` 12 / 12 |
| TSC (WB55 only) | `0x4002_4000` | 1K | **no class** | 39 `TSC_IRQn` | `AHB1ENR.TSCEN` 16 / 16 |
| GPIOA | `0x4800_0000` | `0x2C` | `st.gpio` `variant = "wb"`, `moder-reset = 0xABFFFFFF`, `ospeedr-reset = 0x0C000000`, `pupdr-reset = 0x64000000` | — (EXTI) | `AHB2ENR.GPIOAEN` 0 / 0 |
| GPIOB | `0x4800_0400` | `0x2C` | `st.gpio` `variant = "wb"`, `moder-reset = 0xFFFFFEBF`, `ospeedr-reset = 0x000000C0`, `pupdr-reset = 0x00000100` | — | `AHB2ENR.GPIOBEN` 1 / 1 |
| GPIOC | `0x4800_0800` | `0x2C` | `st.gpio` `variant = "wb"`, `moder-reset = 0xFFFFFFFF` | — | `AHB2ENR.GPIOCEN` 2 / 2 |
| GPIOD (WB55 only) | `0x4800_0C00` | `0x2C` | `st.gpio` `variant = "wb"`, `moder-reset = 0xFFFFFFFF` | — | `AHB2ENR.GPIODEN` 3 / 3 |
| GPIOE | `0x4800_1000` | `0x2C` | `st.gpio` `variant = "wb"`, `moder-reset = 0x000003FF` (PE0–PE4) | — | `AHB2ENR.GPIOEEN` 4 / 4 |
| GPIOH | `0x4800_1C00` | `0x2C` | `st.gpio` `variant = "wb"`, `moder-reset = 0x000000CF` (PH0, PH1, PH3) | — | `AHB2ENR.GPIOHEN` 7 / 7 |
| ADC1 | `0x5004_0000` | 1K | **no class** | 18 `ADC1_IRQn` | `AHB2ENR.ADCEN` 13 / 13 |
| AES1 | `0x5006_0000` | 1K | **no class** | 51 `AES1_IRQn` | `AHB2ENR.AES1EN` 16 / 16 |
| TIM2 | `0x4000_0000` | `0x64` | `st.tim` `variant = "general"`, `width = 32`, `channels = 4` | 28 `TIM2_IRQn` | `APB1ENR1.TIM2EN` 0 / `APB1RSTR1.TIM2RST` 0 |
| LCD (WB55 only) | `0x4000_2400` | 1K | **no class** | 49 `LCD_IRQn` | `APB1ENR1.LCDEN` 9 / 9 |
| RTC & TAMP | `0x4000_2800` | `0xD0` | `st.rtc` (RM0351 map; a WB has 20 `BKPxR`, `0x50`–`0x9C`, and the model's `BKP20R`–`BKP31R` are reserved space) | 2 `TAMP_STAMP_LSECSS` (EXTI 18), 3 `RTC_WKUP` (EXTI 19), 41 `RTC_Alarm` (EXTI 17) | `APB1ENR1.RTCAPBEN` 10 (APB access) and `BDCR.RTCEN` 15; reset is `BDCR.BDRST` |
| WWDG | `0x4000_2C00` | `0x0C` | `st.wwdg` `wdgtb-shift = 11`, `wdgtb-width = 3` (RM0434 §33: `WDGTB[2:0]` at 13:11) | 0 `WWDG_IRQn` | `APB1ENR1.WWDGEN` 11 / none |
| IWDG | `0x4000_3000` | `0x14` | `st.iwdg` `window = true` (the `f4` file plus `WINR`; `PR[2:0]`, RM0434 §32.4) | — | none (LSI) |
| SPI2 (WB55 Rx/Vx) | `0x4000_3800` | `0x400` | `stm32.spi` `variant = "f7"` | 35 `SPI2_IRQn` | `APB1ENR1.SPI2EN` 14 / 14 |
| I2C1 | `0x4000_5400` | `0x2C` | `st.i2c-v2` | 30 `I2C1_EV`, 31 `I2C1_ER` | `APB1ENR1.I2C1EN` 21 / 21 |
| I2C3 | `0x4000_5C00` | `0x2C` | `st.i2c-v2` | 32 `I2C3_EV`, 33 `I2C3_ER` | `APB1ENR1.I2C3EN` 23 / 23 |
| CRS | `0x4000_6000` | 1K | **no class** | 42 `CRS_IRQn` | `APB1ENR1.CRSEN` 24 / 24 |
| USB FS | `0x4000_6800` | `0x60` | `st.usbfs` (the F0x2-style device: `LPMCSR` `+0x54`, `BCDR` `+0x58`) | 19 `USB_HP`, 20 `USB_LP` | `APB1ENR1.USBEN` 26 / 26 |
| USB packet memory | `0x4000_6C00` | `0x400` | `st.usbfs` region `pma` (1024 bytes, RM0434 §40) | — | — |
| LPTIM1 | `0x4000_7C00` | 1K | **no class** | 47 `LPTIM1_IRQn` | `APB1ENR1.LPTIM1EN` 31 / 31 |
| LPUART1 | `0x4000_8000` | `0x34` | `st.usart` `variant = "u5"`, `lpuart = true` (FIFO and `PRESC` present: RM0434 Table 228) | 37 `LPUART1_IRQn` | `APB1ENR2.LPUART1EN` 0 / `APB1RSTR2.LPUART1RST` 0 |
| LPTIM2 | `0x4000_9400` | 1K | **no class** | 48 `LPTIM2_IRQn` | `APB1ENR2.LPTIM2EN` 5 / 5 |
| SYSCFG + VREFBUF | `0x4001_0000` | `0x114` | `st.syscfg` `variant = "wb"`, `ports = 8`; for the boot alias `space`, `boot-size`, `boot-sources` | 54 `FPU_IRQn` is `CFGR1.FPU_IE`'s, not modelled | **none** — the WB's RCC has no `SYSCFGEN`; leave `enable`/`reset` unwired |
| COMP1/COMP2 | `0x4001_0200` | `0x08` | **no class** | 22 `COMP_IRQn` (EXTI 20/21) | — (SYSCFG/COMP share the APB2 clock) |
| TIM1 | `0x4001_2C00` | `0x64` | `st.tim` `variant = "advanced"`, `channels = 4` | 24 `TIM1_BRK` (`irq`), 25 `TIM1_UP_TIM16` (`irq-up`), 26 `TIM1_TRG_COM_TIM17` (`irq-trg`), 27 `TIM1_CC` (`irq-cc`) | `APB2ENR.TIM1EN` 11 / 11 |
| SPI1 | `0x4001_3000` | `0x400` | `stm32.spi` `variant = "f7"` (FIFO, `DS`, `FRXTH`) | 34 `SPI1_IRQn` | `APB2ENR.SPI1EN` 12 / 12 |
| USART1 | `0x4001_3800` | `0x34` | `st.usart` `variant = "u5"` (FIFO, `PRESC`; the U5's `AUTOCR` at `+0x30` is reserved on a WB and reads zero) | 36 `USART1_IRQn` | `APB2ENR.USART1EN` 14 / 14 |
| TIM16 | `0x4001_4400` | `0x64` | `st.tim` `variant = "advanced"`, `channels = 1` (it has `RCR`, `BDTR`, `CH1N`); wire all four `irq*` to 25 | 25 (shared with TIM1_UP) | `APB2ENR.TIM16EN` 17 / 17 |
| TIM17 | `0x4001_4800` | `0x64` | `st.tim` `variant = "advanced"`, `channels = 1`; all `irq*` to 26 | 26 (shared with TIM1_TRG_COM) | `APB2ENR.TIM17EN` 18 / 18 |
| SAI1 | `0x4001_5400` | 1K | **no class** | 38 `SAI1_IRQn` | `APB2ENR.SAI1EN` 21 / 21 |
| Radio (BLE/802.15.4/RF ctrl) | `0x6000_0000`–`0x6000_1FFF` | | CPU2's (`st.wb-cpu2`) | — | no enable bit in `stm32wb55xx.h`; `APB3RSTR.RFRST` 0 |
| DBGMCU | `0xE004_2000` | `0x54` | `st.dbgmcu` `variant = "wb"` (`DEV_ID` `0x495`, `REV_ID` `0x2001` by default) | — | — |
| Signature | `0x1FFF_7500` | `0xE2` | `st.devinfo` `size = 0xE2`, `package-offset = 0`, `package` per part, `uid64-offset = 0x80`, `uid-offset = 0x90`, `flash-size-offset = 0xE0`, `flash-kb` per part, `seed = seed` | — | — |
| CPU2 | — | — | `st.wb-cpu2` (other workstream) | — | — |

TIM1–TIM17 use the default `layout = "f4"`: the WB's `TIM_TypeDef` is the
L4 one, `DCR`/`DMAR` at `0x48`/`0x4C` and `OR` at `0x50`. `CCMR3`, `CCR5`,
`CCR6`, `AF1`, `AF2` (`0x54`–`0x64`) are not decoded by that layout, which is
the same limitation the L4 has; HAL's ordinary PWM/timebase paths do not touch
them.

## CPU1 interrupts (`IRQn_Type`, `stm32wb55xx.h`; RM0434 Table 72)

| IRQ | Name | Source in the model |
| --- | --- | --- |
| 0 | `WWDG_IRQn` | `wwdg` early wakeup |
| 1 | `PVD_PVM_IRQn` | `exti.irq16`, `irq31`, `irq33` (PVD, PVM1, PVM3) |
| 2 | `TAMP_STAMP_LSECSS_IRQn` | `exti.irq18` |
| 3 | `RTC_WKUP_IRQn` | `exti.irq19` |
| 4 | `FLASH_IRQn` | `flash.irq` |
| 5 | `RCC_IRQn` | `rcc` |
| 6–10 | `EXTI0_IRQn`…`EXTI4_IRQn` | `exti.irq0`…`irq4` |
| 11–17 | `DMA1_Channel1_IRQn`…`DMA1_Channel7_IRQn` | `dma1.irq1`…`irq7` |
| 18 | `ADC1_IRQn` | — |
| 19 | `USB_HP_IRQn` | `usb` |
| 20 | `USB_LP_IRQn` | `usb` |
| 21 | `C2SEV_PWR_C2H_IRQn` | `exti.irq40` (C2SEV) and PWR's CPU2-hold wakeup |
| 22 | `COMP_IRQn` | `exti.irq20`, `irq21` |
| 23 | `EXTI9_5_IRQn` | `exti.irq5`…`irq9` |
| 24 | `TIM1_BRK_IRQn` | `tim1.irq` |
| 25 | `TIM1_UP_TIM16_IRQn` | `tim1.irq-up`, all of `tim16` |
| 26 | `TIM1_TRG_COM_TIM17_IRQn` | `tim1.irq-trg`, all of `tim17` |
| 27 | `TIM1_CC_IRQn` | `tim1.irq-cc` |
| 28 | `TIM2_IRQn` | `tim2.irq` |
| 29 | `PKA_IRQn` | — |
| 30 | `I2C1_EV_IRQn` | `i2c1` |
| 31 | `I2C1_ER_IRQn` | `i2c1` |
| 32 | `I2C3_EV_IRQn` | `i2c3` |
| 33 | `I2C3_ER_IRQn` | `i2c3` |
| 34 | `SPI1_IRQn` | `spi1` |
| 35 | `SPI2_IRQn` (**WB55 only**) | `spi2` |
| 36 | `USART1_IRQn` | `usart1.irq` |
| 37 | `LPUART1_IRQn` | `lpuart1.irq` |
| 38 | `SAI1_IRQn` | — |
| 39 | `TSC_IRQn` (**WB55 only**) | — |
| 40 | `EXTI15_10_IRQn` | `exti.irq10`…`irq15` |
| 41 | `RTC_Alarm_IRQn` | `exti.irq17` |
| 42 | `CRS_IRQn` | — |
| 43 | `PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQn` | `pwr` |
| 44 | `IPCC_C1_RX_IRQn` | `ipcc` |
| 45 | `IPCC_C1_TX_IRQn` | `ipcc` |
| 46 | `HSEM_IRQn` | `hsem` |
| 47 | `LPTIM1_IRQn` | — |
| 48 | `LPTIM2_IRQn` | — |
| 49 | `LCD_IRQn` (**WB55 only**) | — |
| 50 | `QUADSPI_IRQn` | — |
| 51 | `AES1_IRQn` | — |
| 52 | `AES2_IRQn` | — |
| 53 | `RNG_IRQn` | `rng.irq` |
| 54 | `FPU_IRQn` | — |
| 55–61 | `DMA2_Channel1_IRQn`…`DMA2_Channel7_IRQn` | `dma2.irq1`…`irq7` |
| 62 | `DMAMUX1_OVR_IRQn` | `dmamux.irq` |

That is 63 vectors (RM0434 §15.1: "63 maskable interrupt channels"), so the
CPU1 `cpu.arm.v7m` needs at least 63 external IRQs. The `exti.irq{n}` request
is the same one for both CPUs (RM0434 Table 79 note 1); with RTC, PVD and COMP
the EXTI line *is* the interrupt — wire the EXTI pin, not the peripheral.

CPU2's 32 vectors (RM0434 Table 73) are mostly ORs of several sources gated by
`SYSCFG_C2IMRx`: 1 PVD/PVM, 2 RTC (wakeup, tamper, alarm), 3 USB/CRS, 4
RCC/FLASH/C1SEV (EXTI 41), 5 EXTI1:0, 6 EXTI3:2, 7 EXTI15:4, 8 TSC/802_IT0, 9
DMA1_CH3:1, 10 DMA1_CH7:4, 11 DMA2 + DMAMUX, 12 ADC/COMP, 13 LPTIM1, 14
LPTIM2, 15 TIM1 (all), 16 TIM2, 17 TIM16, 18 TIM17, 19 IPCC C2 RX/TX + HSEM,
20 PKA/RNG/AES1, 21 AES2, 22 LCD/802_IT1, 23 I2C1, 24 I2C3, 25 SPI1, 26 SPI2,
27 USART1, 28 LPUART1, 29 SAI1, 30 BLE, 31 802.15.4. Only relevant if CPU2 is
ever an executing core.

## EXTI lines (RM0434 Table 74)

| Line | Source | Kind | Event |
| --- | --- | --- | --- |
| 0–15 | `syscfg.exti0`…`exti15` | configurable | yes |
| 16 | PVD | configurable | no |
| 17 | RTC alarm A/B | configurable | yes |
| 18 | RTC tamper / timestamp / LSE CSS | configurable | yes |
| 19 | RTC wakeup | configurable | yes |
| 20, 21 | COMP1, COMP2 | configurable | yes |
| 22, 23 | I2C1, I2C3 wakeup | direct | — |
| 24, 25 | USART1, LPUART1 wakeup | direct | — |
| 26, 27 | reserved | | |
| 28 | USB wakeup | direct | — |
| 29, 30 | LPTIM1, LPTIM2 | direct | — |
| 31 | PVM1 | configurable | no |
| 32 | reserved | | |
| 33 | PVM3 | configurable | no |
| 34, 35 | reserved | | |
| 36, 37 | IPCC CPU1, CPU2 | direct | — |
| 38, 39 | HSEM interrupt 0 (CPU1), 1 (CPU2) | direct | — |
| 40 | C2SEV (CPU2's `SEV`) | configurable | yes |
| 41 | C1SEV (CPU1's `SEV`) | configurable | yes |
| 42 | flash ECC / global | direct | — |
| 43 | LCD wakeup (**WB55 only**) | direct | — |
| 44 | HSE CSS | direct | — |
| 45, 46 | BLE, 802.15.4 | direct | — |
| 47 | reserved | | |
| 48 | `CDBGPWRUPREQ` | direct | — |

## DMAMUX1 request lines (RM0434 Table 64)

1–4 generators; 5 ADC1; 6/7 SPI1_RX/TX; 8/9 SPI2_RX/TX (**WB55 only**); 10/11
I2C1_RX/TX; 12/13 I2C3_RX/TX; 14/15 USART1_RX/TX; 16/17 LPUART1_RX/TX; 18/19
SAI1_A/B; 20 QUADSPI; 21–24 TIM1_CH1–CH4; 25 TIM1_UP; 26 TIM1_TRIG; 27
TIM1_COM; 28–31 TIM2_CH1–CH4; 32 TIM2_UP; 33/34 TIM16_CH1/UP; 35/36
TIM17_CH1/UP; 37/38 AES1_IN/OUT; 39/40 AES2_IN/OUT. Trigger and sync inputs:
0–15 EXTI lines 0–15, 16/17 `dmamux_evt0`/`1`, 18/19 LPTIM1_OUT/LPTIM2_OUT
(RM0434 Table 65).

## Flash and the secure area

* Pages are 4K; `PNB[7:0]`. The default `SFR` is `DDS | SFSA` with the
  `SFSA` of the parts table, i.e. the application may use
  `0x0800_0000`…`0x0800_0000 + SFSA × 4K − 1` (832K on a 1M part, 368K on a
  512K part, 112K on 256K). A board that wants all of the flash for CPU1
  writes `sfr = 0x1100` (FSD set, no secure area).
* Default `SRRVR` = `C2OPT | SNBRSA 0x0F | SBRSA 0x0A | SBRV = SFSA×4K/4`.
  The two SRAM2 page numbers are a value reported from a working WB55 with
  the BLE stack (ST community), not from a manual — **unverified**.
* `IPCCBR` resets to 0, so the IPCC mailbox buffer (and `st.wb-cpu2`'s
  reference table) is at the start of SRAM2a, `0x2003_0000`.
* What CPU1 sees in the secure area: reads return zero and set `SR.RDERR`,
  fetches included; programs and page erases are bus errors; a debugger is
  refused. The `image` property and `Flash::load_image` still write there.

## Reset values a board must give

* `st.syscfg` `memrmp-reset`: `MEM_MODE` follows BOOT0/nBOOT1 (RM0434 §2.3);
  0 for boot from flash.
* `st.gpio` `*-reset`: the GPIO rows above (RM0434 §10.5.1–4).
* `st.devinfo` `package`: `0x0A` (UFQFPN48: WB55Cx, WB35Cx), `0x13`
  (VFQFPN68: WB55Rx), `0x11` (WLCSP100/UFBGA129: WB55Vx); `flash-kb` the part
  size in KiB.

## Peripherals with no class in the tree

ADC1, AES1, AES2, PKA, QUADSPI, SAI1, LPTIM1, LPTIM2, CRS, COMP, TSC, LCD,
the radio blocks. A board that leaves them unmapped takes a bus fault on the
first access; `UnassignedPolicy` or stub RAM is the board's choice. HAL's
`SystemClock_Config` for a WB touches none of them; ST's BLE examples enable
AES1/PKA/RNG clocks for CPU2 via `RCC_C2*ENR` but do not access them from
CPU1.

## Open points

* `FLASH_C2CR`'s reset value: RM0434 §3.10.17 heads it `0xC000 0000` while its
  own bit table has bits 31:27 reserved and Table 18 and ST's SVD give zero;
  the model uses zero.
* `st.rcc`/`st.pwr` WB variants (enable/reset wires, `C2BOOT`) are another
  workstream's; the RCC bits above are from `stm32wb55xx.h` and must match.
* Whether `RCC_APB3ENR` has an `RFEN` bit: `stm32wb55xx.h` defines only
  `RCC_APB3RSTR_RFRST` (bit 0); the radio clock is CPU2's business.
