#!/usr/bin/env python3
"""Generate machines/stm32wb55.machine and machines/stm32wb35.machine.

Usage: scripts/gen-stm32wb-machines.py machines

The two parts are one reference manual (RM0434) and one die family: the WB35
is the WB55 with less SRAM1 and flash and without GPIOD, the LCD, SPI2 and the
touch-sensing controller. A `.machine` file has no include the catalog can
resolve, so both are written from here, from the table in
docs/platforms/stm32wb-inventory.md.
"""
import sys

PARTS = {
    "stm32wb55": dict(
        title="STMicroelectronics STM32WB55RGV6",
        package="VFQFPN68",
        package_code=0x13,
        flash="1M",
        flash_kb=1024,
        sram1="192K",
        ports="abcdeh",
        wb35=False,
        other="stm32wb35",
    ),
    "stm32wb35": dict(
        title="STMicroelectronics STM32WB35CEU6A",
        package="UFQFPN48",
        package_code=0x0A,
        flash="512K",
        flash_kb=512,
        sram1="32K",
        ports="abceh",
        wb35=True,
        other="stm32wb55",
    ),
}

# RM0434 §10.5.1–4 reset values: the debug pins on A and B, the bonded pins of
# E and H; every other pin is analog.
GPIO_RESET = {
    "a": "moder-reset = 0xabffffff, ospeedr-reset = 0x0c000000, pupdr-reset = 0x64000000",
    "b": "moder-reset = 0xfffffebf, ospeedr-reset = 0x000000c0, pupdr-reset = 0x00000100",
    "c": "moder-reset = 0xffffffff",
    "d": "moder-reset = 0xffffffff",
    "e": "moder-reset = 0x000003ff",
    "h": "moder-reset = 0x000000cf",
}
# AHB2ENR / AHB2RSTR bit of each port (`RCC_AHB2ENR_GPIOxEN_Pos`).
GPIO_BIT = {"a": 0, "b": 1, "c": 2, "d": 3, "e": 4, "h": 7}
# The address of each port: 0x48000000 + 0x400 × (letter index).
GPIO_OFF = {"a": 0x0000, "b": 0x0400, "c": 0x0800, "d": 0x0c00, "e": 0x1000, "h": 0x1c00}


def header(name, p):
    if p["wb35"]:
        what = (
            "A Cortex-M4 with FPv4-SP-D16 and a Cortex-M0+ radio coprocessor, 512 KiB\n"
            "# of flash, 32 KiB of SRAM1 and 64 KiB of SRAM2 (2a + 2b), in the UFQFPN48\n"
            "# package. It is the WB55 without GPIOD, the LCD, SPI2 and the TSC (RM0434\n"
            "# Table 71, DS11929 Table 2): see machines/stm32wb55.machine."
        )
    else:
        what = (
            "A Cortex-M4 with FPv4-SP-D16 and a Cortex-M0+ radio coprocessor, 1 MiB of\n"
            "# flash, 192 KiB of SRAM1 and 64 KiB of SRAM2 (2a + 2b), in the VFQFPN68\n"
            "# package used on ST's NUCLEO-WB55RG. The WB35 is the same family with\n"
            "# less memory and fewer peripherals: see machines/stm32wb35.machine."
        )
    return f"""# {name} — an {p['title']}.
#
#   rsemu run {name} --media firmware=app.bin
#
# {what}
#
# Generated, with its sibling, by scripts/gen-stm32wb-machines.py: the two
# parts come from one description and cannot drift. Edit the script, not this
# file.
#
# Sources: ST **RM0434** rev 16 (STM32WB55xx/WB35xx reference manual) for the
# memory map (Table 1), the vector table (Table 72), the EXTI lines (Table 74)
# and every peripheral placed below; ST's CMSIS device headers
# `stm32wb55xx.h` / `stm32wb35xx.h` (Apache-2.0, cmsis-device-wb v1.12.4) for
# every base address, `IRQn_Type` and `RCC_*ENR_*EN_Pos`; ST **DS11929** rev
# 18 for the per-part memory sizes. No emulator source of any licence was
# consulted (ROADMAP.md §1). docs/platforms/stm32wb.md has the full account
# and docs/platforms/stm32wb-inventory.md the table this file is drawn from.
#
# ---------------------------------------------------------------------------
# The two cores
# ---------------------------------------------------------------------------
#
# * **CPU1** is `cpu`, the Cortex-M4 your firmware runs on.
# * **CPU2** is `cpu2`, and it is **not an executing core**. On the silicon it
#   is a Cortex-M0+ running ST's encrypted wireless stack, which cannot be run
#   here; `st.wb-cpu2` is a bus master that behaves like that stack instead
#   (docs/devices/stm32wb-cpu2.md). Firmware sets `PWR_CR4.C2BOOT`, CPU2 reads
#   the mailbox reference table at the start of SRAM2a, posts the system READY
#   event through IPCC channel 2, and from then on answers SHCI commands and
#   the BLE HCI/ACI command set — GAP, GATT, advertising — exactly in ST's
#   formats. **There is no radio**: advertising goes nowhere, no peer ever
#   connects, and no scan ever reports anything.
# * The flash's secure area — where ST's stack would sit — is real: `SFR`
#   defaults to where the full BLE stack v1.25 installs, and CPU1 cannot read,
#   program or erase above it.
#
# ---------------------------------------------------------------------------
# What is not here
# ---------------------------------------------------------------------------
#
# ADC1, AES1, AES2, PKA, QUADSPI, SAI1, LPTIM1/2, CRS, COMP, TSC and LCD:
# none has a class in the tree, and an access to any of their addresses is a
# BusFault. I2C1, I2C3 and the SPIs are placed with nothing on their buses;
# the SPIs drive no DMA request.
"""


def body(name, p):
    out = []
    w = out.append
    w(f'machine "{name}" {{')
    w('  param console = "console"     # USART1 — the ST-LINK virtual COM port')
    w(f"  param flash   = {p['flash']}")
    w("  param seed    = 1")
    w("")
    w("  # The memory map, RM0434 Table 1.")
    w("  param flash-base   = 0x08000000")
    w("  param sram1-base   = 0x20000000")
    w("  param sram2a-base  = 0x20030000   # the mailbox reference table is its first word")
    w("  param sram2b-base  = 0x20038000")
    w("  param sram2-mirror = 0x10000000   # CPU1's alias of SRAM2a + SRAM2b")
    w("  param apb1-base    = 0x40000000")
    w("  param apb2-base    = 0x40010000")
    w("  param ahb1-base    = 0x40020000")
    w("  param ahb2-base    = 0x48000000")
    w("  param ahb4-base    = 0x58000000   # RCC, PWR, EXTI, IPCC, RNG, HSEM, FLASH: shared by both CPUs")
    w("  param dbgmcu-base  = 0xE0042000")
    w("")
    w("  # The reference oscillator is the 32 MHz radio crystal every WB board fits")
    w("  # (RM0434 §6.2.2: HSE32 is required for the RF). The MSI's reset 4 MHz is")
    w("  # hse / 8; every internal source is an exact ratio of it")
    w("  # (docs/techniques/clock-control.md, \"It does not reparent\").")
    w("  osc hse = 32000000 Hz")
    w("  osc lsi = 32000 Hz")
    w("  osc lse = 32768 Hz")
    w("  # One tick of a USART's domain is one character time: 115200 baud, 10")
    w("  # bits a character. The baud rate the firmware programs is not modelled.")
    w("  osc uartclk = 11520 Hz")
    w("")
    w("  # The clock tree RCC drives, at its reset values: SYSCLK on the MSI at")
    w("  # 4 MHz, every prescaler at /1. HCLK2 is CPU2's (EXTCFGR.C2HPRE), HCLK4")
    w("  # the shared AHB4 and flash (EXTCFGR.SHDHPRE). RCC re-rates these.")
    w("  object sysclk  \"clock\" { clock = hse / 8 }")
    w("  object hclk    \"clock\" { clock = sysclk }")
    w("  object hclk2   \"clock\" { clock = sysclk }")
    w("  object hclk4   \"clock\" { clock = sysclk }")
    w("  object pclk1   \"clock\" { clock = hclk }")
    w("  object pclk2   \"clock\" { clock = hclk }")
    w("  object timclk1 \"clock\" { clock = pclk1 }")
    w("  object timclk2 \"clock\" { clock = pclk2 }")
    w("")
    w("  space mem { width = 32 }")
    w("")
    w("  # CPU1. `__NVIC_PRIO_BITS` is 4; 63 external interrupts (RM0434 §15.1).")
    w("  object cpu \"cpu.arm.v7m\" {")
    w("    clock         = hclk")
    w("    space         = mem")
    w("    part          = \"cortex-m4f\"")
    w("    priority-bits = 4")
    w("  }")
    w("")
    w("  # CPU2 and its wireless stack, as a bus master on the same map: it reads")
    w("  # the reference table CPU1 left at SRAM2a and talks through IPCC's")
    w("  # processor-2 registers. `seed` makes its static random address and")
    w("  # LE_Rand repeatable.")
    w("  object cpu2 \"st.wb-cpu2\" { clock = hclk2, space = mem, seed = seed }")
    w("")
    w("  object eflash \"st.flash\" {")
    w("    clock   = hclk4")
    w("    variant = \"wb\"")
    w("    size    = flash")
    w("    image   = \"firmware\"")
    w("  }")
    w(f"  object sram1  \"ram\" {{ size = {p['sram1']} }}")
    w("  object sram2a \"ram\" { size = 32K }")
    w("  object sram2b \"ram\" { size = 32K }")
    w("")
    w("  object rcc \"st.rcc\" {")
    w("    clock   = hse")
    w("    variant = \"wb\"")
    w("    hse     = 32000000")
    w("    lsi     = 32000")
    w("    lse     = 32768")
    w("    sysclk  = \"sysclk\", hclk = \"hclk\", hclk2 = \"hclk2\", hclk4 = \"hclk4\"")
    w("    pclk1   = \"pclk1\", pclk2 = \"pclk2\"")
    w("    timclk1 = \"timclk1\", timclk2 = \"timclk2\"")
    w("  }")
    w("  object pwr \"st.pwr\" { clock = hse, variant = \"wb\" }")
    w("")
    w("  # MEM_MODE (RM0434 §11.2.1): 0 flash, 1 system memory, 3 SRAM1, 6 QUADSPI.")
    w("  # There is no system-bootloader image and no QUADSPI here, so those are 0")
    w("  # (\"no such memory\") and a fetch through them faults.")
    w("  object syscfg \"st.syscfg\" {")
    w("    variant      = \"wb\"")
    w("    ports        = 8")
    w("    space        = mem")
    w("    boot-size    = flash")
    w("    boot-sources = [ flash-base, 0, 0, sram1-base ]")
    w("  }")
    if p["wb35"]:
        w("  # The WB35 has no line 43 (the LCD wakeup; RM0434 Table 75).")
        w("  object exti \"st.exti\" { variant = \"wb\", direct-lines = 0x174f073c00000 }")
    else:
        w("  object exti \"st.exti\" { variant = \"wb\" }")
    w("  object ipcc \"st.ipcc\" { }")
    w("  object hsem \"st.hsem\" { }")
    w("")
    w("  # The factory signature at 0x1FFF7500: PACKAGE (+0), UID64 (+0x80, which")
    w("  # ST's stack builds the public BD address from), UID96 (+0x90) and")
    w("  # FLASH_SIZE (+0xE0). The UIDs are drawn from `seed`.")
    w("  object devinfo \"st.devinfo\" {")
    w(f"    size = 0xe2, flash-kb = {p['flash_kb']}, seed = seed, package = 0x{p['package_code']:02x}")
    w("    package-offset = 0x0, uid64-offset = 0x80, uid-offset = 0x90, flash-size-offset = 0xe0")
    w("  }")
    w("  object dbgmcu \"st.dbgmcu\" { variant = \"wb\" }")
    w("")
    w(f"  # GPIO {', '.join(c.upper() for c in p['ports'])}. Reset values: RM0434 §10.5.")
    for c in p["ports"]:
        w(f"  object gpio{c} \"st.gpio\" {{ variant = \"wb\", {GPIO_RESET[c]} }}")
    w("")
    w("  # USART1 is the console (PB6/PB7 on the Nucleo, the ST-LINK VCP).")
    w("  object usart1  \"st.usart\" { clock = uartclk, variant = \"u5\", port = console }")
    w("  object lpuart1 \"st.usart\" { clock = uartclk, variant = \"u5\", port = \"lpuart1\", lpuart = true }")
    w("")
    w("  # Timers: the L4 register map (`layout = \"f4\"`). TIM2 is 32-bit; TIM16")
    w("  # and TIM17 have RCR/BDTR, so they are one-channel advanced timers.")
    w("  object tim1  \"st.tim\" { clock = timclk2, variant = \"advanced\", channels = 4 }")
    w("  object tim2  \"st.tim\" { clock = timclk1, variant = \"general\", channels = 4, width = 32 }")
    w("  object tim16 \"st.tim\" { clock = timclk2, variant = \"advanced\", channels = 1 }")
    w("  object tim17 \"st.tim\" { clock = timclk2, variant = \"advanced\", channels = 1 }")
    w("")
    w("  object iwdg \"st.iwdg\" { clock = lsi, window = true }")
    w("  # WWDG_CFR.WDGTB is three bits at 13:11 (RM0434 §33).")
    w("  object wwdg \"st.wwdg\" { clock = pclk1, wdgtb-shift = 11, wdgtb-width = 3 }")
    w("  object crc  \"st.crc\"  { variant = \"u5\" }")
    w("  # The RNG has its own clock; its numbers come from `seed`, not the host.")
    w("  object rng  \"st.rng\"  { clock = hse, seed = seed, ced = true }")
    w("  object rtc  \"st.rtc\"  { clock = lse, epoch = \"2024-01-01T00:00:00\" }")
    w("")
    w("  # The 1 KiB OTP area of the information block (RM0434 §3.3.1), as a")
    w("  # part straight from the factory has it: blank, 0xFF. Read-only here:")
    w("  # programming it through FLASH_CR.PG is not modelled.")
    w("  object otp \"rom\" { size = 1K, fill = 0xff }")
    w("")
    w("  # I2C1 and I2C3 (the v2 block, TIMINGR; RM0434 §34) and SPI1 (SPI2 on the")
    w("  # WB55 only): placed with nothing on their buses, so a driver initialises")
    w("  # them and a transfer finds an empty bus, as on a bare board.")
    w("  object i2c1 \"st.i2c-v2\" { clock = pclk1, link = \"wired\" }")
    w("  object i2c3 \"st.i2c-v2\" { clock = pclk1, link = \"wired\" }")
    w("  object spi1 \"stm32.spi\" { clock = pclk2, variant = \"f7\", link = \"wired\" }")
    if not p["wb35"]:
        w("  object spi2 \"stm32.spi\" { clock = pclk1, variant = \"f7\", link = \"wired\" }")
    w("")
    w("  # The USB full-speed device (RM0434 §40): the F0x2-style block with a")
    w("  # linear 1 KiB packet memory. It plugs into the named USB bus `usb`; a")
    w("  # host — a test harness, or anything that opens that bus — enumerates")
    w("  # the guest's device stack through it. Nothing else is on the bus.")
    w("  # Its clock is the 48 MHz USB clock its frame timer counts (\"fixed by")
    w("  # the requirements of the USB standard at 48 MHz\", §40.4.2); which")
    w("  # source CLK48SEL picks is not modelled, so it is an exact ratio of HSE.")
    w("  object usb \"st.usbfs\" { bus = \"usb\", clock = hse * 3 / 2 }")
    w("")
    w("  # DMA1/DMA2, seven channels each, routed by DMAMUX1's fourteen channels")
    w("  # (RM0434 §13–14).")
    w("  object dma1   \"st.dma\"    { clock = hclk, space = mem, variant = \"channel\" }")
    w("  object dma2   \"st.dma\"    { clock = hclk, space = mem, variant = \"channel\" }")
    w("  object dmamux \"st.dmamux\" { channels = 14, requests = 64 }")
    w("")
    w("  # ---------------------------------------------------------------------")
    w("  # The map")
    w("  # ---------------------------------------------------------------------")
    w("  map mem 0x00000000            size flash = syscfg.boot")
    w("  map mem flash-base            size flash = eflash.array")
    w(f"  map mem sram1-base            size {p['sram1']} = sram1")
    w("  map mem sram2a-base           size 32K   = sram2a")
    w("  map mem sram2b-base           size 32K   = sram2b")
    w("  map mem sram2-mirror          size 32K   = sram2a")
    w("  map mem sram2-mirror + 0x8000 size 32K   = sram2b")
    w("  map mem 0x1fff7000            size 1K    = otp       # OTP_AREA_BASE")
    w("  map mem 0x1fff7500            size 0xe2  = devinfo   # PACKAGE_BASE")
    w("")
    w("  map mem apb1-base + 0x0000 size 0x64  = tim2")
    w("  map mem apb1-base + 0x2800 size 0xd0  = rtc")
    w("  map mem apb1-base + 0x2c00 size 0x0c  = wwdg")
    w("  map mem apb1-base + 0x3000 size 0x14  = iwdg")
    w("  map mem apb1-base + 0x8000 size 0x34  = lpuart1")
    if not p["wb35"]:
        w("  map mem apb1-base + 0x3800 size 0x400 = spi2")
    w("  map mem apb1-base + 0x6800 size 0x60  = usb.regs")
    w("  map mem apb1-base + 0x6c00 size 0x400 = usb.pma")
    w("  map mem apb1-base + 0x5400 size 0x2c  = i2c1")
    w("  map mem apb1-base + 0x5c00 size 0x2c  = i2c3")
    w("  map mem apb2-base + 0x3000 size 0x400 = spi1")
    w("  map mem apb2-base + 0x0000 size 0x114 = syscfg")
    w("  map mem apb2-base + 0x2c00 size 0x64  = tim1")
    w("  map mem apb2-base + 0x3800 size 0x34  = usart1")
    w("  map mem apb2-base + 0x4400 size 0x64  = tim16")
    w("  map mem apb2-base + 0x4800 size 0x64  = tim17")
    w("  map mem ahb1-base + 0x0000 size 0xac  = dma1")
    w("  map mem ahb1-base + 0x0400 size 0xac  = dma2")
    w("  map mem ahb1-base + 0x0800 size 0x148 = dmamux")
    w("  map mem ahb1-base + 0x3000 size 0x18  = crc")
    for c in p["ports"]:
        w(f"  map mem ahb2-base + 0x{GPIO_OFF[c]:04x} size 0x2c  = gpio{c}")
    w("  map mem ahb4-base + 0x0000 size 0x188 = rcc")
    w("  map mem ahb4-base + 0x0400 size 0x8c = pwr")
    w("  map mem ahb4-base + 0x0800 size 0x400 = exti")
    w("  map mem ahb4-base + 0x0c00 size 0x400 = ipcc")
    w("  map mem ahb4-base + 0x1000 size 0x0c  = rng")
    w("  map mem ahb4-base + 0x1400 size 0x400 = hsem")
    w("  map mem ahb4-base + 0x4000 size 0x88  = eflash")
    w("  map mem dbgmcu-base        size 0x54  = dbgmcu")
    w("")
    w("  # ---------------------------------------------------------------------")
    w("  # Interrupts: RM0434 Table 72 / `IRQn_Type`, interrupt n -> cpu.irq{n}")
    w("  # ---------------------------------------------------------------------")
    irqs = [
        ("wwdg.ewi", 0, "WWDG"),
        ("exti.irq18", 2, "TAMP_STAMP_LSECSS"),
        ("exti.irq19", 3, "RTC_WKUP"),
        ("eflash.irq", 4, "FLASH"),
        ("rcc.irq", 5, "RCC"),
    ]
    for n in range(5):
        irqs.append((f"exti.irq{n}", 6 + n, f"EXTI{n}"))
    for n in range(7):
        irqs.append((f"dma1.irq{n + 1}", 11 + n, f"DMA1_Channel{n + 1}"))
    for n in range(5, 10):
        irqs.append((f"exti.irq{n}", 23, "EXTI9_5"))
    irqs += [
        ("tim1.irq", 24, "TIM1_BRK"),
        ("tim1.irq-up", 25, "TIM1_UP_TIM16"),
        ("tim1.irq-trg", 26, "TIM1_TRG_COM_TIM17"),
        ("tim1.irq-cc", 27, "TIM1_CC"),
        ("tim2.irq", 28, "TIM2"),
        ("i2c1.ev", 30, "I2C1_EV"),
        ("i2c1.er", 31, "I2C1_ER"),
        ("i2c3.ev", 32, "I2C3_EV"),
        ("i2c3.er", 33, "I2C3_ER"),
        ("spi1.irq", 34, "SPI1"),
        ("usb.irq_hp", 19, "USB_HP (iso/double-buffered CTR)"),
        ("usb.irq", 20, "USB_LP"),
        ("usart1.irq", 36, "USART1"),
        ("lpuart1.irq", 37, "LPUART1"),
    ]
    if not p["wb35"]:
        irqs.append(("spi2.irq", 35, "SPI2"))
    for t, v in (("tim16", 25), ("tim17", 26)):
        for pin in ("irq", "irq-up", "irq-trg", "irq-cc"):
            irqs.append((f"{t}.{pin}", v, f"shared with TIM1 ({t.upper()})"))
    for n in range(10, 16):
        irqs.append((f"exti.irq{n}", 40, "EXTI15_10"))
    irqs += [
        ("exti.irq17", 41, "RTC_Alarm"),
        ("ipcc.c1-rx", 44, "IPCC_C1_RX"),
        ("ipcc.c1-tx", 45, "IPCC_C1_TX"),
        ("hsem.c1-irq", 46, "HSEM"),
        ("rng.irq", 53, "RNG"),
    ]
    for n in range(7):
        irqs.append((f"dma2.irq{n + 1}", 55 + n, f"DMA2_Channel{n + 1}"))
    irqs.append(("dmamux.irq", 62, "DMAMUX1_OVR"))
    width = max(len(s) for s, _, _ in irqs)
    for src, n, label in irqs:
        w(f"  wire {src.ljust(width)} -> cpu.irq{n:<3} # {label}")
    w("")
    w("  # CPU2's side of the mailbox, and the bit that boots it.")
    w("  wire pwr.c2boot  -> cpu2.boot")
    w("  wire ipcc.c2-rx  -> cpu2.ipcc-rx")
    w("  wire ipcc.c2-tx  -> cpu2.ipcc-tx")
    w("")
    w("  # The RTC's three events are EXTI lines 17–19 (RM0434 Table 74).")
    w("  wire rtc.alarm  -> exti.line17")
    w("  wire rtc.stamp  -> exti.line18")
    w("  wire rtc.wakeup -> exti.line19")
    w("  # USB_WAKEUP is direct line 28, \"USB wake-up\" (RM0434 Table 74).")
    w("  wire usb.wakeup -> exti.line28")
    w("")
    w("  # The GPIO mux: every pin into SYSCFG, EXTICR picks one port per line.")
    for c in p["ports"]:
        w(f"  for n in 0..16 {{ wire gpio{c}.p${{n}} -> syscfg.p{c}${{n}} }}")
    w("  for n in 0..16 { wire syscfg.exti${n} -> exti.line${n} }")
    w("")
    w("  # DMAMUX1 channels 0–6 drive DMA1 channels 1–7, 7–13 drive DMA2's.")
    w("  for n in 0..7 { wire dmamux.ch${n} -> dma1.req${n + 1} }")
    w("  for n in 0..7 { wire dmamux.ch${n + 7} -> dma2.req${n + 1} }")
    w("")
    w("  # Peripheral requests into DMAMUX1, numbered as DMAREQ_ID numbers them")
    w("  # (RM0434 Table 64): the I2Cs and the U(S)ARTs. The SPI model has no")
    w("  # request outputs yet.")
    w("  wire i2c1.rx-drq    -> dmamux.req10   # I2C1_RX")
    w("  wire i2c1.tx-drq    -> dmamux.req11   # I2C1_TX")
    w("  wire i2c3.rx-drq    -> dmamux.req12   # I2C3_RX")
    w("  wire i2c3.tx-drq    -> dmamux.req13   # I2C3_TX")
    w("  wire usart1.dma-rx  -> dmamux.req14   # USART1_RX")
    w("  wire usart1.dma-tx  -> dmamux.req15   # USART1_TX")
    w("  wire lpuart1.dma-rx -> dmamux.req16   # LPUART1_RX")
    w("  wire lpuart1.dma-tx -> dmamux.req17   # LPUART1_TX")
    w("")
    w("  # ---------------------------------------------------------------------")
    w("  # Clock enables and resets, RCC -> peripheral. Bit numbers are the")
    w("  # header's RCC_xxxENR_yyyEN_Pos. CPU2's own enable (C2xxxENR) is ORed")
    w("  # onto the same pin: either core's bit clocks a shared block.")
    w("  # SYSCFG and EXTI have no enable bit on a WB; IWDG runs on the LSI.")
    w("  # ---------------------------------------------------------------------")
    gates = [
        ("ahb1", 0, "dma1", True),
        ("ahb1", 1, "dma2", True),
        ("ahb1", 2, "dmamux", True),
        ("ahb1", 12, "crc", True),
    ]
    for c in p["ports"]:
        gates.append(("ahb2", GPIO_BIT[c], f"gpio{c}", True))
    gates += [
        ("ahb3", 18, "rng", True),
        ("ahb3", 19, "hsem", True),
        ("ahb3", 20, "ipcc", True),
        ("apb1", 0, "tim2", True),
        ("apb1", 11, "wwdg", False),
        ("apb1", 21, "i2c1", True),
        ("apb1", 23, "i2c3", True),
        ("apb1", 26, "usb", True),
        ("apb2", 12, "spi1", True),
    ]
    if not p["wb35"]:
        gates.append(("apb1", 14, "spi2", True))
    gates += [
        ("apb1b", 0, "lpuart1", True),
        ("apb2", 11, "tim1", True),
        ("apb2", 14, "usart1", True),
        ("apb2", 17, "tim16", True),
        ("apb2", 18, "tim17", True),
    ]
    # `apb1b` is the second APB1 word (`APB1ENR2`), whose pins take a `b`:
    # `apb1enb0`, `apb1rstb0`, `c2apb1enb0`.
    for bus, bit, dev, has_reset in gates:
        second = bus.endswith("b")
        stem = bus[:-1] if second else bus
        suffix = "b" if second else ""
        w(f"  wire rcc.{stem}en{suffix}{bit} -> {dev}.enable")
        w(f"  wire rcc.c2{stem}en{suffix}{bit} -> {dev}.enable")
        if has_reset:
            w(f"  wire rcc.{stem}rst{suffix}{bit} -> {dev}.reset")
    w("")
    w("  # The RTC: BDCR.RTCEN and the backup-domain reset.")
    w("  wire rcc.rtcen -> rtc.rtcen")
    w("  wire rcc.bdrst -> rtc.bdrst")
    w("")
    w("  # Backup-domain write protection, and the SMPS enable RCC's SMPSSWS reads.")
    w("  wire pwr.dbp    -> rcc.dbp")
    w("  wire pwr.smpsen -> rcc.smpsen")
    w("")
    w("  # Resets. Each watchdog latches its own cause in RCC_CSR; they meet at a")
    w("  # wire.or so firmware can still tell which one rebooted it.")
    w("  # OBL_LAUNCH reloads the option bytes, which is a reset too.")
    w("  object sysreset \"wire.or\" { inputs = 3 }")
    w("  wire iwdg.reset   -> rcc.iwdgrst")
    w("  wire iwdg.reset   -> sysreset.in0")
    w("  wire wwdg.reset   -> rcc.wwdgrst")
    w("  wire wwdg.reset   -> sysreset.in1")
    w("  wire eflash.reset -> rcc.oblrst")
    w("  wire eflash.reset -> sysreset.in2")
    w("  wire sysreset.out -> cpu.reset")
    w("  wire dbgmcu.iwdg  -> iwdg.freeze")
    w("  wire dbgmcu.wwdg  -> wwdg.freeze")
    w("}")
    return "\n".join(out) + "\n"


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    for name, p in PARTS.items():
        with open(f"{sys.argv[1]}/{name}.machine", "w") as f:
            f.write(header(name, p))
            f.write("\n")
            f.write(body(name, p))


if __name__ == "__main__":
    main()
