/*
 * HAL configuration for the STM32WB BLE smoke firmware (tests/stm32wb_ble.rs).
 *
 * Written for rsemu (MIT). ST's HAL reads this header to learn which modules
 * are compiled in and what the board's oscillators are. Only the modules the
 * app uses are enabled, so a HAL file the board cannot back is not linked.
 */
#ifndef STM32WBxx_HAL_CONF_H
#define STM32WBxx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_HSEM_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* Every WB board fits the 32 MHz radio crystal and a 32.768 kHz LSE. */
#define HSE_VALUE            32000000UL
#define HSE_STARTUP_TIMEOUT  100UL
#define MSI_VALUE            4000000UL
#define HSI_VALUE            16000000UL
#define LSI1_VALUE           32000UL
#define LSI2_VALUE           32000UL
#define LSE_VALUE            32768UL
#define HSI48_VALUE          48000000UL
#define LSE_STARTUP_TIMEOUT  5000UL
#define EXTERNAL_SAI1_CLOCK_VALUE 2097000UL

#define VDD_VALUE            3300UL
#define TICK_INT_PRIORITY    ((1UL << __NVIC_PRIO_BITS) - 1UL)
#define USE_RTOS             0U
#define PREFETCH_ENABLE      1U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE    1U
#define USE_SPI_CRC          0U

#define assert_param(expr) ((void)0U)

#include "stm32wbxx_hal_rcc.h"
#include "stm32wbxx_hal_gpio.h"
#include "stm32wbxx_hal_dma.h"
#include "stm32wbxx_hal_cortex.h"
#include "stm32wbxx_hal_flash.h"
#include "stm32wbxx_hal_hsem.h"
#include "stm32wbxx_hal_pwr.h"
#include "stm32wbxx_hal_uart.h"

#endif /* STM32WBxx_HAL_CONF_H */
