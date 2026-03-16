/**
  * @file    stm32h7xx_hal_conf.h
  * @brief   HAL configuration for STM32H745 Cortex-M7 — AP Autopilot
  *
  * Derived from STM32CubeH7 template. Enable only the HAL modules
  * actually used by the autopilot BSP to minimise code size.
  */
#ifndef __STM32H7xx_HAL_CONF_H
#define __STM32H7xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* #################### Module Selection #################################### */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* #################### Oscillator Values ################################### */
/* TODO: Set HSE_VALUE to match your board's crystal frequency (Hz).
 *       Common values: 8000000, 12000000, 25000000.
 *       The PLL multipliers in bsp.c assume 8 MHz — adjust both together. */
#if !defined(HSE_VALUE)
#define HSE_VALUE    ((uint32_t)8000000)
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT  ((uint32_t)100)
#endif

#if !defined(CSI_VALUE)
#define CSI_VALUE    ((uint32_t)4000000)
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE    ((uint32_t)64000000)
#endif

#if !defined(LSE_VALUE)
#define LSE_VALUE    ((uint32_t)32768)
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT  ((uint32_t)5000)
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE    ((uint32_t)32000)
#endif

#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE  12288000U
#endif

/* #################### System Configuration ################################ */
#define VDD_VALUE                  3300UL
#define TICK_INT_PRIORITY          ((uint32_t)0x0F)
#define USE_RTOS                   0
#define USE_SPI_CRC                0U

/* #################### Assert — disabled (F' has its own) ################## */
#define assert_param(expr) ((void)0U)

/* #################### Header Includes ##################################### */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32h7xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32h7xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32h7xx_hal_dma.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32h7xx_hal_exti.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32h7xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32h7xx_hal_flash.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
  #include "stm32h7xx_hal_i2c.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32h7xx_hal_pwr.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
  #include "stm32h7xx_hal_spi.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32h7xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32h7xx_hal_uart.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_HAL_CONF_H */
