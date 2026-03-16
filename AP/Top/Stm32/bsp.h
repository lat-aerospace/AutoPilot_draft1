/**
  * @file    bsp.h
  * @brief   Board Support Package — STM32H745ZIT6 (M7 core only)
  *
  * Declares all BSP initialization functions and peripheral handles
  * that Main.cpp needs.
  *
  * Clock tree:
  *   HSE    = 48 MHz
  *   VOS3
  *   SYSCLK = 200 MHz (PLL1: M=3, N=25, P=2)
  *   HCLK   = 200 MHz (AHB /1)
  *   APB1   = 100 MHz (/2), timer clock = 200 MHz
  *   APB2   = 100 MHz (/2), timer clock = 200 MHz
  *   PLL1Q  = 200 MHz (SPI123 kernel clock)
  *   PLL3Q  =  50 MHz (USART234578 kernel clock)
  *   PLL3R  =  50 MHz (I2C123 kernel clock)
  *   Flash  = 4 wait states
  *
  * Peripheral mapping (actual hardware — matches VajR PCB):
  *   SPI2  : ICM-20948 IMU (Mode 3, 6.25 MHz from PLL1Q 200 MHz/32)
  *           SCK=PB13, MISO=PB14, MOSI=PB15, CS=PD8
  *   I2C2  : BMP390 barometer (Fast Plus, 50 MHz PLL3R)
  *   UART5 : NEO-M9N GPS (38400 default, PC12=TX PB12=RX)
  *   UART8 : MAVLink GCS (115200 baud, PE1=TX PE0=RX, DMA)
  *   TIM1  : PWM servo output CH3/CH4 (50 Hz, PA10/PA11)
  *   TIM3  : PWM servo output CH3     (50 Hz, PB0)
  *   TIM8  : PWM servo output CH1/CH2 (50 Hz, PC6/PC7)
  */
#ifndef AP_BSP_H
#define AP_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* ---- Peripheral handles (defined in bsp.c) ----------------------------- */
extern SPI_HandleTypeDef  hspi2;     /* SPI2  : ICM-20948 IMU (PB13/14/15) */
extern I2C_HandleTypeDef  hi2c2;     /* I2C2  : BMP390 barometer           */
extern UART_HandleTypeDef huart3;    /* USART3: MAVLink GCS via ST-Link VCP (115200, DMA) */
extern UART_HandleTypeDef huart5;    /* UART5 : NEO-M9N GPS (38400, DMA)   */
extern UART_HandleTypeDef huart8;    /* UART8 : MAVLink GCS (115200, DMA)  */
extern TIM_HandleTypeDef  htim1;     /* TIM1  : PWM servo CH3/CH4 (50 Hz)  */
extern TIM_HandleTypeDef  htim3;     /* TIM3  : PWM servo CH3     (50 Hz)  */
extern TIM_HandleTypeDef  htim8;     /* TIM8  : PWM servo CH1/CH2 (50 Hz)  */

extern DMA_HandleTypeDef  hdma_spi2_rx;
extern DMA_HandleTypeDef  hdma_spi2_tx;
extern DMA_HandleTypeDef  hdma_usart3_rx;
extern DMA_HandleTypeDef  hdma_usart3_tx;
extern DMA_HandleTypeDef  hdma_uart5_rx;
extern DMA_HandleTypeDef  hdma_uart8_rx;
extern DMA_HandleTypeDef  hdma_uart8_tx;

/* ---- GPIO pin definitions ----------------------------------------------- */
/* IMU chip-select: GPIOD, PIN_8 (matches VajR PCB) */
extern GPIO_TypeDef* IMU_CS_GPIO_Port;
extern uint16_t      IMU_CS_Pin;

/* ---- BSP init functions ------------------------------------------------- */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_SPI2_Init(void);
void MX_I2C2_Init(void);
void MX_USART3_Init(void);
void MX_UART5_Init(void);
void MX_UART8_Init(void);
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM8_Init(void);

/* ---- Post-init for timer GPIO (called from MX_TIMx_Init) --------------- */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/* ---- Cache / MPU helpers ------------------------------------------------ */
void CPU_CACHE_Enable(void);

#ifdef __cplusplus
}
#endif

#endif /* AP_BSP_H */
