/**
  * @file    stm32h7xx_it.c
  * @brief   Interrupt service routines for STM32H745 — AP Autopilot
  *
  * NOTE: SVC_Handler, PendSV_Handler, and SysTick_Handler are provided by
  *       FreeRTOS via the defines in FreeRTOSConfig.h:
  *         #define vPortSVCHandler    SVC_Handler
  *         #define xPortPendSVHandler PendSV_Handler
  *         #define xPortSysTickHandler SysTick_Handler
  *       Do NOT define them here — doing so causes duplicate symbol errors.
  *
  *       TIM6_DAC_IRQHandler is defined in Stm32Timer.cpp (the F' rate-group
  *       tick driver). Do NOT define it here either.
  *
  * DMA stream assignment:
  *   DMA1_Stream0: SPI2 RX  (ICM-20948 IMU)
  *   DMA1_Stream1: SPI2 TX  (ICM-20948 IMU)
  *   DMA1_Stream2: UART8 RX (MAVLink)
  *   DMA1_Stream3: UART8 TX (MAVLink)
  *   DMA2_Stream3: UART5 RX  (NEO-M9N GPS, per .ioc)
  */

#include "stm32h7xx_hal.h"
#include "bsp.h"

/* ---- Cortex-M7 Processor Exceptions ------------------------------------- */

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void)
{
}

/* ---- DMA Interrupt Handlers --------------------------------------------- */

/* DMA1 Stream0: SPI2 RX (ICM-20948 IMU) */
void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi2_rx);
}

/* DMA1 Stream1: SPI2 TX (ICM-20948 IMU) */
void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

/* DMA1 Stream2: UART8 RX (MAVLink) */
void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart8_rx);
}

/* DMA1 Stream3: UART8 TX (MAVLink) */
void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart8_tx);
}

/* DMA2 Stream3: UART5 RX (NEO-M9N GPS, per .ioc) */
void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart5_rx);
}

/* ---- UART Interrupt Handlers -------------------------------------------- */

void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
}

void UART8_IRQHandler(void)
{
    /* Clear error flags first to prevent ISR storm from noise/framing errors
       (e.g. when USB-TTL is disconnected or RX pin floats) */
    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_ORE))
        __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_OREF);
    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_FE))
        __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_FEF);
    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_NE))
        __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_NEF);
    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_PE))
        __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_PEF);
    HAL_UART_IRQHandler(&huart8);
}

/* ---- USART3 + DMA (MAVLink via ST-Link VCP) ---------------------------- */

/* DMA1 Stream4: USART3 RX (MAVLink via VCP) */
void DMA1_Stream4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

/* DMA1 Stream5: USART3 TX (MAVLink via VCP) */
void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}
