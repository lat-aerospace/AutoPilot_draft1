/**
  * @file    stm32h7xx_hal_msp.c
  * @brief   HAL MSP (MCU Support Package) initialization — AP Autopilot
  *
  * These callbacks are invoked by the HAL when MX_*_Init() calls
  * HAL_*_Init(). They configure the GPIO alternate functions, enable
  * peripheral clocks, and set up DMA channels for each peripheral.
  *
  * Pin assignments (matches VajR PCB):
  *   SPI2  : ICM-20948 IMU — PB13=SCK, PB14=MISO, PB15=MOSI (AF5)
  *           CS=PD8 (GPIO), DMA1_Stream0 (RX), DMA1_Stream1 (TX)
  *   I2C2  : BMP390 baro   — SCL/SDA on GPIOB (PB10/11)
  *   UART5 : NEO-M9N GPS   — TX=PC12, RX=PB12 (AF14), DMA2_Stream3 (RX)
  *   UART8 : MAVLink GCS   — TX=PE1, RX=PE0 (AF8), DMA1_Stream2/3
  *   TIM1  : PWM servos    — CH3=PA10, CH4=PA11 (AF1)
  *   TIM3  : PWM servo     — CH3=PB0 (AF2)
  *   TIM8  : PWM servos    — CH1=PC6, CH2=PC7 (AF3)
  */

#include "stm32h7xx_hal.h"
#include "bsp.h"

/* ---- HAL base MSP (called by HAL_Init) --------------------------------- */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/* ---- SPI2 MSP (ICM-20948 IMU on VajR PCB) ------------------------------ */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (hspi->Instance == SPI2) {
        /* SPI123 kernel clock = PLL1Q (200 MHz) */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            while (1) {}
        }

        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* SPI2: PB13=SCK, PB14=MISO, PB15=MOSI (AF5) — matches VajR */
        GPIO_InitStruct.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;  /* Match VajR — LOW for TXB0104PWR level translator */
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* NOTE: DMA for SPI2 removed to match VajR (no DMA in their HAL_SPI_MspInit).
         * Polled SPI is used for both init and scheduled reads.
         * DMA can be re-added once ICM-20948 communication is confirmed working. */
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI2) {
        __HAL_RCC_SPI2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    }
}

/* ---- I2C2 MSP (BMP390 barometer) --------------------------------------- */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C2) {
        __HAL_RCC_I2C2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* I2C2: PB10=SCL, PB11=SDA (AF4) */
        GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;       /* external pull-ups required */
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C2) {
        __HAL_RCC_I2C2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    }
}

/* ---- UART MSP ----------------------------------------------------------- */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* USART3: PD8=TX, PD9=RX (AF7) — ST-Link VCP on Nucleo */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* DMA for USART3 RX: DMA1_Stream4 — circular for MAVLink */
        __HAL_RCC_DMA1_CLK_ENABLE();
        hdma_usart3_rx.Instance                 = DMA1_Stream4;
        hdma_usart3_rx.Init.Request             = DMA_REQUEST_USART3_RX;
        hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_usart3_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart3_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

        /* DMA for USART3 TX: DMA1_Stream5 */
        hdma_usart3_tx.Instance                 = DMA1_Stream5;
        hdma_usart3_tx.Init.Request             = DMA_REQUEST_USART3_TX;
        hdma_usart3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart3_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart3_tx.Init.Mode                = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_usart3_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_usart3_tx);
        __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

        /* USART3 IRQ */
        HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    else if (huart->Instance == UART5) {
        __HAL_RCC_UART5_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* UART5 TX: PC12 (AF8) */
        GPIO_InitStruct.Pin       = GPIO_PIN_12;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* UART5 RX: PB12 (AF14) — per CubeMX .ioc */
        GPIO_InitStruct.Pin       = GPIO_PIN_12;
        GPIO_InitStruct.Alternate = GPIO_AF14_UART5;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* DMA for UART5 RX: DMA2_Stream3 (per .ioc) — circular for GPS */
        __HAL_RCC_DMA2_CLK_ENABLE();
        hdma_uart5_rx.Instance                 = DMA2_Stream3;
        hdma_uart5_rx.Init.Request             = DMA_REQUEST_UART5_RX;
        hdma_uart5_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_uart5_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_uart5_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart5_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_uart5_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_uart5_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_uart5_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart5_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_uart5_rx);

        /* UART5 IRQ */
        HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
    else if (huart->Instance == UART8) {
        __HAL_RCC_UART8_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /* UART8: PE1=TX, PE0=RX (AF8) — J4 connector */
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* DMA for UART8 RX + TX (MAVLink) */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* RX: DMA1_Stream2 */
        hdma_uart8_rx.Instance                 = DMA1_Stream2;
        hdma_uart8_rx.Init.Request             = DMA_REQUEST_UART8_RX;
        hdma_uart8_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_uart8_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_uart8_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart8_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_uart8_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_uart8_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_uart8_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart8_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_uart8_rx);

        /* TX: DMA1_Stream3 */
        hdma_uart8_tx.Instance                 = DMA1_Stream3;
        hdma_uart8_tx.Init.Request             = DMA_REQUEST_UART8_TX;
        hdma_uart8_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_uart8_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_uart8_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_uart8_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart8_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_uart8_tx.Init.Mode                = DMA_NORMAL;
        hdma_uart8_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_uart8_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(&hdma_uart8_tx);
        __HAL_LINKDMA(huart, hdmatx, hdma_uart8_tx);

        /* UART8 IRQ */
        HAL_NVIC_SetPriority(UART8_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(UART8_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);
        HAL_DMA_DeInit(huart->hdmarx);
        HAL_DMA_DeInit(huart->hdmatx);
    }
    else if (huart->Instance == UART5) {
        __HAL_RCC_UART5_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);
        HAL_DMA_DeInit(huart->hdmarx);
    }
    else if (huart->Instance == UART8) {
        __HAL_RCC_UART8_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1);
        HAL_DMA_DeInit(huart->hdmarx);
        HAL_DMA_DeInit(huart->hdmatx);
    }
}

/* ---- TIM PWM MSP (clock enable only — GPIO in HAL_TIM_MspPostInit) ----- */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
    else if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    else if (htim->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
}

/* ---- TIM Base MSP (for TIM3 which uses HAL_TIM_Base_Init first) -------- */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10 | GPIO_PIN_11);
    }
    else if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
    }
    else if (htim->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
    }
}

/* ---- HAL_TIM_MspPostInit — GPIO AF setup for PWM outputs --------------- */
/*  Called from MX_TIMx_Init() after HAL_TIM_PWM_Init() and channel config  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* TIM1: PA10=CH3, PA11=CH4 (AF1) */
        GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (htim->Instance == TIM3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* TIM3: PB0=CH3 (AF2) */
        GPIO_InitStruct.Pin       = GPIO_PIN_0;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (htim->Instance == TIM8) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /* TIM8: PC6=CH1, PC7=CH2 (AF3) */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}
