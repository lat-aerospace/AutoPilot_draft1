/**
  * @file    bsp.c
  * @brief   Board Support Package — STM32H745ZIT6 (M7 core only)
  *
  * Contains SystemClock_Config(), PeriphCommonClock_Config(),
  * all MX_*_Init() peripheral init functions, and HAL handle definitions.
  *
  *
  * Peripheral mapping (actual hardware):
  *   SPI4  : ICM-20948 IMU (Mode 3, 6.25 MHz), CS = GPIOE PIN_4
  *   I2C2  : BMP390 barometer (400 kHz fast mode plus, addr 0x77)
  *   UART5 : NEO-M9N GPS (starts at 38400, initGps() changes to 57600)
  *   UART8 : MAVLink GCS (115200 baud, PE1=TX PE0=RX, DMA RX+TX)
  *   TIM1  : PWM servo CH3/CH4 (50 Hz, PA10/PA11)
  *   TIM3  : PWM servo CH3     (50 Hz, PB0)
  *   TIM8  : PWM servo CH1/CH2 (50 Hz, PC6/PC7)
  *
  * Clock configuration:
  *   HSE    = 48 MHz
  *   VOS3
  *   PLL1   : M=3, N=25, P=2, Q=2 → SYSCLK=200 MHz, PLL1Q=200 MHz
  *   PLL3   : M=3, N=25, P=2, Q=8, R=8 → PLL3Q=50 MHz (UART/I2C kernels)
  *   HCLK   = 200 MHz (AHB /1)
  *   APB1   = 100 MHz (/2), timer clock = 200 MHz
  *   APB2   = 100 MHz (/2), timer clock = 200 MHz
  *   Flash  = 4 WS
  */

#include "bsp.h"

/* ---- Peripheral handle definitions -------------------------------------- */
SPI_HandleTypeDef  hspi4;
I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart8;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim3;
TIM_HandleTypeDef  htim8;

DMA_HandleTypeDef  hdma_spi4_rx;
DMA_HandleTypeDef  hdma_spi4_tx;
DMA_HandleTypeDef  hdma_usart3_rx;
DMA_HandleTypeDef  hdma_usart3_tx;
DMA_HandleTypeDef  hdma_uart5_rx;
DMA_HandleTypeDef  hdma_uart8_rx;
DMA_HandleTypeDef  hdma_uart8_tx;

/* ---- GPIO pin definitions ----------------------------------------------- */
/* IMU chip-select: GPIOE, PIN_4 */
GPIO_TypeDef* IMU_CS_GPIO_Port = GPIOE;
uint16_t      IMU_CS_Pin       = GPIO_PIN_4;

/* ---- APB1 timer clock for Stm32Timer.cpp -------------------------------- */
/* SYSCLK=200 MHz, AHB/1=200 MHz, APB1/2=100 MHz, timer multiplier=2 → 200 MHz */
uint32_t AP_Tim6_ApbClockHz = 200000000U;

/* ---- Local helpers ------------------------------------------------------- */
static void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

/* ========================================================================== */
/*  SystemClock_Config — 200 MHz via HSE(48 MHz) + PLL1 (VOS3)               */
/*                                                                            */
/* ========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Supply configuration: use internal LDO */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /* VOS3 */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /* ---- PLL1: SYSCLK = 200 MHz, PLL1Q = 200 MHz ----
     * HSE = 48 MHz
     * PLL1 input  = 48 / 3 = 16 MHz   (VCI range 8-16 MHz)
     * PLL1 VCO    = 16 * 25 = 400 MHz  (wide VCO range)
     * PLL1P       = 400 / 2 = 200 MHz  → SYSCLK
     * PLL1Q       = 400 / 2 = 200 MHz
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 3;
    RCC_OscInitStruct.PLL.PLLN       = 25;
    RCC_OscInitStruct.PLL.PLLP       = 2;
    RCC_OscInitStruct.PLL.PLLQ       = 2;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_3;  /* 8-16 MHz */
    RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN   = 0;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    /* ---- Bus clocks ----
     * SYSCLK = 200 MHz, HCLK = 200 MHz (AHB /1)
     * APBx   = 100 MHz (/2), timer clock = 200 MHz
     * Flash  = 4 wait states (VOS3)                                           */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK    |
                                       RCC_CLOCKTYPE_SYSCLK  |
                                       RCC_CLOCKTYPE_PCLK1   |
                                       RCC_CLOCKTYPE_PCLK2   |
                                       RCC_CLOCKTYPE_D3PCLK1 |
                                       RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;        /* 200 MHz */
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;        /* 100 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;        /* 100 MHz */
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================================================================== */
/*  PeriphCommonClock_Config — PLL3 for UART/I2C kernel clocks               */
/*                                                                            */
/* ========================================================================== */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /* PLL3: M=3, N=25, P=2, Q=8, R=8
     * PLL3 input  = 48 / 3 = 16 MHz
     * PLL3 VCO    = 16 * 25 = 400 MHz
     * PLL3Q       = 400 / 8 = 50 MHz  → USART234578 kernel clock
     * PLL3R       = 400 / 8 = 50 MHz  → I2C123 kernel clock
     * NOTE: SPI4/5 NOT routed to PLL3 (uses default pclk2 = 100 MHz)
     */
    PeriphClkInitStruct.PeriphClockSelection =
        RCC_PERIPHCLK_USART234578 |
        RCC_PERIPHCLK_I2C123;

    PeriphClkInitStruct.PLL3.PLL3M      = 3;
    PeriphClkInitStruct.PLL3.PLL3N      = 25;
    PeriphClkInitStruct.PLL3.PLL3P      = 2;
    PeriphClkInitStruct.PLL3.PLL3Q      = 8;
    PeriphClkInitStruct.PLL3.PLL3R      = 8;
    PeriphClkInitStruct.PLL3.PLL3RGE    = RCC_PLL3VCIRANGE_3;  /* 8-16 MHz */
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN  = 0;

    /* USART234578 kernel = PLL3Q (50 MHz) */
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;

    /* I2C123 kernel = PLL3R (50 MHz) */
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_PLL3;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================================================================== */
/*  CPU_CACHE_Enable — L1 instruction and data cache                          */
/* ========================================================================== */
void CPU_CACHE_Enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

/* ========================================================================== */
/*  MX_GPIO_Init — CS pin, LEDs, general GPIO                                */
/* ========================================================================== */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks for all GPIO ports used */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* IMU chip select (GPIOE PIN_4) — default HIGH (deselected) */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = IMU_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

    /* LEDs (PD8 removed — now used by USART3 TX for ST-Link VCP) */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* LED1 = PG6 */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* ========================================================================== */
/*  MX_DMA_Init — enable DMA clocks and NVIC                                 */
/* ========================================================================== */
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt priorities */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);  /* SPI4 RX    */
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);  /* SPI4 TX    */
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);  /* UART8 RX   */
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);  /* UART8 TX   */
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);  /* USART3 RX  */
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);  /* USART3 TX  */
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);  /* UART5 RX   */
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/* ========================================================================== */
/*  MX_SPI4_Init — ICM-20948 IMU (Mode 3, 6.25 MHz)                          */
/*  SPI4 on APB2 → pclk2 = 100 MHz, prescaler 16 → 6.25 MHz                 */
/* ========================================================================== */
void MX_SPI4_Init(void)
{
    hspi4.Instance               = SPI4;
    hspi4.Init.Mode              = SPI_MODE_MASTER;
    hspi4.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi4.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi4.Init.CLKPolarity       = SPI_POLARITY_HIGH;      /* Mode 3 */
    hspi4.Init.CLKPhase          = SPI_PHASE_2EDGE;        /* Mode 3 */
    hspi4.Init.NSS               = SPI_NSS_SOFT;
    /* pclk2 = 100 MHz, prescaler 16 → SPI clock = 6.25 MHz */
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi4.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi4.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi4.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi4.Init.CRCPolynomial     = 0x0;
    hspi4.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    hspi4.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    hspi4.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi4.Init.MasterSSIdleness          = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi4.Init.MasterInterDataIdleness   = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi4.Init.MasterReceiverAutoSusp    = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi4.Init.MasterKeepIOState         = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi4.Init.IOSwap            = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&hspi4) != HAL_OK) { Error_Handler(); }
}

/* ========================================================================== */
/*  MX_I2C2_Init — BMP390 barometer (400 kHz Fast Mode Plus)                 */
/*  I2C123 kernel = PLL3R = 50 MHz                                           */
/* ========================================================================== */
void MX_I2C2_Init(void)
{
    hi2c2.Instance              = I2C2;
    hi2c2.Init.Timing           = 0x0020081F;  /* 400 kHz @ 50 MHz PLL3R */
    hi2c2.Init.OwnAddress1      = 0;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
    HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);

    /* Enable Fast Mode Plus */
    HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
}

/* ========================================================================== */
/*  MX_USART3_Init — ST-Link VCP for MAVLink (115200, PD8=TX PD9=RX)        */
/*  USART234578 kernel = PLL3Q = 50 MHz                                      */
/* ========================================================================== */
void MX_USART3_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
}

/* ========================================================================== */
/*  MX_UART5_Init — NEO-M9N GPS (38400 baud default, initGps() → 57600)     */
/*  USART234578 kernel = PLL3Q = 50 MHz                                      */
/* ========================================================================== */
void MX_UART5_Init(void)
{
    huart5.Instance          = UART5;
    huart5.Init.BaudRate     = 38400;
    huart5.Init.WordLength   = UART_WORDLENGTH_8B;
    huart5.Init.StopBits     = UART_STOPBITS_1;
    huart5.Init.Parity       = UART_PARITY_NONE;
    huart5.Init.Mode         = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart5) != HAL_OK) { Error_Handler(); }
}

/* ========================================================================== */
/*  MX_UART8_Init — MAVLink telemetry (115200 baud, PE1=TX PE0=RX, DMA)     */
/*  USART234578 kernel = PLL3Q = 50 MHz                                      */
/* ========================================================================== */
void MX_UART8_Init(void)
{
    huart8.Instance          = UART8;
    huart8.Init.BaudRate     = 115200;
    huart8.Init.WordLength   = UART_WORDLENGTH_8B;
    huart8.Init.StopBits     = UART_STOPBITS_1;
    huart8.Init.Parity       = UART_PARITY_NONE;
    huart8.Init.Mode         = UART_MODE_TX_RX;
    huart8.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart8.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart8) != HAL_OK) { Error_Handler(); }
}

/* ========================================================================== */
/*  MX_TIM1_Init — PWM servo CH3/CH4 (50 Hz, PA10/PA11)                     */
/*  TIM1 on APB2 → timer clock = 200 MHz                                     */
/*  Prescaler=199, Period=19999 → 1 MHz tick, 50 Hz, CCR = µs directly      */
/* ========================================================================== */
void MX_TIM1_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 200 - 1;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 20000 - 1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    HAL_TIM_MspPostInit(&htim1);
}

/* ========================================================================== */
/*  MX_TIM3_Init — PWM servo CH3 (50 Hz, PB0)                               */
/*  TIM3 on APB1 → timer clock = 200 MHz                                     */
/*  Prescaler=199, Period=19999 → 1 MHz tick, 50 Hz, CCR = µs directly      */
/* ========================================================================== */
void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 200 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 20000 - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity  = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_MspPostInit(&htim3);
}

/* ========================================================================== */
/*  MX_TIM8_Init — PWM servo CH1/CH2 (50 Hz, PC6/PC7)                       */
/*  TIM8 on APB2 → timer clock = 200 MHz                                     */
/*  Prescaler=199, Period=19999 → 1 MHz tick, 50 Hz, CCR = µs directly      */
/* ========================================================================== */
void MX_TIM8_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 200 - 1;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 20000 - 1;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);

    HAL_TIM_MspPostInit(&htim8);
}
