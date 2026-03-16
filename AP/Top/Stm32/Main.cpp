// ======================================================================
// \title  AP/Top/Stm32/Main.cpp
// \brief  STM32H745 hardware deployment – entry point (M7 core only)
//
// Boot sequence:
//   1. CPU_CACHE_Enable() – L1 I/D cache
//   2. HAL_Init() + SystemClock_Config() + PeriphCommonClock_Config()
//   3. Peripheral init (SPI2, I2C2, UART5, UART8, TIM1/3/8, GPIO, DMA)
//   4. Enable DWT cycle counter (FreeRtosRawTime sub-ms precision)
//   5. Os::init() – registers FreeRTOS-backed Os primitives
//   6. Stm32::setupTopology() – wires F' component graph + binds HAL
//   7. Stm32::startRateGroups() – arms TIM6 + timer task
//   8. vTaskStartScheduler() – FreeRTOS takes control; NEVER RETURNS
//
// NOTE: All F' active/queued component tasks are created during
//       setupTopology() but only begin executing after
//       vTaskStartScheduler() is called.
// ======================================================================

#include <AP/Top/Stm32/Stm32Topology.hpp>
#include <Os/Os.hpp>
#include <Os/Console.hpp>
#include <Os/Cpu.hpp>
#include <Os/FileSystem.hpp>
#include <Os/Memory.hpp>
#include <Os/Task.hpp>
#include <FreeRTOS.h>
#include <task.h>

// BSP: peripheral handles, init functions, pin definitions
#include "bsp.h"

// ---------------------------------------------------------------------------
// Enable DWT cycle counter (used by FreeRtosRawTime for sub-ms precision)
// ---------------------------------------------------------------------------
static void enableDwtCycleCounter(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

// ---------------------------------------------------------------------------
// FreeRTOS application hooks
// ---------------------------------------------------------------------------
extern "C" void vApplicationMallocFailedHook(void) {
    FW_ASSERT(false);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t /*xTask*/,
                                               char* /*pcTaskName*/) {
    FW_ASSERT(false);
}

extern "C" void vApplicationTickHook(void) {
    HAL_IncTick();
}

extern "C" void vApplicationIdleHook(void) {
}

// Required by configSUPPORT_STATIC_ALLOCATION = 1
static StaticTask_t xIdleTaskTCB;
static StackType_t  uxIdleTaskStack[configMINIMAL_STACK_SIZE];

extern "C" void vApplicationGetIdleTaskMemory(
    StaticTask_t** ppxIdleTaskTCBBuffer,
    StackType_t**  ppxIdleTaskStackBuffer,
    uint32_t*      pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Debug: blink LED1 (PG6) to show which init stage the CPU has reached.
// Each stage adds one more blink. Call after HAL_Init() at minimum.
// ---------------------------------------------------------------------------
// Simple busy-wait delay. Uses a large volatile loop.
// At 64 MHz (before PLL): ~250ms per call.  At 200 MHz (after PLL): ~80ms.
// Not precise, but clearly visible for debug blinks.
static void roughDelayMs(uint32_t ms) {
    for (uint32_t m = 0; m < ms; m++) {
        for (volatile uint32_t i = 0; i < 50000U; i++) {
            __NOP();
        }
    }
}

static void debugBlink(int blinks) {
    // Enable GPIOG clock (safe to call multiple times)
    __HAL_RCC_GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_6;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &g);

    for (int i = 0; i < blinks; i++) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
        roughDelayMs(300);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        roughDelayMs(300);
    }
    roughDelayMs(1500);  // long pause between stages — clearly separates groups
}

int main(void) {
    // Step 1: Enable L1 instruction and data caches
    CPU_CACHE_Enable();

    // Step 2: STM32 HAL initialization
    HAL_Init();

    // Step 3: System clock (200 MHz via PLL1/VOS3)
    SystemClock_Config();

    // Step 3b: Peripheral common clocks (PLL3 for UART/I2C)
    PeriphCommonClock_Config();

    // Step 4: Peripheral initialization (BSP)
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI2_Init();          // IMU (ICM-20948 + AK09916) — SPI2 PB13/14/15, CS=PD8
    MX_I2C2_Init();          // Barometer (BMP390)
    MX_UART5_Init();         // GPS (NEO-M9N)
    MX_UART8_Init();         // MAVLink (UART8, J4 connector) — kept as fallback
    MX_TIM1_Init();          // PWM servos CH3/CH4 (PA10/PA11)
    MX_TIM3_Init();          // PWM servo  CH3     (PB0)
    MX_TIM8_Init();          // PWM servos CH1/CH2 (PC6/PC7)

    // Step 5: Initialize F' OS abstraction (FreeRTOS-backed)
    Os::Console::init();
    Os::FileSystem::init();
    Os::Cpu::init();
    Os::Memory::init();
    Os::Task::init();

    // Step 6: Build hardware binding struct from BSP handles
    Stm32::HardwareBindings hw;
    hw.hspi2       = &hspi2;
    hw.hi2c2       = &hi2c2;
    hw.huart5      = &huart5;
    hw.huart8      = &huart8;   // MAVLink via UART8 (PE1=TX, PE0=RX, J4 connector)
    hw.htim1       = &htim1;
    hw.htim3       = &htim3;
    hw.htim8       = &htim8;
    hw.imu_cs_port = IMU_CS_GPIO_Port;
    hw.imu_cs_pin  = IMU_CS_Pin;

    // Step 7: Wire F' topology and bind hardware drivers
    Stm32::setupTopology(hw);

    // Step 8: Arm the 400 Hz base-tick timer (TIM6 + FreeRTOS task)
    // Note: startRateGroups creates the timer task. TIM6 hardware init
    // is deferred to inside that task (after s_task_handle is set) so
    // that vTaskNotifyGiveFromISR works correctly from the ISR.
    Stm32::startRateGroups();

    // Step 9: Start FreeRTOS scheduler – all F' component threads now run.
    // This call NEVER returns under normal operation.
    vTaskStartScheduler();

    FW_ASSERT(false);
    return -1;
}
