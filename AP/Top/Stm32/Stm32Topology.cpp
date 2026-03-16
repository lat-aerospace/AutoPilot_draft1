// ======================================================================
// \title  AP/Top/Stm32/Stm32Topology.cpp
// \brief  STM32 hardware deployment – topology setup / teardown
//
// Rate group divisors (base tick = 400 Hz from TIM6):
//   RG1: ÷1  = 400 Hz  (IMU)
//   RG2: ÷4  = 100 Hz  (StateEstimator, Controller)
//   RG3: ÷8  =  50 Hz  (Barometer)
//   RG4: ÷40 =  10 Hz  (Autonomy, MAVLink, GPS)
// ======================================================================

#include <AP/Top/Stm32/Stm32TopologyAc.hpp>   // F' auto-generated
#include <AP/Top/Stm32/Stm32Topology.hpp>
#include <AP/Os/FreeRtos/FreeRtosMemAllocator.hpp>
#include <Fw/Types/MallocAllocator.hpp>

using namespace Stm32;

// ---------------------------------------------------------------------------
// Rate group divisors
// ---------------------------------------------------------------------------
Svc::RateGroupDriver::DividerSet rateGroupDivisorsSet{
    {{1, 0}, {4, 0}, {8, 0}, {40, 0}}
};

// Rate group context arrays (unused values; required by configure())
U32 rateGroup1Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup2Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup3Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};
U32 rateGroup4Context[Svc::ActiveRateGroup::CONNECTION_COUNT_MAX] = {};

// ---------------------------------------------------------------------------
// FreeRTOS memory allocator (backing store for Os::Generic::PriorityQueue)
// ---------------------------------------------------------------------------
static Ap::Os::FreeRtosMemAllocator s_freertos_allocator;

// ---------------------------------------------------------------------------
// Debug blink for topology init (self-contained, no external deps)
// ---------------------------------------------------------------------------
#include "stm32h7xx_hal.h"
static void topoRoughDelay(uint32_t ms) {
    for (uint32_t m = 0; m < ms; m++) {
        for (volatile uint32_t i = 0; i < 50000U; i++) { __NOP(); }
    }
}
static void topoBlink(int blinks) {
    __HAL_RCC_GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef g = {};
    g.Pin = GPIO_PIN_6; g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &g);
    for (int i = 0; i < blinks; i++) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
        topoRoughDelay(300);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        topoRoughDelay(300);
    }
    topoRoughDelay(1500);
}

// ---------------------------------------------------------------------------
// configureTopology – hardware binding + component init
// Called after the auto-generated lifecycle functions have wired the graph.
// ---------------------------------------------------------------------------
static void configureTopology(const HardwareBindings& hw) {
    topoBlink(1);  // entering configureTopology

    // Register FreeRTOS heap as the allocator for F' priority queues
    Fw::MemAllocatorRegistry::getInstance().registerAllocator(
        Fw::MemoryAllocation::MemoryAllocatorType::OS_GENERIC_PRIORITY_QUEUE,
        s_freertos_allocator);

    // Configure rate group driver and rate groups
    rateGroupDriverComp.configure(rateGroupDivisorsSet);
    rateGroup1Comp.configure(rateGroup1Context, FW_NUM_ARRAY_ELEMENTS(rateGroup1Context));
    rateGroup2Comp.configure(rateGroup2Context, FW_NUM_ARRAY_ELEMENTS(rateGroup2Context));
    rateGroup3Comp.configure(rateGroup3Context, FW_NUM_ARRAY_ELEMENTS(rateGroup3Context));
    rateGroup4Comp.configure(rateGroup4Context, FW_NUM_ARRAY_ELEMENTS(rateGroup4Context));

    topoBlink(2);  // rate groups configured

    // TIM6 base-tick timer at 400 Hz (creates its own internal FreeRTOS task)
    stm32Timer.configure(400U);

    topoBlink(3);  // timer configured

    // IMU – ICM-20948 + AK09916 on SPI4
    if (hw.hspi4 != nullptr) {
        stm32Imu.configure(hw.hspi4, hw.imu_cs_port, hw.imu_cs_pin);
        (void)stm32Imu.initSensor();
    }

    topoBlink(4);  // IMU done

    // Barometer – BMP390 on I2C2
    if (hw.hi2c2 != nullptr) {
        stm32Baro.configure(hw.hi2c2);
        (void)stm32Baro.initSensor();
    }

    topoBlink(5);  // Baro done

    // GPS – NEO-M9N UBX-NAV-PVT on UART5 with DMA circular receive
    // DISABLED: GPS init hangs when no GPS module connected (indoor)
    // if (hw.huart5 != nullptr) {
    //     stm32Gps.configure(hw.huart5);
    //     stm32Gps.initGps();
    //     stm32Gps.startRx();
    // }

    topoBlink(6);  // GPS done

    // MAVLink gateway – UART8 at 115200 baud with DMA (J4 connector)
    if (hw.huart8 != nullptr) {
        mavlinkUart.configure(hw.huart8, 1U, 1U);
        mavlinkUart.startRx();
    }

    topoBlink(7);  // MAVLink done

    // PWM servo output – TIM1/TIM3/TIM8 at 50 Hz (5 channels across 3 timers)
    if (hw.htim1 != nullptr && hw.htim3 != nullptr && hw.htim8 != nullptr) {
        stm32Pwm.configure(hw.htim1, hw.htim3, hw.htim8);
        stm32Pwm.startPwm();
    }

    topoBlink(8);  // PWM done, configureTopology complete
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
namespace Stm32 {

void setupTopology(const HardwareBindings& hw) {
    // Build the topology state struct required by auto-generated lifecycle fns.
    // CdhCore::SubtopologyState is empty; no external parameters are needed.
    TopologyState state;

    // Auto-generated F' lifecycle sequence
    initComponents(state);
    setBaseIds();
    connectComponents();
    regCommands();
    configComponents(state);

    // Hardware-specific initialization — blinks 1-8 are inside
    configureTopology(hw);

    loadParameters();
    startTasks(state);
}

void startRateGroups() {
    // Arms the TIM6 ISR and starts the internal timer FreeRTOS task.
    // Rate groups begin ticking once vTaskStartScheduler() is called.
    stm32Timer.startTimer();
}

void teardownTopology(const HardwareBindings& hw) {
    TopologyState state;

    stm32Timer.stopTimer();
    stm32Pwm.disarm();

    stopTasks(state);
    freeThreads(state);
    tearDownComponents(state);
}

}  // namespace Stm32
