// ======================================================================
// \title  AP/Top/Stm32/Stm32Topology.hpp
// \brief  Public API for the STM32 hardware deployment topology
// ======================================================================
#ifndef AP_TOP_STM32_TOPOLOGY_HPP
#define AP_TOP_STM32_TOPOLOGY_HPP

#include <AP/Top/Stm32/Stm32TopologyDefs.hpp>
#include <AP/Hal/Stm32/Stm32Timer/Stm32Timer.hpp>
#include <AP/Hal/Stm32/Stm32SpiImu/Stm32SpiImu.hpp>
#include <AP/Hal/Stm32/Stm32Uart/Stm32Uart.hpp>
#include <AP/Hal/Stm32/Stm32I2cBaro/Stm32I2cBaro.hpp>
#include <AP/Hal/Stm32/Stm32GpsUart/Stm32GpsUart.hpp>
#include <AP/Hal/Stm32/Stm32PwmOutput/Stm32PwmOutput.hpp>

namespace Stm32 {

//! Hardware binding struct – filled in by Main.cpp with STM32 HAL handles.
//! Pass nullptr for any peripheral not populated on the target board.
struct HardwareBindings {
    SPI_HandleTypeDef*  hspi2       = nullptr;  ///< SPI2  : ICM-20948 IMU + AK09916 mag (PB13/14/15)
    I2C_HandleTypeDef*  hi2c2       = nullptr;  ///< I2C2  : BMP390 barometer
    UART_HandleTypeDef* huart5      = nullptr;  ///< UART5 : NEO-M9N GPS (57600 baud, DMA)
    UART_HandleTypeDef* huart8      = nullptr;  ///< UART8 : MAVLink GCS (115200 baud, DMA)
    TIM_HandleTypeDef*  htim1       = nullptr;  ///< TIM1  : PWM servo CH3/CH4 (PA10/PA11)
    TIM_HandleTypeDef*  htim3       = nullptr;  ///< TIM3  : PWM servo CH3     (PB0)
    TIM_HandleTypeDef*  htim8       = nullptr;  ///< TIM8  : PWM servo CH1/CH2 (PC6/PC7)
    GPIO_TypeDef*       imu_cs_port = nullptr;  ///< IMU chip-select GPIO port (GPIOD)
    uint16_t            imu_cs_pin  = 0;        ///< IMU chip-select GPIO pin mask (PIN_8)
};

//! Topology lifecycle functions – call in the order shown from Main.cpp.
void setupTopology(const HardwareBindings& hw);
void startRateGroups();
void teardownTopology(const HardwareBindings& hw);

}  // namespace Stm32

#endif  // AP_TOP_STM32_TOPOLOGY_HPP
