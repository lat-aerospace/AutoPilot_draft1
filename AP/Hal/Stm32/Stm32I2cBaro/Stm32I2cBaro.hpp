// ======================================================================
// \title  AP/Hal/Stm32/Stm32I2cBaro/Stm32I2cBaro.hpp
// \brief  BMP390 barometer driver over STM32 I2C2
//
// BMP390 runs in continuous (normal) mode — both temperature and pressure
// are sampled automatically. No interleaving needed.
//
// schedIn_handler (50 Hz):
//   1. Read 6 raw bytes (3 pressure + 3 temperature)
//   2. Apply BMP390 compensation polynomial using NVM calibration data
//   3. Compute altitude using hypsometric formula
//   4. Publish BaroData
// ======================================================================
#ifndef AP_HAL_STM32_I2C_BARO_HPP
#define AP_HAL_STM32_I2C_BARO_HPP

#include <stm32h7xx_hal.h>
#include <AP/Types/ApTypesSerializableAc.hpp>
#include <AP/Hal/Stm32/Stm32I2cBaro/Stm32I2cBaroComponentAc.hpp>

namespace Ap {

class Stm32I2cBaro final : public Stm32I2cBaroComponentBase {
  public:
    static constexpr uint8_t  I2C_ADDR      = 0x76U << 1U;  // 8-bit for STM32 HAL (SDO→GND)
    static constexpr uint32_t I2C_TIMEOUT   = 20U;           // ms

    explicit Stm32I2cBaro(const char* const compName);
    ~Stm32I2cBaro() override = default;

    //! Bind I2C handle
    void configure(I2C_HandleTypeDef* hi2c);

    //! Initialize BMP390: verify chip ID, configure, read calibration NVM
    bool initSensor();

  private:
    void schedIn_handler(FwIndexType portNum,
                         U32 context) override;

    // I2C helpers
    bool    writeRegister(uint8_t reg, uint8_t value);
    bool    readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);

    // BMP390 compensation
    void    compensate(uint32_t raw_press, uint32_t raw_temp,
                       float& out_press_pa, float& out_temp_c);

    static float pressureToAltitude(float pressure_pa);

    I2C_HandleTypeDef* m_hi2c = nullptr;

    // BMP390 calibration coefficients (from NVM registers 0x31–0x45)
    struct Bmp390Calib {
        float T1, T2, T3;
        float P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
    } m_calib = {};

    // Published values
    float m_altitude_m   = 0.0F;
    float m_pressure_pa  = 101325.0F;
    float m_temp_c       = 20.0F;

    bool m_initialized = false;
};

}  // namespace Ap

#endif  // AP_HAL_STM32_I2C_BARO_HPP
