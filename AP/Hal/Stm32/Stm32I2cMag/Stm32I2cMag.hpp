// ======================================================================
// \title  AP/Hal/Stm32/Stm32I2cMag/Stm32I2cMag.hpp
// \brief  IST8310 3-axis magnetometer driver over STM32 I2C
//
// Device: IST8310
//   I2C 7-bit address : 0x0E  (ADDR pin low)
//   Device ID register: 0x00 → 0x10
//
// Register map (relevant subset):
//   0x00 WIA       Device ID (read-only, = 0x10)
//   0x02 STAT1     Bit 0 = DRDY (1 = data ready)
//   0x03 DATAXL    X low byte
//   0x04 DATAXH    X high byte
//   0x05 DATAYL    Y low byte
//   0x06 DATAYH    Y high byte
//   0x07 DATAZL    Z low byte
//   0x08 DATAZH    Z high byte
//   0x0A CNTL1     Bit 0 = single-measurement trigger
//   0x0B CNTL2     Bit 0 = SRST (soft reset)
//   0x41 AVGCNTL   Averaging: 0x24 → 16-sample avg on X/Y
//   0x42 PDCNTL    Pulse duration: 0xC0 → normal
//
// Sensitivity:
//   X/Y  0.003 Gauss / LSB
//   Z    0.004 Gauss / LSB
//
// Scheduling (50 Hz via RG3):
//   Phase 0 (even cycles): write CNTL1=0x01 → start single measurement
//   Phase 1 (odd cycles) : read STAT1; if DRDY, burst-read 6 data bytes
// ======================================================================
#ifndef AP_HAL_STM32_I2C_MAG_HPP
#define AP_HAL_STM32_I2C_MAG_HPP

#include <stm32h7xx_hal.h>
#include <AP/Types/ApTypesSerializableAc.hpp>
#include <AP/Hal/Stm32/Stm32I2cMag/Stm32I2cMagComponentAc.hpp>

namespace Ap {

class Stm32I2cMag final : public Stm32I2cMagComponentBase {
  public:
    // ----------------------------------------------------------------
    // IST8310 register addresses
    // ----------------------------------------------------------------
    static constexpr uint8_t IST_ADDR_7BIT   = 0x0EU;
    static constexpr uint8_t IST_REG_WIA     = 0x00U;
    static constexpr uint8_t IST_REG_STAT1   = 0x02U;
    static constexpr uint8_t IST_REG_DATAXL  = 0x03U;
    static constexpr uint8_t IST_REG_CNTL1   = 0x0AU;
    static constexpr uint8_t IST_REG_CNTL2   = 0x0BU;
    static constexpr uint8_t IST_REG_AVGCNTL = 0x41U;
    static constexpr uint8_t IST_REG_PDCNTL  = 0x42U;

    static constexpr uint8_t IST_CHIP_ID     = 0x10U;
    static constexpr uint8_t IST_SRST        = 0x01U;  ///< CNTL2 soft reset
    static constexpr uint8_t IST_SINGLE_MEAS = 0x01U;  ///< CNTL1 trigger
    static constexpr uint8_t IST_DRDY        = 0x01U;  ///< STAT1 data-ready

    // Averaging: 16 samples on X and Y axes
    static constexpr uint8_t IST_AVGCNTL_VAL = 0x24U;
    // Normal pulse duration
    static constexpr uint8_t IST_PDCNTL_VAL  = 0xC0U;

    // Gauss per LSB
    static constexpr float SENS_XY = 0.003f;
    static constexpr float SENS_Z  = 0.004f;

    static constexpr uint32_t I2C_TIMEOUT_MS = 10U;

    // ----------------------------------------------------------------

    explicit Stm32I2cMag(const char* const compName);
    ~Stm32I2cMag() override = default;

    //! Bind I2C peripheral handle (call before startTasks)
    void configure(I2C_HandleTypeDef* hi2c);

    //! Soft-reset, verify device ID, and configure averaging.
    //! Returns true on success.
    bool initSensor();

  private:
    void schedIn_handler(FwIndexType portNum,
                         U32 context) override;

    bool startMeasurement();
    bool readData(Ap::MagData& out);

    bool i2cWrite(uint8_t reg, uint8_t val);
    bool i2cRead (uint8_t reg, uint8_t* buf, uint8_t len);

    I2C_HandleTypeDef* m_hi2c        = nullptr;
    bool               m_initialized = false;
    bool               m_pending     = false;  ///< measurement started, not yet read
};

}  // namespace Ap

#endif  // AP_HAL_STM32_I2C_MAG_HPP
