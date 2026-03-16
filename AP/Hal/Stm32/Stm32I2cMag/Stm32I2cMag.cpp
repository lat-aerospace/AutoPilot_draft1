// ======================================================================
// \title  AP/Hal/Stm32/Stm32I2cMag/Stm32I2cMag.cpp
// \brief  IST8310 I2C magnetometer driver implementation
// ======================================================================
#include <AP/Hal/Stm32/Stm32I2cMag/Stm32I2cMag.hpp>
#include <Fw/Types/Assert.hpp>
#include <cstring>

namespace Ap {

// I2C 8-bit write address: 7-bit address << 1
static constexpr uint16_t IST_ADDR_WRITE =
    static_cast<uint16_t>(Stm32I2cMag::IST_ADDR_7BIT << 1U);

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Stm32I2cMag::Stm32I2cMag(const char* const compName)
    : Stm32I2cMagComponentBase(compName) {}

// ---------------------------------------------------------------------------
// configure
// ---------------------------------------------------------------------------
void Stm32I2cMag::configure(I2C_HandleTypeDef* hi2c) {
    m_hi2c = hi2c;
}

// ---------------------------------------------------------------------------
// initSensor – soft-reset, verify ID, configure averaging
// ---------------------------------------------------------------------------
bool Stm32I2cMag::initSensor() {
    FW_ASSERT(m_hi2c != nullptr);

    // Soft reset
    if (!i2cWrite(IST_REG_CNTL2, IST_SRST)) { return false; }
    HAL_Delay(10U);  // wait for reset to complete

    // Verify device ID
    uint8_t wia = 0U;
    if (!i2cRead(IST_REG_WIA, &wia, 1U)) { return false; }
    if (wia != IST_CHIP_ID) { return false; }

    // Configure 16-sample averaging and normal pulse duration
    if (!i2cWrite(IST_REG_AVGCNTL, IST_AVGCNTL_VAL)) { return false; }
    if (!i2cWrite(IST_REG_PDCNTL,  IST_PDCNTL_VAL))  { return false; }

    m_initialized = true;
    m_pending     = false;
    return true;
}

// ---------------------------------------------------------------------------
// schedIn_handler – 50 Hz alternating trigger/read
// ---------------------------------------------------------------------------
void Stm32I2cMag::schedIn_handler(FwIndexType /*portNum*/,
                                   U32 /*context*/) {
    if (!m_initialized) { return; }

    if (!m_pending) {
        // Phase 0: trigger a single measurement
        (void)startMeasurement();
        m_pending = true;
    } else {
        // Phase 1: attempt to read result
        Ap::MagData mag;
        if (readData(mag)) {
            magOut_out(0, mag);
            tlmWrite_magX(mag.get_field_gauss().get_x());
            tlmWrite_magY(mag.get_field_gauss().get_y());
            tlmWrite_magZ(mag.get_field_gauss().get_z());
        }
        m_pending = false;
    }
}

// ---------------------------------------------------------------------------
// startMeasurement – write CNTL1 = 0x01
// ---------------------------------------------------------------------------
bool Stm32I2cMag::startMeasurement() {
    return i2cWrite(IST_REG_CNTL1, IST_SINGLE_MEAS);
}

// ---------------------------------------------------------------------------
// readData – check DRDY, burst-read 6 bytes, convert to Gauss
// ---------------------------------------------------------------------------
bool Stm32I2cMag::readData(Ap::MagData& out) {
    // Check DRDY
    uint8_t stat = 0U;
    if (!i2cRead(IST_REG_STAT1, &stat, 1U)) { return false; }
    if ((stat & IST_DRDY) == 0U) { return false; }

    // Burst read: DATAXL..DATAZH (6 bytes starting at 0x03)
    uint8_t buf[6] = {};
    if (!i2cRead(IST_REG_DATAXL, buf, 6U)) { return false; }

    // Reconstruct signed 16-bit values (little-endian)
    int16_t raw_x, raw_y, raw_z;
    (void)memcpy(&raw_x, &buf[0], 2U);
    (void)memcpy(&raw_y, &buf[2], 2U);
    (void)memcpy(&raw_z, &buf[4], 2U);

    // Convert to Gauss
    const float gx = static_cast<float>(raw_x) * SENS_XY;
    const float gy = static_cast<float>(raw_y) * SENS_XY;
    const float gz = static_cast<float>(raw_z) * SENS_Z;

    Ap::Vec3 field;
    field.set_x(gx);
    field.set_y(gy);
    field.set_z(gz);
    out.set_field_gauss(field);
    out.set_time_s(0.0);  // caller can stamp; FreeRtosTime available via topology

    return true;
}

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------
bool Stm32I2cMag::i2cWrite(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    const HAL_StatusTypeDef rc =
        HAL_I2C_Master_Transmit(m_hi2c, IST_ADDR_WRITE,
                                buf, 2U,
                                static_cast<uint32_t>(I2C_TIMEOUT_MS));
    return rc == HAL_OK;
}

bool Stm32I2cMag::i2cRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    // Send register address
    HAL_StatusTypeDef rc =
        HAL_I2C_Master_Transmit(m_hi2c, IST_ADDR_WRITE,
                                &reg, 1U,
                                static_cast<uint32_t>(I2C_TIMEOUT_MS));
    if (rc != HAL_OK) { return false; }

    // Read data
    rc = HAL_I2C_Master_Receive(m_hi2c, IST_ADDR_WRITE | 1U,
                                buf, static_cast<uint16_t>(len),
                                static_cast<uint32_t>(I2C_TIMEOUT_MS));
    return rc == HAL_OK;
}

}  // namespace Ap
