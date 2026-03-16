// ======================================================================
// \title  AP/Hal/Stm32/Stm32I2cBaro/Stm32I2cBaro.cpp
// \brief  BMP390 I2C barometer driver
// ======================================================================
#include <AP/Hal/Stm32/Stm32I2cBaro/Stm32I2cBaro.hpp>
#include <Fw/Types/Assert.hpp>
#include <cmath>
#include <cstring>

// Busy-wait delay for sensor init (runs before FreeRTOS scheduler starts).
// At 200 MHz SYSCLK this gives approximately 1 ms per iteration.
static void initDelayMs(uint32_t ms) {
    for (uint32_t m = 0; m < ms; m++) {
        for (volatile uint32_t i = 0; i < 50000U; i++) {
            __NOP();
        }
    }
}

// BMP390 register addresses
static constexpr uint8_t REG_CHIP_ID     = 0x00U;
static constexpr uint8_t REG_PRESS_DATA  = 0x04U;  // 3 bytes: XLSB, LSB, MSB
static constexpr uint8_t REG_TEMP_DATA   = 0x07U;  // 3 bytes: XLSB, LSB, MSB
static constexpr uint8_t REG_PWR_CTRL    = 0x1BU;
static constexpr uint8_t REG_OSR         = 0x1CU;
static constexpr uint8_t REG_ODR         = 0x1DU;
static constexpr uint8_t REG_CALIB_START = 0x31U;  // 21 bytes of calibration data
static constexpr uint8_t BMP390_CHIP_ID  = 0x60U;

// Hypsometric constants
static constexpr float P0_PA   = 101325.0F;   // sea level pressure
static constexpr float T0_K    = 288.15F;     // sea level temperature
static constexpr float L_PER_M = 0.0065F;     // temperature lapse rate
static constexpr float G_DIV_RL = 5.2561F;    // g / (R * L)

namespace Ap {

Stm32I2cBaro::Stm32I2cBaro(const char* const compName)
    : Stm32I2cBaroComponentBase(compName) {}

void Stm32I2cBaro::configure(I2C_HandleTypeDef* hi2c) {
    m_hi2c = hi2c;
}

// ---------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------
bool Stm32I2cBaro::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return HAL_I2C_Master_Transmit(m_hi2c, I2C_ADDR, buf, 2, I2C_TIMEOUT) == HAL_OK;
}

bool Stm32I2cBaro::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    if (HAL_I2C_Master_Transmit(m_hi2c, I2C_ADDR, &reg, 1, I2C_TIMEOUT) != HAL_OK) {
        return false;
    }
    return HAL_I2C_Master_Receive(m_hi2c, I2C_ADDR, buf, len, I2C_TIMEOUT) == HAL_OK;
}

// ---------------------------------------------------------------------------
// initSensor – verify chip ID, configure, read calibration NVM
// ---------------------------------------------------------------------------
bool Stm32I2cBaro::initSensor() {
    FW_ASSERT(m_hi2c != nullptr);

    // Check chip ID
    uint8_t chip_id = 0;
    if (!readRegisters(REG_CHIP_ID, &chip_id, 1)) { return false; }
    if (chip_id != BMP390_CHIP_ID) { return false; }

    // Configure: temp + pressure enabled, normal mode
    if (!writeRegister(REG_PWR_CTRL, 0x33U)) { return false; }
    initDelayMs(2);

    // Oversampling: standard resolution (OSR pressure x8, temp x1)
    if (!writeRegister(REG_OSR, 0x01U)) { return false; }
    initDelayMs(2);

    // Output data rate: 100 Hz
    if (!writeRegister(REG_ODR, 0x01U)) { return false; }
    initDelayMs(2);

    // Read calibration NVM (21 bytes from 0x31)
    uint8_t calib_raw[21] = {};
    if (!readRegisters(REG_CALIB_START, calib_raw, 21)) { return false; }

    // Parse calibration data (per BMP390 datasheet Section 8.4)
    // NVM word: unsigned 16-bit or signed 8/16-bit, then float conversion
    auto u16 = [&](int i) -> uint16_t {
        return static_cast<uint16_t>(calib_raw[i]) |
               (static_cast<uint16_t>(calib_raw[i + 1]) << 8U);
    };
    auto s16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(u16(i));
    };
    auto s8 = [&](int i) -> int8_t {
        return static_cast<int8_t>(calib_raw[i]);
    };

    // Temperature calibration
    m_calib.T1 = static_cast<float>(u16(0)) / std::pow(2.0F, -8.0F);
    m_calib.T2 = static_cast<float>(u16(2)) / std::pow(2.0F, 30.0F);
    m_calib.T3 = static_cast<float>(s8(4))  / std::pow(2.0F, 48.0F);

    // Pressure calibration
    m_calib.P1  = (static_cast<float>(s16(5))  - std::pow(2.0F, 14.0F)) / std::pow(2.0F, 20.0F);
    m_calib.P2  = (static_cast<float>(s16(7))  - std::pow(2.0F, 14.0F)) / std::pow(2.0F, 29.0F);
    m_calib.P3  = static_cast<float>(s8(9))    / std::pow(2.0F, 32.0F);
    m_calib.P4  = static_cast<float>(s8(10))   / std::pow(2.0F, 37.0F);
    m_calib.P5  = static_cast<float>(u16(11))  / std::pow(2.0F, -3.0F);
    m_calib.P6  = static_cast<float>(u16(13))  / std::pow(2.0F, 6.0F);
    m_calib.P7  = static_cast<float>(s8(15))   / std::pow(2.0F, 8.0F);
    m_calib.P8  = static_cast<float>(s8(16))   / std::pow(2.0F, 15.0F);
    m_calib.P9  = static_cast<float>(s16(17))  / std::pow(2.0F, 48.0F);
    m_calib.P10 = static_cast<float>(s8(19))   / std::pow(2.0F, 48.0F);
    m_calib.P11 = static_cast<float>(s8(20))   / std::pow(2.0F, 65.0F);

    m_initialized = true;
    return true;
}

// ---------------------------------------------------------------------------
// compensate – BMP390 datasheet compensation polynomial
// ---------------------------------------------------------------------------
void Stm32I2cBaro::compensate(uint32_t raw_press, uint32_t raw_temp,
                                float& out_press_pa, float& out_temp_c) {
    // Temperature compensation
    const float pd1 = static_cast<float>(raw_temp) - m_calib.T1;
    const float pd2 = pd1 * m_calib.T2;
    out_temp_c = pd2 + (pd1 * pd1) * m_calib.T3;

    // Pressure compensation
    const float pp1 = m_calib.P6 * out_temp_c;
    const float pp2 = m_calib.P7 * (out_temp_c * out_temp_c);
    const float pp3 = m_calib.P8 * (out_temp_c * out_temp_c * out_temp_c);
    const float po1 = m_calib.P5 + pp1 + pp2 + pp3;

    const float pp4 = m_calib.P2 * out_temp_c;
    const float pp5 = m_calib.P3 * (out_temp_c * out_temp_c);
    const float pp6 = m_calib.P4 * (out_temp_c * out_temp_c * out_temp_c);
    const float po2 = static_cast<float>(raw_press) *
                       (m_calib.P1 + pp4 + pp5 + pp6);

    const float pp7 = static_cast<float>(raw_press) * static_cast<float>(raw_press);
    const float pp8 = m_calib.P9 + m_calib.P10 * out_temp_c;
    const float pp9 = pp7 * pp8;
    const float pp10 = pp7 * static_cast<float>(raw_press) * m_calib.P11;

    out_press_pa = po1 + po2 + pp9 + pp10;
}

float Stm32I2cBaro::pressureToAltitude(float pressure_pa) {
    return (T0_K / L_PER_M) *
           (1.0F - std::pow(pressure_pa / P0_PA, 1.0F / G_DIV_RL));
}

// ---------------------------------------------------------------------------
// schedIn_handler – 50 Hz continuous read
// ---------------------------------------------------------------------------
void Stm32I2cBaro::schedIn_handler(FwIndexType /*portNum*/,
                                    U32 /*context*/) {
    if (!m_initialized) { return; }

    // Read 6 bytes: pressure (3B at 0x04) + temperature (3B at 0x07)
    uint8_t data[6] = {};
    if (!readRegisters(REG_PRESS_DATA, data, 6)) { return; }

    // Assemble 24-bit raw values (little-endian: XLSB, LSB, MSB)
    const uint32_t raw_press = static_cast<uint32_t>(data[0]) |
                               (static_cast<uint32_t>(data[1]) << 8U) |
                               (static_cast<uint32_t>(data[2]) << 16U);
    const uint32_t raw_temp  = static_cast<uint32_t>(data[3]) |
                               (static_cast<uint32_t>(data[4]) << 8U) |
                               (static_cast<uint32_t>(data[5]) << 16U);

    // Apply compensation
    compensate(raw_press, raw_temp, m_pressure_pa, m_temp_c);

    // Compute altitude
    m_altitude_m = pressureToAltitude(m_pressure_pa);

    // Publish
    Ap::BaroData baro;
    baro.set_altitude_m(m_altitude_m);
    baro.set_pressure_pa(m_pressure_pa);
    baroOut_out(0, baro);
}

}  // namespace Ap
