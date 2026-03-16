// ======================================================================
// \title  AP/Hal/Stm32/Stm32SpiImu/Stm32SpiImu.cpp
// \brief  STM32 SPI driver for ICM-20948 IMU + AK09916 magnetometer
// ======================================================================
#include <AP/Hal/Stm32/Stm32SpiImu/Stm32SpiImu.hpp>
#include <Fw/Types/Assert.hpp>
#include <cstring>
#include <cmath>

// Busy-wait delay for sensor init (runs before FreeRTOS scheduler starts).
static void initDelayMs(uint32_t ms) {
    for (uint32_t m = 0; m < ms; m++) {
        for (volatile uint32_t i = 0; i < 50000U; i++) {
            __NOP();
        }
    }
}

// =====================================================================
// ICM-20948 register addresses (bank-selected via REG_BANK_SEL = 0x7F)
// =====================================================================

// Bank 0
static constexpr uint8_t REG_WHO_AM_I           = 0x00U;
static constexpr uint8_t REG_USER_CTRL           = 0x03U;
static constexpr uint8_t REG_PWR_MGMT_1          = 0x06U;
static constexpr uint8_t REG_ACCEL_XOUT_H        = 0x2DU;
static constexpr uint8_t REG_GYRO_XOUT_H         = 0x33U;
static constexpr uint8_t REG_EXT_SLV_SENS_DATA_00 = 0x3BU;
static constexpr uint8_t REG_BANK_SEL             = 0x7FU;

// Bank 2
static constexpr uint8_t REG_GYRO_SMPLRT_DIV     = 0x00U;
static constexpr uint8_t REG_GYRO_CONFIG_1        = 0x01U;
static constexpr uint8_t REG_ODR_ALIGN_EN         = 0x09U;
static constexpr uint8_t REG_ACCEL_SMPLRT_DIV_1   = 0x10U;
static constexpr uint8_t REG_ACCEL_SMPLRT_DIV_2   = 0x11U;
static constexpr uint8_t REG_ACCEL_CONFIG         = 0x14U;

// Bank 3
static constexpr uint8_t REG_I2C_MST_ODR_CONFIG  = 0x00U;
static constexpr uint8_t REG_I2C_MST_CTRL         = 0x01U;
static constexpr uint8_t REG_I2C_SLV0_ADDR        = 0x03U;
static constexpr uint8_t REG_I2C_SLV0_REG         = 0x04U;
static constexpr uint8_t REG_I2C_SLV0_CTRL        = 0x05U;
static constexpr uint8_t REG_I2C_SLV0_DO          = 0x06U;

// ICM-20948 expected WHO_AM_I
static constexpr uint8_t ICM20948_WHO_AM_I_VAL    = 0xEAU;

// AK09916 I2C address (7-bit)
static constexpr uint8_t AK09916_I2C_ADDR         = 0x0CU;
static constexpr uint8_t AK09916_REG_WIA2         = 0x01U;
static constexpr uint8_t AK09916_WIA2_VAL         = 0x09U;
static constexpr uint8_t AK09916_REG_CNTL2        = 0x31U;
static constexpr uint8_t AK09916_REG_CNTL3        = 0x32U;
static constexpr uint8_t AK09916_REG_ST1          = 0x10U;
static constexpr uint8_t AK09916_MODE_100HZ       = 0x08U;  // Continuous mode 4 = 100 Hz

// Sensitivity: ±16 g FS → 2048 LSB/g, ±2000 °/s → 16.4 LSB/(°/s)
static constexpr float ACCEL_SCALE = 9.80665F / 2048.0F;   // m/s² per LSB
static constexpr float GYRO_SCALE  = (3.14159265F / 180.0F) / 16.4F; // rad/s per LSB

// AK09916 sensitivity: 0.15 µT/LSB → convert to Gauss: 0.15 * 0.01 = 0.0015 G/LSB
static constexpr float MAG_SCALE   = 0.0015F;

// Module-level pointer for DMA callback (only one IMU instance expected)
static Ap::Stm32SpiImu* s_imu_instance = nullptr;

// ---------------------------------------------------------------------------
// STM32 HAL DMA TX/RX complete callback (weak override)
// ---------------------------------------------------------------------------
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* /*hspi*/) {
    if (s_imu_instance != nullptr) {
        s_imu_instance->dmaCompleteCallback();
    }
}

namespace Ap {

Stm32SpiImu::Stm32SpiImu(const char* const compName)
    : Stm32SpiImuComponentBase(compName) {
    (void)memset(m_tx_buf, 0, sizeof(m_tx_buf));
    (void)memset(m_rx_buf, 0, sizeof(m_rx_buf));
    s_imu_instance = this;
}

Stm32SpiImu::~Stm32SpiImu() {
    s_imu_instance = nullptr;
    if (m_dma_sem != nullptr) {
        vSemaphoreDelete(m_dma_sem);
    }
}

// ---------------------------------------------------------------------------
// configure – bind HAL handles
// ---------------------------------------------------------------------------
void Stm32SpiImu::configure(SPI_HandleTypeDef* hspi,
                              GPIO_TypeDef*      cs_port,
                              uint16_t           cs_pin) {
    m_hspi    = hspi;
    m_cs_port = cs_port;
    m_cs_pin  = cs_pin;

    // Binary semaphore used to signal DMA completion to schedIn_handler
    m_dma_sem = xSemaphoreCreateBinaryStatic(&m_dma_sem_buf);
    FW_ASSERT(m_dma_sem != nullptr);
}

// ---------------------------------------------------------------------------
// CS helpers
// ---------------------------------------------------------------------------
void Stm32SpiImu::csAssert()   { HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_RESET); }
void Stm32SpiImu::csDeassert() { HAL_GPIO_WritePin(m_cs_port, m_cs_pin, GPIO_PIN_SET);   }

// ---------------------------------------------------------------------------
// Low-level register access (blocking polled, used during init only)
// Match VajR: separate Transmit + Receive, 1000 ms timeout
// ---------------------------------------------------------------------------
void Stm32SpiImu::writeReg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7FU), value };
    csAssert();
    HAL_SPI_Transmit(m_hspi, tx, 2, 1000);
    csDeassert();
}

uint8_t Stm32SpiImu::readReg(uint8_t reg) {
    uint8_t read_reg = static_cast<uint8_t>(reg | 0x80U);
    uint8_t reg_val  = 0x00U;
    csAssert();
    HAL_SPI_Transmit(m_hspi, &read_reg, 1, 1000);
    HAL_SPI_Receive(m_hspi, &reg_val, 1, 1000);
    csDeassert();
    return reg_val;
}

// ---------------------------------------------------------------------------
// ICM-20948 bank selection
// ---------------------------------------------------------------------------
void Stm32SpiImu::selectBank(uint8_t bank) {
    writeReg(REG_BANK_SEL, static_cast<uint8_t>((bank & 0x03U) << 4U));
}

// ---------------------------------------------------------------------------
// AK09916 I2C master helpers (via ICM-20948 Bank 3 SLV0)
// ---------------------------------------------------------------------------
void Stm32SpiImu::i2cMasterWrite(uint8_t addr, uint8_t reg, uint8_t value) {
    selectBank(3);
    writeReg(REG_I2C_SLV0_ADDR, addr);       // Write (bit7=0)
    writeReg(REG_I2C_SLV0_REG, reg);
    writeReg(REG_I2C_SLV0_DO, value);
    writeReg(REG_I2C_SLV0_CTRL, 0x81U);      // Enable + 1 byte
    selectBank(0);
    initDelayMs(10);
}

uint8_t Stm32SpiImu::i2cMasterRead(uint8_t addr, uint8_t reg) {
    selectBank(3);
    writeReg(REG_I2C_SLV0_ADDR, static_cast<uint8_t>(addr | 0x80U));  // Read (bit7=1)
    writeReg(REG_I2C_SLV0_REG, reg);
    writeReg(REG_I2C_SLV0_CTRL, 0x81U);      // Enable + 1 byte
    selectBank(0);
    initDelayMs(10);
    return readReg(REG_EXT_SLV_SENS_DATA_00);
}

// ---------------------------------------------------------------------------
// initMag – configure AK09916 via ICM-20948's I2C master
// ---------------------------------------------------------------------------
bool Stm32SpiImu::initMag() {
    // Reset I2C master before enabling it
    selectBank(0);
    uint8_t user_ctrl = readReg(REG_USER_CTRL);
    writeReg(REG_USER_CTRL, static_cast<uint8_t>(user_ctrl | 0x02U));  // I2C_MST_RST
    initDelayMs(10);

    // Enable I2C master
    user_ctrl = readReg(REG_USER_CTRL);
    writeReg(REG_USER_CTRL, static_cast<uint8_t>(user_ctrl | 0x20U));  // I2C_MST_EN
    initDelayMs(10);

    // Configure I2C master clock
    selectBank(3);
    writeReg(REG_I2C_MST_CTRL, 0x07U);       // I2C master clock = 345.6 kHz
    writeReg(REG_I2C_MST_ODR_CONFIG, 0x03U); // ODR = 1.1 kHz / (1 + 3) ≈ 275 Hz
    selectBank(0);

    // Check AK09916 WHO_AM_I
    uint8_t mag_id = i2cMasterRead(AK09916_I2C_ADDR, AK09916_REG_WIA2);
    if (mag_id != AK09916_WIA2_VAL) {
        return false;
    }

    // Soft reset AK09916
    i2cMasterWrite(AK09916_I2C_ADDR, AK09916_REG_CNTL3, 0x01U);
    initDelayMs(100);

    // Set continuous measurement mode 4 (100 Hz)
    i2cMasterWrite(AK09916_I2C_ADDR, AK09916_REG_CNTL2, AK09916_MODE_100HZ);
    initDelayMs(10);

    // Configure SLV0 for continuous 8-byte read from AK09916 ST1 register
    // This will auto-read ST1 + HXL + HXH + HYL + HYH + HZL + HZH + ST2
    selectBank(3);
    writeReg(REG_I2C_SLV0_ADDR, static_cast<uint8_t>(AK09916_I2C_ADDR | 0x80U));
    writeReg(REG_I2C_SLV0_REG, AK09916_REG_ST1);
    writeReg(REG_I2C_SLV0_CTRL, 0x88U);      // Enable + 8 bytes
    selectBank(0);

    m_mag_initialized = true;
    return true;
}

// ---------------------------------------------------------------------------
// initSensor – configure ICM-20948 + AK09916
// Matches VajR's icm20948_init() sequence exactly:
//   1. WHO_AM_I retry loop (VajR loops forever; we limit to 50 attempts)
//   2. Device reset (0x80 | 0x41 → PWR_MGMT_1)
//   3. Wakeup (clear sleep bit via read-modify-write)
//   4. Clock source (auto = 1)
//   5. ODR align enable
//   6. SPI slave enable (disable I2C interface)
//   7. Gyro LPF (config 0)
//   8. Accel LPF (config 0)
//   9. Gyro sample rate divider = 0
//  10. Accel sample rate divider = 0
//  11. Gyro full scale ±2000 dps
//  12. Accel full scale ±16 g
// ---------------------------------------------------------------------------
bool Stm32SpiImu::initSensor() {
    FW_ASSERT(m_hspi != nullptr);

    m_dbg_init_stage = 1;  // stage 1: power-on delay

    // Wait for ICM-20948 to boot after power-on (datasheet: 100 ms minimum)
    initDelayMs(200);

    m_dbg_init_stage = 2;  // stage 2: WHO_AM_I retry loop

    // --- Step 1: WHO_AM_I retry loop (like VajR's `while(!icm20948_who_am_i())`) ---
    // VajR loops forever; we limit to 50 attempts (~5 seconds)
    bool who_ok = false;
    for (uint32_t attempt = 0; attempt < 50U; attempt++) {
        m_dbg_who_attempts = attempt + 1;

        selectBank(0);
        uint8_t who = readReg(REG_WHO_AM_I);

        m_dbg_who_am_i   = who;
        m_dbg_hal_status  = m_hspi->ErrorCode;
        m_dbg_spi_error   = m_hspi->ErrorCode;
        m_dbg_rx[0]       = static_cast<uint8_t>(REG_WHO_AM_I | 0x80U);
        m_dbg_rx[1]       = who;

        if (who == ICM20948_WHO_AM_I_VAL) {
            who_ok = true;
            break;
        }
        initDelayMs(100);
    }
    if (!who_ok) {
        m_dbg_init_stage = 0xFF;  // stuck at WHO_AM_I
        return false;
    }

    m_dbg_init_stage = 3;  // stage 3: device reset

    // --- Step 2: Device reset (VajR: icm20948_device_reset) ---
    selectBank(0);
    writeReg(REG_PWR_MGMT_1, 0x80U | 0x41U);
    initDelayMs(100);

    m_dbg_init_stage = 4;  // stage 4: wakeup

    // --- Step 3: Wakeup – clear sleep bit via read-modify-write (VajR style) ---
    selectBank(0);
    {
        uint8_t pwr = readReg(REG_PWR_MGMT_1);
        pwr &= 0xBFU;  // clear bit 6 (SLEEP)
        writeReg(REG_PWR_MGMT_1, pwr);
    }
    initDelayMs(100);

    m_dbg_init_stage = 5;  // stage 5: clock source

    // --- Step 4: Clock source = auto (VajR: icm20948_clock_source(1)) ---
    selectBank(0);
    {
        uint8_t pwr = readReg(REG_PWR_MGMT_1);
        pwr |= 0x01U;  // set CLKSEL = 1 (auto)
        writeReg(REG_PWR_MGMT_1, pwr);
    }

    m_dbg_init_stage = 6;  // stage 6: ODR align

    // --- Step 5: ODR alignment enable ---
    selectBank(2);
    writeReg(REG_ODR_ALIGN_EN, 0x01U);

    m_dbg_init_stage = 7;  // stage 7: SPI slave enable

    // --- Step 6: SPI slave enable (disable I2C slave interface) ---
    selectBank(0);
    {
        uint8_t uc = readReg(REG_USER_CTRL);
        uc |= 0x10U;  // I2C_IF_DIS
        writeReg(REG_USER_CTRL, uc);
    }

    m_dbg_init_stage = 8;  // stage 8: gyro LPF

    // --- Step 7: Gyro low-pass filter (config 0 = no filtering) ---
    selectBank(2);
    {
        uint8_t gc = readReg(REG_GYRO_CONFIG_1);
        gc |= (0U << 3U);  // DLPF_CFG = 0
        writeReg(REG_GYRO_CONFIG_1, gc);
    }

    m_dbg_init_stage = 9;  // stage 9: accel LPF

    // --- Step 8: Accel low-pass filter (config 0 = no filtering) ---
    selectBank(2);
    {
        uint8_t ac = readReg(REG_ACCEL_CONFIG);
        ac |= (0U << 3U);  // DLPF_CFG = 0
        writeReg(REG_ACCEL_CONFIG, ac);
    }

    m_dbg_init_stage = 10;  // stage 10: sample rate dividers

    // --- Step 9: Gyro sample rate divider = 0 (1.1 kHz ODR) ---
    selectBank(2);
    writeReg(REG_GYRO_SMPLRT_DIV, 0x00U);

    // --- Step 10: Accel sample rate divider = 0 (1.125 kHz ODR) ---
    selectBank(2);
    writeReg(REG_ACCEL_SMPLRT_DIV_1, 0x00U);
    writeReg(REG_ACCEL_SMPLRT_DIV_2, 0x00U);

    m_dbg_init_stage = 11;  // stage 11: full scale select

    // --- Step 11: Gyro full scale ±2000 dps (read-modify-write like VajR) ---
    selectBank(2);
    {
        uint8_t gc = readReg(REG_GYRO_CONFIG_1);
        gc |= 0x06U;  // FS_SEL = 11 (2000 dps)
        writeReg(REG_GYRO_CONFIG_1, gc);
    }

    // --- Step 12: Accel full scale ±16 g (read-modify-write like VajR) ---
    selectBank(2);
    {
        uint8_t ac = readReg(REG_ACCEL_CONFIG);
        ac |= 0x06U;  // FS_SEL = 11 (16 g)
        writeReg(REG_ACCEL_CONFIG, ac);
    }

    // Return to Bank 0
    selectBank(0);

    m_dbg_init_stage = 12;  // stage 12: init complete
    m_initialized = true;

    // Initialize AK09916 magnetometer via I2C master
    (void)initMag();

    return true;
}

// ---------------------------------------------------------------------------
// startDmaRead – polled SPI burst read (DMA removed to match VajR)
// ---------------------------------------------------------------------------
void Stm32SpiImu::startDmaRead() {
    // Polled burst read of accel+gyro: 1 address byte + 12 data bytes
    m_tx_buf[0] = static_cast<uint8_t>(REG_ACCEL_XOUT_H | 0x80U);
    csAssert();
    HAL_SPI_TransmitReceive(m_hspi, m_tx_buf, m_rx_buf,
                             static_cast<uint16_t>(DMA_BUF_LEN), 1000);
    csDeassert();
}

// ---------------------------------------------------------------------------
// dmaCompleteCallback – unused (polled mode), kept for future DMA re-enable
// ---------------------------------------------------------------------------
void Stm32SpiImu::dmaCompleteCallback() {
    csDeassert();
    BaseType_t higher_prio = pdFALSE;
    xSemaphoreGiveFromISR(m_dma_sem, &higher_prio);
    portYIELD_FROM_ISR(higher_prio);
}

// ---------------------------------------------------------------------------
// Raw-to-engineering-unit helpers
// ---------------------------------------------------------------------------
float Stm32SpiImu::rawToAccel(int16_t raw) {
    return static_cast<float>(raw) * ACCEL_SCALE;
}
float Stm32SpiImu::rawToGyro(int16_t raw) {
    return static_cast<float>(raw) * GYRO_SCALE;
}

// ---------------------------------------------------------------------------
// schedIn_handler – 400 Hz callback from RG1
// ---------------------------------------------------------------------------
void Stm32SpiImu::schedIn_handler(FwIndexType /*portNum*/,
                                   U32 /*context*/) {
    if (!m_initialized) { return; }

    // 1. Polled SPI burst read of accel+gyro (no DMA — matching VajR)
    startDmaRead();

    // 2. Parse the 12-byte block: [dummy][AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L]
    //                                     [GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L]
    const uint8_t* d = m_rx_buf + 1;  // skip dummy byte
    auto combine = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8U) | lo);
    };

    const int16_t ax_raw = combine(d[0], d[1]);
    const int16_t ay_raw = combine(d[2], d[3]);
    const int16_t az_raw = combine(d[4], d[5]);
    const int16_t gx_raw = combine(d[6], d[7]);
    const int16_t gy_raw = combine(d[8], d[9]);
    const int16_t gz_raw = combine(d[10], d[11]);

    // 4. Build F' ImuData type
    Ap::ImuData imu;
    Ap::Vec3 accel;
    accel.set_x(rawToAccel(ax_raw));
    accel.set_y(rawToAccel(ay_raw));
    accel.set_z(rawToAccel(az_raw));

    Ap::Vec3 gyro;
    gyro.set_x(rawToGyro(gx_raw));
    gyro.set_y(rawToGyro(gy_raw));
    gyro.set_z(rawToGyro(gz_raw));

    imu.set_accel_mps2(accel);
    imu.set_gyro_dps(gyro);

    // 5. Publish IMU data at 400 Hz
    imuOut_out(0, imu);

    // 6. Read magnetometer every 4th tick (100 Hz)
    m_tick_count++;
    if (m_mag_initialized && (m_tick_count % 4U) == 0U) {
        // Polled SPI read of EXT_SLV_SENS_DATA (8 bytes: ST1 + 6 mag + ST2)
        uint8_t mag_tx[9] = {};
        uint8_t mag_rx[9] = {};
        mag_tx[0] = static_cast<uint8_t>(REG_EXT_SLV_SENS_DATA_00 | 0x80U);
        csAssert();
        HAL_SPI_TransmitReceive(m_hspi, mag_tx, mag_rx, 9, 10);
        csDeassert();

        // mag_rx[0] = dummy, mag_rx[1] = ST1
        // mag_rx[2..3] = HXL,HXH, mag_rx[4..5] = HYL,HYH, mag_rx[6..7] = HZL,HZH
        // mag_rx[8] = ST2
        const uint8_t st1 = mag_rx[1];
        if (st1 & 0x01U) {  // DRDY bit
            const int16_t mx_raw = combine(mag_rx[3], mag_rx[2]);  // Note: AK09916 is little-endian
            const int16_t my_raw = combine(mag_rx[5], mag_rx[4]);
            const int16_t mz_raw = combine(mag_rx[7], mag_rx[6]);

            Ap::MagData mag;
            Ap::Vec3 field;
            field.set_x(static_cast<float>(mx_raw) * MAG_SCALE);
            field.set_y(static_cast<float>(my_raw) * MAG_SCALE);
            field.set_z(static_cast<float>(mz_raw) * MAG_SCALE);
            mag.set_field_gauss(field);

            magOut_out(0, mag);
        }
    }
}

}  // namespace Ap
