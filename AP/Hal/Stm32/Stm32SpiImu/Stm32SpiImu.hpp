// ======================================================================
// \title  AP/Hal/Stm32/Stm32SpiImu/Stm32SpiImu.hpp
// \brief  STM32 SPI driver for ICM-20948 IMU + AK09916 magnetometer
//
// Reads accelerometer (±16 g) and gyroscope (±2000 °/s) at 400 Hz.
// Reads AK09916 magnetometer at 100 Hz via ICM-20948 internal I2C master.
// Uses SPI2 with DMA so the bus transfer completes in the background;
// schedIn_handler triggers a DMA read, the DMA complete callback copies
// the result into an ImuData record and calls imuOut.
//
// ICM-20948 is a bank-selected device (REG_BANK_SEL = 0x7F).
// AK09916 is accessed through the ICM-20948's built-in I2C master
// (SPI → internal I2C bridge via Bank 3 registers).
// ======================================================================
#ifndef AP_HAL_STM32_SPI_IMU_HPP
#define AP_HAL_STM32_SPI_IMU_HPP

#include <FreeRTOS.h>
#include <semphr.h>
#include <stm32h7xx_hal.h>
#include <AP/Types/ApTypesSerializableAc.hpp>
#include <AP/Hal/Stm32/Stm32SpiImu/Stm32SpiImuComponentAc.hpp>

namespace Ap {

class Stm32SpiImu final : public Stm32SpiImuComponentBase {
  public:
    explicit Stm32SpiImu(const char* const compName);
    ~Stm32SpiImu() override;

    //! Bind hardware handles (call during topology init before startTasks)
    //! \param hspi    SPI2 handle configured for 5 MHz, CPOL=1, CPHA=1
    //! \param cs_port GPIO port for chip-select (GPIOD)
    //! \param cs_pin  GPIO pin number for chip-select (PIN_8)
    void configure(SPI_HandleTypeDef* hspi,
                   GPIO_TypeDef*      cs_port,
                   uint16_t           cs_pin);

    //! Initialize ICM-20948 + AK09916 registers (call after configure)
    bool initSensor();

    //! DMA transfer-complete callback – called from HAL_SPI_RxCpltCallback
    void dmaCompleteCallback();

  private:
    // F' port handler
    void schedIn_handler(FwIndexType portNum,
                         U32 context) override;

    // Low-level SPI helpers
    void     csAssert();
    void     csDeassert();
    void     writeReg(uint8_t reg, uint8_t value);
    uint8_t  readReg(uint8_t reg);
    void     startDmaRead();

    // ICM-20948 bank selection
    void     selectBank(uint8_t bank);

    // AK09916 I2C master helpers (via ICM-20948 Bank 3)
    void     i2cMasterWrite(uint8_t addr, uint8_t reg, uint8_t value);
    uint8_t  i2cMasterRead(uint8_t addr, uint8_t reg);
    bool     initMag();

    // Convert raw 16-bit two's-complement to engineering units
    static float rawToAccel(int16_t raw);  // m/s²
    static float rawToGyro(int16_t raw);   // rad/s

    // State
    SPI_HandleTypeDef* m_hspi      = nullptr;
    GPIO_TypeDef*      m_cs_port   = nullptr;
    uint16_t           m_cs_pin    = 0;

    // DMA receive buffer: 1 dummy + 12 data bytes (accel X,Y,Z + gyro X,Y,Z each 2B)
    static constexpr size_t DMA_BUF_LEN = 13U;
    uint8_t m_tx_buf[DMA_BUF_LEN];
    uint8_t m_rx_buf[DMA_BUF_LEN];

    SemaphoreHandle_t  m_dma_sem     = nullptr;
    StaticSemaphore_t  m_dma_sem_buf;

    // Mag reading state
    uint32_t m_tick_count     = 0U;
    bool     m_mag_initialized = false;

    bool m_initialized = false;

    // Debug: SPI init diagnostics (read via SWD)
    volatile uint8_t  m_dbg_who_am_i = 0;        // WHO_AM_I value read
    volatile uint32_t m_dbg_hal_status = 0xFF;    // HAL return: 0=OK,1=ERR,2=BUSY,3=TIMEOUT
    volatile uint8_t  m_dbg_rx[2] = {};           // raw SPI rx bytes
    volatile uint32_t m_dbg_spi_error = 0;        // hspi->ErrorCode after transfer
    volatile uint32_t m_dbg_who_attempts = 0;     // how many WHO_AM_I retries
    volatile uint8_t  m_dbg_init_stage = 0;       // which init step we reached
};

}  // namespace Ap

#endif  // AP_HAL_STM32_SPI_IMU_HPP
