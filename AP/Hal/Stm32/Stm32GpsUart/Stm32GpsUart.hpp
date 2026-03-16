// ======================================================================
// \title  AP/Hal/Stm32/Stm32GpsUart/Stm32GpsUart.hpp
// \brief  NEO-M9N UBX-NAV-PVT GPS parser over STM32 UART5 + DMA
//
// Protocol: UBX binary (preferred over NMEA for precision + bandwidth).
// Message:  UBX-NAV-PVT  class=0x01  id=0x07  length=92 bytes
//
// UBX frame structure:
//   0xB5 0x62        sync chars
//   class (1B)       0x01 = NAV
//   id    (1B)       0x07 = PVT
//   length (2B LE)   0x005C = 92
//   payload (92B)    see field map below
//   CK_A CK_B (2B)  Fletcher-8 checksum
//
// Key payload offsets (from start of payload):
//   0   iTOW      U4  GPS time of week [ms]
//   4   year      U2
//   6   month     U1
//   7   day       U1
//   20  fixType   U1  0=none,1=dead,2=2D,3=3D,4=GNSS+DR
//   21  flags     U1  bit0 = gnssFixOk
//   23  numSV     U1  number of satellites used
//   24  lon       I4  longitude [1e-7 deg]
//   28  lat       I4  latitude  [1e-7 deg]
//   32  height    I4  height above ellipsoid [mm]
//   36  hMSL      I4  height above mean sea level [mm]
//   48  velN      I4  NED north velocity [mm/s]
//   52  velE      I4  NED east velocity [mm/s]
//   56  velD      I4  NED down velocity [mm/s]
//   60  gSpeed    I4  ground speed [mm/s]
//   64  headMot   I4  heading of motion [1e-5 deg]
//
// Receive: DMA circular buffer on UART5.
//          schedIn_handler (10 Hz) drains the ring buffer and parses frames.
//
// GPS initialization:
//   initGps() configures the NEO-M9N at startup:
//   1. Send baud rate change command (38400 → 57600)
//   2. Reinitialize UART at 57600
//   3. Disable all NMEA messages
//   4. Enable UBX-NAV-PVT
//   5. Set 10 Hz update rate
//   6. Save config
// ======================================================================
#ifndef AP_HAL_STM32_GPS_UART_HPP
#define AP_HAL_STM32_GPS_UART_HPP

#include <stm32h7xx_hal.h>
#include <AP/Types/ApTypesSerializableAc.hpp>
#include <AP/Hal/Stm32/Stm32GpsUart/Stm32GpsUartComponentAc.hpp>

namespace Ap {

class Stm32GpsUart final : public Stm32GpsUartComponentBase {
  public:
    static constexpr size_t   RX_BUF_SIZE     = 512U;   ///< DMA circular buffer
    static constexpr uint8_t  UBX_SYNC_1      = 0xB5U;
    static constexpr uint8_t  UBX_SYNC_2      = 0x62U;
    static constexpr uint8_t  UBX_CLASS_NAV   = 0x01U;
    static constexpr uint8_t  UBX_ID_PVT      = 0x07U;
    static constexpr uint16_t UBX_PVT_LEN     = 92U;

    explicit Stm32GpsUart(const char* const compName);
    ~Stm32GpsUart() override = default;

    //! Bind UART handle (call before startTasks)
    void configure(UART_HandleTypeDef* huart);

    //! Configure NEO-M9N: baud rate, disable NMEA, enable NAV-PVT, 10 Hz
    void initGps();

    //! Start DMA receive in circular mode
    void startRx();

  private:
    void schedIn_handler(FwIndexType portNum,
                         U32 context) override;

    //! Try to parse one complete UBX-NAV-PVT frame from the ring buffer.
    //! Returns true when a valid frame is found and \a out is populated.
    bool parseNavPvt(Ap::GpsData& out);

    //! Peek at a byte relative to m_rx_read_pos without advancing it
    uint8_t peek(uint16_t offset) const;

    //! Advance read position by n bytes
    void advance(uint16_t n);

    //! Bytes available in the ring buffer
    uint16_t available() const;

    //! UBX Fletcher-8 checksum over payload bytes
    static bool checksum(const uint8_t* payload, uint16_t len,
                         uint8_t expected_a, uint8_t expected_b);

    //! Send a UBX command with checksum
    void sendUbxCommand(uint8_t cls, uint8_t id,
                        const uint8_t* payload, uint16_t len);

    UART_HandleTypeDef* m_huart     = nullptr;
    uint8_t             m_rx_buf[RX_BUF_SIZE];
    uint16_t            m_rx_read   = 0U;

    bool                m_initialized = false;
};

}  // namespace Ap

#endif  // AP_HAL_STM32_GPS_UART_HPP
