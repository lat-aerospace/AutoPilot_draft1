// ======================================================================
// \title  AP/Hal/Stm32/Stm32Uart/Stm32Uart.hpp
// \brief  STM32 UART-based MAVLink gateway for hardware deployment
//
// Replaces MavlinkGateway (UDP) for the STM32 target.  Same port interface
// so the topology wiring is identical.
//
// TX: MAVLink frames assembled in schedIn_handler and sent via DMA.
// RX: Circular DMA; schedIn_handler drains the ring buffer, parsing
//     MAVLink packets and dispatching to appropriate handlers.
//
// Messages sent (10 Hz unless noted):
//   HEARTBEAT (1 Hz), ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD,
//   SYS_STATUS, NAMED_VALUE_FLOAT (debug)
//
// Messages received:
//   MANUAL_CONTROL, RC_CHANNELS_OVERRIDE, HEARTBEAT (GCS),
//   COMMAND_LONG, PARAM_REQUEST_LIST, PARAM_REQUEST_READ,
//   REQUEST_DATA_STREAM, SET_MODE, MISSION_COUNT,
//   MISSION_ITEM_INT, MISSION_REQUEST_LIST
// ======================================================================
#ifndef AP_HAL_STM32_UART_HPP
#define AP_HAL_STM32_UART_HPP

#include <stm32h7xx_hal.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <AP/Types/ApTypesSerializableAc.hpp>
#include <common/mavlink.h>
#include <AP/Hal/Stm32/Stm32Uart/Stm32UartComponentAc.hpp>

namespace Ap {

class Stm32Uart final : public Stm32UartComponentBase {
  public:
    static constexpr size_t RX_BUF_SIZE = 512U;
    static constexpr size_t TX_BUF_SIZE = 280U;

    explicit Stm32Uart(const char* const compName);
    ~Stm32Uart() override;

    void configure(UART_HandleTypeDef* huart,
                   uint8_t             sys_id = 1,
                   uint8_t             comp_id = 1);
    void startRx();
    void rxIdleCallback();

  private:
    // F' port handlers
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;
    void modeIn_handler(FwIndexType portNum, const Ap::FlightMode& mode) override;

    // MAVLink telemetry senders
    void sendHeartbeat();
    void sendAttitude(const Ap::AircraftState& state);
    void sendGlobalPosition(const Ap::AircraftState& state);
    void sendVfrHud(const Ap::AircraftState& state);
    void sendSysStatus();
    void sendNamedFloats(const Ap::AircraftState& state);

    // MAVLink receive + dispatch
    void parseMavlink();
    void handleMessage(const mavlink_message_t& msg);
    void handleManualControl(const mavlink_message_t& msg);
    void handleRcChannelsOverride(const mavlink_message_t& msg);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleSetMode(const mavlink_message_t& msg);
    void handleMissionCount(const mavlink_message_t& msg);
    void handleMissionItemInt(const mavlink_message_t& msg);
    void handleMissionRequestList(const mavlink_message_t& msg);

    // Parameter helpers
    void sendAllParams();
    void sendParamValue(uint16_t index);
    void sendParamByName(const char* name);

    // Low-level TX
    void sendMavlinkMsg(const mavlink_message_t& msg);
    void sendBytes(const uint8_t* data, size_t len);

    // Hardware
    UART_HandleTypeDef* m_huart    = nullptr;
    uint8_t             m_sys_id   = 1;
    uint8_t             m_comp_id  = 1;

    // DMA circular receive buffer
    uint8_t  m_rx_dma_buf[RX_BUF_SIZE];
    uint16_t m_rx_read_pos = 0;

    // TX buffer
    uint8_t  m_tx_buf[TX_BUF_SIZE];

    // MAVLink parser state
    mavlink_message_t  m_mav_msg;
    mavlink_status_t   m_mav_status;

    // Cached state for periodic telemetry
    Ap::AircraftState m_state;
    Ap::FlightMode    m_mode = Ap::FlightMode::FBWB;
    bool              m_state_valid = false;

    // Heartbeat / tick counter
    U32  m_tick_count = 0;
    static constexpr U32 HEARTBEAT_TICKS = 10U;  // 10 Hz / 10 = 1 Hz

    // Param state
    bool m_paramsSentToGcs = false;

    // Mission upload state
    uint16_t m_missionCount    = 0;
    uint16_t m_missionReceived = 0;
};

}  // namespace Ap

#endif  // AP_HAL_STM32_UART_HPP
