#ifndef Ap_MavlinkGateway_HPP
#define Ap_MavlinkGateway_HPP

#include "AP/Components/Mavlink/MavlinkGateway/MavlinkGatewayComponentAc.hpp"

#include <common/mavlink.h>
#include <netinet/in.h>

namespace Ap {

class MavlinkGateway final : public MavlinkGatewayComponentBase {
  public:
    MavlinkGateway(const char* const compName);
    ~MavlinkGateway();

    /// Open UDP socket. Call after construction, before starting.
    void configure(const char* targetIp = "127.0.0.1",
                   U16 targetPort = 14550,
                   U16 bindPort = 14540);

  private:
    // Port handlers
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;
    void modeIn_handler(FwIndexType portNum, const Ap::FlightMode& mode) override;

    // Send a packed MAVLink message over UDP
    void sendMavlinkMsg(const mavlink_message_t& msg);

    // Send HEARTBEAT (called at 1Hz)
    void sendHeartbeat();

    // Send telemetry messages (called at 10Hz)
    void sendTelemetry();

    // Receive and parse incoming MAVLink messages (non-blocking)
    void recvMavlink();

    // Process a decoded MAVLink message
    void handleMessage(const mavlink_message_t& msg);

    // Send all parameters (response to PARAM_REQUEST_LIST)
    void sendAllParams();

    // Send a single PARAM_VALUE by index
    void sendParamValue(uint16_t index);

    // Send a single PARAM_VALUE by name
    void sendParamByName(const char* name);

    // Latest estimated state
    Ap::AircraftState m_state;

    // Current flight mode (from Autonomy, for heartbeat custom_mode)
    Ap::FlightMode m_mode = Ap::FlightMode::FBWB;

    // UDP socket
    int m_sockFd = -1;
    struct sockaddr_in m_targetAddr;

    // Heartbeat counter (send every 10th schedIn = 1Hz)
    U32 m_tickCount    = 0;
    U32 m_msgsSent     = 0;
    U32 m_msgsRecvd    = 0;
    U32 m_rcMsgCnt     = 0;

    // Latest RC stick values for telemetry
    F32 m_rcRoll     = 0.0f;
    F32 m_rcPitch    = 0.0f;
    F32 m_rcThrottle = 0.0f;
    F32 m_rcYaw      = 0.0f;

    // Dynamic GCS address — updated from received packets
    struct sockaddr_in m_gcsAddr;
    bool m_gcsKnown = false;

    // True after we proactively sent params to the GCS (one-shot)
    bool m_paramsSentToGcs = false;

    // MAVLink parser state
    mavlink_message_t m_rxMsg;
    mavlink_status_t  m_rxStatus;

    // MAVLink system/component ID
    static constexpr uint8_t SYS_ID  = 1;
    static constexpr uint8_t COMP_ID = 1;
};

}  // namespace Ap

#endif
