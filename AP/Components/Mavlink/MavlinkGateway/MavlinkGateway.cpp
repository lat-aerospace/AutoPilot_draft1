#include "AP/Components/Mavlink/MavlinkGateway/MavlinkGateway.hpp"
#include "AP/Math/ApMath.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstring>

namespace Ap {

MavlinkGateway::MavlinkGateway(const char* const compName)
    : MavlinkGatewayComponentBase(compName)
{
    memset(&m_targetAddr, 0, sizeof(m_targetAddr));
    memset(&m_rxMsg, 0, sizeof(m_rxMsg));
    memset(&m_rxStatus, 0, sizeof(m_rxStatus));
}

MavlinkGateway::~MavlinkGateway() {
    if (m_sockFd >= 0) {
        ::close(m_sockFd);
    }
}

// -------------------------------------------------------------------------
// configure — open UDP socket, set target address, bind for recv
// -------------------------------------------------------------------------
void MavlinkGateway::configure(const char* targetIp, U16 targetPort, U16 bindPort) {
    m_sockFd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockFd < 0) {
        this->log_WARNING_HI_MavlinkSocketFailed(errno);
        return;
    }

    // Set non-blocking so recvfrom doesn't block the rate group
    int flags = fcntl(m_sockFd, F_GETFL, 0);
    fcntl(m_sockFd, F_SETFL, flags | O_NONBLOCK);

    // Bind to receive from GCS
    struct sockaddr_in bindAddr;
    memset(&bindAddr, 0, sizeof(bindAddr));
    bindAddr.sin_family      = AF_INET;
    bindAddr.sin_port        = htons(bindPort);
    bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(m_sockFd, reinterpret_cast<struct sockaddr*>(&bindAddr), sizeof(bindAddr)) < 0) {
        this->log_WARNING_HI_MavlinkSocketFailed(errno);
        ::close(m_sockFd);
        m_sockFd = -1;
        return;
    }

    // Target address for sending telemetry
    m_targetAddr.sin_family = AF_INET;
    m_targetAddr.sin_port   = htons(targetPort);
    inet_pton(AF_INET, targetIp, &m_targetAddr.sin_addr);

    this->log_ACTIVITY_HI_MavlinkSocketOpened();
}

// -------------------------------------------------------------------------
// schedIn — called at 10Hz from RG4
// -------------------------------------------------------------------------
void MavlinkGateway::schedIn_handler(FwIndexType portNum, U32 context) {
    if (m_sockFd < 0) return;

    m_tickCount++;

    // Receive incoming MAVLink messages (non-blocking, drain all available)
    recvMavlink();

    // Heartbeat at 1Hz (every 10th tick)
    if (m_tickCount % 10 == 0) {
        sendHeartbeat();
    }

    // Telemetry at 10Hz (every tick)
    sendTelemetry();

    this->tlmWrite_mavMsgsSent(m_msgsSent);
    this->tlmWrite_mavMsgsRecvd(m_msgsRecvd);
    this->tlmWrite_lastRecvMsgId(m_lastMsgId);
}

// -------------------------------------------------------------------------
// stateIn — async handler, stores latest estimated state
// -------------------------------------------------------------------------
void MavlinkGateway::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state = state;
}

// -------------------------------------------------------------------------
// modeIn — async handler, stores current flight mode
// -------------------------------------------------------------------------
void MavlinkGateway::modeIn_handler(FwIndexType portNum, const Ap::FlightMode& mode) {
    m_mode = mode;
}

// -------------------------------------------------------------------------
// recvMavlink — non-blocking recv, parse all available MAVLink messages
// -------------------------------------------------------------------------
void MavlinkGateway::recvMavlink() {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    while (true) {
        ssize_t n = ::recvfrom(m_sockFd, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n <= 0) break;  // EAGAIN/EWOULDBLOCK or error — no more data

        // Feed each byte to the MAVLink parser
        for (ssize_t i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &m_rxMsg, &m_rxStatus)) {
                handleMessage(m_rxMsg);
                m_msgsRecvd++;
            }
        }
    }
}

// -------------------------------------------------------------------------
// handleMessage — process a decoded MAVLink message
// -------------------------------------------------------------------------
void MavlinkGateway::handleMessage(const mavlink_message_t& msg) {
    m_lastMsgId = msg.msgid;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            mavlink_manual_control_t mc;
            mavlink_msg_manual_control_decode(&msg, &mc);

            // MANUAL_CONTROL: x=pitch, y=roll, z=throttle, r=yaw
            // Range: -1000 to +1000 (z: 0 to 1000 for throttle)
            Ap::RcChannels rc;
            rc.set_roll(static_cast<F32>(mc.y) / 1000.0f);
            rc.set_pitch(static_cast<F32>(mc.x) / 1000.0f);
            rc.set_throttle(static_cast<F32>(mc.z) / 1000.0f);
            rc.set_yaw(static_cast<F32>(mc.r) / 1000.0f);

            if (this->isConnected_rcOut_OutputPort(0)) {
                this->rcOut_out(0, rc);
            }
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            mavlink_rc_channels_override_t rc_ov;
            mavlink_msg_rc_channels_override_decode(&msg, &rc_ov);

            // RC_CHANNELS_OVERRIDE: chan1=roll, chan2=pitch, chan3=throttle, chan4=yaw
            // Range: 1000-2000 (1500 = center)
            Ap::RcChannels rc;
            rc.set_roll(static_cast<F32>(rc_ov.chan1_raw - 1500) / 500.0f);
            rc.set_pitch(static_cast<F32>(rc_ov.chan2_raw - 1500) / 500.0f);
            rc.set_throttle(static_cast<F32>(rc_ov.chan3_raw - 1000) / 1000.0f);
            rc.set_yaw(static_cast<F32>(rc_ov.chan4_raw - 1500) / 500.0f);

            if (this->isConnected_rcOut_OutputPort(0)) {
                this->rcOut_out(0, rc);
            }
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);

            // ACK every command as accepted (keeps MissionPlanner happy)
            mavlink_message_t ack;
            mavlink_msg_command_ack_pack(
                SYS_ID, COMP_ID, &ack,
                cmd.command,
                MAV_RESULT_ACCEPTED,
                0, 0,
                msg.sysid, msg.compid
            );
            sendMavlinkMsg(ack);
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            // GCS heartbeat — ignore
            break;
        }
        default:
            break;
    }
}

// -------------------------------------------------------------------------
// sendMavlinkMsg — serialize and send one MAVLink message over UDP
// -------------------------------------------------------------------------
void MavlinkGateway::sendMavlinkMsg(const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    ::sendto(m_sockFd, buf, len, 0,
             reinterpret_cast<const struct sockaddr*>(&m_targetAddr),
             sizeof(m_targetAddr));
    m_msgsSent++;
}

// -------------------------------------------------------------------------
// sendHeartbeat — tells GCS "I'm an autopilot and I'm alive"
// -------------------------------------------------------------------------
void MavlinkGateway::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        SYS_ID, COMP_ID, &msg,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        static_cast<uint32_t>(m_mode),  // custom_mode = FBWB(0) or AUTO(1)
        MAV_STATE_ACTIVE
    );
    sendMavlinkMsg(msg);
}

// -------------------------------------------------------------------------
// sendTelemetry — pack and send attitude, position, HUD data
// -------------------------------------------------------------------------
void MavlinkGateway::sendTelemetry() {
    const auto euler = m_state.get_euler_deg();
    const auto rate  = m_state.get_angular_rate_dps();
    const auto pos   = m_state.get_position_ned();
    const auto vel   = m_state.get_velocity_ned();

    // --- ATTITUDE ---
    {
        mavlink_message_t msg;
        mavlink_msg_attitude_pack(
            SYS_ID, COMP_ID, &msg,
            static_cast<uint32_t>(m_state.get_time_s() * 1000.0),
            euler.get_x() * Ap::DEG2RAD,
            euler.get_y() * Ap::DEG2RAD,
            euler.get_z() * Ap::DEG2RAD,
            rate.get_x() * Ap::DEG2RAD,
            rate.get_y() * Ap::DEG2RAD,
            rate.get_z() * Ap::DEG2RAD
        );
        sendMavlinkMsg(msg);
    }

    // --- GLOBAL_POSITION_INT ---
    {
        constexpr double REF_LAT = 35.0;
        constexpr double REF_LON = -106.0;
        constexpr double REF_ALT = 1600.0;
        constexpr double R_EARTH = 6378137.0;

        double lat = REF_LAT + (pos.get_x() / R_EARTH) * Ap::RAD2DEG;
        double lon = REF_LON + (pos.get_y() / (R_EARTH * std::cos(REF_LAT * Ap::DEG2RAD))) * Ap::RAD2DEG;
        double altMsl = REF_ALT - pos.get_z();

        mavlink_message_t msg;
        mavlink_msg_global_position_int_pack(
            SYS_ID, COMP_ID, &msg,
            static_cast<uint32_t>(m_state.get_time_s() * 1000.0),
            static_cast<int32_t>(lat * 1e7),
            static_cast<int32_t>(lon * 1e7),
            static_cast<int32_t>(altMsl * 1000),
            static_cast<int32_t>(altMsl * 1000),
            static_cast<int16_t>(vel.get_x() * 100),
            static_cast<int16_t>(vel.get_y() * 100),
            static_cast<int16_t>(vel.get_z() * 100),
            static_cast<uint16_t>(euler.get_z() * 100)
        );
        sendMavlinkMsg(msg);
    }

    // --- VFR_HUD ---
    {
        constexpr double REF_ALT = 1600.0;
        double altMsl = REF_ALT - pos.get_z();

        mavlink_message_t msg;
        mavlink_msg_vfr_hud_pack(
            SYS_ID, COMP_ID, &msg,
            static_cast<float>(m_state.get_airspeed_ms()),
            static_cast<float>(m_state.get_airspeed_ms()),
            static_cast<int16_t>(euler.get_z()),
            0,
            static_cast<float>(altMsl),
            static_cast<float>(-vel.get_z())
        );
        sendMavlinkMsg(msg);
    }

    // --- SYS_STATUS ---
    {
        mavlink_message_t msg;
        mavlink_msg_sys_status_pack(
            SYS_ID, COMP_ID, &msg,
            0, 0, 0, 500, 12000, -1,
            0, 0, 0, 0, 0, 0,
            75, 0, 0, 0
        );
        sendMavlinkMsg(msg);
    }
}

}  // namespace Ap
