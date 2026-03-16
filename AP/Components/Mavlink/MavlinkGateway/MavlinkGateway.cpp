#include "AP/Components/Mavlink/MavlinkGateway/MavlinkGateway.hpp"
#include "AP/Math/ApMath.hpp"

#if !defined(TGT_OS_TYPE_FREERTOS)
// POSIX / Linux (SITL builds)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#endif

#include <cmath>
#include <cstring>

namespace Ap {

// -------------------------------------------------------------------------
// Minimal parameter table — satisfies MissionPlanner's connection handshake
// -------------------------------------------------------------------------
struct ParamEntry {
    char name[16];   // MAVLink param name (max 16 chars, null-padded)
    float value;
};

static const ParamEntry PARAM_TABLE[] = {
    {"SYSID_THISMAV",  1.0f},
    {"ARMING_CHECK",   0.0f},
    {"FORMAT_VERSION", 1.0f},
};

static constexpr uint16_t PARAM_COUNT = sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

MavlinkGateway::MavlinkGateway(const char* const compName)
    : MavlinkGatewayComponentBase(compName)
{
    memset(&m_targetAddr, 0, sizeof(m_targetAddr));
    memset(&m_gcsAddr, 0, sizeof(m_gcsAddr));
    memset(&m_rxMsg, 0, sizeof(m_rxMsg));
    memset(&m_rxStatus, 0, sizeof(m_rxStatus));
}

MavlinkGateway::~MavlinkGateway() {
#if !defined(TGT_OS_TYPE_FREERTOS)
    if (m_sockFd >= 0) {
        ::close(m_sockFd);
    }
#endif
}

// -------------------------------------------------------------------------
// configure — open UDP socket, set target address, bind for recv
// -------------------------------------------------------------------------
void MavlinkGateway::configure(const char* targetIp, U16 targetPort, U16 bindPort) {
#if defined(TGT_OS_TYPE_FREERTOS)
    // No POSIX sockets on bare-metal.
    // On STM32 the MAVLink transport is UART-based; this UDP path is
    // only used in SITL.  Mark socket as invalid so schedIn is a no-op.
    (void)targetIp; (void)targetPort; (void)bindPort;
    m_sockFd = -1;
#else
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
#endif
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

    this->tlmWrite_rcRoll(m_rcRoll);
    this->tlmWrite_rcPitch(m_rcPitch);
    this->tlmWrite_rcThrottle(m_rcThrottle);
    this->tlmWrite_rcYaw(m_rcYaw);
    this->tlmWrite_rcMsgCount(m_rcMsgCnt);
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
#if defined(TGT_OS_TYPE_FREERTOS)
    // No POSIX sockets on bare-metal; UART-based receive is handled
    // by a separate STM32-specific component.
#else
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    struct sockaddr_in srcAddr;
    socklen_t srcLen = sizeof(srcAddr);

    while (true) {
        srcLen = sizeof(srcAddr);
        ssize_t n = ::recvfrom(m_sockFd, buf, sizeof(buf), 0,
                               reinterpret_cast<struct sockaddr*>(&srcAddr), &srcLen);
        if (n <= 0) break;  // EAGAIN/EWOULDBLOCK or error — no more data

        // Capture the GCS address from the first non-loopback packet,
        // so responses go back to wherever the GCS actually is.
        if (!m_gcsKnown) {
            m_gcsAddr = srcAddr;
            // Update target to respond to actual GCS address + port 14550
            m_targetAddr.sin_addr = srcAddr.sin_addr;
            m_gcsKnown = true;
        }

        // Feed each byte to the MAVLink parser
        for (ssize_t i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &m_rxMsg, &m_rxStatus)) {
                handleMessage(m_rxMsg);
                m_msgsRecvd++;
            }
        }
    }
#endif
}

// -------------------------------------------------------------------------
// handleMessage — process a decoded MAVLink message
// -------------------------------------------------------------------------
void MavlinkGateway::handleMessage(const mavlink_message_t& msg) {
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

            m_rcRoll     = rc.get_roll();
            m_rcPitch    = rc.get_pitch();
            m_rcThrottle = rc.get_throttle();
            m_rcYaw      = rc.get_yaw();
            m_rcMsgCnt++;

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

            m_rcRoll     = rc.get_roll();
            m_rcPitch    = rc.get_pitch();
            m_rcThrottle = rc.get_throttle();
            m_rcYaw      = rc.get_yaw();
            m_rcMsgCnt++;

            if (this->isConnected_rcOut_OutputPort(0)) {
                this->rcOut_out(0, rc);
            }
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);

            // ACK every command as accepted
            mavlink_message_t ack;
            mavlink_msg_command_ack_pack(
                SYS_ID, COMP_ID, &ack,
                cmd.command,
                MAV_RESULT_ACCEPTED,
                0, 0,
                msg.sysid, msg.compid
            );
            sendMavlinkMsg(ack);

            // Handle specific commands that expect follow-up data
            if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ||
                (cmd.command == MAV_CMD_REQUEST_MESSAGE &&
                 static_cast<uint32_t>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION)) {
                uint8_t zeros8[8] = {};
                uint8_t zeros18[18] = {};
                mavlink_message_t ver;
                mavlink_msg_autopilot_version_pack(
                    SYS_ID, COMP_ID, &ver,
                    MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                    MAV_PROTOCOL_CAPABILITY_MAVLINK2,
                    0, 0, 0, 0,        // flight/middleware/os/board sw version
                    zeros8, zeros8, zeros8,  // custom versions (8-byte arrays)
                    0, 0, 0,           // vendor/product/uid
                    zeros18            // uid2
                );
                sendMavlinkMsg(ver);
            }
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
            // MissionPlanner requests all params during connection handshake.
            // Must respond with PARAM_VALUE for each param or MP won't
            // consider itself connected (and joystick thread won't start).
            sendAllParams();
            this->log_ACTIVITY_HI_ParamRequestReceived(PARAM_COUNT);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(&msg, &req);

            if (req.param_index >= 0 && req.param_index < PARAM_COUNT) {
                sendParamValue(static_cast<uint16_t>(req.param_index));
            } else {
                // Lookup by name
                sendParamByName(req.param_id);
            }
            break;
        }
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
            // MissionPlanner requests data streams — we already send
            // telemetry at 10Hz unconditionally, so just ACK by sending
            // a DATA_STREAM message confirming the rate.
            mavlink_request_data_stream_t ds;
            mavlink_msg_request_data_stream_decode(&msg, &ds);

            mavlink_message_t resp;
            mavlink_msg_data_stream_pack(
                SYS_ID, COMP_ID, &resp,
                ds.req_stream_id,
                ds.req_message_rate,
                ds.start_stop
            );
            sendMavlinkMsg(resp);
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            // Only process GCS heartbeats (type 6 = MAV_TYPE_GCS), not our own echoes
            if (hb.type == MAV_TYPE_GCS && !m_paramsSentToGcs) {
                // Proactively send all params on first GCS heartbeat detection.
                // This handles the case where MissionPlanner's PARAM_REQUEST_LIST
                // was lost or never sent (UDP auto-detect mode).
                m_paramsSentToGcs = true;
                sendAllParams();
                this->log_ACTIVITY_HI_ParamRequestReceived(PARAM_COUNT);
            }
            break;
        }
        case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t sm;
            mavlink_msg_set_mode_decode(&msg, &sm);

            // custom_mode: 0=FBWB, 1=AUTO (matches our FlightMode enum)
            Ap::FlightMode newMode = (sm.custom_mode == 1)
                                     ? Ap::FlightMode::AUTO
                                     : Ap::FlightMode::FBWB;

            if (this->isConnected_modeCommandOut_OutputPort(0)) {
                this->modeCommandOut_out(0, newMode);
            }
            break;
        }
        case MAVLINK_MSG_ID_MISSION_COUNT: {
            mavlink_mission_count_t mc;
            mavlink_msg_mission_count_decode(&msg, &mc);

            m_missionCount = mc.count;
            m_missionReceived = 0;

            // ACK by requesting the first waypoint
            if (m_missionCount > 0) {
                mavlink_message_t req;
                mavlink_msg_mission_request_int_pack(
                    SYS_ID, COMP_ID, &req,
                    msg.sysid, msg.compid,
                    0,  // first waypoint seq
                    MAV_MISSION_TYPE_MISSION
                );
                sendMavlinkMsg(req);
            }
            break;
        }
        case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
            mavlink_mission_item_int_t mi;
            mavlink_msg_mission_item_int_decode(&msg, &mi);

            // Forward waypoint to Autonomy
            Ap::MissionWaypoint wp;
            wp.set_seq(mi.seq);
            wp.set_lat_deg(static_cast<double>(mi.x) * 1e-7);  // degE7 → deg
            wp.set_lon_deg(static_cast<double>(mi.y) * 1e-7);
            wp.set_alt_msl_m(mi.z);
            wp.set_speed_ms(mi.param1 > 0.0f ? mi.param1 : 50.0f);  // default 50 m/s

            if (this->isConnected_missionWaypointOut_OutputPort(0)) {
                this->missionWaypointOut_out(0, wp);
            }

            m_missionReceived++;

            // ACK this item
            mavlink_message_t ack;
            mavlink_msg_mission_ack_pack(
                SYS_ID, COMP_ID, &ack,
                msg.sysid, msg.compid,
                MAV_MISSION_ACCEPTED,
                MAV_MISSION_TYPE_MISSION,
                0  // opaque_id
            );

            if (m_missionReceived < m_missionCount) {
                // Request next waypoint
                mavlink_message_t req;
                mavlink_msg_mission_request_int_pack(
                    SYS_ID, COMP_ID, &req,
                    msg.sysid, msg.compid,
                    m_missionReceived,
                    MAV_MISSION_TYPE_MISSION
                );
                sendMavlinkMsg(req);
            } else {
                // All waypoints received — send final ACK
                sendMavlinkMsg(ack);
            }
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            // GCS asks how many waypoints we have — respond with our count
            mavlink_message_t resp;
            mavlink_msg_mission_count_pack(
                SYS_ID, COMP_ID, &resp,
                msg.sysid, msg.compid,
                m_missionCount,
                MAV_MISSION_TYPE_MISSION,
                0  // opaque_id
            );
            sendMavlinkMsg(resp);
            break;
        }
        default:
            this->log_ACTIVITY_LO_MavMsgReceived(msg.msgid);
            break;
    }
}

// -------------------------------------------------------------------------
// sendMavlinkMsg — serialize and send one MAVLink message over UDP
// -------------------------------------------------------------------------
void MavlinkGateway::sendMavlinkMsg(const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#if defined(TGT_OS_TYPE_FREERTOS)
    // Bare-metal: UART-based send would go here.
    (void)buf; (void)len;
#else
    ::sendto(m_sockFd, buf, len, 0,
             reinterpret_cast<const struct sockaddr*>(&m_targetAddr),
             sizeof(m_targetAddr));
#endif
    m_msgsSent++;
}

// -------------------------------------------------------------------------
// sendHeartbeat — tells GCS "I'm an autopilot and I'm alive"
// -------------------------------------------------------------------------
void MavlinkGateway::sendHeartbeat() {
    mavlink_message_t msg;

    // base_mode flags that tell GCS what this autopilot supports/is doing:
    // - CUSTOM_MODE_ENABLED: custom_mode field is meaningful
    // - STABILIZE_ENABLED: autopilot is stabilizing
    // - MANUAL_INPUT_ENABLED: RC/joystick input accepted
    // - SAFETY_ARMED: vehicle is armed (required by some GCS for RC override)
    uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                     | MAV_MODE_FLAG_STABILIZE_ENABLED
                     | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                     | MAV_MODE_FLAG_SAFETY_ARMED;

    mavlink_msg_heartbeat_pack(
        SYS_ID, COMP_ID, &msg,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_GENERIC,
        baseMode,
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
        double lat = Ap::REF_LAT_DEG + (pos.get_x() / Ap::R_EARTH) * Ap::RAD2DEG;
        double lon = Ap::REF_LON_DEG + (pos.get_y() / (Ap::R_EARTH * std::cos(Ap::REF_LAT_RAD))) * Ap::RAD2DEG;
        double altMsl = Ap::REF_ALT_MSL - pos.get_z();

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
        double altMsl = Ap::REF_ALT_MSL - pos.get_z();

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

    // --- NAMED_VALUE_FLOAT: internal debug variables ---
    // These appear in Mission Planner's Status tab
    {
        uint32_t timeBootMs = static_cast<uint32_t>(m_state.get_time_s() * 1000.0);
        double altMsl = Ap::REF_ALT_MSL - pos.get_z();

        auto sendNamedFloat = [&](const char* name, float value) {
            mavlink_message_t msg;
            mavlink_msg_named_value_float_pack(
                SYS_ID, COMP_ID, &msg,
                timeBootMs, name, value
            );
            sendMavlinkMsg(msg);
        };

        sendNamedFloat("posN",    static_cast<float>(pos.get_x()));
        sendNamedFloat("posE",    static_cast<float>(pos.get_y()));
        sendNamedFloat("altMSL",  static_cast<float>(altMsl));
        sendNamedFloat("roll",    static_cast<float>(euler.get_x()));
        sendNamedFloat("pitch",   static_cast<float>(euler.get_y()));
        sendNamedFloat("heading", static_cast<float>(euler.get_z()));
        sendNamedFloat("ias",     static_cast<float>(m_state.get_airspeed_ms()));
        sendNamedFloat("velN",    static_cast<float>(vel.get_x()));
        sendNamedFloat("velE",    static_cast<float>(vel.get_y()));
        sendNamedFloat("velD",    static_cast<float>(vel.get_z()));
        sendNamedFloat("mode",    static_cast<float>(m_mode));
        sendNamedFloat("rcRoll",  m_rcRoll);
        sendNamedFloat("rcPitch", m_rcPitch);
        sendNamedFloat("rcThr",   m_rcThrottle);
    }
}

// -------------------------------------------------------------------------
// sendAllParams — respond to PARAM_REQUEST_LIST with every parameter
// -------------------------------------------------------------------------
void MavlinkGateway::sendAllParams() {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        sendParamValue(i);
    }
}

// -------------------------------------------------------------------------
// sendParamValue — send a single PARAM_VALUE by index
// -------------------------------------------------------------------------
void MavlinkGateway::sendParamValue(uint16_t index) {
    if (index >= PARAM_COUNT) return;

    mavlink_message_t msg;
    mavlink_msg_param_value_pack(
        SYS_ID, COMP_ID, &msg,
        PARAM_TABLE[index].name,
        PARAM_TABLE[index].value,
        MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT,
        index
    );
    sendMavlinkMsg(msg);
}

// -------------------------------------------------------------------------
// sendParamByName — send a single PARAM_VALUE by name lookup
// -------------------------------------------------------------------------
void MavlinkGateway::sendParamByName(const char* name) {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(PARAM_TABLE[i].name, name, 16) == 0) {
            sendParamValue(i);
            return;
        }
    }
}

}  // namespace Ap
