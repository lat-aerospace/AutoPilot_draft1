// ======================================================================
// \title  AP/Hal/Stm32/Stm32Uart/Stm32Uart.cpp
// \brief  STM32 UART-based MAVLink gateway for hardware deployment
//
// Full MAVLink gateway over USART2 / DMA.  Feature parity with the
// SITL MavlinkGateway (UDP) so that QGC and MissionPlanner display
// all processed sensor data and accept joystick / mission uploads.
// ======================================================================
#include <AP/Hal/Stm32/Stm32Uart/Stm32Uart.hpp>
#include <Fw/Types/Assert.hpp>
#include <cstring>
#include <cmath>

// Reference datum constants (must match AP/Math/ApMath.hpp)
// Duplicated here to avoid pulling in Eigen via ApMath.hpp on STM32.
namespace ApRef {
    static constexpr double PI       = 3.14159265358979323846;
    static constexpr double DEG2RAD  = PI / 180.0;
    static constexpr double RAD2DEG  = 180.0 / PI;
    static constexpr double REF_LAT_DEG = 35.0;
    static constexpr double REF_LON_DEG = -106.0;
    static constexpr double REF_ALT_MSL = 1600.0;
    static constexpr double REF_LAT_RAD = REF_LAT_DEG * DEG2RAD;
    static constexpr double R_EARTH     = 6378137.0;
}

// Singleton for ISR callback (only one MAVLink UART)
static Ap::Stm32Uart* s_uart_instance = nullptr;

extern "C" void AP_Uart_IdleCallback(UART_HandleTypeDef* /*huart*/) {
    if (s_uart_instance != nullptr) {
        s_uart_instance->rxIdleCallback();
    }
}

namespace Ap {

// -------------------------------------------------------------------------
// Minimal parameter table — satisfies MissionPlanner's connection handshake
// -------------------------------------------------------------------------
struct ParamEntry {
    char name[16];
    float value;
};

static const ParamEntry PARAM_TABLE[] = {
    {"SYSID_THISMAV",  1.0f},
    {"ARMING_CHECK",   0.0f},
    {"FORMAT_VERSION", 1.0f},
};
static constexpr uint16_t PARAM_COUNT =
    sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

// -------------------------------------------------------------------------
// Construction / configuration
// -------------------------------------------------------------------------
Stm32Uart::Stm32Uart(const char* const compName)
    : Stm32UartComponentBase(compName)
{
    (void)memset(m_rx_dma_buf, 0, sizeof(m_rx_dma_buf));
    (void)memset(&m_mav_msg,    0, sizeof(m_mav_msg));
    (void)memset(&m_mav_status, 0, sizeof(m_mav_status));
    m_state = Ap::AircraftState();
    s_uart_instance = this;
}

Stm32Uart::~Stm32Uart() {
    s_uart_instance = nullptr;
}

void Stm32Uart::configure(UART_HandleTypeDef* huart,
                           uint8_t             sys_id,
                           uint8_t             comp_id) {
    m_huart   = huart;
    m_sys_id  = sys_id;
    m_comp_id = comp_id;
}

// -------------------------------------------------------------------------
// startRx – start circular DMA receive (IDLE interrupt not needed;
//           parseMavlink() polls the DMA counter at 10 Hz)
// -------------------------------------------------------------------------
void Stm32Uart::startRx() {
    FW_ASSERT(m_huart != nullptr);
    HAL_UART_Receive_DMA(m_huart, m_rx_dma_buf,
                         static_cast<uint16_t>(RX_BUF_SIZE));
}

void Stm32Uart::rxIdleCallback() {
    // Unused — parseMavlink() polls the DMA counter.
}

// -------------------------------------------------------------------------
// Low-level TX — polled (blocking).
// DMA TX is unsafe here: (1) local stack buffers go out of scope before
// DMA finishes, and (2) DMA can't access DTCM-RAM on STM32H7.
// At 115200 baud, 50 bytes ≈ 4.3 ms — acceptable for a 10 Hz task.
// -------------------------------------------------------------------------
void Stm32Uart::sendBytes(const uint8_t* data, size_t len) {
    if (m_huart == nullptr || data == nullptr || len == 0) { return; }
    HAL_UART_Transmit(m_huart, const_cast<uint8_t*>(data),
                      static_cast<uint16_t>(len), 50);
}

void Stm32Uart::sendMavlinkMsg(const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendBytes(buf, len);
}

// =========================================================================
// MAVLink receive + dispatch
// =========================================================================
void Stm32Uart::parseMavlink() {
    const uint16_t dma_ndtr =
        static_cast<uint16_t>(__HAL_DMA_GET_COUNTER(m_huart->hdmarx));
    const uint16_t write_pos =
        static_cast<uint16_t>(RX_BUF_SIZE - dma_ndtr);

    while (m_rx_read_pos != write_pos) {
        const uint8_t byte = m_rx_dma_buf[m_rx_read_pos];
        m_rx_read_pos = static_cast<uint16_t>((m_rx_read_pos + 1U) % RX_BUF_SIZE);

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &m_mav_msg, &m_mav_status) == 1) {
            handleMessage(m_mav_msg);
        }
    }
}

void Stm32Uart::handleMessage(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            handleManualControl(msg);
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            handleRcChannelsOverride(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleCommandLong(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            sendAllParams();
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(&msg, &req);
            if (req.param_index >= 0 && req.param_index < PARAM_COUNT) {
                sendParamValue(static_cast<uint16_t>(req.param_index));
            } else {
                sendParamByName(req.param_id);
            }
            break;
        }
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
            mavlink_request_data_stream_t ds;
            mavlink_msg_request_data_stream_decode(&msg, &ds);
            mavlink_message_t resp;
            mavlink_msg_data_stream_pack(m_sys_id, m_comp_id, &resp,
                ds.req_stream_id, ds.req_message_rate, ds.start_stop);
            sendMavlinkMsg(resp);
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (hb.type == MAV_TYPE_GCS && !m_paramsSentToGcs) {
                m_paramsSentToGcs = true;
                sendAllParams();
            }
            break;
        }
        case MAVLINK_MSG_ID_SET_MODE:
            handleSetMode(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            handleMissionCount(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            handleMissionItemInt(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            handleMissionRequestList(msg);
            break;
        default:
            break;
    }
}

// -------------------------------------------------------------------------
// Incoming message handlers
// -------------------------------------------------------------------------
void Stm32Uart::handleManualControl(const mavlink_message_t& msg) {
    mavlink_manual_control_t mc;
    mavlink_msg_manual_control_decode(&msg, &mc);

    Ap::RcChannels rc;
    rc.set_roll(static_cast<F32>(mc.y) / 1000.0F);
    rc.set_pitch(static_cast<F32>(mc.x) / 1000.0F);
    rc.set_throttle(static_cast<F32>(mc.z) / 1000.0F);
    rc.set_yaw(static_cast<F32>(mc.r) / 1000.0F);
    rcOut_out(0, rc);
}

void Stm32Uart::handleRcChannelsOverride(const mavlink_message_t& msg) {
    mavlink_rc_channels_override_t rc_ov;
    mavlink_msg_rc_channels_override_decode(&msg, &rc_ov);

    Ap::RcChannels rc;
    rc.set_roll(static_cast<F32>(rc_ov.chan1_raw - 1500) / 500.0F);
    rc.set_pitch(static_cast<F32>(rc_ov.chan2_raw - 1500) / 500.0F);
    rc.set_throttle(static_cast<F32>(rc_ov.chan3_raw - 1000) / 1000.0F);
    rc.set_yaw(static_cast<F32>(rc_ov.chan4_raw - 1500) / 500.0F);
    rcOut_out(0, rc);
}

void Stm32Uart::handleCommandLong(const mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    // ACK every command
    mavlink_message_t ack;
    mavlink_msg_command_ack_pack(m_sys_id, m_comp_id, &ack,
        cmd.command, MAV_RESULT_ACCEPTED, 0, 0, msg.sysid, msg.compid);
    sendMavlinkMsg(ack);

    // Handle AUTOPILOT_VERSION request
    if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ||
        (cmd.command == MAV_CMD_REQUEST_MESSAGE &&
         static_cast<uint32_t>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION)) {
        uint8_t zeros8[8] = {};
        uint8_t zeros18[18] = {};
        mavlink_message_t ver;
        mavlink_msg_autopilot_version_pack(m_sys_id, m_comp_id, &ver,
            MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT | MAV_PROTOCOL_CAPABILITY_MAVLINK2,
            0, 0, 0, 0, zeros8, zeros8, zeros8, 0, 0, 0, zeros18);
        sendMavlinkMsg(ver);
    }
}

void Stm32Uart::handleSetMode(const mavlink_message_t& msg) {
    mavlink_set_mode_t sm;
    mavlink_msg_set_mode_decode(&msg, &sm);

    Ap::FlightMode newMode = (sm.custom_mode == 1)
                             ? Ap::FlightMode::AUTO
                             : Ap::FlightMode::FBWB;
    if (this->isConnected_modeCommandOut_OutputPort(0)) {
        this->modeCommandOut_out(0, newMode);
    }
}

void Stm32Uart::handleMissionCount(const mavlink_message_t& msg) {
    mavlink_mission_count_t mc;
    mavlink_msg_mission_count_decode(&msg, &mc);
    m_missionCount = mc.count;
    m_missionReceived = 0;

    if (m_missionCount > 0) {
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(m_sys_id, m_comp_id, &req,
            msg.sysid, msg.compid, 0, MAV_MISSION_TYPE_MISSION);
        sendMavlinkMsg(req);
    }
}

void Stm32Uart::handleMissionItemInt(const mavlink_message_t& msg) {
    mavlink_mission_item_int_t mi;
    mavlink_msg_mission_item_int_decode(&msg, &mi);

    Ap::MissionWaypoint wp;
    wp.set_seq(mi.seq);
    wp.set_lat_deg(static_cast<double>(mi.x) * 1e-7);
    wp.set_lon_deg(static_cast<double>(mi.y) * 1e-7);
    wp.set_alt_msl_m(mi.z);
    wp.set_speed_ms(mi.param1 > 0.0f ? mi.param1 : 50.0f);

    if (this->isConnected_missionWaypointOut_OutputPort(0)) {
        this->missionWaypointOut_out(0, wp);
    }
    m_missionReceived++;

    if (m_missionReceived < m_missionCount) {
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(m_sys_id, m_comp_id, &req,
            msg.sysid, msg.compid, m_missionReceived, MAV_MISSION_TYPE_MISSION);
        sendMavlinkMsg(req);
    } else {
        mavlink_message_t ackMsg;
        mavlink_msg_mission_ack_pack(m_sys_id, m_comp_id, &ackMsg,
            msg.sysid, msg.compid, MAV_MISSION_ACCEPTED,
            MAV_MISSION_TYPE_MISSION, 0);
        sendMavlinkMsg(ackMsg);
    }
}

void Stm32Uart::handleMissionRequestList(const mavlink_message_t& msg) {
    mavlink_message_t resp;
    mavlink_msg_mission_count_pack(m_sys_id, m_comp_id, &resp,
        msg.sysid, msg.compid, m_missionCount, MAV_MISSION_TYPE_MISSION, 0);
    sendMavlinkMsg(resp);
}

// -------------------------------------------------------------------------
// Parameter helpers
// -------------------------------------------------------------------------
void Stm32Uart::sendAllParams() {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        sendParamValue(i);
    }
}

void Stm32Uart::sendParamValue(uint16_t index) {
    if (index >= PARAM_COUNT) return;
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(m_sys_id, m_comp_id, &msg,
        PARAM_TABLE[index].name, PARAM_TABLE[index].value,
        MAV_PARAM_TYPE_REAL32, PARAM_COUNT, index);
    sendMavlinkMsg(msg);
}

void Stm32Uart::sendParamByName(const char* name) {
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(PARAM_TABLE[i].name, name, 16) == 0) {
            sendParamValue(i);
            return;
        }
    }
}

// =========================================================================
// MAVLink telemetry senders
// =========================================================================
void Stm32Uart::sendHeartbeat() {
    mavlink_message_t msg;
    uint8_t baseMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                     | MAV_MODE_FLAG_STABILIZE_ENABLED
                     | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                     | MAV_MODE_FLAG_SAFETY_ARMED;

    mavlink_msg_heartbeat_pack(m_sys_id, m_comp_id, &msg,
        MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC,
        baseMode, static_cast<uint32_t>(m_mode), MAV_STATE_ACTIVE);
    sendMavlinkMsg(msg);
}

void Stm32Uart::sendAttitude(const Ap::AircraftState& state) {
    mavlink_message_t msg;
    static constexpr float D2R = 3.14159265358979323846F / 180.0F;
    const Ap::Vec3& att   = state.get_euler_deg();
    const Ap::Vec3& rates = state.get_angular_rate_dps();

    mavlink_msg_attitude_pack(m_sys_id, m_comp_id, &msg,
        xTaskGetTickCount(),
        att.get_x() * D2R, att.get_y() * D2R, att.get_z() * D2R,
        rates.get_x() * D2R, rates.get_y() * D2R, rates.get_z() * D2R);
    sendMavlinkMsg(msg);
}

void Stm32Uart::sendGlobalPosition(const Ap::AircraftState& state) {
    mavlink_message_t msg;
    const Ap::Vec3& pos = state.get_position_ned();
    const Ap::Vec3& vel = state.get_velocity_ned();

    // Reverse NED → LLA using the same reference datum as StateEstimator
    double lat = ApRef::REF_LAT_DEG + (pos.get_x() / ApRef::R_EARTH) * ApRef::RAD2DEG;
    double lon = ApRef::REF_LON_DEG + (pos.get_y() / (ApRef::R_EARTH * std::cos(ApRef::REF_LAT_RAD))) * ApRef::RAD2DEG;
    double altMsl = ApRef::REF_ALT_MSL - pos.get_z();  // NED down → altitude up

    mavlink_msg_global_position_int_pack(m_sys_id, m_comp_id, &msg,
        xTaskGetTickCount(),
        static_cast<int32_t>(lat * 1e7),
        static_cast<int32_t>(lon * 1e7),
        static_cast<int32_t>(altMsl * 1000.0),
        static_cast<int32_t>(altMsl * 1000.0),
        static_cast<int16_t>(vel.get_x() * 100.0F),
        static_cast<int16_t>(vel.get_y() * 100.0F),
        static_cast<int16_t>(vel.get_z() * 100.0F),
        static_cast<uint16_t>(state.get_euler_deg().get_z() * 100.0F));
    sendMavlinkMsg(msg);
}

void Stm32Uart::sendVfrHud(const Ap::AircraftState& state) {
    mavlink_message_t msg;
    const Ap::Vec3& vel = state.get_velocity_ned();
    const float alt = static_cast<float>(ApRef::REF_ALT_MSL - state.get_position_ned().get_z());

    mavlink_msg_vfr_hud_pack(m_sys_id, m_comp_id, &msg,
        static_cast<float>(state.get_airspeed_ms()),
        static_cast<float>(state.get_airspeed_ms()),
        static_cast<int16_t>(state.get_euler_deg().get_z()),
        0,
        alt,
        static_cast<float>(-vel.get_z()));
    sendMavlinkMsg(msg);
}

void Stm32Uart::sendSysStatus() {
    mavlink_message_t msg;
    mavlink_msg_sys_status_pack(m_sys_id, m_comp_id, &msg,
        0, 0, 0,    // sensors present/enabled/health
        500,         // load (50%)
        12000,       // battery voltage mV
        -1,          // battery current
        75,          // battery remaining %
        0, 0, 0, 0, 0, 0, 0, 0, 0);
    sendMavlinkMsg(msg);
}

void Stm32Uart::sendNamedFloats(const Ap::AircraftState& state) {
    const Ap::Vec3& euler = state.get_euler_deg();
    const Ap::Vec3& pos   = state.get_position_ned();
    const Ap::Vec3& vel   = state.get_velocity_ned();
    const uint32_t timeMs = xTaskGetTickCount();
    double altMsl = ApRef::REF_ALT_MSL - pos.get_z();

    auto send = [&](const char* name, float value) {
        mavlink_message_t msg;
        mavlink_msg_named_value_float_pack(m_sys_id, m_comp_id, &msg,
                                           timeMs, name, value);
        sendMavlinkMsg(msg);
    };

    send("posN",    static_cast<float>(pos.get_x()));
    send("posE",    static_cast<float>(pos.get_y()));
    send("altMSL",  static_cast<float>(altMsl));
    send("roll",    static_cast<float>(euler.get_x()));
    send("pitch",   static_cast<float>(euler.get_y()));
    send("heading", static_cast<float>(euler.get_z()));
    send("ias",     static_cast<float>(state.get_airspeed_ms()));
    send("velN",    static_cast<float>(vel.get_x()));
    send("velE",    static_cast<float>(vel.get_y()));
    send("velD",    static_cast<float>(vel.get_z()));
    send("mode",    static_cast<float>(m_mode));
}

// =========================================================================
// F' port handlers
// =========================================================================
void Stm32Uart::stateIn_handler(FwIndexType /*portNum*/,
                                  Ap::AircraftState& state) {
    m_state       = state;
    m_state_valid = true;
}

void Stm32Uart::modeIn_handler(FwIndexType /*portNum*/,
                                 const Ap::FlightMode& mode) {
    m_mode = mode;
}

void Stm32Uart::schedIn_handler(FwIndexType /*portNum*/, U32 /*context*/) {
    if (m_huart == nullptr) { return; }

    m_tick_count++;

    // 1. Parse incoming MAVLink messages
    parseMavlink();

    // 2. Heartbeat at 1 Hz (send even without valid state so MP connects)
    if (m_tick_count % HEARTBEAT_TICKS == 0) {
        sendHeartbeat();
    }

    // 3. Send telemetry if we have valid state
    //    Throttled to 2 Hz to avoid overloading the 10 Hz task with
    //    16+ polled-UART transmissions per tick.  At 115200 baud each
    //    ~40-byte MAVLink message takes ~3.5 ms; 16 msgs = ~56 ms,
    //    which risks queue overflow from the 100 Hz stateIn port.
    if (m_state_valid && (m_tick_count % 5 == 0)) {
        sendAttitude(m_state);
        sendGlobalPosition(m_state);
        sendVfrHud(m_state);
        sendSysStatus();
    }
}

}  // namespace Ap
