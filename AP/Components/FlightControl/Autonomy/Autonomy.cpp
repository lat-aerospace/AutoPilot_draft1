#include "AP/Components/FlightControl/Autonomy/Autonomy.hpp"
#include <cmath>

namespace Ap {

// C++14: out-of-class definitions for ODR-used static constexpr members
constexpr double Autonomy::ALT_RATE;
constexpr double Autonomy::HEADING_RATE;
constexpr double Autonomy::SPEED_MIN;
constexpr double Autonomy::SPEED_MAX;
constexpr double Autonomy::ALT_MIN;
constexpr double Autonomy::ALT_MAX;
constexpr double Autonomy::WP_RADIUS;
constexpr double Autonomy::REF_LAT_DEG;
constexpr double Autonomy::REF_LON_DEG;
constexpr double Autonomy::REF_ALT_M;
constexpr double Autonomy::R_EARTH;

Autonomy::Autonomy(const char* const compName)
    : AutonomyComponentBase(compName)
{
    // Default RC throttle to 0.5 → desSpeed = 50 m/s
    m_rc.set_throttle(0.5f);
}

Autonomy::~Autonomy() {}

// -------------------------------------------------------------------------
// schedIn — called at 10Hz (RG4)
// -------------------------------------------------------------------------
void Autonomy::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    constexpr double DT = 0.1;  // 10Hz

    if (m_mode == Ap::FlightMode::FBWB) {
        // -------------------------------------------------------------------
        // FBWB: integrate RC sticks to produce guidance setpoints
        // -------------------------------------------------------------------

        // Pitch stick → altitude rate
        m_desAlt += static_cast<double>(m_rc.get_pitch()) * ALT_RATE * DT;
        m_desAlt  = std::max(ALT_MIN, std::min(ALT_MAX, m_desAlt));

        // Roll stick → heading rate
        m_desHeading += static_cast<double>(m_rc.get_roll()) * HEADING_RATE * DT;
        if (m_desHeading >= 360.0) m_desHeading -= 360.0;
        if (m_desHeading <    0.0) m_desHeading += 360.0;

        // Throttle → airspeed
        const double thr = static_cast<double>(m_rc.get_throttle());
        m_desSpeed = SPEED_MIN + thr * (SPEED_MAX - SPEED_MIN);

    } else if (m_mode == Ap::FlightMode::AUTO) {
        // -------------------------------------------------------------------
        // AUTO: follow mission waypoints via AP_Mission
        // -------------------------------------------------------------------
        if (m_mission.has_active_mission()) {
            const Ap::Waypoint* wp = m_mission.get_current();
            if (wp != nullptr) {
                // Set altitude and speed setpoints from waypoint
                m_desAlt   = static_cast<double>(wp->alt_msl_m);
                m_desSpeed = static_cast<double>(wp->speed_ms);

                // Compute bearing from current position to waypoint (NED)
                const Vec3d ref_lla(REF_LAT_DEG * DEG2RAD,
                                    REF_LON_DEG * DEG2RAD,
                                    REF_ALT_M);
                const Vec3d wp_ned = m_mission.get_current_ned(ref_lla);
                const double posN  = m_state.get_position_ned().get_x();
                const double posE  = m_state.get_position_ned().get_y();
                const double dN    = wp_ned.x() - posN;
                const double dE    = wp_ned.y() - posE;
                m_desHeading = std::atan2(dE, dN) * RAD2DEG;
                if (m_desHeading < 0.0) m_desHeading += 360.0;

                // Check waypoint acceptance radius — advance sequencer
                const double dist2D = std::sqrt(dN * dN + dE * dE);
                if (dist2D < WP_RADIUS) {
                    if (!m_mission.advance_to_next()) {
                        // Mission complete: return to FBWB
                        m_mode = Ap::FlightMode::FBWB;
                    }
                }
            }
        } else {
            // No active mission: fall back to FBWB
            m_mode = Ap::FlightMode::FBWB;
        }
    }

    // --- Output guidance command ---
    Ap::GuidanceCmd cmd;
    cmd.set_desired_alt_m(m_desAlt);
    cmd.set_desired_airspeed_ms(m_desSpeed);
    cmd.set_desired_heading_deg(m_desHeading);

    if (this->isConnected_guidanceCmdOut_OutputPort(0)) {
        this->guidanceCmdOut_out(0, cmd);
    }
    if (this->isConnected_guidanceCmdOut_OutputPort(1)) {
        this->guidanceCmdOut_out(1, cmd);
    }

    // Broadcast current mode
    if (this->isConnected_modeOut_OutputPort(0)) {
        this->modeOut_out(0, m_mode);
    }

    // Telemetry
    this->tlmWrite_currentMode(m_mode);
    this->tlmWrite_desAlt(m_desAlt);
    this->tlmWrite_desAirspeed(m_desSpeed);
    this->tlmWrite_desHeading(m_desHeading);
    this->tlmWrite_wpCount(m_mission.num_waypoints());
}

// -------------------------------------------------------------------------
// Async input handlers
// -------------------------------------------------------------------------
void Autonomy::rcIn_handler(FwIndexType portNum, Ap::RcChannels& rc) {
    m_rc = rc;
}

void Autonomy::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state = state;
}

// -------------------------------------------------------------------------
// Command handlers — mission management
// -------------------------------------------------------------------------
void Autonomy::AddWaypoint_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                      F64 lat, F64 lon, F32 alt, F32 speed) {
    Ap::Waypoint wp;
    wp.lat_deg   = lat;
    wp.lon_deg   = lon;
    wp.alt_msl_m = alt;
    wp.speed_ms  = speed;
    wp.seq       = m_mission.num_waypoints();

    m_mission.add_waypoint(wp);

    // Switch to AUTO mode when first waypoint arrives
    if (m_mission.num_waypoints() == 1) {
        m_mode = Ap::FlightMode::AUTO;
    }

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void Autonomy::ClearMission_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_mission.clear();
    m_mode = Ap::FlightMode::FBWB;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Ap
