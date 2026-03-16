#include "AP/Components/FlightControl/Autonomy/Autonomy.hpp"
#include "AP/Math/ApMath.hpp"
#include <cmath>
#include <cstring>

namespace Ap {

Autonomy::Autonomy(const char* const compName)
    : AutonomyComponentBase(compName)
{
    memset(m_waypoints, 0, sizeof(m_waypoints));
}

Autonomy::~Autonomy() {}

// -------------------------------------------------------------------------
// schedIn — called at 10Hz (RG4)
// -------------------------------------------------------------------------
void Autonomy::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    constexpr double DT = 0.1;  // 10Hz

    if (m_mode == Ap::FlightMode::FBWB) {
        // --- FBWB mode: sticks → integrated guidance command ---

        // Altitude: integrate pitch stick
        m_desAlt += static_cast<double>(m_rc.get_pitch()) * ALT_RATE * DT;
        if (m_desAlt < ALT_MIN) m_desAlt = ALT_MIN;
        if (m_desAlt > ALT_MAX) m_desAlt = ALT_MAX;

        // Heading: integrate roll stick
        m_desHeading += static_cast<double>(m_rc.get_roll()) * HEADING_RATE * DT;
        // Wrap to [0, 360)
        if (m_desHeading >= 360.0) m_desHeading -= 360.0;
        if (m_desHeading < 0.0)    m_desHeading += 360.0;

        // Airspeed: throttle maps directly
        double thr = static_cast<double>(m_rc.get_throttle());
        m_desSpeed = SPEED_MIN + thr * (SPEED_MAX - SPEED_MIN);

    } else if (m_mode == Ap::FlightMode::AUTO) {
        // --- AUTO mode: waypoint navigation ---

        if (m_wpCount == 0 || m_wpCurrent >= m_wpCount) {
            // No waypoints loaded or past final — hold current guidance (loiter)
        } else {
            const auto& wp = m_waypoints[m_wpCurrent];

            // Convert target WP lat/lon to NED using centralized reference datum
            double dLat = (wp.get_lat_deg() - Ap::REF_LAT_DEG) * Ap::DEG2RAD;
            double dLon = (wp.get_lon_deg() - Ap::REF_LON_DEG) * Ap::DEG2RAD;
            double wpN = dLat * Ap::R_EARTH;
            double wpE = dLon * Ap::R_EARTH * std::cos(Ap::REF_LAT_RAD);

            // Current aircraft NED position
            double posN = m_state.get_position_ned().get_x();
            double posE = m_state.get_position_ned().get_y();

            // Vector from aircraft to waypoint
            double dN = wpN - posN;
            double dE = wpE - posE;
            double dist = std::sqrt(dN * dN + dE * dE);

            // Desired bearing to waypoint
            double bearing = std::atan2(dE, dN) * Ap::RAD2DEG;
            if (bearing < 0.0) bearing += 360.0;

            m_desHeading = bearing;
            m_desAlt = static_cast<double>(wp.get_alt_msl_m());
            m_desSpeed = static_cast<double>(wp.get_speed_ms());

            // Waypoint advance: when within acceptance radius
            if (dist < WP_ACCEPT_RADIUS) {
                if (m_wpCurrent + 1 < m_wpCount) {
                    m_wpCurrent++;
                }
                // At final WP: hold position (loiter toward the waypoint)
            }

            // Telemetry for AUTO-specific data
            this->tlmWrite_wpDist(dist);
        }

        this->tlmWrite_wpCurrent(m_wpCurrent);
        this->tlmWrite_wpCount(m_wpCount);
    }

    // --- Output guidance command ---
    Ap::GuidanceCmd cmd;
    cmd.set_desired_alt_m(m_desAlt);
    cmd.set_desired_airspeed_ms(m_desSpeed);
    cmd.set_desired_heading_deg(m_desHeading);

    if (this->isConnected_guidanceCmdOut_OutputPort(0)) {
        this->guidanceCmdOut_out(0, cmd);
    }

    // Broadcast current mode
    for (FwIndexType i = 0; i < 1; i++) {
        if (this->isConnected_modeOut_OutputPort(i)) {
            this->modeOut_out(i, m_mode);
        }
    }

    // Telemetry
    this->tlmWrite_currentMode(m_mode);
    this->tlmWrite_desAlt(m_desAlt);
    this->tlmWrite_desAirspeed(m_desSpeed);
    this->tlmWrite_desHeading(m_desHeading);
}

// -------------------------------------------------------------------------
// Async input handlers — store latest values
// -------------------------------------------------------------------------
void Autonomy::rcIn_handler(FwIndexType portNum, Ap::RcChannels& rc) {
    m_rc = rc;
}

void Autonomy::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state = state;
}

void Autonomy::missionWaypointIn_handler(FwIndexType portNum, Ap::MissionWaypoint& wp) {
    U16 seq = wp.get_seq();
    if (seq < MAX_WAYPOINTS) {
        m_waypoints[seq] = wp;
        // Track highest waypoint index to determine count
        if (seq + 1 > m_wpCount) {
            m_wpCount = seq + 1;
        }
    }
}

void Autonomy::modeCommandIn_handler(FwIndexType portNum, const Ap::FlightMode& mode) {
    if (mode == m_mode) return;

    if (mode == Ap::FlightMode::AUTO) {
        // Entering AUTO: start from first waypoint
        m_wpCurrent = 0;
    } else if (mode == Ap::FlightMode::FBWB) {
        // Switching to FBWB: snapshot current guidance to avoid discontinuity
        // m_desAlt, m_desSpeed, m_desHeading already hold the last guidance values
    }

    m_mode = mode;
}

}  // namespace Ap
