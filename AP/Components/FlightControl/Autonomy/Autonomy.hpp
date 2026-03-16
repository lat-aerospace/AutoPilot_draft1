#ifndef Ap_Autonomy_HPP
#define Ap_Autonomy_HPP

#include "AP/Components/FlightControl/Autonomy/AutonomyComponentAc.hpp"

namespace Ap {

class Autonomy final : public AutonomyComponentBase {
  public:
    Autonomy(const char* const compName);
    ~Autonomy();

  private:
    // Port handlers
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void rcIn_handler(FwIndexType portNum, Ap::RcChannels& rc) override;
    void stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;
    void missionWaypointIn_handler(FwIndexType portNum, Ap::MissionWaypoint& wp) override;
    void modeCommandIn_handler(FwIndexType portNum, const Ap::FlightMode& mode) override;

    // Latest inputs
    Ap::RcChannels    m_rc;
    Ap::AircraftState m_state;
    Ap::FlightMode    m_mode = Ap::FlightMode::FBWB;

    // FBWB guidance state (integrated from sticks)
    double m_desAlt     = 2600.0;  // MSL (m) — match initial condition
    double m_desSpeed   = 50.0;    // m/s — match initial condition
    double m_desHeading = 0.0;     // deg — match initial condition

    // FBWB stick rates
    static constexpr double ALT_RATE     = 5.0;    // m/s per full stick
    static constexpr double HEADING_RATE = 30.0;    // deg/s per full stick
    static constexpr double SPEED_MIN    = 20.0;    // m/s
    static constexpr double SPEED_MAX    = 80.0;    // m/s
    static constexpr double ALT_MIN      = 1650.0;  // m MSL (50m AGL)
    static constexpr double ALT_MAX      = 4000.0;  // m MSL

    // AUTO mode — waypoint storage
    static constexpr U16 MAX_WAYPOINTS = 64;
    static constexpr double WP_ACCEPT_RADIUS = 100.0;  // metres

    Ap::MissionWaypoint m_waypoints[MAX_WAYPOINTS];
    U16 m_wpCount   = 0;
    U16 m_wpCurrent = 0;
};

}  // namespace Ap

#endif
