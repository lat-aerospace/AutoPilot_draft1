#ifndef Ap_Autonomy_HPP
#define Ap_Autonomy_HPP

#include "AP/Components/FlightControl/Autonomy/AutonomyComponentAc.hpp"
#include "AP_Mission/AP_Mission.hpp"
#include "AP_Math/AP_Math.hpp"
#include <cmath>

namespace Ap {

class Autonomy final : public AutonomyComponentBase {
  public:
    Autonomy(const char* const compName);
    ~Autonomy();

  private:
    // ---------------------------------------------------------------
    // Port handlers
    // ---------------------------------------------------------------
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void rcIn_handler(FwIndexType portNum, Ap::RcChannels& rc) override;
    void stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;

    // ---------------------------------------------------------------
    // Command handlers
    // ---------------------------------------------------------------
    void AddWaypoint_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                F64 lat, F64 lon, F32 alt, F32 speed) override;
    void ClearMission_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    // ---------------------------------------------------------------
    // Latest inputs
    // ---------------------------------------------------------------
    Ap::RcChannels    m_rc;
    Ap::AircraftState m_state;
    Ap::FlightMode    m_mode = Ap::FlightMode::FBWB;

    // ---------------------------------------------------------------
    // FBWB guidance state (integrated from sticks)
    // ---------------------------------------------------------------
    double m_desAlt     = INIT_ALT_M;
    double m_desSpeed   = INIT_SPEED_MS;
    double m_desHeading = 0.0;     // deg

    // Initial guidance setpoints — must match sim default reset()
    // RigidBody6DOF::REF_ALT_MSL (1600 m) + default AGL (1000 m) = 2600 m
    static constexpr double INIT_ALT_M    = 2600.0;  // m MSL
    static constexpr double INIT_SPEED_MS = 50.0;    // m/s (= AircraftParams::V_trim)

    // FBWB stick rates
    static constexpr double ALT_RATE     = 5.0;    // m/s per full stick
    static constexpr double HEADING_RATE = 30.0;   // deg/s per full stick
    static constexpr double SPEED_MIN    = 20.0;   // m/s
    static constexpr double SPEED_MAX    = 80.0;   // m/s
    static constexpr double ALT_MIN      = 1650.0; // m MSL (50m AGL)
    static constexpr double ALT_MAX      = 4000.0; // m MSL

    // ---------------------------------------------------------------
    // AUTO mode: mission sequencer
    // ---------------------------------------------------------------
    Ap::AP_Mission m_mission;

    // Waypoint acceptance radius (m) — advance to next when within this distance
    static constexpr double WP_RADIUS = 50.0;

    // NED reference for LLA → NED conversion
    static constexpr double REF_LAT_DEG = 35.0;
    static constexpr double REF_LON_DEG = -106.0;
    static constexpr double REF_ALT_M   = 1600.0;
    static constexpr double R_EARTH     = 6378137.0;
};

}  // namespace Ap

#endif
