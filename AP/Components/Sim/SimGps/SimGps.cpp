#include "AP/Components/Sim/SimGps/SimGps.hpp"
#include "AP/Math/ApMath.hpp"
#include "AP/Math/CoordTransforms.hpp"

namespace Ap {

SimGps::SimGps(const char* const compName)
    : SimGpsComponentBase(compName)
{}

SimGps::~SimGps() {}

// -------------------------------------------------------------------------
// schedIn — called at 10Hz (RG4)
// -------------------------------------------------------------------------
void SimGps::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    const auto pos = m_truth.get_position_ned();
    const auto vel = m_truth.get_velocity_ned();

    // Add noise to NED position
    double nN = pos.get_x() + m_posNoise(m_rng);
    double nE = pos.get_y() + m_posNoise(m_rng);
    double nD = pos.get_z() + m_posNoise(m_rng);

    // Convert noisy NED to LLA (reference: origin at 0,0,0 in rad,rad,m)
    // Using a fixed reference point (e.g. mid-US)
    constexpr double REF_LAT = 35.0 * Ap::DEG2RAD;  // ~35 deg N
    constexpr double REF_LON = -106.0 * Ap::DEG2RAD; // ~106 deg W
    constexpr double REF_ALT = 1600.0;               // ~1600 m MSL
    Ap::Vec3d refLla(REF_LAT, REF_LON, REF_ALT);
    Ap::Vec3d ned(nN, nE, nD);
    Ap::Vec3d lla = Ap::llaFromNed(refLla, ned);

    // Noisy velocity
    double vN = vel.get_x() + m_velNoise(m_rng);
    double vE = vel.get_y() + m_velNoise(m_rng);
    double vD = vel.get_z() + m_velNoise(m_rng);

    Ap::GpsData gps;
    gps.set_lat_deg(lla.x() * Ap::RAD2DEG);
    gps.set_lon_deg(lla.y() * Ap::RAD2DEG);
    gps.set_alt_msl_m(lla.z());
    gps.set_vel_ned_mps(Ap::Vec3(vN, vE, vD));
    gps.set_time_s(m_truth.get_time_s());

    if (this->isConnected_gpsOut_OutputPort(0)) {
        this->gpsOut_out(0, gps);
    }
}

// -------------------------------------------------------------------------
// truthStateIn — async handler
// -------------------------------------------------------------------------
void SimGps::truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_truth = state;
}

}  // namespace Ap
