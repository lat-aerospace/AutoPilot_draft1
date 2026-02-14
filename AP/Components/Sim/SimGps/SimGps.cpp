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

    // --- Delay buffer: read oldest, write current ---
    // Use the delayed state for output (simulates GPS processing lag)
    const Ap::AircraftState& delayed = m_bufferFull
        ? m_delayBuf[m_delayIdx]
        : m_truth;  // before buffer fills, use current

    // Store current truth into ring buffer
    m_delayBuf[m_delayIdx] = m_truth;
    m_delayIdx = (m_delayIdx + 1) % GPS_DELAY_SAMPLES;
    if (!m_bufferFull && m_delayIdx == 0) {
        m_bufferFull = true;
    }

    // --- Read from delayed state ---
    const auto pos = delayed.get_position_ned();
    const auto vel = delayed.get_velocity_ned();

    // Add position noise
    double nN = pos.get_x() + POS_NOISE_SIGMA * m_noise(m_rng);
    double nE = pos.get_y() + POS_NOISE_SIGMA * m_noise(m_rng);
    double nD = pos.get_z() + POS_NOISE_SIGMA * m_noise(m_rng);

    // Convert noisy NED to LLA
    constexpr double REF_LAT = 35.0 * Ap::DEG2RAD;
    constexpr double REF_LON = -106.0 * Ap::DEG2RAD;
    constexpr double REF_ALT = 1600.0;
    Ap::Vec3d refLla(REF_LAT, REF_LON, REF_ALT);
    Ap::Vec3d ned(nN, nE, nD);
    Ap::Vec3d lla = Ap::llaFromNed(refLla, ned);

    // Add velocity noise
    double vN = vel.get_x() + VEL_NOISE_SIGMA * m_noise(m_rng);
    double vE = vel.get_y() + VEL_NOISE_SIGMA * m_noise(m_rng);
    double vD = vel.get_z() + VEL_NOISE_SIGMA * m_noise(m_rng);

    Ap::GpsData gps;
    gps.set_lat_deg(lla.x() * Ap::RAD2DEG);
    gps.set_lon_deg(lla.y() * Ap::RAD2DEG);
    gps.set_alt_msl_m(lla.z());
    gps.set_vel_ned_mps(Ap::Vec3(vN, vE, vD));
    gps.set_time_s(delayed.get_time_s());

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
