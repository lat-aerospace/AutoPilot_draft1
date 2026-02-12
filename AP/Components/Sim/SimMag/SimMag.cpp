#include "AP/Components/Sim/SimMag/SimMag.hpp"
#include "AP/Math/ApMath.hpp"
#include "AP/Math/CoordTransforms.hpp"

namespace Ap {

SimMag::SimMag(const char* const compName)
    : SimMagComponentBase(compName)
{}

SimMag::~SimMag() {}

// -------------------------------------------------------------------------
// schedIn — called at 100Hz (RG2)
// -------------------------------------------------------------------------
void SimMag::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    const auto quat = m_truth.get_attitude_quat();

    // Earth's magnetic field in NED (simplified, ~Albuquerque NM)
    // Roughly: 0.23 gauss North, 0.05 gauss East, 0.40 gauss Down
    constexpr double MAG_N = 0.23;
    constexpr double MAG_E = 0.05;
    constexpr double MAG_D = 0.40;

    // Rotate NED magnetic field to body frame using quaternion inverse
    Ap::Quatd q(quat.get_w(), quat.get_x(), quat.get_y(), quat.get_z());
    q.normalize();
    Ap::Vec3d magNED(MAG_N, MAG_E, MAG_D);
    Ap::Vec3d magBody = q.inverse() * magNED;

    // Add noise
    double mx = magBody.x() + m_magNoise(m_rng);
    double my = magBody.y() + m_magNoise(m_rng);
    double mz = magBody.z() + m_magNoise(m_rng);

    Ap::MagData mag;
    mag.set_field_gauss(Ap::Vec3(mx, my, mz));
    mag.set_time_s(m_truth.get_time_s());

    if (this->isConnected_magOut_OutputPort(0)) {
        this->magOut_out(0, mag);
    }
}

// -------------------------------------------------------------------------
// truthStateIn — async handler
// -------------------------------------------------------------------------
void SimMag::truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_truth = state;
}

}  // namespace Ap
