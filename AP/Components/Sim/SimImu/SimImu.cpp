#include "AP/Components/Sim/SimImu/SimImu.hpp"
#include "AP/Math/ApMath.hpp"

namespace Ap {

SimImu::SimImu(const char* const compName)
    : SimImuComponentBase(compName)
{}

SimImu::~SimImu() {}

// -------------------------------------------------------------------------
// schedIn — called at 400Hz
// -------------------------------------------------------------------------
void SimImu::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    // Compute true specific force in body frame:
    // accel = (velocity_derivative + gravity) rotated to body frame
    // For simplicity, use velocity as proxy and add gravity in NED then rotate
    const auto vel = m_truth.get_velocity_ned();
    const auto quat = m_truth.get_attitude_quat();
    const auto rate = m_truth.get_angular_rate_dps();

    // True accel ≈ gravity in body frame (simplified — full model would difference velocities)
    // Rotate gravity [0, 0, 9.81] from NED to body using quaternion inverse
    Ap::Quatd q(quat.get_w(), quat.get_x(), quat.get_y(), quat.get_z());
    q.normalize();
    Ap::Vec3d gravNED(0.0, 0.0, Ap::GRAVITY);
    Ap::Vec3d gravBody = q.inverse() * gravNED;

    // Sensed accel = -gravBody (accelerometer at rest reads +1g upward)
    double ax = -gravBody.x() + m_accelNoise(m_rng);
    double ay = -gravBody.y() + m_accelNoise(m_rng);
    double az = -gravBody.z() + m_accelNoise(m_rng);

    // Gyro = true angular rate + noise
    double gx = rate.get_x() + m_gyroNoise(m_rng);
    double gy = rate.get_y() + m_gyroNoise(m_rng);
    double gz = rate.get_z() + m_gyroNoise(m_rng);

    Ap::ImuData imu;
    imu.set_accel_mps2(Ap::Vec3(ax, ay, az));
    imu.set_gyro_dps(Ap::Vec3(gx, gy, gz));
    imu.set_time_s(m_truth.get_time_s());

    if (this->isConnected_imuOut_OutputPort(0)) {
        this->imuOut_out(0, imu);
    }
}

// -------------------------------------------------------------------------
// truthStateIn — async handler, stores latest truth
// -------------------------------------------------------------------------
void SimImu::truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_truth = state;
}

}  // namespace Ap
