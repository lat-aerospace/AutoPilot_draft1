#include "AP/Components/Sim/SimImu/SimImu.hpp"
#include "AP/Math/ApMath.hpp"
#include <cmath>

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

    const auto quat = m_truth.get_attitude_quat();
    const auto rate = m_truth.get_angular_rate_dps();

    // Rotate gravity [0, 0, 9.81] from NED to body frame
    Ap::Quatd q(quat.get_w(), quat.get_x(), quat.get_y(), quat.get_z());
    q.normalize();
    Ap::Vec3d gravNED(0.0, 0.0, Ap::GRAVITY);
    Ap::Vec3d gravBody = q.inverse() * gravNED;

    // --- Update bias random walk ---
    const double sqrt_dt = std::sqrt(DT);
    for (int i = 0; i < 3; i++) {
        m_accelBias[i] += ACCEL_BIAS_WALK * sqrt_dt * m_noise(m_rng);
        m_gyroBias[i]  += GYRO_BIAS_WALK  * sqrt_dt * m_noise(m_rng);
    }

    // --- Accelerometer: truth + bias + noise ---
    // Sensed accel = -gravBody (at rest reads +1g upward)
    double ax = -gravBody.x() + m_accelBias[0] + ACCEL_NOISE_SIGMA * m_noise(m_rng);
    double ay = -gravBody.y() + m_accelBias[1] + ACCEL_NOISE_SIGMA * m_noise(m_rng);
    double az = -gravBody.z() + m_accelBias[2] + ACCEL_NOISE_SIGMA * m_noise(m_rng);

    // --- Gyroscope: truth * (1 + scale_error) + bias + noise ---
    double gx = rate.get_x() * (1.0 + GYRO_SCALE_ERR) + m_gyroBias[0]
                + GYRO_NOISE_SIGMA * m_noise(m_rng);
    double gy = rate.get_y() * (1.0 + GYRO_SCALE_ERR) + m_gyroBias[1]
                + GYRO_NOISE_SIGMA * m_noise(m_rng);
    double gz = rate.get_z() * (1.0 + GYRO_SCALE_ERR) + m_gyroBias[2]
                + GYRO_NOISE_SIGMA * m_noise(m_rng);

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
