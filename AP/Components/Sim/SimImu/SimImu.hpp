#ifndef Ap_SimImu_HPP
#define Ap_SimImu_HPP

#include "AP/Components/Sim/SimImu/SimImuComponentAc.hpp"
#include <random>

namespace Ap {

class SimImu final : public SimImuComponentBase {
  public:
    SimImu(const char* const compName);
    ~SimImu();

  private:
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;

    Ap::AircraftState m_truth;

    // -----------------------------------------------------------------
    // IMU error parameters (tune these)
    // -----------------------------------------------------------------
    // White noise (zero-mean Gaussian per sample)
    static constexpr double ACCEL_NOISE_SIGMA = 0.05;   // [m/s^2] per sample
    static constexpr double GYRO_NOISE_SIGMA  = 0.05;   // [deg/s] per sample

    // Bias random walk (slow drift over time)
    static constexpr double ACCEL_BIAS_WALK = 0.0005;   // [m/s^2] per sqrt(s)
    static constexpr double GYRO_BIAS_WALK  = 0.001;    // [deg/s] per sqrt(s)

    // Scale factor error (constant percentage offset)
    static constexpr double GYRO_SCALE_ERR  = 0.001;    // 0.1% scale error

    // Time step (400 Hz)
    static constexpr double DT = 0.0025;
    // -----------------------------------------------------------------

    // Evolving bias states
    double m_accelBias[3] = {0.0, 0.0, 0.0};  // [m/s^2]
    double m_gyroBias[3]  = {0.0, 0.0, 0.0};  // [deg/s]

    // RNG
    std::mt19937 m_rng{42};
    std::normal_distribution<double> m_noise{0.0, 1.0};
};

}  // namespace Ap

#endif
