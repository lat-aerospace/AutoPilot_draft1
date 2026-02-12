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
    std::mt19937 m_rng{42};
    std::normal_distribution<double> m_accelNoise{0.0, 0.05};  // m/s^2
    std::normal_distribution<double> m_gyroNoise{0.0, 0.01};   // deg/s
};

}  // namespace Ap

#endif
