#ifndef Ap_SimGps_HPP
#define Ap_SimGps_HPP

#include "AP/Components/Sim/SimGps/SimGpsComponentAc.hpp"
#include <random>

namespace Ap {

class SimGps final : public SimGpsComponentBase {
  public:
    SimGps(const char* const compName);
    ~SimGps();

  private:
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;

    Ap::AircraftState m_truth;
    std::mt19937 m_rng{123};
    std::normal_distribution<double> m_posNoise{0.0, 1.5};   // metres
    std::normal_distribution<double> m_velNoise{0.0, 0.1};   // m/s
};

}  // namespace Ap

#endif
