#ifndef Ap_SimBaro_HPP
#define Ap_SimBaro_HPP

#include "AP/Components/Sim/SimBaro/SimBaroComponentAc.hpp"
#include <random>

namespace Ap {

class SimBaro final : public SimBaroComponentBase {
  public:
    SimBaro(const char* const compName);
    ~SimBaro();

  private:
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;

    Ap::AircraftState m_truth;
    std::mt19937 m_rng{77};
    std::normal_distribution<double> m_altNoise{0.0, 0.5};  // metres
};

}  // namespace Ap

#endif
