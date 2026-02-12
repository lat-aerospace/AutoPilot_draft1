#ifndef Ap_SimMag_HPP
#define Ap_SimMag_HPP

#include "AP/Components/Sim/SimMag/SimMagComponentAc.hpp"
#include <random>

namespace Ap {

class SimMag final : public SimMagComponentBase {
  public:
    SimMag(const char* const compName);
    ~SimMag();

  private:
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;

    Ap::AircraftState m_truth;
    std::mt19937 m_rng{99};
    std::normal_distribution<double> m_magNoise{0.0, 0.005};  // gauss
};

}  // namespace Ap

#endif
