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

    // -----------------------------------------------------------------
    // Baro error parameters (tune these)
    // -----------------------------------------------------------------
    static constexpr double ALT_NOISE_SIGMA = 1e-4;//0.5;     // [m] white noise per sample
    static constexpr double BIAS_DRIFT_RATE = 1e-6;//0.001;   // [m] per sqrt(s) — slow random walk
    static constexpr double DT = 0.02;                  // 50 Hz
    // -----------------------------------------------------------------

    double m_baroBias = 0.0;  // evolving altitude bias [m]

    std::mt19937 m_rng{77};
    std::normal_distribution<double> m_noise{0.0, 1.0};
};

}  // namespace Ap

#endif
