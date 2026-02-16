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

    // -----------------------------------------------------------------
    // GPS error parameters (tune these)
    // -----------------------------------------------------------------
    static constexpr double POS_NOISE_SIGMA = 1e-4;//1.5;    // [m] position noise 1-sigma
    static constexpr double VEL_NOISE_SIGMA = 1e-6;//0.1;    // [m/s] velocity noise 1-sigma

    // Measurement delay: ring buffer of past truth states
    // At 10Hz, 3 samples = 300ms delay (typical consumer GPS)
    static constexpr int GPS_DELAY_SAMPLES = 3;
    // -----------------------------------------------------------------

    Ap::AircraftState m_delayBuf[GPS_DELAY_SAMPLES];
    int  m_delayIdx   = 0;
    bool m_bufferFull = false;

    std::mt19937 m_rng{123};
    std::normal_distribution<double> m_noise{0.0, 1.0};
};

}  // namespace Ap

#endif
