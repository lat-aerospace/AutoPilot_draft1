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

    // -----------------------------------------------------------------
    // Magnetometer error parameters (tune these)
    // -----------------------------------------------------------------
    // Hard-iron offset [gauss] — constant bias from magnetized components
    static constexpr double HARD_IRON_X =  0.02;   // body-X offset
    static constexpr double HARD_IRON_Y = -0.01;   // body-Y offset
    static constexpr double HARD_IRON_Z =  0.03;   // body-Z offset

    // Soft-iron scale factors — distortion from nearby ferromagnetic material
    // 1.0 = no distortion; typical range 0.95–1.05
    static constexpr double SOFT_IRON_X = 1.02;
    static constexpr double SOFT_IRON_Y = 0.98;
    static constexpr double SOFT_IRON_Z = 1.01;

    static constexpr double MAG_NOISE_SIGMA = 1e-6;//0.005;  // [gauss] white noise 1-sigma
    // -----------------------------------------------------------------

    std::mt19937 m_rng{99};
    std::normal_distribution<double> m_noise{0.0, 1.0};
};

}  // namespace Ap

#endif
