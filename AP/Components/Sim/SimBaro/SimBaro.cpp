#include "AP/Components/Sim/SimBaro/SimBaro.hpp"
#include <cmath>

namespace Ap {

SimBaro::SimBaro(const char* const compName)
    : SimBaroComponentBase(compName)
{}

SimBaro::~SimBaro() {}

// -------------------------------------------------------------------------
// schedIn — called at 50Hz (RG3)
// -------------------------------------------------------------------------
void SimBaro::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    const auto pos = m_truth.get_position_ned();

    constexpr double REF_ALT = 1600.0;  // metres MSL

    // True altitude MSL = ref_alt - posD (NED down is negative altitude)
    double trueAlt = REF_ALT - pos.get_z();

    // Update slow bias drift (random walk)
    m_baroBias += BIAS_DRIFT_RATE * std::sqrt(DT) * m_noise(m_rng);

    // Noisy altitude = truth + bias + white noise
    double noisyAlt = trueAlt + m_baroBias + ALT_NOISE_SIGMA * m_noise(m_rng);

    // ISA pressure model: P = P0 * (1 - L*h/T0)^(g/(R*L))
    constexpr double P0  = 101325.0;
    constexpr double L   = 0.0065;
    constexpr double T0  = 288.15;
    constexpr double G   = 9.80665;
    constexpr double R   = 287.05;
    constexpr double EXP = G / (R * L);

    double pressure = P0 * std::pow(1.0 - L * noisyAlt / T0, EXP);

    Ap::BaroData baro;
    baro.set_pressure_pa(pressure);
    baro.set_altitude_m(noisyAlt);
    baro.set_time_s(m_truth.get_time_s());

    if (this->isConnected_baroOut_OutputPort(0)) {
        this->baroOut_out(0, baro);
    }
}

// -------------------------------------------------------------------------
// truthStateIn — async handler
// -------------------------------------------------------------------------
void SimBaro::truthStateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_truth = state;
}

}  // namespace Ap
