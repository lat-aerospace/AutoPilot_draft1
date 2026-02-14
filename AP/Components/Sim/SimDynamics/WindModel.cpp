#include "WindModel.hpp"

namespace Sim {

WindModel::WindModel() {}

void WindModel::setSteadyWind(const Ap::Vec3d& windNED) {
    m_steadyWind = windNED;
}

void WindModel::enableTurbulence(bool enable, double intensity_mps) {
    m_turbEnabled = enable;
    m_turbIntensity = intensity_mps;
}

Ap::Vec3d WindModel::getWind(double time_s, double altitude_m) const {
    if (!m_turbEnabled) {
        return m_steadyWind;
    }

    // Compute dt since last call
    double dt = (m_lastTime < 0.0) ? 0.0025 : (time_s - m_lastTime);
    m_lastTime = time_s;
    if (dt <= 0.0) {
        return m_steadyWind + m_gustState;
    }

    // Dryden turbulence (MIL-HDBK-1797, low-altitude model)
    //
    // Each axis is a first-order low-pass filter driven by white noise:
    //   x[k+1] = a * x[k] + b * w[k]
    // where a = exp(-dt/tau), b = sigma * sqrt(1 - a^2)
    //
    // Time constant tau = L / V_ref, where L is the Dryden length scale.

    double h = altitude_m;
    if (h < 10.0) h = 10.0;  // prevent divide-by-zero

    // Dryden length scales (low altitude, h < 1000 ft ~ 305 m)
    double Lu = h / std::pow(0.177 + 0.000823 * h, 1.2);
    double Lv = Lu;
    double Lw = h;

    // Turbulence intensities per axis
    double sigma_u = m_turbIntensity;
    double sigma_v = m_turbIntensity;
    double sigma_w = m_turbIntensity * 0.5;  // vertical gusts are weaker

    // Reference airspeed for filter bandwidth
    constexpr double V_ref = 50.0;  // m/s

    // Update each axis with first-order filter
    auto filterStep = [&](double L, double sigma, double& state) {
        double tau = L / V_ref;
        double a = std::exp(-dt / tau);
        double b = sigma * std::sqrt(1.0 - a * a);
        state = a * state + b * m_noise(m_rng);
    };

    double gu = m_gustState.x();
    double gv = m_gustState.y();
    double gw = m_gustState.z();

    filterStep(Lu, sigma_u, gu);
    filterStep(Lv, sigma_v, gv);
    filterStep(Lw, sigma_w, gw);

    m_gustState = Ap::Vec3d(gu, gv, gw);

    return m_steadyWind + m_gustState;
}

}  // namespace Sim
