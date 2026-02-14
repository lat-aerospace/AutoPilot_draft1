#pragma once

#include "AP/Math/ApMath.hpp"
#include <random>
#include <cmath>

namespace Sim {

// =========================================================================
// WindModel — steady wind + Dryden turbulence gusts
// =========================================================================
// Defaults to zero wind (sim behaves as before).
// Call setSteadyWind() and/or enableTurbulence() to activate.
//
// Tunable parameters:
//   - Steady wind vector (NED, m/s)
//   - Turbulence on/off, intensity (m/s RMS)
// =========================================================================

class WindModel {
public:
    WindModel();

    /// Set constant wind in NED frame [m/s]
    /// Example: Vec3d(5, 0, 0) = 5 m/s from south (blowing north)
    void setSteadyWind(const Ap::Vec3d& windNED);

    /// Enable Dryden turbulence gusts
    /// intensity: RMS gust magnitude [m/s] (light=1, moderate=3, severe=6)
    void enableTurbulence(bool enable, double intensity_mps = 1.0);

    /// Get total wind (steady + gust) in NED frame
    /// time_s: simulation time, altitude_m: altitude MSL for gust scaling
    Ap::Vec3d getWind(double time_s, double altitude_m) const;

    const Ap::Vec3d& steadyWind() const { return m_steadyWind; }

private:
    Ap::Vec3d m_steadyWind{0.0, 0.0, 0.0};

    bool   m_turbEnabled   = false;
    double m_turbIntensity = 1.0;   // [m/s] RMS gust

    // Dryden filter state (first-order low-pass for each NED axis)
    mutable Ap::Vec3d m_gustState{0.0, 0.0, 0.0};
    mutable double    m_lastTime = -1.0;

    mutable std::mt19937 m_rng{314159};
    mutable std::normal_distribution<double> m_noise{0.0, 1.0};
};

}  // namespace Sim
