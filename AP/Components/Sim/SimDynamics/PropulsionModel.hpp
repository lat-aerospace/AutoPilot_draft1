#pragma once

#include "AircraftParams.hpp"

namespace Sim {

// =========================================================================
// PropulsionModel — simple fixed-pitch propeller thrust
// =========================================================================
// Thrust = throttle * T_static * (rho/rho0) * max(0, 1 - V/V_zero)
//
// T_static:  max thrust at sea level, V=0, full throttle
// rho/rho0:  density ratio (accounts for altitude)
// V_zero:    airspeed at which thrust drops to zero (propeller unloading)
//
// Tunable params in AircraftParams: maxThrust_static, V_prop_zero
// =========================================================================

class PropulsionModel {
public:
    explicit PropulsionModel(const AircraftParams& params);

    /// Compute thrust along body X-axis [N]
    /// throttle: 0..1,  V_air: true airspeed [m/s],  rho: density [kg/m^3]
    double computeThrust(double throttle, double V_air, double rho) const;

private:
    const AircraftParams& m_p;

    // ISA sea-level density for ratio computation
    static constexpr double RHO0 = 1.225;  // kg/m^3
};

}  // namespace Sim
