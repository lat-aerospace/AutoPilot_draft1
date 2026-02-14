#include "PropulsionModel.hpp"
#include <cmath>

namespace Sim {

PropulsionModel::PropulsionModel(const AircraftParams& params)
    : m_p(params)
{}

double PropulsionModel::computeThrust(double throttle, double V_air,
                                       double rho) const {
    // Density ratio: thrust falls with altitude
    const double density_ratio = rho / RHO0;

    // Speed factor: thrust decreases as prop advance ratio increases
    // At V=0 → factor=1 (static), at V=V_prop_zero → factor=0
    double speed_factor = 1.0 - V_air / m_p.V_prop_zero;
    if (speed_factor < 0.0) speed_factor = 0.0;

    return throttle * m_p.maxThrust_static * density_ratio * speed_factor;
}

}  // namespace Sim
