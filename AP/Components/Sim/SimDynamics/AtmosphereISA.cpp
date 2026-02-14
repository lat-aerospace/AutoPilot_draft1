#include "AtmosphereISA.hpp"

namespace Sim {

AtmosphereState AtmosphereISA::compute(double altitude_m) {
    // Clamp to troposphere [0, 11000 m]
    double h = altitude_m;
    if (h < 0.0)     h = 0.0;
    if (h > 11000.0) h = 11000.0;

    AtmosphereState atm;

    // Temperature: linear lapse from sea level
    //   T(h) = T0 - L * h
    const double tempRatio = 1.0 - LAPSE * h / T0;
    atm.temperature = T0 * tempRatio;

    // Pressure: barometric formula
    //   P(h) = P0 * (T/T0)^(g/(R*L))
    const double exponent = G_STD / (R_AIR * LAPSE);
    atm.pressure = P0 * std::pow(tempRatio, exponent);

    // Density: ideal gas law  rho = P / (R * T)
    atm.density = atm.pressure / (R_AIR * atm.temperature);

    // Speed of sound: a = sqrt(gamma * R * T)
    atm.speedOfSound = std::sqrt(GAMMA * R_AIR * atm.temperature);

    return atm;
}

}  // namespace Sim
