#pragma once

#include <cmath>

namespace Sim {

// =========================================================================
// ISA atmosphere output
// =========================================================================
struct AtmosphereState {
    double density;       // kg/m^3
    double temperature;   // K
    double pressure;      // Pa
    double speedOfSound;  // m/s
};

// =========================================================================
// International Standard Atmosphere (ISA) — troposphere model
// Valid from sea level to 11,000 m (36,089 ft).
// =========================================================================
class AtmosphereISA {
public:
    /// Compute atmosphere properties at a given altitude MSL (metres).
    /// Clamped to [0, 11000] m.
    static AtmosphereState compute(double altitude_m);

    // -----------------------------------------------------------------
    // ISA constants (all easily changeable here)
    // -----------------------------------------------------------------
    static constexpr double P0    = 101325.0;    // Pa   — sea-level pressure
    static constexpr double T0    = 288.15;      // K    — sea-level temperature (15°C)
    static constexpr double RHO0  = 1.225;       // kg/m^3 — sea-level density
    static constexpr double LAPSE = 0.0065;      // K/m  — temperature lapse rate
    static constexpr double R_AIR = 287.05287;   // J/(kg·K) — specific gas constant, dry air
    static constexpr double G_STD = 9.80665;     // m/s^2 — standard gravity
    static constexpr double GAMMA = 1.4;         // —    — ratio of specific heats (cp/cv)
};

}  // namespace Sim
