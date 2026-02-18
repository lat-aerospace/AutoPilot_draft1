#pragma once

// AP_TECS — Total Energy Control System for altitude + airspeed control.
// Pure C++17, no F Prime dependency.
//
// TECS manages pitch and throttle together using energy concepts:
//   - Total energy = kinetic (0.5*V^2) + potential (g*h)
//   - Energy distribution = potential - kinetic
//
// Throttle → total energy
// Pitch    → energy distribution (trade altitude for speed and vice versa)
//
// This is the same fundamental approach as ArduPlane's AP_TECS library.
// Reference: Lambregts, 1983 "Integrated system design for flight and propulsion
//            control using total energy principles"

#include "AP_Math/AP_Math.hpp"

namespace Ap {

struct TecsOutput {
    double throttle;    // 0..1
    double pitch_rad;   // desired pitch angle [rad]
    // Diagnostics
    double e_total_err; // total energy error [m^2/s^2]
    double e_dist_err;  // energy distribution error [m^2/s^2]
    double spe;         // specific potential energy
    double ske;         // specific kinetic energy
};

struct TecsParams {
    // Throttle controller gains
    double K_thr       = 0.5;     // proportional total-energy → throttle
    double K_thr_d     = 0.05;    // derivative term (damps throttle oscillations)
    double thr_trim    = 0.72;    // trim throttle (level cruise at 50 m/s, 2600 m MSL)
    double thr_min     = 0.0;
    double thr_max     = 1.0;

    // Pitch controller gains
    double K_pit       = 0.35;    // proportional energy-distribution → pitch
    double K_pit_d     = 0.08;    // derivative
    double pitch_min   = -0.35;   // rad (-20 deg)
    double pitch_max   =  0.35;   // rad (+20 deg)

    // Time constants for rate filters
    double alt_rate_tau = 2.0;    // altitude rate low-pass [s]
    double spd_rate_tau = 2.0;    // speed rate low-pass [s]

    // Climb/sink rate limits (for energy demand capping)
    double max_climb_ms = 5.0;    // [m/s]
    double max_sink_ms  = 3.0;    // [m/s]
};

class AP_TECS {
public:
    AP_TECS() = default;
    explicit AP_TECS(const TecsParams& params);

    void init(double alt_m, double airspeed_ms, double throttle);
    void setParams(const TecsParams& p) { m_params = p; }

    // Update at 10Hz (NavController rate)
    // target_alt: desired altitude [m MSL]
    // target_speed: desired airspeed [m/s]
    // current_alt: measured altitude [m MSL]
    // current_speed: measured airspeed [m/s]
    void update(double target_alt, double target_speed,
                double current_alt, double current_speed, double dt);

    const TecsOutput& get() const { return m_out; }

    // Parameter setters (for F Prime commands)
    void setThrGain(double k) { m_params.K_thr = k; }
    void setPitGain(double k) { m_params.K_pit = k; }
    void setTrimThrottle(double thr) { m_params.thr_trim = thr; }

private:
    TecsParams m_params;
    TecsOutput m_out{};

    LowPassFilter m_alt_rate_filter{0.2};
    LowPassFilter m_spd_rate_filter{0.2};

    double m_prev_alt   = 0.0;
    double m_prev_speed = 0.0;
    double m_prev_e_total = 0.0;
    double m_prev_e_dist  = 0.0;
    bool   m_first = true;
};

}  // namespace Ap
