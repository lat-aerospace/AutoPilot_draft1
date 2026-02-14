#pragma once

// =========================================================================
// AircraftParams — all tunable aircraft constants in one place
// =========================================================================
// Default values: Cessna 172 Skyhawk at typical cruise loading.
// Source: NASA TN D-2999, UIUC airfoil database, JSBSim Cessna 172P model.
//
// To tune: change any value below and rebuild.
// =========================================================================

namespace Sim {

struct AircraftParams {

    // =================================================================
    // GEOMETRY
    // =================================================================
    double wingArea   = 16.17;     // S   [m^2]  — wing reference area
    double wingspan   = 10.91;     // b   [m]    — wing span
    double meanChord  = 1.49;      // c   [m]    — mean aerodynamic chord
    // Derived: AR = b^2/S = 7.36 (computed at runtime)

    // =================================================================
    // MASS & INERTIA
    // =================================================================
    double mass = 1043.0;          // [kg]       — gross weight (~2300 lbs)
    double Ixx  = 1285.0;         // [kg m^2]   — roll inertia
    double Iyy  = 1825.0;         // [kg m^2]   — pitch inertia
    double Izz  = 2667.0;         // [kg m^2]   — yaw inertia
    double Ixz  = 0.0;            // [kg m^2]   — product of inertia (small, ignored)

    // =================================================================
    // LONGITUDINAL STABILITY DERIVATIVES
    // =================================================================
    // All per-radian. Non-dimensional rates use q_hat = q * c / (2V).

    // -- Lift --
    double CL0   = 0.307;         // CL at zero alpha
    double CLa   = 4.59;          // dCL/d_alpha            [/rad]
    double CLq   = 3.9;           // dCL/d_q_hat            [/rad]
    double CLde  = 0.36;          // dCL/d_delta_elevator    [/rad]

    // -- Drag (parabolic polar: CD = CD0 + CL^2 / (pi * e * AR)) --
    double CD0     = 0.027;       // parasite drag coefficient
    double e_oswald = 0.8;        // Oswald span efficiency factor

    // -- Pitching moment --
    double Cm0   = 0.04;          // Cm at zero alpha (positive = nose up)
    double Cma   = -0.89;         // dCm/d_alpha  (MUST be negative for stability)  [/rad]
    double Cmq   = -12.4;         // dCm/d_q_hat  (pitch damping)                   [/rad]
    double Cmde  = -0.96;         // dCm/d_delta_elevator                            [/rad]

    // =================================================================
    // LATERAL-DIRECTIONAL STABILITY DERIVATIVES
    // =================================================================
    // Non-dimensional rates: p_hat = p*b/(2V), r_hat = r*b/(2V).

    // -- Side force --
    double CYb   = -0.31;         // dCY/d_beta             [/rad]
    double CYp   = -0.037;        // dCY/d_p_hat            [/rad]
    double CYr   = 0.21;          // dCY/d_r_hat            [/rad]
    double CYda  = 0.0;           // dCY/d_delta_aileron    [/rad]
    double CYdr  = 0.187;         // dCY/d_delta_rudder     [/rad]

    // -- Rolling moment --
    double Clb   = -0.089;        // dCl/d_beta  (dihedral effect)    [/rad]
    double Clp   = -0.47;         // dCl/d_p_hat (roll damping)       [/rad]
    double Clr   = 0.096;         // dCl/d_r_hat                      [/rad]
    double Clda  = -0.178;        // dCl/d_delta_aileron              [/rad]
    double Cldr  = 0.0147;        // dCl/d_delta_rudder               [/rad]

    // -- Yawing moment --
    double Cnb   = 0.065;         // dCn/d_beta  (weathercock stability) [/rad]
    double Cnp   = -0.03;         // dCn/d_p_hat (adverse yaw)          [/rad]
    double Cnr   = -0.125;        // dCn/d_r_hat (yaw damping)          [/rad]
    double Cnda  = -0.053;        // dCn/d_delta_aileron                 [/rad]
    double Cndr  = -0.066;        // dCn/d_delta_rudder                  [/rad]

    // =================================================================
    // CONTROL SURFACE LIMITS
    // =================================================================
    double de_max = 0.4887;        // elevator max deflection [rad] (28 deg)
    double da_max = 0.3491;        // aileron max deflection  [rad] (20 deg)
    double dr_max = 0.2793;        // rudder max deflection   [rad] (16 deg)

    // =================================================================
    // STALL
    // =================================================================
    double alpha_stall = 0.2793;   // stall angle of attack [rad] (16 deg)
    // Post-stall CL drops via exponential rolloff (handled in AeroModel)

    // =================================================================
    // PROPULSION
    // =================================================================
    double maxThrust_static = 3000.0;   // [N]  max static thrust at sea level, full throttle
    double V_prop_zero      = 110.0;    // [m/s] airspeed at which propeller thrust → 0

    // =================================================================
    // REFERENCE / TRIM CONDITIONS
    // =================================================================
    double V_trim   = 50.0;       // [m/s]  cruise true airspeed
    double alt_trim = 1524.0;     // [m]    cruise altitude MSL (5000 ft)
};

}  // namespace Sim
