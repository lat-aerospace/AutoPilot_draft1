#pragma once

// AircraftParams — all tunable aircraft constants in one place.
// Default: Cessna 172 Skyhawk at typical cruise loading.
// Source: NASA TN D-2999, UIUC airfoil database, JSBSim C172P model.
// Enhanced with blade-element propeller, flap, and ground-contact params.

namespace Ap {

struct AircraftParams {

    // =================================================================
    // GEOMETRY
    // =================================================================
    double wingArea   = 16.17;     // S  [m^2]  wing reference area
    double wingspan   = 10.91;     // b  [m]    wing span
    double meanChord  = 1.49;      // c  [m]    mean aerodynamic chord
    // AR = b^2/S = 7.36

    // =================================================================
    // MASS & INERTIA
    // =================================================================
    double mass = 1043.0;          // [kg]
    double Ixx  = 1285.0;         // [kg m^2]  roll
    double Iyy  = 1825.0;         // [kg m^2]  pitch
    double Izz  = 2667.0;         // [kg m^2]  yaw
    double Ixz  = 0.0;            // [kg m^2]  product of inertia

    // =================================================================
    // LONGITUDINAL STABILITY DERIVATIVES (per radian)
    // q_hat = q*c/(2V)
    // =================================================================
    double CL0   = 0.307;
    double CLa   = 4.59;
    double CLq   = 3.9;
    double CLde  = 0.36;

    double CD0      = 0.027;      // parasite drag
    double e_oswald = 0.80;       // Oswald span efficiency

    double Cm0   = 0.04;
    double Cma   = -0.89;         // MUST be negative (pitch stability)
    double Cmq   = -12.4;         // pitch damping
    double Cmde  = -0.96;

    // =================================================================
    // LATERAL-DIRECTIONAL STABILITY DERIVATIVES (per radian)
    // p_hat = p*b/(2V),  r_hat = r*b/(2V)
    // =================================================================
    double CYb   = -0.31;
    double CYp   = -0.037;
    double CYr   =  0.21;
    double CYda  =  0.0;
    double CYdr  =  0.187;

    double Clb   = -0.089;        // dihedral effect
    double Clp   = -0.47;         // roll damping
    double Clr   =  0.096;
    double Clda  = -0.178;
    double Cldr  =  0.0147;

    double Cnb   =  0.065;        // weathercock stability
    double Cnp   = -0.030;        // adverse yaw
    double Cnr   = -0.125;        // yaw damping
    double Cnda  = -0.053;
    double Cndr  = -0.066;

    // =================================================================
    // CONTROL SURFACE LIMITS
    // =================================================================
    double de_max = 0.4887;       // elevator [rad] (28 deg)
    double da_max = 0.3491;       // aileron  [rad] (20 deg)
    double dr_max = 0.2793;       // rudder   [rad] (16 deg)

    // =================================================================
    // STALL & POST-STALL
    // =================================================================
    double alpha_stall = 0.2793;  // stall AoA [rad] (16 deg)
    // Post-stall: Kirchhoff flat-plate model (see AeroModel)

    // =================================================================
    // PROPULSION — Blade element theory
    // =================================================================
    // Thrust = CT(J) * rho * n^2 * D^4
    // Torque  = CQ(J) * rho * n^2 * D^5
    // where J = V / (n*D) is the advance ratio

    double prop_diameter   = 1.88;      // [m]   propeller diameter
    double CT0             = 0.50;      // thrust coefficient at J=0 (tuned for V_trim=50 m/s)
    double CQ0             = 0.015;     // torque coefficient at J=0
    double J_max           = 1.3;       // advance ratio at zero thrust (raised for high-speed cruise)
    double max_rpm         = 2700.0;    // [rpm]  max engine RPM
    double prop_inertia    = 0.22;      // [kg m^2]  propeller+engine Izz
    double propwash_factor = 1.5;       // dynamic pressure multiplier on tail surfaces

    // =================================================================
    // GROUND CONTACT (tricycle gear: nose, left main, right main)
    // =================================================================
    double gear_k        = 50000.0;  // [N/m]   spring stiffness per gear leg
    double gear_c        = 5000.0;   // [N s/m] damper per gear leg
    double gear_mu       = 0.4;      // rolling friction coefficient
    // Gear positions in body frame [m] (X=fwd, Y=right, Z=down)
    double nose_x  = 1.5;   double nose_y  =  0.0;  double nose_z  = 1.0;
    double lmain_x = -0.3;  double lmain_y = -1.5;  double lmain_z = 1.0;
    double rmain_x = -0.3;  double rmain_y =  1.5;  double rmain_z = 1.0;

    // =================================================================
    // REFERENCE / TRIM
    // =================================================================
    double V_trim   = 50.0;    // [m/s]  cruise airspeed
    double alt_trim = 1524.0;  // [m]    cruise altitude MSL (5000 ft)
    double Re_ref   = 4.5e6;   // reference Reynolds number for CD0
};

}  // namespace Ap
