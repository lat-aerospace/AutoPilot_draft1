#pragma once

#include "AP/Math/ApMath.hpp"
#include "AircraftParams.hpp"

namespace Sim {

// =========================================================================
// Flight conditions input to the aero model
// =========================================================================
struct AeroConditions {
    double alpha;       // angle of attack    [rad]
    double beta;        // sideslip angle     [rad]
    double V_air;       // true airspeed      [m/s]
    double p, q, r;     // body angular rates [rad/s]
    double da, de, dr;  // surface deflections [rad] (NOT normalized)
    double rho;         // air density        [kg/m^3]
};

// =========================================================================
// Aerodynamic forces & moments output (body frame)
// =========================================================================
struct AeroOutput {
    Ap::Vec3d force;    // (Fx, Fy, Fz) body frame [N]
    Ap::Vec3d moment;   // (L,  M,  N)  body frame [N m]

    // Individual coefficients (for telemetry / debugging)
    double CL, CD, CY;
    double Cl, Cm, Cn;
};

// =========================================================================
// AeroModel — stability-derivative coefficient buildup
// =========================================================================
class AeroModel {
public:
    explicit AeroModel(const AircraftParams& params);

    /// Compute body-frame aerodynamic forces and moments
    AeroOutput compute(const AeroConditions& c) const;

private:
    double computeCL(double alpha, double qhat, double de) const;
    double computeCD(double CL) const;
    double computeCm(double alpha, double qhat, double de) const;
    double computeCY(double beta, double phat, double rhat,
                     double da, double dr) const;
    double computeCl(double beta, double phat, double rhat,
                     double da, double dr) const;
    double computeCn(double beta, double phat, double rhat,
                     double da, double dr) const;

    const AircraftParams& m_p;
    double m_AR;  // aspect ratio, precomputed
};

}  // namespace Sim
