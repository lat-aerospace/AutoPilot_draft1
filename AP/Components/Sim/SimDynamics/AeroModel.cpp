#include "AeroModel.hpp"
#include <cmath>

namespace Sim {

AeroModel::AeroModel(const AircraftParams& params)
    : m_p(params)
    , m_AR(params.wingspan * params.wingspan / params.wingArea)
{}

AeroOutput AeroModel::compute(const AeroConditions& c) const {
    AeroOutput out;

    // Dynamic pressure: q_bar = 0.5 * rho * V^2
    const double qbar = 0.5 * c.rho * c.V_air * c.V_air;

    // Guard against very low airspeed (taxiing / stationary)
    const double V_safe = (c.V_air > 1.0) ? c.V_air : 1.0;

    // Non-dimensional angular rates
    const double phat = c.p * m_p.wingspan  / (2.0 * V_safe);  // p * b / (2V)
    const double qhat = c.q * m_p.meanChord / (2.0 * V_safe);  // q * c / (2V)
    const double rhat = c.r * m_p.wingspan  / (2.0 * V_safe);  // r * b / (2V)

    // --- Coefficient buildup ---
    out.CL = computeCL(c.alpha, qhat, c.de);
    out.CD = computeCD(out.CL);
    out.Cm = computeCm(c.alpha, qhat, c.de);
    out.CY = computeCY(c.beta, phat, rhat, c.da, c.dr);
    out.Cl = computeCl(c.beta, phat, rhat, c.da, c.dr);
    out.Cn = computeCn(c.beta, phat, rhat, c.da, c.dr);

    // --- Stability axis → body axis rotation ---
    // Stability axes: X_stab = -Drag direction, Z_stab = -Lift direction
    // Body axes:
    //   Fx_body = -CD*cos(alpha) + CL*sin(alpha)
    //   Fz_body = -CD*sin(alpha) - CL*cos(alpha)
    const double ca = std::cos(c.alpha);
    const double sa = std::sin(c.alpha);

    const double CX_body = -out.CD * ca + out.CL * sa;
    const double CZ_body = -out.CD * sa - out.CL * ca;

    const double S = m_p.wingArea;
    out.force.x() = qbar * S * CX_body;
    out.force.y() = qbar * S * out.CY;
    out.force.z() = qbar * S * CZ_body;

    out.moment.x() = qbar * S * m_p.wingspan  * out.Cl;  // L — roll
    out.moment.y() = qbar * S * m_p.meanChord * out.Cm;  // M — pitch
    out.moment.z() = qbar * S * m_p.wingspan  * out.Cn;  // N — yaw

    return out;
}

// -------------------------------------------------------------------------
// Coefficient computations
// -------------------------------------------------------------------------

double AeroModel::computeCL(double alpha, double qhat, double de) const {
    double CL = m_p.CL0 + m_p.CLa * alpha + m_p.CLq * qhat + m_p.CLde * de;

    // Post-stall: exponential CL rolloff beyond alpha_stall
    if (std::abs(alpha) > m_p.alpha_stall) {
        double CL_at_stall = m_p.CL0 + m_p.CLa * m_p.alpha_stall;
        double excess = std::abs(alpha) - m_p.alpha_stall;
        double stall_factor = std::exp(-4.0 * excess);
        double sign = (alpha > 0.0) ? 1.0 : -1.0;
        CL = sign * CL_at_stall * stall_factor + m_p.CLq * qhat + m_p.CLde * de;
    }

    return CL;
}

double AeroModel::computeCD(double CL) const {
    // Parabolic drag polar: CD = CD0 + CL^2 / (pi * e * AR)
    double CDi = CL * CL / (M_PI * m_p.e_oswald * m_AR);
    return m_p.CD0 + CDi;
}

double AeroModel::computeCm(double alpha, double qhat, double de) const {
    return m_p.Cm0 + m_p.Cma * alpha + m_p.Cmq * qhat + m_p.Cmde * de;
}

double AeroModel::computeCY(double beta, double phat, double rhat,
                             double da, double dr) const {
    return m_p.CYb * beta + m_p.CYp * phat + m_p.CYr * rhat
           + m_p.CYda * da + m_p.CYdr * dr;
}

double AeroModel::computeCl(double beta, double phat, double rhat,
                             double da, double dr) const {
    return m_p.Clb * beta + m_p.Clp * phat + m_p.Clr * rhat
           + m_p.Clda * da + m_p.Cldr * dr;
}

double AeroModel::computeCn(double beta, double phat, double rhat,
                             double da, double dr) const {
    return m_p.Cnb * beta + m_p.Cnp * phat + m_p.Cnr * rhat
           + m_p.Cnda * da + m_p.Cndr * dr;
}

}  // namespace Sim
