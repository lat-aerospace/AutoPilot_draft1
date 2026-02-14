#include "RigidBody6DOF.hpp"
#include "AP/Math/CoordTransforms.hpp"
#include <cmath>

namespace Sim {

// -------------------------------------------------------------------------
// Construction & reset
// -------------------------------------------------------------------------

RigidBody6DOF::RigidBody6DOF(const AircraftParams& params)
    : m_params(params)
    , m_aero(m_params)
    , m_prop(m_params)
{
    m_state.setZero();
    reset();
}

void RigidBody6DOF::reset(double airspeed_ms, double altitude_m) {
    m_state.setZero();

    // Position: NED down is negative altitude
    m_state[Idx::PZ] = -altitude_m;

    // Velocity: forward in body frame
    m_state[Idx::U] = airspeed_ms;

    // Quaternion: identity (level, heading north)
    m_state[Idx::QW] = 1.0;

    m_surfaces = {};
    m_time = 0.0;
}

// -------------------------------------------------------------------------
// Surface input
// -------------------------------------------------------------------------

void RigidBody6DOF::setSurfaces(const SurfaceInput& cmd) {
    m_surfaces = cmd;
}

// -------------------------------------------------------------------------
// Integration (RK4 — unchanged)
// -------------------------------------------------------------------------

void RigidBody6DOF::step(double dt) {
    const StateVec k1 = computeDerivative(m_state, m_surfaces);

    StateVec s2 = m_state + 0.5 * dt * k1;
    normalizeQuat(s2);
    const StateVec k2 = computeDerivative(s2, m_surfaces);

    StateVec s3 = m_state + 0.5 * dt * k2;
    normalizeQuat(s3);
    const StateVec k3 = computeDerivative(s3, m_surfaces);

    StateVec s4 = m_state + dt * k3;
    normalizeQuat(s4);
    const StateVec k4 = computeDerivative(s4, m_surfaces);

    m_state += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    normalizeQuat(m_state);
    m_time += dt;
}

// -------------------------------------------------------------------------
// State derivative — stability-derivative aero model
// -------------------------------------------------------------------------
//   x_dot = F(x, u)
//
//   Forces:  AeroModel (CL,CD,CY → body frame) + PropulsionModel + gravity
//   Moments: AeroModel (Cl,Cm,Cn → body frame)
//   EOM:     standard 6-DOF rigid body (same as before)
// -------------------------------------------------------------------------

StateVec RigidBody6DOF::computeDerivative(const StateVec& s,
                                           const SurfaceInput& cmd) const {
    StateVec ddt;
    ddt.setZero();

    const double u = s[Idx::U], v = s[Idx::V], w = s[Idx::W];
    const double p = s[Idx::P], q = s[Idx::Q], r = s[Idx::R];

    // --- Rotation matrix ---
    const Ap::Mat3d C_bn = dcmFromState(s);   // body-from-NED
    const Ap::Mat3d C_nb = C_bn.transpose();   // NED-from-body

    // --- Altitude MSL ---
    const double alt_msl = REF_ALT - s[Idx::PZ];

    // --- Atmosphere ---
    const AtmosphereState atm = AtmosphereISA::compute(alt_msl);

    // --- Wind (NED → body) ---
    const Ap::Vec3d windNED  = m_wind.getWind(m_time, alt_msl);
    const Ap::Vec3d windBody = C_bn * windNED;

    // --- Air-relative velocity in body frame ---
    const double u_air = u - windBody.x();
    const double v_air = v - windBody.y();
    const double w_air = w - windBody.z();
    const double V_air = std::sqrt(u_air * u_air + v_air * v_air + w_air * w_air);

    // --- Aero angles ---
    const double alpha = (V_air > 1.0) ? std::atan2(w_air, u_air) : 0.0;
    double beta_arg = (V_air > 1.0) ? v_air / V_air : 0.0;
    if (beta_arg >  1.0) beta_arg =  1.0;
    if (beta_arg < -1.0) beta_arg = -1.0;
    const double beta = std::asin(beta_arg);

    // --- Control surfaces: normalized [-1,+1] → radians ---
    const double de = cmd.elevator * m_params.de_max;
    const double da = cmd.aileron  * m_params.da_max;
    const double dr = cmd.rudder   * m_params.dr_max;

    // --- Aerodynamic forces & moments (body frame) ---
    AeroConditions cond;
    cond.alpha = alpha;
    cond.beta  = beta;
    cond.V_air = V_air;
    cond.p = p;  cond.q = q;  cond.r = r;
    cond.da = da;  cond.de = de;  cond.dr = dr;
    cond.rho = atm.density;

    const AeroOutput aero = m_aero.compute(cond);

    // --- Propulsion (thrust along body X) ---
    const double thrust = m_prop.computeThrust(cmd.throttle, V_air, atm.density);

    // --- Gravity in body frame ---
    const Ap::Vec3d g_body = C_bn * Ap::Vec3d(0.0, 0.0, Ap::GRAVITY);

    // --- Total forces (body frame) ---
    const double Fx = thrust + aero.force.x() + m_params.mass * g_body.x();
    const double Fy =          aero.force.y() + m_params.mass * g_body.y();
    const double Fz =          aero.force.z() + m_params.mass * g_body.z();

    // --- Total moments (body frame) ---
    const double L_tot = aero.moment.x();
    const double M_tot = aero.moment.y();
    const double N_tot = aero.moment.z();

    // =====================================================================
    // Equations of motion (unchanged)
    // =====================================================================

    // Position derivative: velocity in NED frame
    const Ap::Vec3d vel_body(u, v, w);
    const Ap::Vec3d vel_ned = C_nb * vel_body;
    ddt[Idx::PX] = vel_ned.x();
    ddt[Idx::PY] = vel_ned.y();
    ddt[Idx::PZ] = vel_ned.z();

    // Velocity derivative (body frame): dv/dt = F/m - omega x v
    ddt[Idx::U] = Fx / m_params.mass + r * v - q * w;
    ddt[Idx::V] = Fy / m_params.mass - r * u + p * w;
    ddt[Idx::W] = Fz / m_params.mass + q * u - p * v;

    // Quaternion derivative: dq/dt = 0.5 * q * omega
    const double qw = s[Idx::QW], qx = s[Idx::QX];
    const double qy = s[Idx::QY], qz = s[Idx::QZ];
    ddt[Idx::QW] = 0.5 * (-p * qx - q * qy - r * qz);
    ddt[Idx::QX] = 0.5 * ( p * qw + r * qy - q * qz);
    ddt[Idx::QY] = 0.5 * ( q * qw - r * qx + p * qz);
    ddt[Idx::QZ] = 0.5 * ( r * qw + q * qx - p * qy);

    // Angular rate derivative: I * domega/dt = M - omega x (I * omega)
    ddt[Idx::P] = (L_tot - (m_params.Izz - m_params.Iyy) * q * r) / m_params.Ixx;
    ddt[Idx::Q] = (M_tot - (m_params.Ixx - m_params.Izz) * p * r) / m_params.Iyy;
    ddt[Idx::R] = (N_tot - (m_params.Iyy - m_params.Ixx) * p * q) / m_params.Izz;

    return ddt;
}

// -------------------------------------------------------------------------
// Utility: quaternion normalization
// -------------------------------------------------------------------------

void RigidBody6DOF::normalizeQuat(StateVec& s) {
    const double norm = std::sqrt(
        s[Idx::QW] * s[Idx::QW] + s[Idx::QX] * s[Idx::QX] +
        s[Idx::QY] * s[Idx::QY] + s[Idx::QZ] * s[Idx::QZ]);
    if (norm > 1e-10) {
        const double inv = 1.0 / norm;
        s[Idx::QW] *= inv;
        s[Idx::QX] *= inv;
        s[Idx::QY] *= inv;
        s[Idx::QZ] *= inv;
    }
}

// -------------------------------------------------------------------------
// Utility: DCM from quaternion in state vector
// -------------------------------------------------------------------------

Ap::Mat3d RigidBody6DOF::dcmFromState(const StateVec& s) {
    const Ap::Quatd q(s[Idx::QW], s[Idx::QX], s[Idx::QY], s[Idx::QZ]);
    return q.normalized().toRotationMatrix();
}

// -------------------------------------------------------------------------
// Accessors
// -------------------------------------------------------------------------

Ap::Vec3d RigidBody6DOF::positionNED() const {
    return Ap::Vec3d(m_state[Idx::PX], m_state[Idx::PY], m_state[Idx::PZ]);
}

Ap::Vec3d RigidBody6DOF::velocityBody() const {
    return Ap::Vec3d(m_state[Idx::U], m_state[Idx::V], m_state[Idx::W]);
}

Ap::Vec3d RigidBody6DOF::velocityNED() const {
    const Ap::Mat3d C_nb = dcmFromState(m_state).transpose();
    return C_nb * velocityBody();
}

Ap::Quatd RigidBody6DOF::quaternion() const {
    return Ap::Quatd(m_state[Idx::QW], m_state[Idx::QX],
                     m_state[Idx::QY], m_state[Idx::QZ]).normalized();
}

Ap::Vec3d RigidBody6DOF::eulerDeg() const {
    const Ap::Vec3d rad = Ap::eulerFromQuat(quaternion());
    return rad * Ap::RAD2DEG;
}

Ap::Vec3d RigidBody6DOF::angularRateDps() const {
    return Ap::Vec3d(m_state[Idx::P], m_state[Idx::Q], m_state[Idx::R]) * Ap::RAD2DEG;
}

double RigidBody6DOF::airspeed() const {
    return velocityBody().norm();
}

}  // namespace Sim
