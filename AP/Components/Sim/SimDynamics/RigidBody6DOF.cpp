#include "RigidBody6DOF.hpp"
#include "AP/Math/CoordTransforms.hpp"

namespace Sim {

// -------------------------------------------------------------------------
// Construction & reset
// -------------------------------------------------------------------------

RigidBody6DOF::RigidBody6DOF(const SimpleAircraftParams& params)
    : m_params(params)
{
    m_state.setZero();
    reset();
}

void RigidBody6DOF::reset(double airspeed_ms, double altitude_m) {
    m_state.setZero();

    // Position: at altitude, NED down is negative altitude
    m_state[Idx::PZ] = -altitude_m;

    // Velocity: forward in body frame
    m_state[Idx::U] = airspeed_ms;

    // Quaternion: identity (level, heading north)
    m_state[Idx::QW] = 1.0;
    m_state[Idx::QX] = 0.0;
    m_state[Idx::QY] = 0.0;
    m_state[Idx::QZ] = 0.0;

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
// Integration
// -------------------------------------------------------------------------
//
// Can be made better by using RK4 or other integrators
//

void RigidBody6DOF::step(double dt) {
    // Forward Euler — simplest integrator. Swap for RK4 later.
    const StateVec ddt = computeDerivative(m_state, m_surfaces);
    m_state += dt * ddt;
    normalizeQuat(m_state);
    m_time += dt;
}

// -------------------------------------------------------------------------
// State derivative — this is where the physics lives
// -------------------------------------------------------------------------
//
// Replace the body of this function with real stability-derivative
// aerodynamics when ready. The integrator doesn't change.
// This block basically is the main physics:  x_dot = F(x,u)
//

StateVec RigidBody6DOF::computeDerivative(const StateVec& s,
                                           const SurfaceInput& cmd) const {
    StateVec ddt;
    ddt.setZero();

    const double u = s[Idx::U], v = s[Idx::V], w = s[Idx::W];
    const double p = s[Idx::P], q = s[Idx::Q], r = s[Idx::R];

    // --- Rotation matrix (body-from-NED) ---
    const Ap::Mat3d C_bn = dcmFromState(s);
    const Ap::Mat3d C_nb = C_bn.transpose();  // NED-from-body

    // --- Airspeed ---
    const double V_air = std::sqrt(u * u + v * v + w * w);

    // --- Forces in body frame ---

    // Thrust along body X
    const double thrust = cmd.throttle * m_params.maxThrust;

    // Simple quadratic drag opposing body-X velocity
    const double drag = m_params.dragCoeff * V_air * V_air;

    // Simple lift along body -Z (opposes weight in level flight)
    const double lift = m_params.liftCoeff * V_air * V_air;

    // Gravity in body frame:  C_bn * [0, 0, g]^T
    const Ap::Vec3d g_body = C_bn * Ap::Vec3d(0.0, 0.0, Ap::GRAVITY);

    const double Fx = thrust - drag + m_params.mass * g_body.x();
    const double Fy =                 m_params.mass * g_body.y();
    const double Fz = -lift         + m_params.mass * g_body.z();

    // --- Moments in body frame ---
    const double L = cmd.aileron  * m_params.lAil  - m_params.dampP * p;
    const double M = cmd.elevator * m_params.mElev - m_params.dampQ * q;
    const double N = cmd.rudder   * m_params.nRud  - m_params.dampR * r;

    // =====================================================================
    // Equations of motion
    // =====================================================================

    // --- Position derivative: velocity in NED frame ---
    const Ap::Vec3d vel_body(u, v, w);
    const Ap::Vec3d vel_ned = C_nb * vel_body;
    ddt[Idx::PX] = vel_ned.x();
    ddt[Idx::PY] = vel_ned.y();
    ddt[Idx::PZ] = vel_ned.z();

    // --- Velocity derivative (body frame, including Coriolis) ---
    //   dv/dt = F/m - omega x v
    ddt[Idx::U] = Fx / m_params.mass + r * v - q * w;
    ddt[Idx::V] = Fy / m_params.mass - r * u + p * w;
    ddt[Idx::W] = Fz / m_params.mass + q * u - p * v;

    // --- Quaternion derivative ---
    //   dq/dt = 0.5 * q * omega_quat
    const double qw = s[Idx::QW], qx = s[Idx::QX];
    const double qy = s[Idx::QY], qz = s[Idx::QZ];
    ddt[Idx::QW] = 0.5 * (-p * qx - q * qy - r * qz);
    ddt[Idx::QX] = 0.5 * ( p * qw + r * qy - q * qz);
    ddt[Idx::QY] = 0.5 * ( q * qw - r * qx + p * qz);
    ddt[Idx::QZ] = 0.5 * ( r * qw + q * qx - p * qy);

    // --- Angular rate derivative (Euler's rotation equations) ---
    //   I * domega/dt = M - omega x (I * omega)
    ddt[Idx::P] = (L - (m_params.Izz - m_params.Iyy) * q * r) / m_params.Ixx;
    ddt[Idx::Q] = (M - (m_params.Ixx - m_params.Izz) * p * r) / m_params.Iyy;
    ddt[Idx::R] = (N - (m_params.Iyy - m_params.Ixx) * p * q) / m_params.Izz;

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
