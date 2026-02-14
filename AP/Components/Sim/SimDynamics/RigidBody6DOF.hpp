#pragma once

#include "AP/Math/ApMath.hpp"

namespace Sim {

// -------------------------------------------------------------------------
// State vector layout (13 elements)
// -------------------------------------------------------------------------
//   [0..2]   position NED          (m)
//   [3..5]   velocity body (u,v,w) (m/s)
//   [6..9]   quaternion (w,x,y,z)  body-from-NED
//   [10..12] angular rate body (p,q,r) (rad/s)
// -------------------------------------------------------------------------

constexpr int STATE_SIZE = 13;
using StateVec = Eigen::Matrix<double, STATE_SIZE, 1>;

// Indices into the state vector
namespace Idx {
    constexpr int PX = 0, PY = 1, PZ = 2;          // position NED
    constexpr int U  = 3, V  = 4, W  = 5;          // velocity body
    constexpr int QW = 6, QX = 7, QY = 8, QZ = 9;  // quaternion
    constexpr int P  = 10, Q = 11, R = 12;          // angular rate body
}

// -------------------------------------------------------------------------
// Surface command input
// -------------------------------------------------------------------------
struct SurfaceInput {
    double aileron  = 0.0;   // -1 to +1
    double elevator = 0.0;   // -1 to +1
    double rudder   = 0.0;   // -1 to +1
    double throttle = 0.0;   //  0 to  1
};

// -------------------------------------------------------------------------
// Simple aircraft constants — tune these, swap for real aero later
// -------------------------------------------------------------------------
struct SimpleAircraftParams {
    // Mass & inertia
    double mass   = 1043.0;   // kg (Cessna 172)
    double Ixx    = 1285.0;   // kg*m^2
    double Iyy    = 1825.0;   // kg*m^2
    double Izz    = 2667.0;   // kg*m^2

    // Propulsion
    double maxThrust = 2000.0; // N
    double dragCoeff = 0.40;   // N/(m/s)^2  — simple quadratic drag

    // Lift — must balance weight at cruise speed
    // Trim: liftCoeff = mass*g / V_cruise^2 = 10228/2500 = 4.09
    double liftCoeff = 4.09;   // N/(m/s)^2

    // Control surface moment gains (Nm per unit deflection)
    double lAil  = 2000.0;    // roll moment per aileron
    double mElev = 3000.0;    // pitch moment per elevator
    double nRud  = 1500.0;    // yaw moment per rudder

    // Angular rate damping (Nm per rad/s)
    double dampP = 200.0;     // roll damping
    double dampQ = 300.0;     // pitch damping
    double dampR = 150.0;     // yaw damping

    // Static stability (Nm per radian)
    double pitchStiffness = 5000.0;  // Cma — restoring pitch moment per rad of alpha
    double yawStiffness   = 2000.0;  // Cnb — weathercock restoring yaw moment per rad of beta
};

// -------------------------------------------------------------------------
// RigidBody6DOF — 13-state 6-DOF integrator with simple force model
// -------------------------------------------------------------------------
class RigidBody6DOF {
public:
    explicit RigidBody6DOF(const SimpleAircraftParams& params = {});

    /// Set surface commands (call before step)
    void setSurfaces(const SurfaceInput& cmd);

    /// Advance state by dt seconds using RK4
    void step(double dt);

    /// Reset to initial conditions (level flight, given airspeed & altitude)
    void reset(double airspeed_ms = 50.0, double altitude_m = 1000.0);

    // --- Accessors ---
    const StateVec& state() const { return m_state; }

    Ap::Vec3d positionNED() const;
    Ap::Vec3d velocityNED() const;
    Ap::Vec3d velocityBody() const;
    Ap::Quatd quaternion() const;
    Ap::Vec3d eulerDeg() const;
    Ap::Vec3d angularRateDps() const;
    double    airspeed() const;
    double    simTime() const { return m_time; }

private:
    /// Compute state derivative given state and surfaces
    StateVec computeDerivative(const StateVec& s, const SurfaceInput& cmd) const;

    /// Normalize the quaternion in-place
    static void normalizeQuat(StateVec& s);

    /// Extract rotation matrix (body-from-NED) from quaternion in state
    static Ap::Mat3d dcmFromState(const StateVec& s);

    SimpleAircraftParams m_params;
    SurfaceInput         m_surfaces;
    StateVec             m_state;
    double               m_time = 0.0;
};

}  // namespace Sim
