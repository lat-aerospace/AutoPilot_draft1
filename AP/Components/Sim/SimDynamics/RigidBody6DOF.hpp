#pragma once

#include "AP/Math/ApMath.hpp"
#include "AircraftParams.hpp"
#include "AtmosphereISA.hpp"
#include "AeroModel.hpp"
#include "PropulsionModel.hpp"
#include "WindModel.hpp"

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
// Surface command input (normalized, from autopilot)
// -------------------------------------------------------------------------
struct SurfaceInput {
    double aileron  = 0.0;   // -1 to +1
    double elevator = 0.0;   // -1 to +1
    double rudder   = 0.0;   // -1 to +1
    double throttle = 0.0;   //  0 to  1
};

// -------------------------------------------------------------------------
// RigidBody6DOF — 13-state 6-DOF integrator with modular force model
// -------------------------------------------------------------------------
class RigidBody6DOF {
public:
    explicit RigidBody6DOF(const AircraftParams& params = {});

    /// Set surface commands (call before step)
    void setSurfaces(const SurfaceInput& cmd);

    /// Advance state by dt seconds using RK4
    void step(double dt);

    /// Reset to initial conditions (level flight, given airspeed & altitude)
    void reset(double airspeed_ms = 50.0, double altitude_m = 1000.0);

    // --- Accessors (unchanged public API) ---
    const StateVec& state() const { return m_state; }

    Ap::Vec3d positionNED() const;
    Ap::Vec3d velocityNED() const;
    Ap::Vec3d velocityBody() const;
    Ap::Quatd quaternion() const;
    Ap::Vec3d eulerDeg() const;
    Ap::Vec3d angularRateDps() const;
    double    airspeed() const;
    double    simTime() const { return m_time; }

    // --- Sub-model access (for configuration) ---
    WindModel& wind() { return m_wind; }

private:
    /// Compute state derivative given state and surfaces
    StateVec computeDerivative(const StateVec& s, const SurfaceInput& cmd) const;

    /// Normalize the quaternion in-place
    static void normalizeQuat(StateVec& s);

    /// Extract rotation matrix (body-from-NED) from quaternion in state
    static Ap::Mat3d dcmFromState(const StateVec& s);

    // Sub-models
    AircraftParams    m_params;
    AeroModel         m_aero;
    PropulsionModel   m_prop;
    WindModel         m_wind;

    SurfaceInput      m_surfaces;
    StateVec          m_state;
    double            m_time = 0.0;

    // Reference altitude for NED ↔ MSL conversion
    static constexpr double REF_ALT = 1600.0;  // m MSL
};

}  // namespace Sim
