#ifndef Ap_Controller_HPP
#define Ap_Controller_HPP

#include "ControllerComponentAc.hpp"

namespace Ap {

class Controller final : public ControllerComponentBase {
  public:
    Controller(const char* const compName);
    ~Controller();

  private:
    // ---------------------------------------------------------------
    // Three scheduled handlers (one per rate group)
    // ---------------------------------------------------------------
    void schedIn_nav_handler     (FwIndexType portNum, U32 context) override;  // 10Hz
    void schedIn_attitude_handler(FwIndexType portNum, U32 context) override;  // 50Hz
    void schedIn_rate_handler    (FwIndexType portNum, U32 context) override;  // 400Hz

    void guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) override;
    void stateIn_handler      (FwIndexType portNum, Ap::AircraftState& state) override;

    static double wrapDeg(double deg);
    static double clamp(double val, double lo, double hi);

    // ---------------------------------------------------------------
    // Latest inputs (written by async handlers)
    // ---------------------------------------------------------------
    Ap::GuidanceCmd   m_cmd;
    Ap::AircraftState m_state;

    // ---------------------------------------------------------------
    // Level 1 → Level 2 handoff  (written at 10Hz, read at 50Hz)
    // ---------------------------------------------------------------
    double m_desRoll_deg  = 0.0;   // desired bank angle
    double m_desPitch_deg = 0.0;   // desired pitch angle

    // ---------------------------------------------------------------
    // Level 2 → Level 3 handoff  (written at 50Hz, read at 400Hz)
    // ---------------------------------------------------------------
    double m_desRollRate_dps  = 0.0;
    double m_desPitchRate_dps = 0.0;
    double m_desYawRate_dps   = 0.0;
    double m_desThrottle      = 0.6;

    // ---------------------------------------------------------------
    // Integrators for PI rate loops (Level 3)
    // ---------------------------------------------------------------
    double m_rollRateI  = 0.0;
    double m_pitchRateI = 0.0;
    double m_yawRateI   = 0.0;

    // Speed integrator (Level 1)
    double m_spdI = 0.0;

    // ---------------------------------------------------------------
    // Initial cmd defaults — match sim reset() initial conditions
    // RigidBody6DOF::REF_ALT_MSL (1600 m) + default AGL (1000 m) = 2600 m
    // ---------------------------------------------------------------
    static constexpr double INIT_ALT_M    = 2600.0;  // m MSL
    static constexpr double INIT_SPEED_MS = 50.0;    // m/s

    // ---------------------------------------------------------------
    // Level 1 gains (Nav — 10Hz)
    // ---------------------------------------------------------------
    static constexpr double KP_HDG   = 0.08;   // heading err (deg) → desired roll (deg)
    static constexpr double KP_ALT   = 0.02;   // alt err (m)       → desired pitch (deg)
    static constexpr double KP_SPD   = 0.008;  // speed err (m/s)   → throttle P term
    static constexpr double KI_SPD   = 0.001;  // speed err integral → throttle I term
    static constexpr double SPD_IMAX = 0.3;    // integrator clamp (throttle units)

    // ---------------------------------------------------------------
    // Level 2 gains (Attitude — 50Hz)
    // ---------------------------------------------------------------
    static constexpr double KP_ROLL_ATT  = 5.0;   // roll err (deg)  → roll rate (dps)
    static constexpr double KP_PITCH_ATT = 5.0;   // pitch err (deg) → pitch rate (dps)
    static constexpr double KP_YAW_ATT   = 2.0;   // yaw coordination gain

    // ---------------------------------------------------------------
    // Level 3 gains (Rate — 400Hz)
    // ---------------------------------------------------------------
    static constexpr double KP_ROLL_RATE  = 0.10;
    static constexpr double KI_ROLL_RATE  = 0.05;
    static constexpr double KP_PITCH_RATE = 0.12;
    static constexpr double KI_PITCH_RATE = 0.06;
    static constexpr double KP_YAW_RATE   = 0.08;
    static constexpr double KI_YAW_RATE   = 0.03;

    static constexpr double IMAX = 0.3;   // integrator clamp (normalized surface units)
};

}  // namespace Ap
#endif