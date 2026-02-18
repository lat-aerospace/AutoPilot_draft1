#include "AP/Components/FlightControl/Controller/Controller.hpp"
#include <cmath>

namespace Ap {

Controller::Controller(const char* const compName)
    : ControllerComponentBase(compName)
{
    m_cmd.set_desired_alt_m(INIT_ALT_M);
    m_cmd.set_desired_airspeed_ms(INIT_SPEED_MS);
    m_cmd.set_desired_heading_deg(0.0);
}

Controller::~Controller() {}

// =========================================================================
// LEVEL 1 — Navigation / Guidance  (10Hz, RG4)
// Inputs:  GuidanceCmd (desired alt, speed, heading) + AircraftState
// Outputs: m_desRoll_deg, m_desPitch_deg, m_desThrottle
// =========================================================================
void Controller::schedIn_nav_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    // --- Lateral: heading error → desired bank angle ---
    double hdgErr = wrapDeg(m_cmd.get_desired_heading_deg()
                            - m_state.get_euler_deg().get_z());
    m_desRoll_deg = clamp(KP_HDG * hdgErr, -30.0, 30.0);

    // --- Longitudinal: altitude error → desired pitch ---
    constexpr double REF_ALT = 1600.0;
    double estAlt = REF_ALT - m_state.get_position_ned().get_z();
    double altErr = m_cmd.get_desired_alt_m() - estAlt;
    m_desPitch_deg = clamp(KP_ALT * altErr, -15.0, 15.0);

    // --- Speed: airspeed error → throttle (PI) ---
    constexpr double DT_NAV = 0.1;   // 10Hz nav loop
    double spdErr  = m_cmd.get_desired_airspeed_ms() - m_state.get_airspeed_ms();
    m_spdI = clamp(m_spdI + KI_SPD * spdErr * DT_NAV, -SPD_IMAX, SPD_IMAX);
    m_desThrottle  = clamp(0.6 + KP_SPD * spdErr + m_spdI, 0.0, 1.0);

}

// =========================================================================
// LEVEL 2 — Attitude Control  (50Hz, RG3)
// Inputs:  m_desRoll_deg, m_desPitch_deg (from L1) + AircraftState
// Outputs: m_desRollRate_dps, m_desPitchRate_dps, m_desYawRate_dps
// =========================================================================
void Controller::schedIn_attitude_handler(FwIndexType portNum, U32 context) {

    double rollErr  = m_desRoll_deg  - m_state.get_euler_deg().get_x();
    double pitchErr = m_desPitch_deg - m_state.get_euler_deg().get_y();

    // P controller: attitude error → desired body rate
    m_desRollRate_dps  = clamp(KP_ROLL_ATT  * rollErr,  -60.0, 60.0);
    m_desPitchRate_dps = clamp(KP_PITCH_ATT * pitchErr, -30.0, 30.0);

    // Yaw coordination: keep sideslip near zero
    // desired yaw rate ≈ (g/V) * tan(bank) — coordinated turn formula
    double bankRad = m_state.get_euler_deg().get_x() * M_PI / 180.0;
    double V       = (m_state.get_airspeed_ms() > 10.0)
                     ? m_state.get_airspeed_ms() : 50.0;
    m_desYawRate_dps = clamp(
        KP_YAW_ATT * (9.81 / V) * std::tan(bankRad) * 180.0 / M_PI,
        -30.0, 30.0);

}

// =========================================================================
// LEVEL 3 — Rate Control / Inner Loop  (400Hz, RG1)
// Inputs:  m_desRollRate/PitchRate/YawRate (from L2) + AircraftState
// Outputs: SurfaceCmd → SimServoDriver
// =========================================================================
void Controller::schedIn_rate_handler(FwIndexType portNum, U32 context) {
    constexpr double DT = 1.0 / 400.0;

    const auto& rates = m_state.get_angular_rate_dps();

    // --- Roll rate PI ---
    double rollRateErr  = m_desRollRate_dps  - rates.get_x();
    m_rollRateI = clamp(m_rollRateI + rollRateErr * DT, -IMAX, IMAX);
    double aileron = clamp(
        KP_ROLL_RATE  * rollRateErr  + KI_ROLL_RATE  * m_rollRateI,
        -1.0, 1.0);

    // --- Pitch rate PI ---
    double pitchRateErr = m_desPitchRate_dps - rates.get_y();
    m_pitchRateI = clamp(m_pitchRateI + pitchRateErr * DT, -IMAX, IMAX);
    double elevator = clamp(
        KP_PITCH_RATE * pitchRateErr + KI_PITCH_RATE * m_pitchRateI,
        -1.0, 1.0);

    // --- Yaw rate PI ---
    double yawRateErr   = m_desYawRate_dps   - rates.get_z();
    m_yawRateI = clamp(m_yawRateI + yawRateErr * DT, -IMAX, IMAX);
    double rudder = clamp(
        KP_YAW_RATE   * yawRateErr   + KI_YAW_RATE   * m_yawRateI,
        -1.0, 1.0);

    // --- Output surface command ---
    Ap::SurfaceCmd cmd;
    cmd.set_aileron (aileron);
    cmd.set_elevator(elevator);
    cmd.set_rudder  (rudder);
    cmd.set_throttle(m_desThrottle);

    if (this->isConnected_surfaceCmdOut_OutputPort(0)) {
        this->surfaceCmdOut_out(0, cmd);
    }

}

// -------------------------------------------------------------------------
// Async input handlers
// -------------------------------------------------------------------------
void Controller::guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) {
    m_cmd = cmd;
}

void Controller::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state = state;
}

double Controller::wrapDeg(double deg) {
    while (deg >  180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double Controller::clamp(double val, double lo, double hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

}  // namespace Ap