#include "AP/Components/FlightControl/Controller/Controller.hpp"
#include <cmath>

namespace Ap {

Controller::Controller(const char* const compName)
    : ControllerComponentBase(compName)
{
    // Default guidance: hold initial conditions (2600m MSL, 50 m/s, heading north)
    m_cmd.set_desired_alt_m(2600.0);
    m_cmd.set_desired_airspeed_ms(60.0);
    m_cmd.set_desired_heading_deg(0.0);
}

Controller::~Controller() {}

// -------------------------------------------------------------------------
// schedIn — called at 100Hz (RG2)
// -------------------------------------------------------------------------
void Controller::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    // --- Lateral: heading → roll → aileron ---
    double hdgErr = wrapDeg(m_cmd.get_desired_heading_deg() -
                            m_state.get_euler_deg().get_z());

    double desRoll = KP_HDG * hdgErr;
    desRoll = clamp(desRoll, -30.0, 30.0);  // max bank 30°

    double rollErr = desRoll - m_state.get_euler_deg().get_x();
    double aileron = KP_ROLL * rollErr;
    aileron = clamp(aileron, -1.0, 1.0);

    // --- Longitudinal: altitude → pitch → elevator ---
    // position_ned.z is posD; alt_MSL = ref_alt - posD
    constexpr double REF_ALT = 1600.0;
    double estAltMsl = REF_ALT - m_state.get_position_ned().get_z();
    double altErr = m_cmd.get_desired_alt_m() - estAltMsl;

    double desPitch = KP_ALT * altErr;
    desPitch = clamp(desPitch, -15.0, 15.0);  // max pitch ±15°

    double pitchErr = desPitch - m_state.get_euler_deg().get_y();
    double elevator = KP_PITCH * pitchErr;
    elevator = clamp(elevator, -1.0, 1.0);

    // --- Speed: airspeed error → throttle ---
    double spdErr = m_cmd.get_desired_airspeed_ms() - m_state.get_airspeed_ms();
    double throttle = 0.6 + KP_SPD * spdErr;
    throttle = clamp(throttle, 0.0, 1.0);

    // --- Rudder: zero for now ---
    double rudder = 0.0;

    // --- Output ---
    Ap::SurfaceCmd cmd;
    cmd.set_aileron(aileron);
    cmd.set_elevator(elevator);
    cmd.set_rudder(rudder);
    cmd.set_throttle(throttle);

    if (this->isConnected_surfaceCmdOut_OutputPort(0)) {
        this->surfaceCmdOut_out(0, cmd);
    }

    // --- Telemetry ---
    this->tlmWrite_cmdAileron(aileron);
    this->tlmWrite_cmdElevator(elevator);
    this->tlmWrite_cmdThrottle(throttle);
    this->tlmWrite_headingErr(hdgErr);
    this->tlmWrite_altErr(altErr);
}

// -------------------------------------------------------------------------
// Async input handlers — store latest values
// -------------------------------------------------------------------------
void Controller::guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) {
    m_cmd = cmd;
}

void Controller::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state = state;
}

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------
double Controller::wrapDeg(double deg) {
    while (deg > 180.0)  deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double Controller::clamp(double val, double lo, double hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

}  // namespace Ap
