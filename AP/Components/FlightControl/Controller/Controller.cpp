#include "AP/Components/FlightControl/Controller/Controller.hpp"
#include "AP/Math/ApMath.hpp"
#include <cmath>

namespace Ap {

Controller::Controller(const char* const compName)
    : ControllerComponentBase(compName)
{
    // Default guidance: hold initial conditions (2000m MSL, 50 m/s, heading north)
    m_cmd.set_desired_alt_m(2000.0);
    m_cmd.set_desired_airspeed_ms(50.0);
    m_cmd.set_desired_heading_deg(0.0);
}

Controller::~Controller() {}

// -------------------------------------------------------------------------
// schedIn — called at 100Hz (RG2)
// -------------------------------------------------------------------------
void Controller::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    constexpr double DT = 0.01;  // 100Hz

    // --- Lateral: heading → roll → aileron ---
    double hdgErr = wrapDeg(m_cmd.get_desired_heading_deg() -
                            m_state.get_euler_deg().get_z());

    double desRoll = m_pidHeading.update(hdgErr, DT);

    double rollErr = desRoll - m_state.get_euler_deg().get_x();
    double aileron = m_pidRoll.update(rollErr, DT);

    // --- Longitudinal: altitude → pitch → elevator ---
    // position_ned.z is posD; alt_MSL = ref_alt - posD
    double estAltMsl = Ap::REF_ALT_MSL - m_state.get_position_ned().get_z();
    double altErr = m_cmd.get_desired_alt_m() - estAltMsl;

    double desPitch = m_pidAlt.update(altErr, DT);

    double pitchErr = desPitch - m_state.get_euler_deg().get_y();
    double elevator = m_pidPitch.update(pitchErr, DT);

    // --- Speed: airspeed error → throttle ---
    double spdErr = m_cmd.get_desired_airspeed_ms() - m_state.get_airspeed_ms();
    double throttle = 0.5 + m_pidSpeed.update(spdErr, DT);
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
