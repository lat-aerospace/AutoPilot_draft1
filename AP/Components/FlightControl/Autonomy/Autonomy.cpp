#include "AP/Components/FlightControl/Autonomy/Autonomy.hpp"
#include <cmath>

namespace Ap {

Autonomy::Autonomy(const char* const compName)
    : AutonomyComponentBase(compName)
{}

Autonomy::~Autonomy() {}

// -------------------------------------------------------------------------
// schedIn — called at 10Hz (RG4)
// -------------------------------------------------------------------------
void Autonomy::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    constexpr double DT = 0.1;  // 10Hz

    // --- FBWB mode: sticks → integrated guidance command ---
    //
    // Pitch stick (m_rc.pitch):  +1 = climb, -1 = descend
    //   → integrates desired altitude at ALT_RATE m/s per full stick
    //
    // Roll stick (m_rc.roll):    +1 = turn right, -1 = turn left
    //   → integrates desired heading at HEADING_RATE deg/s per full stick
    //
    // Throttle (m_rc.throttle):  0..1 → maps linearly to SPEED_MIN..SPEED_MAX
    //

    // Altitude: integrate pitch stick
    m_desAlt += static_cast<double>(m_rc.get_pitch()) * ALT_RATE * DT;
    if (m_desAlt < ALT_MIN) m_desAlt = ALT_MIN;
    if (m_desAlt > ALT_MAX) m_desAlt = ALT_MAX;

    // Heading: integrate roll stick
    m_desHeading += static_cast<double>(m_rc.get_roll()) * HEADING_RATE * DT;
    // Wrap to [0, 360)
    if (m_desHeading >= 360.0) m_desHeading -= 360.0;
    if (m_desHeading < 0.0)    m_desHeading += 360.0;

    // Airspeed: throttle maps directly
    double thr = static_cast<double>(m_rc.get_throttle());
    m_desSpeed = SPEED_MIN + thr * (SPEED_MAX - SPEED_MIN);

    // --- Output guidance command ---
    Ap::GuidanceCmd cmd;
    cmd.set_desired_alt_m(m_desAlt);
    cmd.set_desired_airspeed_ms(m_desSpeed);
    cmd.set_desired_heading_deg(m_desHeading);

    if (this->isConnected_guidanceCmdOut_OutputPort(0)) {
        this->guidanceCmdOut_out(0, cmd);
    }

    // Broadcast current mode
    for (FwIndexType i = 0; i < 1; i++) {
        if (this->isConnected_modeOut_OutputPort(i)) {
            this->modeOut_out(i, m_mode);
        }
    }

    // Telemetry
    this->tlmWrite_currentMode(m_mode);
    this->tlmWrite_desAlt(m_desAlt);
    this->tlmWrite_desAirspeed(m_desSpeed);
    this->tlmWrite_desHeading(m_desHeading);
}

// -------------------------------------------------------------------------
// Async input handlers — store latest values
// -------------------------------------------------------------------------
void Autonomy::rcIn_handler(FwIndexType portNum, Ap::RcChannels& rc) {
    m_rc = rc;
}

void Autonomy::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state = state;
}

}  // namespace Ap
