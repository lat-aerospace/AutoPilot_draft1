#include "AP/Components/FlightControl/NavController/NavController.hpp"
#include <cmath>

namespace Ap {

NavController::NavController(const char* compName)
    : NavControllerComponentBase(compName)
{
    // Default guidance: hold starting altitude + trim speed, heading north
    m_cmd.set_desired_alt_m(INIT_ALT_M);
    m_cmd.set_desired_airspeed_ms(INIT_SPEED_MS);
    m_cmd.set_desired_heading_deg(0.0);
}

NavController::~NavController() {}

// =========================================================================
// schedIn — called at 10Hz (RG4)
// =========================================================================
void NavController::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    constexpr double DT = 0.1;  // 10Hz

    const auto& pos = m_state.get_position_ned();
    const auto& vel = m_state.get_velocity_ned();

    // Altitude MSL (NED reference altitude = 1600m MSL, posD is negative of AGL)
    const double alt_msl = 1600.0 - pos.get_z();
    const double vn = vel.get_x();
    const double ve = vel.get_y();
    const double gs = std::sqrt(vn * vn + ve * ve);  // ground speed

    // -------------------------------------------------------------------
    // TECS update: total energy → throttle + pitch
    // -------------------------------------------------------------------
    m_tecs.update(
        m_cmd.get_desired_alt_m(),
        m_cmd.get_desired_airspeed_ms(),
        alt_msl,
        m_state.get_airspeed_ms(),
        DT
    );
    const auto& tecs = m_tecs.get();

    // -------------------------------------------------------------------
    // L1 heading hold: desired heading → roll
    // -------------------------------------------------------------------
    const double hdg_rad     = m_state.get_euler_deg().get_z() * Ap::DEG2RAD;
    const double des_hdg_rad = m_cmd.get_desired_heading_deg() * Ap::DEG2RAD;
    m_l1.updateHeadingHold(des_hdg_rad, hdg_rad, gs > 5.0 ? gs : 50.0);
    const auto& l1 = m_l1.get();

    // -------------------------------------------------------------------
    // Build desired attitude and output
    // -------------------------------------------------------------------
    Ap::DesiredAttitude att;
    att.set_roll_deg(l1.roll_rad * Ap::RAD2DEG);
    att.set_pitch_deg(tecs.pitch_rad * Ap::RAD2DEG);
    att.set_throttle(tecs.throttle);

    if (this->isConnected_desiredAttOut_OutputPort(0)) {
        this->desiredAttOut_out(0, att);
    }

    // -------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------
    this->tlmWrite_desiredRoll(att.get_roll_deg());
    this->tlmWrite_desiredPitch(att.get_pitch_deg());
    this->tlmWrite_desiredThrottle(tecs.throttle);

    double hdgErr = m_cmd.get_desired_heading_deg() - m_state.get_euler_deg().get_z();
    while (hdgErr >  180.0) hdgErr -= 360.0;
    while (hdgErr < -180.0) hdgErr += 360.0;
    this->tlmWrite_headingErr(hdgErr);
    this->tlmWrite_altErr(m_cmd.get_desired_alt_m() - alt_msl);
}

// =========================================================================
// Async input handlers
// =========================================================================
void NavController::guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) {
    m_cmd = cmd;
}

void NavController::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state = state;
}

// =========================================================================
// Command handlers — live gain tuning from fprime-gds
// =========================================================================
void NavController::SetL1Period_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 period_s) {
    m_l1.setL1Period(static_cast<double>(period_s));
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void NavController::SetAltGain_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 kp) {
    m_tecs.setPitGain(static_cast<double>(kp));
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void NavController::SetThrGain_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 kp) {
    m_tecs.setThrGain(static_cast<double>(kp));
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Ap
