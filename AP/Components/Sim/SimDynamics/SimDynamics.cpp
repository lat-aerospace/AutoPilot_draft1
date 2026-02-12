#include "AP/Components/Sim/SimDynamics/SimDynamics.hpp"

namespace Ap {

SimDynamics::SimDynamics(const char* const compName)
    : SimDynamicsComponentBase(compName)
{}

SimDynamics::~SimDynamics() {}

// -------------------------------------------------------------------------
// schedIn — called at 400Hz by rate group 1
// -------------------------------------------------------------------------
void SimDynamics::schedIn_handler(FwIndexType portNum, U32 context) {
    // Drain async queue (processes any queued surfaceCmdIn messages)
    this->dispatchCurrentMessages();

    // One-time startup event
    if (!m_started) {
        this->log_ACTIVITY_HI_SimStarted();
        m_started = true;
    }

    // Step physics: dt = 2.5ms (400Hz)
    constexpr double DT = 0.0025;
    m_body.setSurfaces(m_latestCmd);
    m_body.step(DT);

    // --- Build truth state for output ---
    const auto pos  = m_body.positionNED();
    const auto vel  = m_body.velocityNED();
    const auto quat = m_body.quaternion();
    const auto eul  = m_body.eulerDeg();
    const auto rate = m_body.angularRateDps();

    Ap::AircraftState truth;
    truth.set_position_ned(Ap::Vec3(pos.x(), pos.y(), pos.z()));
    truth.set_velocity_ned(Ap::Vec3(vel.x(), vel.y(), vel.z()));
    truth.set_attitude_quat(Ap::Quat(quat.w(), quat.x(), quat.y(), quat.z()));
    truth.set_euler_deg(Ap::Vec3(eul.x(), eul.y(), eul.z()));
    truth.set_angular_rate_dps(Ap::Vec3(rate.x(), rate.y(), rate.z()));
    truth.set_airspeed_ms(m_body.airspeed());
    truth.set_time_s(m_body.simTime());

    // Output truth state to all connected sim sensors
    for (FwIndexType i = 0; i < 4; i++) {
        if (this->isConnected_truthStateOut_OutputPort(i)) {
            this->truthStateOut_out(i, truth);
        }
    }

    // --- Telemetry ---
    this->tlmWrite_posN(pos.x());
    this->tlmWrite_posE(pos.y());
    this->tlmWrite_posD(pos.z());
    this->tlmWrite_roll(eul.x());
    this->tlmWrite_pitch(eul.y());
    this->tlmWrite_yaw(eul.z());
    this->tlmWrite_airspeed(m_body.airspeed());
    this->tlmWrite_simTime(m_body.simTime());
}

// -------------------------------------------------------------------------
// surfaceCmdIn — async handler, stores latest command
// -------------------------------------------------------------------------
void SimDynamics::surfaceCmdIn_handler(FwIndexType portNum, Ap::SurfaceCmd& cmd) {
    m_latestCmd.aileron  = cmd.get_aileron();
    m_latestCmd.elevator = cmd.get_elevator();
    m_latestCmd.rudder   = cmd.get_rudder();
    m_latestCmd.throttle = cmd.get_throttle();
}

}  // namespace Ap
