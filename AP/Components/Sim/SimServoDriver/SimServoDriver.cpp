#include "AP/Components/Sim/SimServoDriver/SimServoDriver.hpp"

namespace Ap {

SimServoDriver::SimServoDriver(const char* const compName)
    : SimServoDriverComponentBase(compName)
{}

SimServoDriver::~SimServoDriver() {}

// -------------------------------------------------------------------------
// surfaceCmdIn — sync handler, forwards command to SimDynamics
// -------------------------------------------------------------------------
void SimServoDriver::surfaceCmdIn_handler(FwIndexType portNum, Ap::SurfaceCmd& cmd) {
    if (!m_firstCmd) {
        this->log_ACTIVITY_HI_FirstCmdReceived();
        m_firstCmd = true;
    }

    // Telemetry
    this->tlmWrite_aileron(cmd.get_aileron());
    this->tlmWrite_elevator(cmd.get_elevator());
    this->tlmWrite_rudder(cmd.get_rudder());
    this->tlmWrite_throttle(cmd.get_throttle());

    // Forward to SimDynamics
    if (this->isConnected_surfaceCmdOut_OutputPort(0)) {
        this->surfaceCmdOut_out(0, cmd);
    }
}

}  // namespace Ap
