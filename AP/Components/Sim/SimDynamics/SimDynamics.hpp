#ifndef Ap_SimDynamics_HPP
#define Ap_SimDynamics_HPP

#include "AP/Components/Sim/SimDynamics/SimDynamicsComponentAc.hpp"
#include "AP/Components/Sim/SimDynamics/RigidBody6DOF.hpp"

namespace Ap {

class SimDynamics final : public SimDynamicsComponentBase {
  public:
    SimDynamics(const char* const compName);
    ~SimDynamics();

  private:
    // --- Handler implementations ---

    //! schedIn: called at 400Hz by RG1
    void schedIn_handler(FwIndexType portNum, U32 context) override;

    //! surfaceCmdIn: async — queued surface commands from SimServoDriver
    void surfaceCmdIn_handler(FwIndexType portNum, Ap::SurfaceCmd& cmd) override;

    // --- Members ---
    Sim::RigidBody6DOF m_body;
    Sim::SurfaceInput  m_latestCmd{0.0, 0.0, 0.0, 0.6};  // trim throttle
    bool m_started = false;
};

}  // namespace Ap

#endif
