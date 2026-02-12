#ifndef Ap_SimServoDriver_HPP
#define Ap_SimServoDriver_HPP

#include "AP/Components/Sim/SimServoDriver/SimServoDriverComponentAc.hpp"

namespace Ap {

class SimServoDriver final : public SimServoDriverComponentBase {
  public:
    SimServoDriver(const char* const compName);
    ~SimServoDriver();

  private:
    //! surfaceCmdIn: sync — receives command, forwards to SimDynamics
    void surfaceCmdIn_handler(FwIndexType portNum, Ap::SurfaceCmd& cmd) override;

    bool m_firstCmd = false;
};

}  // namespace Ap

#endif
