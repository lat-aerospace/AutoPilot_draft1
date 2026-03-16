#ifndef Ap_Controller_HPP
#define Ap_Controller_HPP

#include "AP/Components/FlightControl/Controller/ControllerComponentAc.hpp"
#include "AP/Math/PidController.hpp"

namespace Ap {

class Controller final : public ControllerComponentBase {
  public:
    Controller(const char* const compName);
    ~Controller();

  private:
    // Port handlers
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) override;
    void stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;

    // Wrap angle to [-180, +180] degrees
    static double wrapDeg(double deg);

    // Clamp value to [lo, hi]
    static double clamp(double val, double lo, double hi);

    // Latest inputs
    Ap::GuidanceCmd   m_cmd;
    Ap::AircraftState m_state;

    // Cascaded PID controllers
    //                        kp     ki     kd    iMax  outMin  outMax
    PidController m_pidHeading{0.8,   0.05,  0.1,  15.0, -30.0,  30.0};   // heading err → desired roll (deg)
    PidController m_pidRoll   {0.02,  0.005, 0.003, 0.3, -1.0,   1.0};    // roll err → aileron
    PidController m_pidAlt    {0.15,  0.01,  0.05, 10.0, -15.0,  15.0};   // alt err → desired pitch (deg)
    PidController m_pidPitch  {0.03,  0.005, 0.005, 0.3, -1.0,   1.0};    // pitch err → elevator
    PidController m_pidSpeed  {0.05,  0.01,  0.01,  0.3, -0.5,   0.5};    // speed err → throttle offset
};

}  // namespace Ap

#endif
