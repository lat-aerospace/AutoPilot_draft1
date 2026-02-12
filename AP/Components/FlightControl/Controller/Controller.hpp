#ifndef Ap_Controller_HPP
#define Ap_Controller_HPP

#include "AP/Components/FlightControl/Controller/ControllerComponentAc.hpp"

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

    // Hardcoded gains
    static constexpr double KP_HDG   = 0.005;   // heading err → desired roll (deg/deg)
    static constexpr double KP_ROLL  = 0.002;  // roll err → aileron
    static constexpr double KP_ALT   = 0.01;   // alt err → desired pitch (deg/m)
    static constexpr double KP_PITCH = 0.003;  // pitch err → elevator
    static constexpr double KP_SPD   = 0.005;   // speed err → throttle
};

}  // namespace Ap

#endif
