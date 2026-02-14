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

    // Gains (tuned for Cessna 172 at ~50 m/s)
    static constexpr double KP_HDG   = 0.08;    // heading err → desired roll (deg/deg)
    static constexpr double KP_ROLL  = 0.001;   // roll err → aileron (reduced: 5x more authority)
    static constexpr double KP_ALT   = 0.02;    // alt err → desired pitch (deg/m)
    static constexpr double KP_PITCH = -0.002;  // pitch err → elevator (negative: Cmde < 0)
    static constexpr double KP_SPD   = 0.008;   // speed err → throttle
};

}  // namespace Ap

#endif
