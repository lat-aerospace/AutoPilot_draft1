#ifndef Ap_NavController_HPP
#define Ap_NavController_HPP

#include "AP/Components/FlightControl/NavController/NavControllerComponentAc.hpp"
#include "AP_TECS/AP_TECS.hpp"
#include "AP_L1_Control/AP_L1_Control.hpp"

namespace Ap {

class NavController final : public NavControllerComponentBase {
  public:
    NavController(const char* compName);
    ~NavController();

  private:
    // ---------------------------------------------------------------
    // Port handlers
    // ---------------------------------------------------------------
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) override;
    void stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;

    // ---------------------------------------------------------------
    // Command handlers
    // ---------------------------------------------------------------
    void SetL1Period_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 period_s) override;
    void SetAltGain_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 kp) override;
    void SetThrGain_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, F32 kp) override;

    // ---------------------------------------------------------------
    // Initial cmd defaults — match sim reset() initial conditions
    // RigidBody6DOF::REF_ALT_MSL (1600 m) + default AGL (1000 m) = 2600 m
    // ---------------------------------------------------------------
    static constexpr double INIT_ALT_M    = 2600.0;  // m MSL
    static constexpr double INIT_SPEED_MS = 50.0;    // m/s (= AircraftParams::V_trim)

    // ---------------------------------------------------------------
    // Latest inputs (updated by async handlers)
    // ---------------------------------------------------------------
    Ap::GuidanceCmd   m_cmd;
    Ap::AircraftState m_state;

    // ---------------------------------------------------------------
    // Algorithm libraries
    // ---------------------------------------------------------------
    Ap::AP_TECS       m_tecs;
    Ap::AP_L1_Control m_l1;
};

}  // namespace Ap

#endif
