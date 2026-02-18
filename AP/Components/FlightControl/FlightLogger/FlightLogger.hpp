#ifndef Ap_FlightLogger_HPP
#define Ap_FlightLogger_HPP

#include "AP/Components/FlightControl/FlightLogger/FlightLoggerComponentAc.hpp"
#include <fstream>

namespace Ap {

// FlightLogger — buffers the latest AircraftState and GuidanceCmd, then
// writes one CSV row per schedIn call (10 Hz).
//
// Output file: logs/flight_log.csv
// Columns: time_s, airspeed_ms, alt_msl_m,
//          posN_m, posE_m, posD_m,
//          roll_deg, pitch_deg, yaw_deg,
//          des_alt_m, des_airspeed_ms, des_heading_deg
class FlightLogger final : public FlightLoggerComponentBase {
  public:
    FlightLogger(const char* compName);
    ~FlightLogger();

  private:
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) override;
    void guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) override;

    void openFile();

    std::ofstream     m_file;
    Ap::AircraftState m_state;
    Ap::GuidanceCmd   m_cmd;
    bool              m_hasState = false;
};

}  // namespace Ap
#endif
