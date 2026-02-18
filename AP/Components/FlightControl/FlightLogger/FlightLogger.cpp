#include "AP/Components/FlightControl/FlightLogger/FlightLogger.hpp"
#include <cstdio>
#include <ctime>
#include <sys/stat.h>

namespace Ap {

FlightLogger::FlightLogger(const char* compName)
    : FlightLoggerComponentBase(compName)
{}

FlightLogger::~FlightLogger() {
    if (m_file.is_open()) {
        m_file.close();
    }
}

void FlightLogger::openFile() {
    // Ensure logs/ directory exists
    ::mkdir("logs", 0755);

    // Build timestamped filename: logs/flight_YYYYMMDD_HHMMSS.csv
    std::time_t now = std::time(nullptr);
    struct std::tm* t = std::localtime(&now);
    char fname[64];
    std::snprintf(fname, sizeof(fname),
                  "logs/flight_%04d%02d%02d_%02d%02d%02d.csv",
                  t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                  t->tm_hour, t->tm_min, t->tm_sec);

    m_file.open(fname);
    if (!m_file.is_open()) return;

    // CSV header
    m_file << "time_s,"
           << "airspeed_ms,"
           << "alt_msl_m,"
           << "posN_m,posE_m,posD_m,"
           << "roll_deg,pitch_deg,yaw_deg,"
           << "des_alt_m,des_airspeed_ms,des_heading_deg\n";
}

// -------------------------------------------------------------------------
// schedIn — 10 Hz, writes one CSV row from buffered state
// -------------------------------------------------------------------------
void FlightLogger::schedIn_handler(FwIndexType portNum, U32 context) {
    this->dispatchCurrentMessages();

    if (!m_hasState) return;

    if (!m_file.is_open()) {
        openFile();
        if (!m_file.is_open()) return;
    }

    const auto& pos   = m_state.get_position_ned();
    const auto& euler = m_state.get_euler_deg();

    m_file << m_state.get_time_s()              << ','
           << m_state.get_airspeed_ms()          << ','
           << (1600.0 - pos.get_z())             << ','   // alt MSL = REF(1600) - posD
           << pos.get_x()                        << ','   // posN
           << pos.get_y()                        << ','   // posE
           << pos.get_z()                        << ','   // posD
           << euler.get_x()                      << ','   // roll
           << euler.get_y()                      << ','   // pitch
           << euler.get_z()                      << ','   // yaw
           << m_cmd.get_desired_alt_m()           << ','
           << m_cmd.get_desired_airspeed_ms()     << ','
           << m_cmd.get_desired_heading_deg()     << '\n';
}

// -------------------------------------------------------------------------
// Async handlers — buffer latest values
// -------------------------------------------------------------------------
void FlightLogger::stateIn_handler(FwIndexType portNum, Ap::AircraftState& state) {
    m_state    = state;
    m_hasState = true;
}

void FlightLogger::guidanceCmdIn_handler(FwIndexType portNum, Ap::GuidanceCmd& cmd) {
    m_cmd = cmd;
}

}  // namespace Ap
