#ifndef Ap_StateEstimator_HPP
#define Ap_StateEstimator_HPP

#include "AP/Components/FlightControl/StateEstimator/StateEstimatorComponentAc.hpp"
#include "AP/Math/ApMath.hpp"

namespace Ap {

class StateEstimator final : public StateEstimatorComponentBase {
  public:
    StateEstimator(const char* const compName);
    ~StateEstimator();

  private:
    // Port handlers
    void schedIn_handler(FwIndexType portNum, U32 context) override;
    void imuIn_handler(FwIndexType portNum, Ap::ImuData& data) override;
    void gpsIn_handler(FwIndexType portNum, Ap::GpsData& data) override;
    void baroIn_handler(FwIndexType portNum, Ap::BaroData& data) override;
    void magIn_handler(FwIndexType portNum, Ap::MagData& data) override;

    // Complementary filter blend factor (0..1, higher = more gyro trust)
    static constexpr double ALPHA = 0.98;

    // Estimated attitude (Euler angles in radians, internally)
    double m_roll  = 0.0;
    double m_pitch = 0.0;
    double m_yaw   = 0.0;

    // Latest sensor data
    Ap::ImuData  m_imu;
    Ap::GpsData  m_gps;
    Ap::BaroData m_baro;
    Ap::MagData  m_mag;

    // Position NED (from GPS)
    double m_posN = 0.0;
    double m_posE = 0.0;
    double m_posD = -1000.0;  // match initial altitude (1000m AGL)

    // Velocity NED (from GPS)
    double m_velN = 50.0;  // match initial airspeed (heading north)
    double m_velE = 0.0;
    double m_velD = 0.0;

    bool m_initialized = false;

    // Track sensor availability — don't output until all have reported
    bool m_hasImu  = false;
    bool m_hasBaro = false;
    bool m_hasGps  = false;
    bool m_hasMag  = false;
};

}  // namespace Ap

#endif
