#include "AP/Components/FlightControl/StateEstimator/StateEstimator.hpp"
#include "AP_Math/AP_Math.hpp"
#include <cmath>

namespace Ap {

StateEstimator::StateEstimator(const char* const compName)
    : StateEstimatorComponentBase(compName)
{
    m_ahrs.init();
}

StateEstimator::~StateEstimator() {}

// -------------------------------------------------------------------------
// schedIn — called at 100Hz (RG2)
// -------------------------------------------------------------------------
void StateEstimator::schedIn_handler(FwIndexType portNum, U32 context) {
    // Drain all queued sensor messages first
    this->dispatchCurrentMessages();

    // Wait until all sensors have delivered at least one reading
    if (!(m_hasImu && m_hasBaro && m_hasGps && m_hasMag)) {
        return;
    }

    constexpr double DT = 0.01;  // 100Hz

    // -------------------------------------------------------------------
    // Convert F Prime sensor types to AP_ measurement types
    // -------------------------------------------------------------------
    ImuMeasurement imu;
    imu.accel_mps2 = Vec3d(m_imu.get_accel_mps2().get_x(),
                           m_imu.get_accel_mps2().get_y(),
                           m_imu.get_accel_mps2().get_z());
    imu.gyro_dps   = Vec3d(m_imu.get_gyro_dps().get_x(),
                           m_imu.get_gyro_dps().get_y(),
                           m_imu.get_gyro_dps().get_z());
    imu.time_s     = m_imu.get_time_s();

    GpsMeasurement gps;
    gps.lat_deg     = m_gps.get_lat_deg();
    gps.lon_deg     = m_gps.get_lon_deg();
    gps.alt_msl_m   = m_gps.get_alt_msl_m();
    gps.vel_ned_mps = Vec3d(m_gps.get_vel_ned_mps().get_x(),
                            m_gps.get_vel_ned_mps().get_y(),
                            m_gps.get_vel_ned_mps().get_z());
    gps.time_s      = m_gps.get_time_s();
    gps.fix_type    = GpsFixType::FIX_3D;
    gps.num_sats    = 10;
    gps.hdop        = 1.2;
    gps.vdop        = 1.8;

    BaroMeasurement baro;
    baro.pressure_pa = m_baro.get_pressure_pa();
    baro.altitude_m  = m_baro.get_altitude_m();
    baro.time_s      = m_baro.get_time_s();

    CompassMeasurement mag;
    mag.field_gauss     = Vec3d(m_mag.get_field_gauss().get_x(),
                                m_mag.get_field_gauss().get_y(),
                                m_mag.get_field_gauss().get_z());
    mag.declination_deg = 8.9;   // Albuquerque, NM
    mag.inclination_deg = 60.0;  // Albuquerque, NM
    mag.time_s          = m_mag.get_time_s();

    // -------------------------------------------------------------------
    // Run Mahony filter update
    // -------------------------------------------------------------------
    m_ahrs.update(imu,
                  gps,  m_gpsNew,
                  baro, m_baroNew,
                  mag,  m_magNew,
                  DT);

    // Reset freshness flags
    m_gpsNew  = false;
    m_baroNew = false;
    m_magNew  = false;

    const AhrsState& s = m_ahrs.get();
    if (!s.initialized) {
        return;
    }

    // -------------------------------------------------------------------
    // Convert AhrsState → F Prime AircraftState
    // -------------------------------------------------------------------
    Ap::AircraftState est;
    est.set_position_ned(Ap::Vec3(s.position_ned.x(),
                                  s.position_ned.y(),
                                  s.position_ned.z()));
    est.set_velocity_ned(Ap::Vec3(s.velocity_ned.x(),
                                  s.velocity_ned.y(),
                                  s.velocity_ned.z()));
    est.set_attitude_quat(Ap::Quat(s.attitude.w(),
                                   s.attitude.x(),
                                   s.attitude.y(),
                                   s.attitude.z()));
    est.set_euler_deg(Ap::Vec3(s.euler_deg.x(),
                               s.euler_deg.y(),
                               s.euler_deg.z()));
    est.set_angular_rate_dps(Ap::Vec3(s.angular_rate_dps.x(),
                                      s.angular_rate_dps.y(),
                                      s.angular_rate_dps.z()));
    est.set_airspeed_ms(s.airspeed_ms);
    est.set_time_s(s.time_s);

    // Output to all 6 connected consumers
    for (FwIndexType i = 0; i < 6; i++) {
        if (this->isConnected_stateOut_OutputPort(i)) {
            this->stateOut_out(i, est);
        }
    }

    // -------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------
    this->tlmWrite_estRoll(s.euler_deg.x());
    this->tlmWrite_estPitch(s.euler_deg.y());
    this->tlmWrite_estYaw(s.euler_deg.z());
    this->tlmWrite_estAlt(s.altitude_m);
    this->tlmWrite_estAirspeed(s.airspeed_ms);
}

// -------------------------------------------------------------------------
// Sensor input handlers — store latest and mark as fresh
// -------------------------------------------------------------------------
void StateEstimator::imuIn_handler(FwIndexType portNum, Ap::ImuData& data) {
    m_imu    = data;
    m_hasImu = true;
}

void StateEstimator::gpsIn_handler(FwIndexType portNum, Ap::GpsData& data) {
    m_gps    = data;
    m_hasGps = true;
    m_gpsNew = true;
}

void StateEstimator::baroIn_handler(FwIndexType portNum, Ap::BaroData& data) {
    m_baro    = data;
    m_hasBaro = true;
    m_baroNew = true;
}

void StateEstimator::magIn_handler(FwIndexType portNum, Ap::MagData& data) {
    m_mag    = data;
    m_hasMag = true;
    m_magNew = true;
}

// -------------------------------------------------------------------------
// Command handler — live Mahony gain tuning
// -------------------------------------------------------------------------
void StateEstimator::SetMahonyGains_cmdHandler(FwOpcodeType opCode, U32 cmdSeq,
                                                F32 kp, F32 ki) {
    AhrsParams p = m_ahrs.params();
    p.Kp = static_cast<double>(kp);
    p.Ki = static_cast<double>(ki);
    m_ahrs.setParams(p);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Ap
