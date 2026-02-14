#include "AP/Components/FlightControl/StateEstimator/StateEstimator.hpp"
#include "AP/Math/ApMath.hpp"
#include <cmath>

namespace Ap {

StateEstimator::StateEstimator(const char* const compName)
    : StateEstimatorComponentBase(compName)
{}

StateEstimator::~StateEstimator() {}

// -------------------------------------------------------------------------
// schedIn — called at 100Hz (RG2)
// -------------------------------------------------------------------------
void StateEstimator::schedIn_handler(FwIndexType portNum, U32 context) {
    // Drain all queued sensor messages
    this->dispatchCurrentMessages();

    // Wait until all sensors have reported at least once
    if (!(m_hasImu && m_hasBaro && m_hasGps && m_hasMag)) {
        return;
    }

    constexpr double DT = 0.01;  // 100Hz

    // --- Attitude estimation (complementary filter) ---

    // Gyro rates (deg/s → rad/s): p, q, r in body frame
    const double p = m_imu.get_gyro_dps().get_x() * Ap::DEG2RAD;
    const double q = m_imu.get_gyro_dps().get_y() * Ap::DEG2RAD;
    const double r = m_imu.get_gyro_dps().get_z() * Ap::DEG2RAD;

    // Euler angle kinematic equations (body rates → Euler rates)
    // roll_dot  = p + (q*sin(roll) + r*cos(roll)) * tan(pitch)
    // pitch_dot = q*cos(roll) - r*sin(roll)
    // yaw_dot   = (q*sin(roll) + r*cos(roll)) / cos(pitch)
    const double sr = std::sin(m_roll);
    const double cr = std::cos(m_roll);
    const double tp = std::tan(m_pitch);
    const double cp = std::cos(m_pitch);

    double roll_dot  = p + (q * sr + r * cr) * tp;
    double pitch_dot = q * cr - r * sr;
    double yaw_dot   = (cp > 0.01) ? (q * sr + r * cr) / cp : 0.0;

    double roll_gyro  = m_roll  + roll_dot  * DT;
    double pitch_gyro = m_pitch + pitch_dot * DT;
    double yaw_gyro   = m_yaw   + yaw_dot   * DT;

    // Accel reference: roll and pitch from gravity direction
    const double ax = m_imu.get_accel_mps2().get_x();
    const double ay = m_imu.get_accel_mps2().get_y();
    const double az = m_imu.get_accel_mps2().get_z();

    double roll_accel  = std::atan2(-ay, -az);
    double pitch_accel = std::atan2(-ax, std::sqrt(ay * ay + az * az));

    // Mag reference: yaw (heading) from tilt-compensated magnetometer
    const double mx = m_mag.get_field_gauss().get_x();
    const double my = m_mag.get_field_gauss().get_y();
    const double mz = m_mag.get_field_gauss().get_z();

    // Tilt compensation (reuse sr/cr/cp from kinematic equations above)
    double sinP = std::sin(m_pitch);

    double mx2 = mx * cp + my * sr * sinP + mz * cr * sinP;
    double my2 = my * cr - mz * sr;

    double yaw_mag = std::atan2(-my2, mx2);

    // On first update, snap to sensor values (no history to blend with)
    if (!m_initialized) {
        m_roll  = roll_accel;
        m_pitch = pitch_accel;
        m_yaw   = yaw_mag;
        m_initialized = true;
    } else {
        // Complementary blend
        m_roll  = ALPHA * roll_gyro  + (1.0 - ALPHA) * roll_accel;
        m_pitch = ALPHA * pitch_gyro + (1.0 - ALPHA) * pitch_accel;

        // Yaw wrapping: handle ±pi discontinuity
        double yaw_err = yaw_mag - yaw_gyro;
        if (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
        if (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
        m_yaw = yaw_gyro + (1.0 - ALPHA) * yaw_err;
    }

    // Wrap yaw to [0, 2*pi)
    if (m_yaw < 0.0)       m_yaw += 2.0 * M_PI;
    if (m_yaw >= 2.0 * M_PI) m_yaw -= 2.0 * M_PI;

    // --- Position & velocity: pass through from GPS/baro ---
    // GPS provides lat/lon → we store NED offsets relative to a reference point.
    // For now, use the GPS-derived NED directly (SimGps already provides vel_ned).
    // Altitude: use baro for vertical (more frequent updates than GPS).

    m_posN = m_gps.get_vel_ned_mps().get_x() != 0.0 ? m_posN : 0.0;  // updated via GPS below
    m_velN = m_gps.get_vel_ned_mps().get_x();
    m_velE = m_gps.get_vel_ned_mps().get_y();
    m_velD = m_gps.get_vel_ned_mps().get_z();

    // Altitude from baro (negative of posD in NED)
    // Reference altitude is 1600m MSL (same as sim)
    constexpr double REF_ALT = 1600.0;
    m_posD = -(m_baro.get_altitude_m() - REF_ALT);

    // Horizontal position from GPS: convert lat/lon to NED offset from reference
    constexpr double REF_LAT = 35.0;   // deg
    constexpr double REF_LON = -106.0; // deg
    double dLat = (m_gps.get_lat_deg() - REF_LAT) * Ap::DEG2RAD;
    double dLon = (m_gps.get_lon_deg() - REF_LON) * Ap::DEG2RAD;
    constexpr double R_EARTH = 6378137.0;
    m_posN = dLat * R_EARTH;
    m_posE = dLon * R_EARTH * std::cos(REF_LAT * Ap::DEG2RAD);

    // Airspeed from velocity magnitude (simplified — no wind model yet)
    double airspeed = std::sqrt(m_velN * m_velN + m_velE * m_velE + m_velD * m_velD);

    // --- Build estimated state ---
    double rollDeg  = m_roll  * Ap::RAD2DEG;
    double pitchDeg = m_pitch * Ap::RAD2DEG;
    double yawDeg   = m_yaw   * Ap::RAD2DEG;

    // Euler → quaternion (half-angle trig)
    double chr = std::cos(m_roll  * 0.5);
    double shr = std::sin(m_roll  * 0.5);
    double chp = std::cos(m_pitch * 0.5);
    double shp = std::sin(m_pitch * 0.5);
    double chy = std::cos(m_yaw   * 0.5);
    double shy = std::sin(m_yaw   * 0.5);

    double qw = chr * chp * chy + shr * shp * shy;
    double qx = shr * chp * chy - chr * shp * shy;
    double qy = chr * shp * chy + shr * chp * shy;
    double qz = chr * chp * shy - shr * shp * chy;

    Ap::AircraftState est;
    est.set_position_ned(Ap::Vec3(m_posN, m_posE, m_posD));
    est.set_velocity_ned(Ap::Vec3(m_velN, m_velE, m_velD));
    est.set_attitude_quat(Ap::Quat(qw, qx, qy, qz));
    est.set_euler_deg(Ap::Vec3(rollDeg, pitchDeg, yawDeg));
    est.set_angular_rate_dps(m_imu.get_gyro_dps());
    est.set_airspeed_ms(airspeed);
    est.set_time_s(m_imu.get_time_s());

    // Output to all connected consumers
    for (FwIndexType i = 0; i < 3; i++) {
        if (this->isConnected_stateOut_OutputPort(i)) {
            this->stateOut_out(i, est);
        }
    }

    // Telemetry
    this->tlmWrite_estRoll(rollDeg);
    this->tlmWrite_estPitch(pitchDeg);
    this->tlmWrite_estYaw(yawDeg);
    this->tlmWrite_estAlt(m_baro.get_altitude_m());
    this->tlmWrite_estAirspeed(airspeed);
}

// -------------------------------------------------------------------------
// Sensor input handlers — just store latest values
// -------------------------------------------------------------------------
void StateEstimator::imuIn_handler(FwIndexType portNum, Ap::ImuData& data) {
    m_imu = data;
    m_hasImu = true;
}

void StateEstimator::gpsIn_handler(FwIndexType portNum, Ap::GpsData& data) {
    m_gps = data;
    m_hasGps = true;
}

void StateEstimator::baroIn_handler(FwIndexType portNum, Ap::BaroData& data) {
    m_baro = data;
    m_hasBaro = true;
}

void StateEstimator::magIn_handler(FwIndexType portNum, Ap::MagData& data) {
    m_mag = data;
    m_hasMag = true;
}

}  // namespace Ap
